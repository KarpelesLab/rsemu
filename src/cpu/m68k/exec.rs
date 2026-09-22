//! The MC68000 interpreter.
//!
//! # One bus access is four cycles, and nothing else costs anything
//!
//! A 68000 bus cycle is four clocks (MC68000UM §5.1, *Data Transfer
//! Operations*), and every published instruction time is a sum of bus cycles
//! and internal cycles. So this interpreter has no per-instruction cycle
//! table: [`Exec::read_word`] and friends charge four each, and
//! [`Exec::internal`] charges the microcode idle time the manual's section 8
//! tables call for, at the point in the instruction where it happens. A
//! `MOVE.B (d16,An),(d16,An)` costs 20 because it makes five bus accesses,
//! not because a table says 20.
//!
//! # The prefetch queue is real state
//!
//! The 68000 keeps two instruction words on hand and refills them one bus
//! cycle at a time. That is observable — it is why `MOVE.W <mem>,($xxxxxxxx).L`
//! writes its operand *before* its last instruction fetch while the same move
//! from a register does not, and why the program counter an address-error
//! frame pushes is not the address of the faulting instruction — so it is
//! modelled explicitly rather than approximated by "fetch the whole
//! instruction, then execute it".
//!
//! The invariant is one line:
//!
//! > [`State::prefetch`]`[0]` is the word at [`State::pc`], and `prefetch[1]`
//! > is the word at `pc + 2`.
//!
//! Executing an instruction *slides* the queue once per instruction word:
//! [`Exec::slide`] shifts `prefetch[1]` down, reads a fresh word from
//! `pc + 4`, and advances `pc` by two. An extension word is therefore taken
//! from `prefetch[1]` (which is where the assembler put it) and the slide that
//! follows is the bus cycle the manual counts against that operand. The last
//! slide of an instruction is what leaves `prefetch[0]` holding the *next*
//! opcode.
//!
//! Because `pc` only moves when a slide completes, the value an exception
//! frame pushes is exactly the hardware's — which is the whole reason to model
//! the queue rather than a byte cursor.
//!
//! # One interpreter, several processors
//!
//! The 68010 and 68020 run through the same code, with the model deciding the
//! few places they differ: which rows decode (`isa.rs`), which frame an
//! exception builds (`enter_exception`, `fault`), whether an odd operand is an
//! address error, whether a fetch that fails faults at once or when its word is
//! used, and how an indexed extension word is read. The 68020's time comes from
//! `timing.rs` and replaces the per-access count at the end of each step; the
//! accesses themselves are the same ones, so a device sees the same bus cycles
//! in the same order.
//!
//! # Sources
//!
//! *M68000 Family Programmer's Reference Manual* (M68000PRM) for every
//! instruction's operation and condition-code rules — the per-instruction
//! pages, which are the only place the irregular rules are stated. The
//! *MC68000 User's Manual* (MC68000UM) §6 for exception processing and the two
//! stack-frame formats, and §8 for instruction timing. `docs/cpu/m68k.md`
//! records where to find both. No copyleft emulator was consulted.

use crate::core::space::{AddressSpace, MemAttrs};
use crate::core::value::Width;

use super::fpu::{self, Fpu, bits as fpbits};
use super::isa::fp::{self, Fmt, Forced, FpOp, ListMode, Pred};
use super::isa::pmmu;
use super::isa::{
    Arg, Cond, Copro, EaSet, FieldSpec, FullExt, Indirect, Insn, Mode, Model, Op, Size, ctrl,
    decode_for, decode_with, ea_of, is_full_format,
};
use super::mmu::{self, Entry, Mmu, mmusr, tc, tt};
use super::mmu040::{self, Regs040 as Mmu040, mmusr as mmusr040, tcr, ttr};
use super::timing;
use super::{Config, Lines, flags, vector};
use crate::float::x87::{self, F80};
use crate::float::{Flags, Spec};

/// Function codes, as they appear on FC0–FC2 and in a group-0 stack frame's
/// special status word (MC68000UM §3.1.1).
mod fc {
    /// User data space.
    pub(super) const USER_DATA: u8 = 1;
    /// User program space.
    pub(super) const USER_PROGRAM: u8 = 2;
    /// Supervisor data space.
    pub(super) const SUPER_DATA: u8 = 5;
    /// Supervisor program space.
    pub(super) const SUPER_PROGRAM: u8 = 6;
}

/// Everything one core owns, minus the interrupt pins.
///
/// Split from [`super::M68k`] because the pins live outside the execution
/// lock: a device asserting an interrupt from inside a CPU-initiated write
/// would otherwise re-enter the CPU's own critical section and deadlock (the
/// re-entrancy contract, `ROADMAP.md` §4.7).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct State {
    /// Which processor this is. Configuration rather than state — it is never
    /// saved — but every method below needs it, so it travels with the
    /// registers it decides the shape of.
    pub model: Model,
    /// The eight data registers.
    pub d: [u32; 8],
    /// The eight address registers. `a[7]` is whichever stack pointer the
    /// current privilege state selects.
    pub a: [u32; 8],
    /// The stack pointers, by [`Bank`]: user, interrupt (the 68000's "SSP")
    /// and master.
    ///
    /// The active one is stale here — `a[7]` is authoritative for it — so
    /// every `a[reg]` access in the interpreter stays a plain array index and
    /// the bank swap happens in exactly one place ([`State::set_sr`]). A 68000
    /// or 68010 never selects the master bank, because it has no **M** bit.
    pub banks: [u32; 3],
    /// The program counter: the address of `prefetch[0]`.
    pub pc: u32,
    /// The status register. See [`super::flags`].
    pub sr: u16,
    /// The two-word instruction prefetch queue.
    pub prefetch: [u16; 2],
    /// Bus and internal cycles since power-on.
    pub cycles: u64,
    /// A double bus fault stopped the processor; only a reset restarts it.
    pub halted: bool,
    /// `STOP` was executed and no interrupt has arrived yet.
    pub stopped: bool,
    /// A reset was requested and its sequence has not run yet.
    pub reset_pending: bool,
    /// How many accesses the address space refused.
    pub faults: u64,
    /// Address of the most recent refused access.
    pub last_fault: u32,
    /// Clocks owed to the next scheduler budget.
    ///
    /// A 68000 cannot be stopped mid-instruction, so a budget that runs out
    /// part-way through one is overshot. The scheduler refuses a `Consumed`
    /// larger than the budget it handed out — rightly, since that would put
    /// the domain ahead of the timeline — so the overshoot is carried here and
    /// charged against the next budget instead. Architectural, because a
    /// restored machine that forgot its debt runs one instruction free.
    pub debt: u64,
    /// The vector base register (68010 on): where the vector table starts.
    pub vbr: u32,
    /// The source function code register, three bits (68010 on).
    pub sfc: u8,
    /// The destination function code register, three bits (68010 on).
    pub dfc: u8,
    /// The cache control register's **E** and **F** bits (68020).
    pub cacr: u32,
    /// The cache address register (68020).
    pub caar: u32,
    /// A bus access software has already completed, owed to the instruction
    /// an `RTE` from a long fault frame is restarting. See [`Replay`].
    pub replay: Option<Replay>,
    /// Instruction words a 68020 failed to fetch into `prefetch[0]` and
    /// `prefetch[1]`, by address: the bus error is owed to whatever consumes
    /// them (MC68020UM §6.1.2). Always `None` on the other models, which
    /// fault on the fetch itself.
    pub poison: [Option<u32>; 2],
    /// The vector of the most recent exception taken, for tests that need to
    /// know *which* exception a step ended in. Never saved.
    pub last_vector: Option<u8>,
    /// The floating-point coprocessor's registers.
    ///
    /// Present on every model for the same reason the MMU is: [`State`] is
    /// one type. A core with no coprocessor never reaches them, because the
    /// instructions that name them are not in its opcode map.
    pub fpu: Fpu,
    /// The 68030's memory management unit: six registers and a cache.
    ///
    /// Present on every model, because [`State`] is one type; a part without
    /// an MMU never reaches it, and nothing can write its registers because
    /// the instructions that do are not in that model's opcode map.
    pub mmu: Mmu,
    /// The 68040's memory management unit, which shares nothing with the
    /// 68030's but the job — see `mmu040.rs` for the table of differences.
    pub mmu040: Mmu040,
}

/// The three stack-pointer banks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Bank {
    /// `USP`, selected when **S** is clear.
    User = 0,
    /// `ISP` — the 68000's `SSP` — selected when **S** is set and **M** is
    /// clear.
    Interrupt = 1,
    /// `MSP`, selected when both **S** and **M** are set; 68020 only.
    Master = 2,
}

/// One bus access that an exception handler completed in software.
///
/// A 68010 handler that sets the rerun flag in a long frame's special status
/// word, or a 68020 handler that clears the data-fault flag, is saying "I did
/// that access myself" — the read's data is in the frame's data input buffer,
/// or the write has been made. This core cannot resume an instruction half
/// way through it, so `RTE` restarts the instruction from its first word,
/// and the one access the frame described is satisfied from here instead of
/// from the bus when the restarted instruction reaches it (MC68000UM §6.3.9.2;
/// MC68020UM §6.2.2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct Replay {
    /// The address of the access.
    pub addr: u32,
    /// Whether it was a read.
    pub read: bool,
    /// One or two bytes.
    pub width: u8,
    /// What the read returns; ignored for a write, which is simply dropped.
    pub data: u32,
}

impl State {
    /// Power-on state, before the reset sequence has run.
    pub(super) const fn new(model: Model) -> State {
        State {
            model,
            d: [0; 8],
            a: [0; 8],
            banks: [0; 3],
            pc: 0,
            // Supervisor state with every interrupt masked, which is what a
            // reset leaves behind (MC68000UM §6.2.6).
            sr: flags::S | flags::IPL,
            prefetch: [0; 2],
            cycles: 0,
            halted: false,
            stopped: false,
            reset_pending: true,
            faults: 0,
            last_fault: 0,
            debt: 0,
            vbr: 0,
            sfc: 0,
            dfc: 0,
            cacr: 0,
            caar: 0,
            replay: None,
            poison: [None, None],
            last_vector: None,
            fpu: Fpu::RESET,
            mmu: Mmu::RESET,
            mmu040: Mmu040::RESET,
        }
    }

    /// Whether the core is in supervisor state.
    #[inline]
    pub(super) const fn supervisor(&self) -> bool {
        self.sr & flags::S != 0
    }

    /// The bank a status register value selects.
    #[inline]
    pub(super) const fn bank_of(&self, sr: u16) -> Bank {
        if sr & flags::S == 0 {
            Bank::User
        } else if sr & flags::M != 0 && self.model.has_020() {
            Bank::Master
        } else {
            Bank::Interrupt
        }
    }

    /// A stack pointer, whichever bank it is in.
    #[must_use]
    pub(super) const fn sp(&self, bank: Bank) -> u32 {
        if bank as u8 == self.bank_of(self.sr) as u8 {
            self.a[7]
        } else {
            self.banks[bank as usize]
        }
    }

    /// Overwrite a stack pointer, whichever bank it is in.
    pub(super) const fn set_sp(&mut self, bank: Bank, value: u32) {
        if bank as u8 == self.bank_of(self.sr) as u8 {
            self.a[7] = value;
        } else {
            self.banks[bank as usize] = value;
        }
    }

    /// The user stack pointer, whichever bank it is in.
    #[must_use]
    pub(super) const fn usp(&self) -> u32 {
        self.sp(Bank::User)
    }

    /// The supervisor stack pointer — the interrupt stack pointer on a 68020 —
    /// whichever bank it is in.
    #[must_use]
    pub(super) const fn ssp(&self) -> u32 {
        self.sp(Bank::Interrupt)
    }

    /// Overwrite the user stack pointer.
    pub(super) const fn set_usp(&mut self, value: u32) {
        self.set_sp(Bank::User, value);
    }

    /// The status register bits this model has storage for.
    #[inline]
    pub(super) const fn sr_mask(&self) -> u16 {
        flags::implemented(self.model)
    }

    /// The `CACR` bits this model has storage for.
    #[inline]
    pub(super) const fn cacr_mask(&self) -> u32 {
        if self.model.has_040() {
            CACR_STORED_040
        } else if self.model.has_030() {
            CACR_STORED_030
        } else if self.model.has_020() {
            CACR_STORED_020
        } else {
            0
        }
    }

    /// Write the status register, swapping stack pointers if **S** or **M**
    /// changed.
    ///
    /// The bank swap is the whole reason this is a method: `A7` names a
    /// different physical register in each state, and a `MOVE to SR` that
    /// left the old one in place is the classic supervisor-mode bug
    /// (M68000PRM, *MOVE to SR*).
    pub(super) const fn set_sr(&mut self, value: u16) {
        let value = value & self.sr_mask();
        let old = self.bank_of(self.sr);
        let new = self.bank_of(value);
        if old as u8 != new as u8 {
            self.banks[old as usize] = self.a[7];
            self.a[7] = self.banks[new as usize];
        }
        self.sr = value;
    }

    /// The condition code register: the low byte of `SR`.
    #[inline]
    pub(super) const fn ccr(&self) -> u8 {
        (self.sr & flags::CCR) as u8
    }

    /// Whether a status flag is set.
    #[inline]
    pub(super) const fn flag(&self, mask: u16) -> bool {
        self.sr & mask != 0
    }

    /// The interrupt priority mask, 0–7.
    #[inline]
    pub(super) const fn ipl_mask(&self) -> u8 {
        ((self.sr & flags::IPL) >> 8) as u8
    }

    /// Evaluate one of the sixteen condition codes against the current flags.
    ///
    /// M68000PRM §3.2, *Condition Tests*. Written out rather than derived from
    /// a formula because the manual writes it out: `GT` is
    /// `N·V·Z̄ + N̄·V̄·Z̄`, and any "simplification" of that is where the bugs
    /// live.
    #[must_use]
    pub(super) const fn test(&self, cond: Cond) -> bool {
        let c = self.flag(flags::C);
        let v = self.flag(flags::V);
        let z = self.flag(flags::Z);
        let n = self.flag(flags::N);
        match cond.0 {
            0x0 => true,
            0x1 => false,
            0x2 => !c && !z,
            0x3 => c || z,
            0x4 => !c,
            0x5 => c,
            0x6 => !z,
            0x7 => z,
            0x8 => !v,
            0x9 => v,
            0xa => !n,
            0xb => n,
            0xc => n == v,
            0xd => n != v,
            0xe => !z && (n == v),
            _ => z || (n != v),
        }
    }
}

/// Why an instruction stopped early.
///
/// Every 68000 exception aborts the instruction that raised it, so the
/// interpreter's operand helpers return `Result<_, Trap>` and the instruction
/// body is written with `?`. The alternative — a status flag checked after
/// every access — is how a half-completed instruction gets committed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Trap {
    /// A word or long access to an odd address, or an instruction fetch from
    /// one. Group 0: a fourteen-byte frame on a 68000, format `$8` on a 68010
    /// and format `$A` on a 68020 — which only ever takes one for an
    /// instruction fetch.
    Address {
        /// The address the instruction tried to reach, untruncated.
        addr: u32,
        /// Whether the access was a read.
        read: bool,
        /// The function code that would have been driven.
        fc: u8,
        /// How many bytes the faulted bus cycle carried.
        width: u8,
        /// What a faulted write was writing.
        data: u32,
    },
    /// The address space refused the access. Group 0, same frame shapes, but
    /// format `$B` on a 68020.
    Bus {
        /// The address the instruction tried to reach, untruncated.
        addr: u32,
        /// Whether the access was a read.
        read: bool,
        /// The function code that would have been driven.
        fc: u8,
        /// How many bytes the faulted bus cycle carried.
        width: u8,
        /// What a faulted write was writing.
        data: u32,
    },
    /// An ordinary vectored exception with the six-byte frame.
    Vectored {
        /// The vector number.
        vector: u8,
        /// The program counter to push.
        pc: u32,
        /// Which group it belongs to, which decides its frame on a 68020 and
        /// whether a trace follows it.
        kind: Kind,
    },
}

/// What sort of vectored exception a [`Trap::Vectored`] is (MC68020UM §6.1.11,
/// Table 6-4).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Kind {
    /// Group 3: the instruction was never executed — illegal, line A, line F,
    /// privilege violation. Never traced.
    Rejected,
    /// Group 2 with a four-word frame: `TRAP #n`, a format error.
    Trap,
    /// Group 2 with the 68020's six-word frame, which also carries the
    /// address of the instruction responsible: `CHK`, `CHK2`, `TRAPcc`,
    /// `TRAPV`, a zero divide (MC68020UM Table 6-5). A four-word frame on
    /// earlier parts.
    Six(u32),
    /// The 68040's floating-point **post-instruction** exception: a format
    /// `$3` frame, the same six words as [`Kind::Six`] but with the
    /// *effective address* in the last two and a format code that says so
    /// (M68040UM §8.4.4).
    Post(u32),
}

impl Trap {
    /// An exception through `vector` for an instruction that was never
    /// executed, pushing `pc`.
    const fn at(vector: u8, pc: u32) -> Trap {
        Trap::Vectored {
            vector,
            pc,
            kind: Kind::Rejected,
        }
    }

    /// An exception an instruction raises by executing, pushing `pc`.
    const fn raised(vector: u8, pc: u32) -> Trap {
        Trap::Vectored {
            vector,
            pc,
            kind: Kind::Trap,
        }
    }

    /// An exception an instruction raises by executing, whose 68020 frame
    /// also records the instruction's own address.
    const fn six(vector: u8, pc: u32, insn: u32) -> Trap {
        Trap::Vectored {
            vector,
            pc,
            kind: Kind::Six(insn),
        }
    }

    /// A 68040 floating-point post-instruction exception, whose format `$3`
    /// frame carries the effective address the instruction calculated.
    const fn post_instruction(vector: u8, pc: u32, ea: u32) -> Trap {
        Trap::Vectored {
            vector,
            pc,
            kind: Kind::Post(ea),
        }
    }
}

/// A resolved operand: where a value is, not what it is.
///
/// Resolving separately from reading is what makes a read-modify-write
/// instruction address memory once — `ADDQ #1,(A0)+` increments `A0` once, not
/// twice — and what lets `MOVE` compute its destination address before the
/// write without duplicating the addressing-mode logic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Loc {
    /// Data register `n`.
    D(u8),
    /// Address register `n`.
    A(u8),
    /// A memory location.
    Mem(u32),
    /// A value with no home: an immediate, or a field extracted from the
    /// opcode.
    Value(u32),
    /// A memory location whose current contents have already been read.
    ///
    /// `ADDX`, `SUBX`, `ABCD` and `SBCD` fetch their destination as part of
    /// the predecrement walk that computes its address, so re-reading it would
    /// put a bus cycle on the wire that hardware does not.
    Prefetched(u32, u32),
}

/// The address and stack-pointer registers as an instruction found them, so a
/// fault part way through it can say which ones it has already moved.
#[derive(Debug, Clone, Copy, Default)]
struct Snap {
    a: [u32; 7],
    sp: [u32; 3],
}

/// One instruction's worth of execution, borrowing everything it needs.
pub(super) struct Exec<'a> {
    state: &'a mut State,
    space: &'a AddressSpace,
    cfg: &'a Config,
    lines: &'a Lines,
    /// Which processor, copied out of the configuration because nearly every
    /// path asks.
    model: Model,
    /// The address pins this model has.
    mask: u32,
    /// The opcode word being executed, kept for a group-0 frame's instruction
    /// register field.
    opcode: u16,
    /// The address of that opcode word: where a restarted instruction starts.
    pc0: u32,
    /// The registers an instruction started with, on the models whose fault
    /// frames can be returned from. Unused on a 68000.
    snap: Snap,
    /// Whether the instruction in progress changed the flow of control — a
    /// branch taken, a jump, a return, a write to `SR` — which is what a
    /// 68020 traces when only **T0** is set (MC68020UM §6.1.7).
    flow: bool,
    /// A function code `MOVES` substitutes for the ordinary data one.
    fc_override: Option<u8>,
    /// What the instruction in progress did that a 68020's time depends on.
    facts: timing::Facts,
    /// The 68020's cache-case time for this step, from the tables; zero on
    /// the other models, whose time is counted per access.
    table: u32,
    /// The exception being taken is an interrupt.
    interrupting: bool,
    /// Cycles this step has charged.
    used: u64,
    /// Whether the source operand of the `MOVE` in progress came from memory.
    source_was_memory: bool,
    /// A `MOVE` destination's postincrement, owed once its write lands.
    deferred_postincrement: Option<(u8, u32)>,
    /// Internal cycles the next exception spends before it pushes anything.
    prologue: u32,
    /// Which coprocessor answers the F line, copied out of the
    /// configuration because decode asks on every instruction.
    copro: Copro,
    /// Whether address translation is switched on: a part with the paged
    /// MMU, and `TC`'s **E** bit set. Recomputed whenever a `PMOVE` changes
    /// it; a transparent block needs no check here, because with translation
    /// off every address is already its own.
    mmu_on: bool,
    /// The same for the 68040's unit: a part with it, and `TC`'s **E** bit
    /// set. The two are never both true — a core is one model.
    mmu040_on: bool,
    /// Whether the fault in progress came from the memory management unit
    /// rather than from the address space refusing the cycle — the 68040's
    /// `ATC` bit in the special status word (M68040UM §8.4.6.2).
    atc_fault: bool,
    /// Whether the fault in progress was a `MOVES` through `SFC` or `DFC`,
    /// which the 68040 reports as an *alternate logical* transfer type.
    fault_alternate: bool,
    /// The effective address the floating-point instruction in progress
    /// calculated, for the 68040's format `$2` unimplemented-instruction
    /// frame. Zero when the operand came from a register or the instruction
    /// stream.
    fp_last_address: u32,
    /// Slides an instruction deferred past its operand write.
    ///
    /// `MOVE <ea>,(xxx).L` performs its write *before* the last instruction
    /// fetch, which is visible in the program counter an address-error frame
    /// pushes. Rather than special-case that in three places, the destination
    /// resolver records the debt and [`Exec::settle`] pays it.
    deferred_slides: u32,
    /// The guest-physical [`WRITE_PAGE`] pages this step wrote to, deduplicated
    /// against the ones already here.
    ///
    /// **Not architectural, and not a timing column**: it is the other half of
    /// the translated engine's self-modifying-code answer. A block reports its
    /// own stores through `jit::StoreLog`, which the dispatcher drains against
    /// the block cache; every instruction *outside* the lifted subset runs
    /// here instead, and without this a `MOVEM` that wrote over a cached
    /// translation would leave it cached. `cpu::riscv::exec` keeps the same
    /// log for the same reason, and `engine::drain` is what reads it.
    ///
    /// Four entries because the widest access a step makes is a `MOVEM` of
    /// sixteen long registers — sixty-four contiguous bytes, so two pages —
    /// and an exception frame is narrower still. [`Exec::wrote_all`] is what
    /// happens if that is ever wrong.
    ///
    /// Behind the translated engine's own gate, because a build with no block
    /// cache has nothing to tell and the interpreter's write path is hot
    /// enough that it should not carry three dead stores.
    #[cfg(all(feature = "cpu-m68k-lift", feature = "jit"))]
    wrote: [u32; 4],
    /// How many of [`Exec::wrote`] are live.
    #[cfg(all(feature = "cpu-m68k-lift", feature = "jit"))]
    wrote_n: u8,
    /// This step wrote to more pages than [`Exec::wrote`] holds, so the reader
    /// must assume every translation is stale.
    #[cfg(all(feature = "cpu-m68k-lift", feature = "jit"))]
    wrote_all: bool,
}

/// The granularity [`Exec::wrote`] records a write at.
///
/// `lift::WINDOW` and `jit::PAGE_SIZE` are the same four kilobytes and both
/// live behind features this file does not have, so the number is written here
/// and `engine`'s tests assert the three agree.
#[cfg(all(feature = "cpu-m68k-lift", feature = "jit"))]
pub(super) const WRITE_PAGE: u32 = 4096;

impl<'a> Exec<'a> {
    /// Borrow a core for one step.
    pub(super) fn new(
        state: &'a mut State,
        space: &'a AddressSpace,
        cfg: &'a Config,
        lines: &'a Lines,
    ) -> Exec<'a> {
        let model = state.model;
        let state_enables_mmu = state.mmu.enabled();
        let state_enables_mmu040 = state.mmu040.active();
        Exec {
            state,
            space,
            cfg,
            lines,
            model,
            mask: model.address_mask(),
            opcode: 0,
            pc0: 0,
            snap: Snap::default(),
            flow: false,
            fc_override: None,
            facts: timing::Facts::default(),
            table: 0,
            interrupting: false,
            used: 0,
            prologue: 4,
            source_was_memory: false,
            deferred_postincrement: None,
            copro: if cfg.fpu.present() {
                Copro::FPU
            } else {
                Copro::NONE
            },
            mmu_on: model.has_mmu() && state_enables_mmu,
            mmu040_on: model.has_040() && state_enables_mmu040,
            atc_fault: false,
            fault_alternate: false,
            fp_last_address: 0,
            deferred_slides: 0,
            #[cfg(all(feature = "cpu-m68k-lift", feature = "jit"))]
            wrote: [0; 4],
            #[cfg(all(feature = "cpu-m68k-lift", feature = "jit"))]
            wrote_n: 0,
            #[cfg(all(feature = "cpu-m68k-lift", feature = "jit"))]
            wrote_all: false,
        }
    }

    /// The pages this step wrote, and whether there were more than fit.
    ///
    /// Read by `engine::drain` after an interpreted step, and by nothing else:
    /// it is derived state for the block cache (CLAUDE.md, "Devices").
    #[cfg(all(feature = "cpu-m68k-lift", feature = "jit"))]
    pub(super) fn wrote(&self) -> (&[u32], bool) {
        (&self.wrote[..self.wrote_n as usize], self.wrote_all)
    }

    /// Record a write at bus address `at`, for the block cache.
    ///
    /// On the guest-**physical** address the access actually reached — the one
    /// already masked to the pins this model drives — because that is the
    /// address a translation was lifted from and a store has to be matched
    /// against it there.
    #[inline]
    fn note_write(&mut self, at: u64) {
        // Nothing reads the log in a build with no translated engine, and the
        // interpreter's write path is hot enough that it should not carry a
        // dead store.
        #[cfg(all(feature = "cpu-m68k-lift", feature = "jit"))]
        {
            let page = (at as u32) & !(WRITE_PAGE - 1);
            if self.wrote[..self.wrote_n as usize].contains(&page) {
                return;
            }
            match self.wrote.get_mut(self.wrote_n as usize) {
                Some(slot) => {
                    *slot = page;
                    self.wrote_n += 1;
                }
                None => self.wrote_all = true,
            }
        }
        #[cfg(not(all(feature = "cpu-m68k-lift", feature = "jit")))]
        {
            let _ = at;
        }
    }
    /// Run one reset sequence, exception sequence, or instruction.
    ///
    /// Returns the cycles charged; zero only when the core is halted, which a
    /// scheduler must notice rather than spin on.
    pub(super) fn step(&mut self) -> u64 {
        let before = self.state.cycles;
        let used = self.step_inner();
        if self.table != 0 {
            // A 68020's time is its table entry, not the accesses made on the
            // way — see `timing.rs` for why, and for the column.
            self.used = u64::from(self.table);
            self.state.cycles = before.wrapping_add(self.used);
            return self.used;
        }
        used
    }

    fn step_inner(&mut self) -> u64 {
        self.state.last_vector = None;
        if self.state.reset_pending {
            self.reset_sequence();
            return self.used;
        }
        if self.state.halted {
            return 0;
        }
        if let Some(level) = self.pending_interrupt() {
            self.state.stopped = false;
            self.take_interrupt(level);
            return self.used;
        }
        if self.state.stopped {
            // A stopped 68000 still drives the bus refresh; nothing is
            // fetched, so charge the four cycles a bus cycle would have taken
            // and let the scheduler move time forward.
            self.internal(4);
            return self.used;
        }
        self.instruction();
        self.used
    }

    // ------------------------------------------------------------------
    // The clock
    // ------------------------------------------------------------------

    /// Charge internal (non-bus) cycles.
    fn internal(&mut self, cycles: u32) {
        // Wrapping on both, deliberately: a cycle counter that panics after
        // 2^64 clocks would be a worse failure than one that wraps, and a
        // debug build must not behave differently from a release one.
        self.used = self.used.wrapping_add(u64::from(cycles));
        self.state.cycles = self.state.cycles.wrapping_add(u64::from(cycles));
    }

    /// The attributes every access this core makes carries.
    ///
    /// `MOVES` reaches the address space its function code register names;
    /// the bus here has no function codes, so the part of one it can carry —
    /// whether the space is a supervisor one, bit 2 — becomes the privilege
    /// attribute and the rest is lost (MC68020UM §3.1, *Function Codes*).
    fn attrs(&self) -> MemAttrs {
        let privileged = match self.fc_override {
            Some(fc) => fc & 4 != 0,
            None => self.state.supervisor(),
        };
        MemAttrs::DEFAULT
            .with_requester(self.cfg.requester)
            .with_privileged(privileged)
    }

    /// The function code for a data access in the current privilege state.
    fn data_fc(&self) -> u8 {
        if let Some(fc) = self.fc_override {
            return fc;
        }
        if self.state.supervisor() {
            fc::SUPER_DATA
        } else {
            fc::USER_DATA
        }
    }

    /// The function code for an instruction fetch.
    fn program_fc(&self) -> u8 {
        if self.state.supervisor() {
            fc::SUPER_PROGRAM
        } else {
            fc::USER_PROGRAM
        }
    }

    /// The data a restarted instruction's access gets from a completed fault,
    /// if this is that access. See [`Replay`].
    fn replayed(&mut self, addr: u32, read: bool, width: u8) -> Option<u32> {
        match self.state.replay {
            Some(r) if r.addr == addr && r.read == read && r.width == width => {
                self.state.replay = None;
                Some(r.data)
            }
            _ => None,
        }
    }

    // ------------------------------------------------------------------
    // Address translation
    // ------------------------------------------------------------------

    /// The physical address an access reaches, or `Err` when the memory
    /// management unit refuses it.
    ///
    /// Called from the four leaf accesses — a byte and a word in each
    /// direction — because everything wider is built out of those, and the
    /// 68030 translates each bus cycle rather than each operand. On any other
    /// model, and on a 68030 with `TC`'s **E** bit clear, this is the
    /// identity and costs one predictable branch.
    #[inline]
    fn bus_addr(&mut self, addr: u32, fc: u8, write: bool) -> Result<u64, ()> {
        if self.mmu040_on {
            return self
                .translate_040(addr, fc, write)
                .map(|pa| u64::from(pa & self.mask));
        }
        if !self.mmu_on {
            return Ok(u64::from(addr & self.mask));
        }
        self.translate(addr, fc, write)
            .map(|pa| u64::from(pa & self.mask))
    }

    /// Translate one logical address on a 68040 (M68040UM Figure 3-22).
    ///
    /// The flowchart's four branches, in its order: a transparent
    /// translation register answers, or the cache does, or the cache misses
    /// and a table search fills it, or the entry says the access may not
    /// happen.
    ///
    /// Everything that aborts sets [`Exec::atc_fault`], which is the `ATC`
    /// bit of the format `$7` frame's special status word: "set for an ATC
    /// fault due to a nonresident entry ... or privilege violation (write
    /// protected or supervisor only) ... cleared for a bus-errored
    /// instruction, data, or cache line-push access" (§8.4.6.2). A
    /// transparently translated block's write protection is counted with
    /// them: it is not a bus error, and the manual's flowchart takes the
    /// same "abort cycle, take access error exception" exit for it.
    fn translate_040(&mut self, la: u32, fc: u8, write: bool) -> Result<u32, ()> {
        self.atc_fault = false;
        let supervisor = fc & 4 != 0;
        // "The TTRs operate independently of the E-bit in the TCR and the
        // state of the MDIS signal" (§3.1.3), so this comes first and happens
        // whether or not paged translation is switched on.
        let program = fc & 3 == 2;
        let pair = *self.state.mmu040.ttr_pair(program);
        if let Some(block) = mmu040::transparent(&pair, la, supervisor) {
            if write && block.write_protected {
                self.atc_fault = true;
                return Err(());
            }
            return Ok(la);
        }
        if !self.state.mmu040.enabled() {
            // Translation off: "logical addresses are used as physical
            // addresses" with the default attributes (§3.1.2, **E**).
            return Ok(la);
        }
        let mut entry = match self.state.mmu040.lookup(la, supervisor) {
            Some(entry) => entry,
            None => {
                // "When a table search is required, the processor suspends
                // instruction execution activity and, at the end of a
                // successful table search, stores the address mapping in the
                // appropriate ATC and retries the access" (§3.5).
                let found = self.walk_040(la, supervisor, write);
                self.state.mmu040.install(found.entry);
                found.entry
            }
        };
        if entry.data & mmusr040::R == 0 {
            // "If an access hits in the ATC but an access error or invalid
            // page descriptor was detected during the table search that
            // created the ATC entry, the access is aborted" (§3.5).
            self.atc_fault = true;
            return Err(());
        }
        if entry.data & mmusr040::S != 0 && !supervisor {
            // §3.2.6.2: the entry is created with the S bit set, and "a
            // subsequent retry of the user access results in an access error
            // exception being taken".
            self.atc_fault = true;
            return Err(());
        }
        if write && entry.data & mmusr040::W != 0 {
            self.atc_fault = true;
            return Err(());
        }
        if write && entry.data & mmusr040::M == 0 {
            // "If the M-bit is clear and a write access to this logical
            // address is attempted, the M68040 suspends the access, initiates
            // a table search to set the M-bit in the page descriptor, and
            // writes over the old ATC entry" (§3.3, **M**).
            let found = self.walk_040(la, supervisor, true);
            self.state.mmu040.install(found.entry);
            entry = found.entry;
            if entry.data & (mmusr040::R | mmusr040::W) != mmusr040::R {
                self.atc_fault = true;
                return Err(());
            }
        }
        Ok(self.state.mmu040.physical(entry, la))
    }

    /// Run one 68040 table search, driving the real bus for every descriptor.
    ///
    /// Only the registers are copied out, not the cache: the search cannot
    /// see the entry it is about to create, and the closure below is free to
    /// borrow the whole core.
    fn walk_040(&mut self, la: u32, supervisor: bool, write: bool) -> mmu040::Found040 {
        let registers = mmu040::Regs040 {
            tcr: self.state.mmu040.tcr,
            urp: self.state.mmu040.urp,
            srp: self.state.mmu040.srp,
            ..mmu040::Regs040::RESET
        };
        let mut bus = |at: u32, value: Option<u32>| -> Option<u32> {
            match value {
                None => self.phys_read_long(at),
                Some(word) => self.phys_write_long(at, word).then_some(0),
            }
        };
        mmu040::search(&registers, la, supervisor, write, &mut bus)
    }

    /// Translate one logical address (MC68030UM Figure 9-8).
    fn translate(&mut self, la: u32, fc: u8, write: bool) -> Result<u32, ()> {
        // A transparently translated block is used as a physical address
        // "without modification and without protection checking" (§9.3), and
        // the TTx registers work whatever the E bit says.
        if self.state.mmu.transparent(la, fc, write).is_some() {
            return Ok(la);
        }
        let mut entry = match self.state.mmu.lookup(la, fc) {
            Some(entry) => entry,
            None => {
                // "When a table search is required, the CPU suspends
                // instruction execution activity and, at the end of a
                // successful table search, stores the address mapping in the
                // ATC and retries the access" (§9.5.2).
                let found = self.walk(la, fc, write, 7);
                self.state.mmu.install(found.entry);
                found.entry
            }
        };
        if entry.data & Entry::BERR != 0 {
            return Err(());
        }
        if write && entry.data & Entry::WP != 0 {
            return Err(());
        }
        if write && entry.data & Entry::M == 0 {
            // The first write to a page a read brought in: "the MC68030
            // aborts the access and initiates a table search, setting the M
            // bit in the page descriptor, invalidating the old ATC entry, and
            // creating a new entry with the M bit set" (§9.4, **M**).
            let found = self.walk(la, fc, true, 7);
            self.state.mmu.install(found.entry);
            entry = found.entry;
            if entry.data & (Entry::BERR | Entry::WP) != 0 {
                return Err(());
            }
        }
        Ok(self.state.mmu.physical(entry, la))
    }

    /// Run one table search, driving the real bus for every descriptor.
    ///
    /// The registers are copied out first so the search cannot see the entry
    /// it is about to create, and so the closure below is free to borrow the
    /// whole core.
    fn walk(&mut self, la: u32, fc: u8, write: bool, levels: u8) -> mmu::Found {
        let registers = self.state.mmu;
        let mut bus = |at: u32, value: Option<u32>| -> Option<u32> {
            match value {
                None => self.phys_read_long(at),
                Some(word) => self.phys_write_long(at, word).then_some(0),
            }
        };
        mmu::search(&registers, la, fc, write, levels, &mut bus)
    }

    /// Charge a table search's bus cycles.
    ///
    /// They are real cycles on a real bus, so they are charged like any
    /// other — and on a model whose time comes from `timing.rs` they are
    /// *added* to the table entry rather than replaced by it, because the
    /// 68020 tables this core borrows have no table search in them.
    fn search_cycles(&mut self, cycles: u32) {
        self.internal(cycles);
        if self.model.has_020() {
            self.table = self.table.saturating_add(cycles);
        }
    }

    /// One word read at a physical address, for a descriptor fetch.
    ///
    /// Straight to the space: a table search is already physical, and
    /// translating it would be a loop. `MemAttrs` carries no function code,
    /// so the only thing it can say about a search is that the MMU made it,
    /// which is a supervisor access.
    fn phys_read_word(&mut self, at: u32) -> Option<u16> {
        self.search_cycles(4);
        let attrs = MemAttrs::DEFAULT
            .with_requester(self.cfg.requester)
            .with_privileged(true);
        match self
            .space
            .read(u64::from(at & self.mask), Width::U16, attrs)
        {
            Ok(value) => Some(value as u16),
            Err(_) => {
                self.state.faults = self.state.faults.wrapping_add(1);
                self.state.last_fault = at;
                None
            }
        }
    }

    /// One word write at a physical address, for a history-bit update.
    fn phys_write_word(&mut self, at: u32, value: u16) -> bool {
        self.search_cycles(4);
        let attrs = MemAttrs::DEFAULT
            .with_requester(self.cfg.requester)
            .with_privileged(true);
        match self.space.write(
            u64::from(at & self.mask),
            Width::U16,
            u64::from(value),
            attrs,
        ) {
            Ok(()) => {
                self.note_write(u64::from(at & self.mask));
                true
            }
            Err(_) => {
                self.state.faults = self.state.faults.wrapping_add(1);
                self.state.last_fault = at;
                false
            }
        }
    }

    /// One long read at a physical address: two word cycles, high word
    /// first, as every other long access on this bus is.
    fn phys_read_long(&mut self, at: u32) -> Option<u32> {
        let hi = self.phys_read_word(at)?;
        let lo = self.phys_read_word(at.wrapping_add(2))?;
        Some((u32::from(hi) << 16) | u32::from(lo))
    }

    /// One long write at a physical address.
    fn phys_write_long(&mut self, at: u32, value: u32) -> bool {
        self.phys_write_word(at, (value >> 16) as u16)
            && self.phys_write_word(at.wrapping_add(2), value as u16)
    }

    // ------------------------------------------------------------------
    // Memory accesses
    // ------------------------------------------------------------------

    /// One byte read. Byte accesses have no alignment rule.
    fn read_byte(&mut self, addr: u32) -> Result<u8, Trap> {
        if self.state.replay.is_some()
            && let Some(data) = self.replayed(addr, true, 1)
        {
            return Ok(data as u8);
        }
        let fc = self.data_fc();
        let Ok(at) = self.bus_addr(addr, fc, false) else {
            return Err(self.bus_fault(addr, true, fc, 1, 0));
        };
        self.internal(4);
        match self.space.read(at, Width::U8, self.attrs()) {
            Ok(v) => Ok(v as u8),
            Err(_) => Err(self.bus_fault(addr, true, fc, 1, 0)),
        }
    }

    /// One word read, faulting on an odd address.
    fn read_word(&mut self, addr: u32) -> Result<u16, Trap> {
        let fc = self.data_fc();
        self.read_word_fc(addr, fc)
    }

    /// One word read with an explicit function code, so an instruction fetch
    /// can report itself as program space in a group-0 frame.
    ///
    /// An odd address is an address error on a 68000 and a 68010. A 68020
    /// takes one only for an instruction fetch; an operand may sit anywhere,
    /// and the processor splits it into the bus cycles it needs (MC68020UM
    /// §6.1.3, and §5.2.2 on misaligned operands). With a 16-bit port — which
    /// is what this bus model is — a word at an odd address is two byte
    /// cycles.
    fn read_word_fc(&mut self, addr: u32, fc: u8) -> Result<u16, Trap> {
        let program = fc & 3 == 2;
        // Before the alignment check: an address error a handler completed
        // in software is exactly a misaligned access that must not fault
        // again.
        if self.state.replay.is_some()
            && !program
            && let Some(data) = self.replayed(addr, true, 2)
        {
            return Ok(data as u16);
        }
        if addr & 1 != 0 {
            if !self.model.has_020() || program {
                return Err(Trap::Address {
                    addr,
                    read: true,
                    fc,
                    width: 2,
                    data: 0,
                });
            }
            let hi = self.read_byte(addr)?;
            let lo = self.read_byte(addr.wrapping_add(1))?;
            return Ok((u16::from(hi) << 8) | u16::from(lo));
        }
        let Ok(at) = self.bus_addr(addr, fc, false) else {
            return Err(self.bus_fault(addr, true, fc, 2, 0));
        };
        self.internal(4);
        match self.space.read(at, Width::U16, self.attrs()) {
            Ok(v) => Ok(v as u16),
            Err(_) => Err(self.bus_fault(addr, true, fc, 2, 0)),
        }
    }

    /// One long read: two word accesses, high word first.
    ///
    /// The 68000 has a 16-bit data bus, so there is no such thing as a 32-bit
    /// bus cycle; a device watching the bus sees two. A 68020 on a 16-bit port
    /// does the same, and a long at an odd address is a byte, a word and a
    /// byte (MC68020UM §5.2.2).
    fn read_long(&mut self, addr: u32) -> Result<u32, Trap> {
        if addr & 1 != 0 && self.model.has_020() {
            let b0 = self.read_byte(addr)?;
            let mid = self.read_word(addr.wrapping_add(1))?;
            let b3 = self.read_byte(addr.wrapping_add(3))?;
            return Ok((u32::from(b0) << 24) | (u32::from(mid) << 8) | u32::from(b3));
        }
        let hi = self.read_word(addr)?;
        let lo = self.read_word(addr.wrapping_add(2))?;
        Ok((u32::from(hi) << 16) | u32::from(lo))
    }

    /// One byte write.
    fn write_byte(&mut self, addr: u32, value: u8) -> Result<(), Trap> {
        if self.state.replay.is_some() && self.replayed(addr, false, 1).is_some() {
            return Ok(());
        }
        let fc = self.data_fc();
        let Ok(at) = self.bus_addr(addr, fc, true) else {
            return Err(self.bus_fault(addr, false, fc, 1, u32::from(value)));
        };
        self.internal(4);
        match self
            .space
            .write(at, Width::U8, u64::from(value), self.attrs())
        {
            Ok(()) => {
                self.note_write(at);
                Ok(())
            }
            Err(_) => Err(self.bus_fault(addr, false, fc, 1, u32::from(value))),
        }
    }

    /// One word write, faulting on an odd address — except on a 68020, which
    /// splits it (see [`Exec::read_word_fc`]).
    fn write_word(&mut self, addr: u32, value: u16) -> Result<(), Trap> {
        let fc = self.data_fc();
        if self.state.replay.is_some() && self.replayed(addr, false, 2).is_some() {
            return Ok(());
        }
        if addr & 1 != 0 {
            if !self.model.has_020() {
                return Err(Trap::Address {
                    addr,
                    read: false,
                    fc,
                    width: 2,
                    data: u32::from(value),
                });
            }
            self.write_byte(addr, (value >> 8) as u8)?;
            return self.write_byte(addr.wrapping_add(1), value as u8);
        }
        let Ok(at) = self.bus_addr(addr, fc, true) else {
            return Err(self.bus_fault(addr, false, fc, 2, u32::from(value)));
        };
        self.internal(4);
        match self
            .space
            .write(at, Width::U16, u64::from(value), self.attrs())
        {
            Ok(()) => {
                self.note_write(at);
                Ok(())
            }
            Err(_) => Err(self.bus_fault(addr, false, fc, 2, u32::from(value))),
        }
    }

    /// One long write: two word accesses, high word first.
    fn write_long(&mut self, addr: u32, value: u32) -> Result<(), Trap> {
        if addr & 1 != 0 && self.model.has_020() {
            self.write_byte(addr, (value >> 24) as u8)?;
            self.write_word(addr.wrapping_add(1), (value >> 8) as u16)?;
            return self.write_byte(addr.wrapping_add(3), value as u8);
        }
        self.write_word(addr, (value >> 16) as u16)?;
        self.write_word(addr.wrapping_add(2), value as u16)
    }

    /// A long write that puts the **low** word out first.
    ///
    /// Two cases need it, for the same reason: the word the datapath is
    /// already holding goes out first. `MOVE.L <ea>,-(An)` has decremented the
    /// register by four and writes the low half before fetching the other one,
    /// and every read-modify-write — `ADD.L D0,(A0)`, `CLR.L`, `NOT.L`, a
    /// memory shift — finished its read with the low word and writes that
    /// back first. It is visible on the bus, so a device with side effects can
    /// tell, and it is not an implementation detail we get to choose.
    fn write_long_low_first(&mut self, addr: u32, value: u32) -> Result<(), Trap> {
        if addr & 1 != 0 && self.model.has_020() {
            return self.write_long(addr, value);
        }
        self.write_word(addr.wrapping_add(2), value as u16)?;
        self.write_word(addr, (value >> 16) as u16)
    }

    /// Record a refused access and turn it into a bus error.
    ///
    /// **Known hazard**, and the same one `ROADMAP.md` §4.1 records against
    /// the 6502 core: an access that meets a retopology in flight comes back
    /// as [`BusError::Retry`](crate::core::error::BusError::Retry), meaning
    /// "nothing happened, reissue" rather than "the hardware refused". This
    /// core cannot reissue — it holds `BUS` across the access and the
    /// retopology takes `BUS` underneath `TOPOLOGY`, so spinning here closes a
    /// deadlock cycle — so a retry becomes a bus-error exception, which is
    /// guest-visible and depends on host timing. The safe-point protocol
    /// (§4.7) is what makes it unreachable, and it does not exist yet. Until
    /// then a machine that remaps a space under a running CPU can see a
    /// spurious vector-2 exception, and `bus_faults` is where it shows up.
    fn bus_fault(&mut self, addr: u32, read: bool, fc: u8, width: u8, data: u32) -> Trap {
        self.state.faults = self.state.faults.wrapping_add(1);
        self.state.last_fault = addr;
        // Captured here rather than read back when the frame is built: a
        // `MOVES` clears its function-code override before the trap has
        // propagated out of it, and the 68040's special status word needs to
        // know which kind of access this was (M68040UM Table 3-2).
        self.fault_alternate = self.fc_override.is_some();
        Trap::Bus {
            addr,
            read,
            fc,
            width,
            data,
        }
    }
    // ------------------------------------------------------------------
    // The prefetch queue
    // ------------------------------------------------------------------

    /// Fetch one instruction word for the queue.
    ///
    /// On a 68000 or 68010 a refused fetch is a bus error on the spot. A 68020
    /// fetches ahead of itself and "may delay taking the exception until it
    /// attempts to use the prefetched information" (MC68020UM §6.1.2) — which
    /// matters, because a routine ending in `RTS` at the last word of a mapped
    /// region prefetches past it and must not fault. So there a refused fetch
    /// *poisons* the word instead: `Ok(None)`, and the fault is raised only if
    /// something consumes it (see [`Exec::queued`] and
    /// [`Exec::instruction`]). An odd address is still an address error at
    /// once, since no bus cycle is attempted (MC68020UM §6.1.3).
    fn fetch(&mut self, addr: u32) -> Result<Option<u16>, Trap> {
        let fc = self.program_fc();
        match self.read_word_fc(addr, fc) {
            Ok(word) => Ok(Some(word)),
            Err(Trap::Bus { .. }) if self.model.has_020() => Ok(None),
            Err(trap) => Err(trap),
        }
    }

    /// Slide the queue one word: shift, refill from `pc + 4`, advance `pc`.
    ///
    /// This is the only place `pc` moves during an instruction, which is what
    /// makes the value an address-error frame pushes correct.
    fn slide(&mut self) -> Result<(), Trap> {
        let fetch = self.state.pc.wrapping_add(4);
        let word = self.fetch(fetch)?;
        self.state.prefetch[0] = self.state.prefetch[1];
        self.state.prefetch[1] = word.unwrap_or(0);
        self.state.poison = [
            self.state.poison[1],
            if word.is_none() { Some(fetch) } else { None },
        ];
        self.state.pc = self.state.pc.wrapping_add(2);
        Ok(())
    }

    /// The word in `prefetch[1]`, or the bus error a 68020 deferred when it
    /// failed to fetch it.
    ///
    /// Everything that reads an extension word out of the queue goes through
    /// here, so a poisoned word is never mistaken for a displacement.
    fn queued(&mut self) -> Result<u16, Trap> {
        if let Some(addr) = self.state.poison[1] {
            return Err(self.fetch_fault(addr));
        }
        Ok(self.state.prefetch[1])
    }

    /// The bus error for a poisoned instruction word, now that it is needed.
    ///
    /// Not [`Exec::bus_fault`]: the refused access was counted when it was
    /// made, and this is the same access being owned up to.
    fn fetch_fault(&self, addr: u32) -> Trap {
        Trap::Bus {
            addr,
            read: true,
            fc: self.program_fc(),
            width: 2,
            data: 0,
        }
    }

    /// Take the next extension word, charging `delay` internal cycles between
    /// reading it and the refill it causes.
    ///
    /// The delay is where an indexed mode's two internal cycles go: the
    /// manual's timing puts them before the refill, and a bus trace shows them
    /// there (MC68000UM Table 8-1).
    fn ext(&mut self, delay: u32) -> Result<u16, Trap> {
        let word = self.queued()?;
        if delay != 0 {
            self.internal(delay);
        }
        self.slide()?;
        Ok(word)
    }

    /// Take the next extension word but leave its refill for [`Exec::settle`].
    fn ext_deferred(&mut self) -> Result<u16, Trap> {
        let word = self.queued()?;
        self.deferred_slides += 1;
        Ok(word)
    }

    /// Pay off any deferred refill, then perform the instruction's final
    /// slide, which is what leaves the next opcode in `prefetch[0]`.
    fn settle(&mut self) -> Result<(), Trap> {
        while self.deferred_slides > 0 {
            self.deferred_slides -= 1;
            self.slide()?;
        }
        self.slide()
    }

    /// Reload both prefetch words from `target` — a branch, a jump, a return,
    /// or the last step of exception processing.
    ///
    /// `gap` is internal time spent between the two fetches. A branch has
    /// none; exception processing has two cycles there, and they are in every
    /// published exception time.
    ///
    /// The program counter moves to `target - 4` *before* the first fetch and
    /// steps as each word lands, because the queue invariant is that `pc`
    /// addresses `prefetch[0]` and neither word has arrived yet. That is not
    /// bookkeeping for its own sake: a branch to an odd address takes an
    /// address error here, and the frame it pushes carries exactly this value.
    fn refill(&mut self, target: u32, gap: u32) -> Result<(), Trap> {
        self.deferred_slides = 0;
        self.flow = true;
        self.state.pc = target.wrapping_sub(4);
        let first = self.fetch(target)?;
        self.state.pc = target.wrapping_sub(2);
        if gap != 0 {
            self.internal(gap);
        }
        let second = self.fetch(target.wrapping_add(2))?;
        self.state.pc = target;
        self.state.prefetch = [first.unwrap_or(0), second.unwrap_or(0)];
        self.state.poison = [
            if first.is_none() { Some(target) } else { None },
            if second.is_none() {
                Some(target.wrapping_add(2))
            } else {
                None
            },
        ];
        Ok(())
    }

    /// [`Exec::refill`] for exception processing and reset, where a fetch
    /// that fails is not deferred: the first prefetches are part of the
    /// exception sequence, and a bus error in them is a double bus fault
    /// (MC68020UM §6.1.1, Figure 6-1).
    fn refill_strict(&mut self, target: u32, gap: u32) -> Result<(), Trap> {
        self.refill(target, gap)?;
        match self.state.poison {
            [Some(addr), _] | [None, Some(addr)] => Err(self.fetch_fault(addr)),
            [None, None] => Ok(()),
        }
    }
    // ------------------------------------------------------------------
    // Reset, interrupts and exceptions
    // ------------------------------------------------------------------

    /// The reset sequence: supervisor state, interrupts masked, `SSP` and `PC`
    /// from vectors 0 and 1 (MC68000UM §6.2.6).
    fn reset_sequence(&mut self) {
        self.state.reset_pending = false;
        self.state.halted = false;
        self.state.stopped = false;
        // Through set_sr, not by assignment: a reset taken in user state has
        // to swap the stack-pointer banks before vector 0 is loaded, or the
        // supervisor stack pointer lands in the user bank and the user one is
        // silently lost.
        self.state.set_sr(flags::S | flags::IPL);
        // The 68010 and 68020 also zero the vector base, and the 68020 clears
        // its cache's enable and freeze bits (MC68000UM §6.2.1; MC68020UM
        // §6.1.1). A fault a handler was completing is abandoned with it.
        self.state.vbr = 0;
        self.state.cacr = 0;
        self.state.replay = None;
        // "The assertion of RESET disables translations by clearing the E
        // bits of the TC and TTx registers, but it does not flush the ATC"
        // (MC68030UM §9.2.2) — which is why an operating system has to flush
        // it itself before turning translation back on.
        self.state.mmu.reset_pin();
        self.mmu_on = false;
        // The 68040's unit says the same in its own words, and adds that
        // `TC`'s **P** bit — the page size — is *not* affected and "must be
        // initialized after a reset" (M68040UM §3.1.2, §3.6.1).
        self.state.mmu040.reset_pin();
        self.mmu040_on = false;
        // "A reset function ... sets FP0-FP7 to positive non-signaling
        // not-a-numbers" and clears FPCR, FPSR and FPIAR (M68881UM §2.1,
        // §2.2, §2.4).
        self.state.fpu = Fpu::RESET;
        self.internal(4);
        let outcome = (|| -> Result<(), Trap> {
            let ssp = self.read_long(0)?;
            let pc = self.read_long(4)?;
            self.state.a[7] = ssp;
            self.refill_strict(pc, 0)
        })();
        if outcome.is_err() {
            // Nothing can be done about a reset vector that cannot be read.
            self.state.halted = true;
        }
    }

    /// The interrupt level that should be taken now, if any.
    ///
    /// Levels one to six are level-sensitive and are taken while they exceed
    /// the mask in `SR`. Level seven is the non-maskable one, and it is
    /// **edge-triggered**: the 68000 recognises a *transition* to seven, not
    /// the level itself, so a source that holds the pins at seven interrupts
    /// once rather than forever (MC68000UM §6.3.2). Treating it as
    /// level-sensitive is the classic way to make a machine with a wired
    /// non-maskable button lock up the moment it is pressed.
    fn pending_interrupt(&self) -> Option<u8> {
        // The latch is consumed whatever the pins say *now*. A step can cover
        // many clocks, so a source that pulses level seven and lets go is the
        // normal case rather than a race; checking the current level first
        // would drop the edge and then deliver it at some unrelated later
        // moment when the pins happened to read seven again.
        if self.lines.take_level_seven() {
            return Some(7);
        }
        let level = self.lines.ipl();
        if level != 0 && level > self.state.ipl_mask() {
            Some(level)
        } else {
            None
        }
    }

    /// Acknowledge and vector an interrupt.
    ///
    /// # The acknowledge cycle is timed but not driven
    ///
    /// Hardware runs an interrupt-acknowledge bus cycle here — function code
    /// 7, CPU space, the level on A3–A1 — and either a device answers with a
    /// vector number or `VPA` asks for the autovector. This core charges that
    /// cycle's four clocks (the published time is 44 cycles, five reads and
    /// three writes: the fifth read is the acknowledge) but does not put it on
    /// the bus, because CPU space is a function code and `MemAttrs` does not
    /// carry one. A controller therefore answers through `core::wire`'s
    /// [`IntAck`](crate::core::wire::IntAck) if one is attached to an `IPL`
    /// net, or through
    /// [`M68k::set_interrupt_vector`](super::M68k::set_interrupt_vector) if a
    /// caller arms one by hand. Either way the vector is *consumed*: the next
    /// acknowledge autovectors again unless something answers again, which is
    /// what asserting `VPA` means and what most 68000 boards do.
    ///
    /// What A3-A1 would carry does reach the controllers, though, as
    /// [`IntAckCycle::at_level`](crate::core::wire::IntAckCycle::at_level): the
    /// level is the whole reason several controllers can share one processor,
    /// since each one compares it with its own and declines the rest.
    fn take_interrupt(&mut self, level: u8) {
        let vector = self
            .lines
            .acknowledge(level)
            .unwrap_or(vector::AUTOVECTOR_BASE.wrapping_add(level));
        let pc = self.state.pc;
        let sr = self.state.sr;
        // The mask rises to the level being serviced, so the handler is not
        // immediately re-entered by its own source.
        let raised = (sr & !flags::IPL) | (u16::from(level) << 8);
        // Ten cycles more prologue than any other exception: four of them are
        // the acknowledge cycle above, and the rest is the microcode deciding
        // what to do with the answer (MC68000UM Table 8-14, 44(5/3)). The
        // 68010 spends two fewer: its interrupt is 46(5/4) with one more
        // write for the format word (MC68000UM Table 9-19).
        self.prologue = if self.model == Model::M68010 { 12 } else { 14 };
        self.interrupting = true;
        let frame = match self.model {
            Model::M68000 => Frame::Classic(None),
            // An interrupt taken on the master stack leaves a throwaway frame
            // on the interrupt stack too, and the handler runs there
            // (MC68020UM §6.1.9).
            _ if self.model.has_020() && sr & flags::M != 0 => Frame::format(0).throwaway(),
            _ => Frame::format(0),
        };
        self.enter_exception(vector, pc, raised, frame);
    }

    /// Perform exception processing.
    ///
    /// `new_sr` is the status register the handler starts with, before **S**
    /// is forced and **T** cleared; passing it in is how an interrupt raises
    /// the mask and everything else does not.
    fn enter_exception(&mut self, vector: u8, pc: u32, new_sr: u16, frame: Frame) {
        let saved_sr = self.state.sr;
        // Supervisor state, tracing off — both trace bits on a 68020. The
        // status register pushed is the one from *before* this (MC68000UM
        // §6.2). **M** is left as it was: a 68020 stacks on whichever
        // supervisor stack it selects (MC68020UM §6.1).
        self.state
            .set_sr((new_sr | flags::S) & !(flags::T | flags::T0));
        // Any exception resumes a stopped processor, including the trace
        // exception a `STOP` executed with T set leaves behind.
        self.state.stopped = false;
        self.state.last_vector = Some(vector);
        if self.model.has_020()
            && let Frame::Format(image) = &frame
        {
            self.table += timing::exception(image.format, self.interrupting, image.throwaway);
        }
        // Four cycles of deciding what to do, for most exceptions. `TRAPV`
        // spends none — it already knew — `CHK` spends two more unless the
        // bound test is what failed, and an interrupt spends ten more because
        // it has an acknowledge cycle to run first.
        self.internal(self.prologue);
        let outcome = (|| -> Result<(), Trap> {
            match frame {
                Frame::Classic(group0) => {
                    let mut sp = self.state.a[7];
                    // The 68000 writes the frame in this order, which is
                    // neither ascending nor descending; it is visible on the
                    // bus.
                    sp = sp.wrapping_sub(2);
                    self.write_word(sp, pc as u16)?;
                    sp = sp.wrapping_sub(4);
                    self.write_word(sp, saved_sr)?;
                    self.write_word(sp.wrapping_add(2), (pc >> 16) as u16)?;
                    if let Some(g0) = group0 {
                        sp = sp.wrapping_sub(2);
                        self.write_word(sp, g0.ir)?;
                        sp = sp.wrapping_sub(2);
                        self.write_word(sp, g0.addr as u16)?;
                        sp = sp.wrapping_sub(4);
                        self.write_word(sp, g0.ssw)?;
                        self.write_word(sp.wrapping_add(2), (g0.addr >> 16) as u16)?;
                    }
                    self.state.a[7] = sp;
                }
                Frame::Format(image) => {
                    self.push_frame(&image, vector, saved_sr, pc)?;
                    if image.throwaway {
                        // The frame just written is on the master stack. Clear
                        // **M**, which moves to the interrupt stack, and leave a
                        // format $1 copy there with **S** set in its status
                        // word — so an `RTE` from the handler lands back on the
                        // master stack to find the real one (MC68020UM §6.1.9,
                        // §6.1.12).
                        let sr = self.state.sr & !flags::M;
                        self.state.set_sr(sr);
                        self.push_frame(&FrameImage::new(1), vector, saved_sr | flags::S, pc)?;
                    }
                }
            }
            let base = self
                .state
                .vbr
                .wrapping_add(u32::from(vector).wrapping_mul(4));
            let target = self.read_long(base)?;
            self.refill_strict(target, 2)
        })();
        if outcome.is_err() {
            // A fault while taking an exception is the double bus fault: the
            // 68000 asserts HALT and stops until reset (MC68000UM §6.2.5).
            self.state.halted = true;
        }
    }

    /// Write a format-word frame (68010 and 68020) onto the active stack.
    ///
    /// The four words every format shares go at the bottom: `SR`, the
    /// program counter, and the format and vector offset (MC68000UM Figure
    /// 6-6; MC68020UM Table 6-5). The rest of the image goes above them,
    /// written from the top down; the 68020 manual guarantees only the layout
    /// and not the order of the bus cycles (MC68020UM §6.1: "all individual bus
    /// cycles ... are not guaranteed to occur in the order in which they are
    /// described"), and the 68010's is not published, so the base words go
    /// out in the 68000's own order after the format word.
    fn push_frame(&mut self, image: &FrameImage, vector: u8, sr: u16, pc: u32) -> Result<(), Trap> {
        let total = 4 + u32::from(image.len);
        let sp = self.state.a[7].wrapping_sub(total * 2);
        for i in (0..image.len as usize).rev() {
            if image.skip & (1u64 << i) != 0 {
                continue;
            }
            let at = sp.wrapping_add(8 + 2 * i as u32);
            self.write_word(at, image.words[i])?;
        }
        let format = (u16::from(image.format) << 12) | ((u16::from(vector) * 4) & 0x0fff);
        self.write_word(sp.wrapping_add(6), format)?;
        self.write_word(sp.wrapping_add(4), pc as u16)?;
        self.write_word(sp, sr)?;
        self.write_word(sp.wrapping_add(2), (pc >> 16) as u16)?;
        self.state.a[7] = sp;
        Ok(())
    }

    /// The special status word a group-0 frame carries.
    ///
    /// Bits 2–0 are the function code the failed access drove, bit 3 is the
    /// instruction/not bit and bit 4 is read/write. The remaining eleven bits
    /// are documented as undefined; the hardware leaves the instruction
    /// register's bits there, and the corpus this core is measured against
    /// expects them, so they are reproduced rather than zeroed
    /// (MC68000UM §6.3.9).
    ///
    /// Bit 3 is set for a *program-space* access and clear for a data one,
    /// which is worth spelling out because the manual's name for it invites
    /// the opposite reading. "I/N" is instruction / **not** — meaning "was the
    /// processor in the middle of executing an instruction (0) or between
    /// instructions (1)" — and an instruction fetch is by definition the
    /// latter, while the data access an instruction makes is the former. The
    /// polarity here is the one the conformance corpus measures on every
    /// address error it contains.
    fn special_status(&self, read: bool, fc: u8) -> u16 {
        let not_instruction = fc == self.program_fc();
        (self.opcode & !0x001f)
            | (u16::from(read) << 4)
            | (u16::from(not_instruction) << 3)
            | u16::from(fc & 7)
    }

    /// Turn a [`Trap`] into exception processing.
    fn service(&mut self, trap: Trap) {
        match trap {
            Trap::Address { addr, read, fc, .. } | Trap::Bus { addr, read, fc, .. }
                if self.model == Model::M68000 =>
            {
                let vector = if matches!(trap, Trap::Address { .. }) {
                    vector::ADDRESS_ERROR
                } else {
                    vector::BUS_ERROR
                };
                let ssw = self.special_status(read, fc);
                let g0 = Group0 {
                    ssw,
                    addr,
                    ir: self.opcode,
                };
                let pc = self.state.pc;
                let sr = self.state.sr;
                self.enter_exception(vector, pc, sr, Frame::Classic(Some(g0)));
            }
            Trap::Address { .. } | Trap::Bus { .. } => self.fault(trap),
            Trap::Vectored { vector, pc, kind } => {
                let sr = self.state.sr;
                let frame = match (self.model, kind) {
                    (Model::M68000, _) => Frame::Classic(None),
                    (model, Kind::Six(insn)) if model.has_020() => {
                        let mut image = FrameImage::new(2);
                        image.push((insn >> 16) as u16);
                        image.push(insn as u16);
                        Frame::Format(image)
                    }
                    (model, Kind::Post(ea)) if model.has_040() => {
                        let mut image = FrameImage::new(3);
                        image.push((ea >> 16) as u16);
                        image.push(ea as u16);
                        Frame::Format(image)
                    }
                    _ => Frame::format(0),
                };
                self.enter_exception(vector, pc, sr, frame);
            }
        }
    }

    /// A bus or address error on a 68010 or 68020: the long frame that lets
    /// an `RTE` finish the job.
    ///
    /// # Restart, not continuation
    ///
    /// Both processors save enough internal state to *continue* the faulted
    /// instruction from the bus cycle that failed (MC68000UM §6.3.9.2;
    /// MC68020UM §6.2). That state is microcode state and this interpreter
    /// does not have any; what it has is the address of the instruction and
    /// the registers it started with. So the frame's internal words — which
    /// the manuals leave to the processor — carry the instruction's address
    /// (on the 68010; the 68020's frame has it in its PC field already) and
    /// the old values of every address register and stack pointer the
    /// instruction had moved before the fault, and `RTE` puts those back and
    /// **restarts the instruction from its first word**. The handler sees the
    /// registers as the partly executed instruction left them, exactly as it
    /// would on hardware; what differs is only how `RTE` finishes, and the
    /// result is the same one.
    ///
    /// A bus cycle software completed itself — the 68010's rerun flag set, the
    /// 68020's data-fault flag cleared — is honoured through [`Replay`].
    fn fault(&mut self, trap: Trap) {
        let (vector, addr, read, fc, width, data) = match trap {
            Trap::Address {
                addr,
                read,
                fc,
                width,
                data,
            } => (vector::ADDRESS_ERROR, addr, read, fc, width, data),
            Trap::Bus {
                addr,
                read,
                fc,
                width,
                data,
            } => (vector::BUS_ERROR, addr, read, fc, width, data),
            Trap::Vectored { .. } => return,
        };
        let program = fc & 3 == 2;
        let sr = self.state.sr;
        let undo = self.undo_list();
        if self.model.has_040() {
            self.fault_040(vector, addr, read, fc, width, sr, &undo);
            return;
        }
        if self.model == Model::M68010 {
            // Format $8, twenty-nine words, twenty-six of them written: the
            // three marked "unused, reserved" are skipped, and the note under
            // MC68000UM Figure 6-8 says as much.
            let rmw = matches!(decode_for(self.model, self.opcode).op, Op::Tas);
            let ssw = (u16::from(program) << 13)
                | (u16::from(!program && read) << 12)
                | (u16::from(rmw) << 11)
                | (u16::from(width == 1) << 9)
                | (u16::from(read) << 8)
                | u16::from(fc & 7);
            let mut image = FrameImage::new(8);
            image.push(ssw); // +$08
            image.push((addr >> 16) as u16); // +$0A
            image.push(addr as u16); // +$0C
            image.skip_one(); // +$0E unused, reserved
            image.push(data as u16); // +$10 data output buffer
            image.skip_one(); // +$12 unused, reserved
            image.push(0); // +$14 data input buffer: nothing arrived
            image.skip_one(); // +$16 unused, reserved
            image.push(self.opcode); // +$18 instruction input buffer
            image.push(VERSION_68010 << 10); // +$1A version number
            image.push((self.pc0 >> 16) as u16); // +$1C restart address
            image.push(self.pc0 as u16);
            image.push_undo(&undo, 4); // +$20..+$37
            image.push(0); // +$38, the last internal word
            // The stacked program counter is where the prefetch had got to,
            // which the manual allows to be "advanced by as many as five
            // words" past the instruction; it is the 68000's value.
            let pc = self.state.pc;
            self.enter_exception(vector, pc, sr, Frame::Format(image));
            return;
        }
        if matches!(trap, Trap::Address { .. }) {
            // A 68020 takes an address error only fetching an instruction
            // from an odd address, before any bus cycle, and the frame is the
            // short one: the "next instruction" is the one at that address,
            // and both pipe stages want rerunning (MC68020UM §6.1.3, §6.2.1).
            let mut image = FrameImage::new(0xa);
            image.push(0); // +$08 internal register
            image.push(SSW_RC | SSW_RB); // +$0A special status word
            image.push(0); // +$0C stage C
            image.push(0); // +$0E stage B
            image.push(0); // +$10 data cycle fault address: not a data cycle
            image.push(0);
            image.push(0); // +$14, +$16 internal
            image.push(0);
            image.push(0); // +$18 data output buffer
            image.push(0);
            image.push(0); // +$1C, +$1E internal
            image.push(0);
            self.enter_exception(vector, addr, sr, Frame::Format(image));
            return;
        }
        // Format $B, the long bus fault frame: the fault happened inside an
        // instruction, and the stacked PC is that instruction's address
        // (MC68020UM Table 6-5).
        let size = match width {
            1 => 0b01,
            2 => 0b10,
            _ => 0b00,
        };
        let rmw = matches!(
            decode_for(self.model, self.opcode).op,
            Op::Tas | Op::Cas | Op::Cas2
        );
        let ssw = if program {
            SSW_FB | SSW_RB
        } else {
            SSW_DF
                | (u16::from(rmw) << 7)
                | (u16::from(read) << 6)
                | (size << 4)
                | u16::from(fc & 7)
        };
        let mut image = FrameImage::new(0xb);
        // +$08 is an internal register; this core keeps "the fault was a data
        // cycle" in its low bit, which the handler may clear DF over.
        image.push(u16::from(!program));
        image.push(ssw); // +$0A
        image.push(0); // +$0C stage C
        image.push(0); // +$0E stage B
        image.push((addr >> 16) as u16); // +$10 data cycle fault address
        image.push(addr as u16);
        image.push(0); // +$14 internal
        image.push(0); // +$16 internal
        image.push((data >> 16) as u16); // +$18 data output buffer
        image.push(data as u16);
        for _ in 0..4 {
            image.push(0); // +$1C..+$23 internal
        }
        let stage_b = self.pc0.wrapping_add(4);
        image.push((stage_b >> 16) as u16); // +$24 stage B address
        image.push(stage_b as u16);
        image.push(0); // +$28, +$2A internal
        image.push(0);
        image.push(0); // +$2C data input buffer: nothing arrived
        image.push(0);
        for _ in 0..3 {
            image.push(0); // +$30..+$35 internal
        }
        image.push(VERSION_68020 << 12); // +$36 version number
        image.push_undo(&undo, 6); // +$38..+$5B
        let pc0 = self.pc0;
        self.enter_exception(vector, pc0, sr, Frame::Format(image));
    }

    /// A bus or address error on a 68040 (M68040UM §8.2.1, §8.2.2, §8.4.6).
    ///
    /// # Two frames, not one
    ///
    /// The 68040 splits what the 68020 put in formats `$A` and `$B`:
    ///
    /// - An **address error** — "the processor attempts to prefetch an
    ///   instruction from an odd address" (§8.2.2) — takes a **format `$2`**
    ///   frame carrying the instruction's address in the `PC` field and the
    ///   referenced address, with bit 0 cleared, in the address field.
    /// - Everything else takes the **format `$7`** access error frame, thirty
    ///   words, laid out in §8.4.6 and written here field by field.
    ///
    /// # No pending write-backs, and why that is a legal frame
    ///
    /// The 68040's frame exists to let a handler finish what the pipeline had
    /// half-done: up to three write-backs, plus a cache line to push. This
    /// interpreter has neither a write-back pipeline nor a cache, so there is
    /// never anything pending, and all three write-back status bytes are
    /// written **invalid** (`V = 0`). That is not an evasion — it is the
    /// first row of M68040UM Table 8-6, "All Read Access Errors ... WB1S 0,
    /// WB2S 0, WB3S 0, Easy Cleanup: None", and a conforming handler that
    /// checks the `V` bits does nothing and returns.
    ///
    /// `RTE` then **restarts the instruction**, which is the same bargain
    /// this core already makes on the 68010 and the 68020, and is what
    /// §8.2.1 describes anyway: "The saved PC value is the logical address of
    /// the instruction executing at the time the fault was detected". The
    /// registers the partly executed instruction had stepped are put back
    /// from the frame's unused write-back fields, which a handler ignores
    /// because the status bytes say they are invalid.
    #[allow(clippy::too_many_arguments)]
    fn fault_040(
        &mut self,
        vector: u8,
        addr: u32,
        read: bool,
        fc: u8,
        width: u8,
        sr: u16,
        undo: &UndoList,
    ) {
        if vector == vector::ADDRESS_ERROR {
            // Format $2: "the address of the instruction that caused the
            // address error as well as the actual address referenced ... bit
            // 0 of the referenced address is cleared" (§8.2.2, §8.4.3).
            let referenced = addr & !1;
            let mut image = FrameImage::new(2);
            image.push((referenced >> 16) as u16);
            image.push(referenced as u16);
            let pc0 = self.pc0;
            self.enter_exception(vector, pc0, sr, Frame::Format(image));
            return;
        }
        let ssw = self.ssw_040(read, fc, width);
        let mut image = FrameImage::new(7);
        // +$08 effective address. Only meaningful when one of the four
        // continuation flags is set in the SSW, and none ever is here: this
        // core takes a floating-point post-instruction, unimplemented or
        // trace exception at the instruction boundary, never stacked behind
        // an access error (§8.4.6.1).
        image.push(0);
        image.push(0);
        image.push(ssw); // +$0C special status word
        image.push(0); // +$0E write-back 3 status: $00, then V=0
        image.push(0); // +$10 write-back 2 status
        image.push(0); // +$12 write-back 1 status
        image.push((addr >> 16) as u16); // +$14 fault address
        image.push(addr as u16);
        // +$18 .. +$3B: three write-back address/data pairs and four long
        // words of push data, all ignored because the status bytes above say
        // the write-backs are invalid and the SSW says this was not a push.
        // The register undo list this core's `RTE` needs goes here, in the
        // same spirit as the 68010's and 68020's "internal" words.
        image.push_undo(undo, 6); // +$18..+$3B, six slots of three words
        let pc0 = self.pc0;
        self.enter_exception(vector, pc0, sr, Frame::Format(image));
    }

    /// The 68040's special status word (M68040UM Figure 8-7).
    ///
    /// `CP`, `CU`, `CT` and `CM` are the four continuation flags and are
    /// always clear here (see [`Exec::fault_040`]). `MA` is for the second
    /// page of an access that spans two, which this core never reports
    /// separately. `ATC` distinguishes a translation failure from a physical
    /// bus error, and the caller says which through `fc`'s companion — see
    /// [`Exec::atc_fault`].
    fn ssw_040(&self, read: bool, fc: u8, width: u8) -> u16 {
        let size = match width {
            1 => 0b01,
            2 => 0b10,
            16 => 0b11, // a line, which only `MOVE16` and a push produce
            _ => 0b00,
        };
        let (tt, tm) = ssw_transfer(fc, self.fault_alternate);
        (u16::from(self.atc_fault) << 10)
            | (u16::from(self.locked_transfer()) << 9)
            | (u16::from(read) << 8)
            | (size << 5)
            | (u16::from(tt) << 3)
            | u16::from(tm)
    }

    /// Whether the faulted access was part of a read-modify-write, which the
    /// 68040's `LK` bit reports (M68040UM §8.4.6.2).
    fn locked_transfer(&self) -> bool {
        matches!(
            decode_for(self.model, self.opcode).op,
            Op::Tas | Op::Cas | Op::Cas2
        )
    }

    /// Whether the memory management unit can translate `addr` for an
    /// instruction fetch.
    ///
    /// This is how a **deferred** prefetch fault gets its `ATC` bit right.
    /// [`Exec::atc_fault`] is set where the translation fails, but a 68040
    /// prefetch failure is not reported there — "bus errors that occur during
    /// instruction prefetches are deferred until the processor attempts to
    /// use the information" (M68040UM §8.2.1), and by then the flag belongs
    /// to a later access, in a later `Exec` that has never seen it. So the
    /// answer is re-derived from the cache instead, which costs a lookup and
    /// no table search: a failed translation has already installed an entry
    /// with **R** clear, and a translation that succeeded before the *bus*
    /// refused the cycle has left one with **R** set.
    ///
    /// The privilege mode read here is the one at the instruction boundary
    /// rather than the one the prefetch ran under. The two differ only when
    /// the instruction between them changed `SR`, and an entry is tagged by
    /// `FC2`, so the worst case is an `ATC` bit that reports the wrong *kind*
    /// of fault for one instruction after a privilege change.
    fn fetch_translates_040(&self, addr: u32) -> bool {
        if !self.mmu040_on {
            return true;
        }
        let supervisor = self.state.supervisor();
        // The instruction pair, because this is a prefetch (§3.4).
        let pair = self.state.mmu040.ttr_pair(true);
        if mmu040::transparent(pair, addr, supervisor).is_some() {
            return true;
        }
        if !self.state.mmu040.enabled() {
            return true;
        }
        match self.state.mmu040.lookup(addr, supervisor) {
            Some(entry) => {
                entry.data & mmusr040::R != 0 && (supervisor || entry.data & mmusr040::S == 0)
            }
            None => false,
        }
    }

    /// The address registers and stack pointers the faulting instruction has
    /// moved, with the values they had when it started, most important first.
    ///
    /// Data registers never need undoing: nothing writes one and then goes on
    /// to a bus cycle that could fault, except `MOVEM`, which loads it again
    /// when restarted. The registers the opcode names come first, because the
    /// one a `MOVEM` is walking must survive a list too long for the frame.
    fn undo_list(&self) -> UndoList {
        let mut out = [(0u8, 0u32); 10];
        let mut n = 0usize;
        let now_a = self.state.a;
        let now_sp = [
            self.state.sp(Bank::User),
            self.state.sp(Bank::Interrupt),
            self.state.sp(Bank::Master),
        ];
        let consider = |code: u8, out: &mut [(u8, u32); 10], n: &mut usize| {
            if out[..*n].iter().any(|(c, _)| *c == code) {
                return;
            }
            let (old, now) = if code < 7 {
                (self.snap.a[code as usize], now_a[code as usize])
            } else {
                let bank = (code - 7) as usize;
                (self.snap.sp[bank], now_sp[bank])
            };
            if old != now && *n < out.len() {
                out[*n] = (code, old);
                *n += 1;
            }
        };
        for reg in [self.opcode & 7, (self.opcode >> 9) & 7] {
            if reg < 7 {
                consider(reg as u8, &mut out, &mut n);
            }
        }
        for code in 7..10 {
            consider(code, &mut out, &mut n);
        }
        for code in 0..7 {
            consider(code, &mut out, &mut n);
        }
        (out, n)
    }

    // ------------------------------------------------------------------
    // Instruction dispatch
    // ------------------------------------------------------------------

    /// Fetch, decode and execute one instruction, then service any exception
    /// it raised and any pending trace.
    fn instruction(&mut self) {
        self.opcode = self.state.prefetch[0];
        let pc0 = self.state.pc;
        self.pc0 = pc0;
        self.flow = false;
        let sr0 = self.state.sr;
        let traced = sr0 & flags::T != 0;
        // A 68020 traces only a change of flow when **T0** alone is set
        // (MC68020UM §6.1.7, Table 6-2); **T1** with **T0** is reserved, and
        // is read as **T1**.
        let traced_flow = self.model.has_020() && sr0 & flags::T0 != 0;
        if self.model.has_010() {
            self.snap = Snap {
                a: [
                    self.state.a[0],
                    self.state.a[1],
                    self.state.a[2],
                    self.state.a[3],
                    self.state.a[4],
                    self.state.a[5],
                    self.state.a[6],
                ],
                sp: [
                    self.state.sp(Bank::User),
                    self.state.sp(Bank::Interrupt),
                    self.state.sp(Bank::Master),
                ],
            };
        }
        // An opcode a 68020 could not fetch is a bus error now, at the
        // instruction boundary: the short frame, whose `RTE` refetches it
        // (MC68020UM §6.1.2, §6.2).
        if let Some(addr) = self.state.poison[0] {
            self.fault_at_boundary(addr);
            return;
        }
        let insn = decode_with(self.model, self.copro, self.opcode);
        let restarted = self.state.replay.is_some();
        self.facts = timing::Facts::default();

        let outcome = if insn.privileged && !self.state.supervisor() {
            // A privilege violation is detected before anything is fetched, so
            // the pushed program counter is the instruction's own.
            Err(Trap::at(vector::PRIVILEGE, pc0))
        } else {
            self.execute(insn, pc0)
        };
        // A completed fault a restarted instruction did not reach is dropped
        // with the instruction: it described this one and no other.
        if restarted {
            self.state.replay = None;
        }
        if self.model.has_020() {
            let size = insn.size.resolve(self.opcode).unwrap_or(Size::Word);
            self.table += match outcome {
                Ok(()) => timing::instruction(insn, self.opcode, size, &self.facts),
                // A jump, branch or return that got as far as fetching from
                // an odd address has done all its work first.
                Err(Trap::Address { .. }) if self.flow => {
                    timing::instruction(insn, self.opcode, size, &self.facts)
                }
                Err(_) => timing::before_exception(insn, &self.facts),
            };
        }

        match outcome {
            Ok(()) => {
                if traced || (traced_flow && self.flow) {
                    let pc = self.state.pc;
                    self.service(Trap::six(vector::TRACE, pc, pc0));
                }
            }
            Err(trap) => {
                self.service(trap);
                // On a 68020 an instruction whose own execution raised an
                // exception is still traced, straight after it: the trap is
                // processed first, then the trace, so the trace handler
                // returns into the trap handler (MC68020UM §6.1.11). A 68000
                // or 68010 does not, and this core keeps what they do.
                let executed = matches!(
                    trap,
                    Trap::Vectored {
                        kind: Kind::Trap | Kind::Six(_),
                        ..
                    }
                );
                if self.model.has_020() && executed && !self.state.halted && (traced || traced_flow)
                {
                    let pc = self.state.pc;
                    self.service(Trap::six(vector::TRACE, pc, pc0));
                }
            }
        }
    }

    /// A 68020 bus error on an opcode fetch, raised when the processor
    /// reaches it: format $A, "at instruction boundary", both pipe stages to
    /// be rerun (MC68020UM §6.2.1).
    fn fault_at_boundary(&mut self, addr: u32) {
        if self.model.has_040() {
            return self.fault_at_boundary_040(addr);
        }
        let mut image = FrameImage::new(0xa);
        image.push(0); // +$08 internal register
        image.push(SSW_FC | SSW_RC | SSW_RB); // +$0A
        for _ in 0..2 {
            image.push(0); // +$0C, +$0E stages C and B
        }
        image.push((addr >> 16) as u16); // +$10 fault address
        image.push(addr as u16);
        for _ in 0..6 {
            image.push(0); // +$14..+$1F
        }
        let pc = self.state.pc;
        let sr = self.state.sr;
        self.enter_exception(vector::BUS_ERROR, pc, sr, Frame::Format(image));
    }

    /// The same, on a 68040: format `$7` with the instruction's own
    /// transfer modifier (M68040UM §8.2.1, §8.4.6).
    ///
    /// "Bus errors that occur during instruction prefetches are deferred
    /// until the processor attempts to use the information", and when the
    /// exception does arrive "the stacked PC points to the exceptional
    /// instruction, and the stacked FA points to the first longword in the
    /// missing page" (§3.5). "Since the processor allows all pending
    /// accesses to complete before reporting an instruction fault, the stack
    /// frame for an instruction fault will not contain any pending
    /// write-backs" — which is the frame this core builds for every fault
    /// anyway.
    fn fault_at_boundary_040(&mut self, addr: u32) {
        let fc = self.program_fc();
        let (tt, tm) = ssw_transfer(fc, false);
        // Read, word wide: this core fetches instructions sixteen bits at a
        // time on every model, where a 68040 fetches a long word or a line.
        // That is the dynamic-bus-sizing approximation the 68020 notes in
        // the ledger already, showing through into the frame.
        let atc = !self.fetch_translates_040(addr);
        let ssw =
            (u16::from(atc) << 10) | (1 << 8) | (0b10 << 5) | (u16::from(tt) << 3) | u16::from(tm);
        let mut image = FrameImage::new(7);
        image.push(0); // +$08 effective address: no continuation pending
        image.push(0);
        image.push(ssw); // +$0C
        image.push(0); // +$0E write-back 3 status
        image.push(0); // +$10 write-back 2 status
        image.push(0); // +$12 write-back 1 status
        image.push((addr >> 16) as u16); // +$14 fault address
        image.push(addr as u16);
        for _ in 0..18 {
            image.push(0); // +$18..+$3B write-back and push data
        }
        let pc = self.state.pc;
        let sr = self.state.sr;
        self.enter_exception(vector::BUS_ERROR, pc, sr, Frame::Format(image));
    }

    /// Execute one decoded instruction.
    ///
    /// `pc0` is the address of the opcode word, which several exceptions push.
    #[allow(clippy::too_many_lines)]
    fn execute(&mut self, insn: Insn, pc0: u32) -> Result<(), Trap> {
        let opcode = self.opcode;
        let size = match insn.size.resolve(opcode) {
            Some(size) => size,
            // decode() already turned this into Op::Illegal; the row it
            // returned carries no size, so any value will do.
            None => Size::Word,
        };
        match insn.op {
            Op::Illegal => Err(Trap::at(vector::ILLEGAL, pc0)),
            Op::LineA => Err(Trap::at(vector::LINE_A, pc0)),
            Op::LineF => Err(Trap::at(vector::LINE_F, pc0)),
            Op::Nop => self.settle(),
            Op::Reset => {
                // 124 clocks of RESET asserted, then the final prefetch.
                self.internal(128);
                self.lines.pulse_reset();
                self.settle()
            }
            Op::Stop => {
                let word = self.ext(0)?;
                self.state.set_sr(word);
                // The queue is settled *before* stopping, so the program
                // counter an arriving interrupt pushes is the instruction
                // after this one. A stopped 68000 cannot fetch, so hardware
                // makes these two bus cycles on the way out instead of the way
                // in; the state either way is the same, and the alternative is
                // an RTE that returns into the middle of the STOP.
                self.settle()?;
                self.state.stopped = true;
                Ok(())
            }

            Op::Move | Op::Movea => self.op_move(insn, size),
            Op::Moveq => {
                let value = i32::from(opcode as i8) as u32;
                self.state.d[reg_hi(opcode)] = value;
                self.set_logic_flags(value, Size::Long);
                self.settle()
            }

            Op::Add | Op::Addi | Op::Addq => self.op_binary(insn, size, BinOp::Add),
            Op::Sub | Op::Subi | Op::Subq => self.op_binary(insn, size, BinOp::Sub),
            Op::And | Op::Andi => self.op_binary(insn, size, BinOp::And),
            Op::Or | Op::Ori => self.op_binary(insn, size, BinOp::Or),
            Op::Eor | Op::Eori => self.op_binary(insn, size, BinOp::Eor),
            Op::Cmp | Op::Cmpi => self.op_compare(insn, size),
            Op::Cmpm => self.op_cmpm(size),
            Op::Adda | Op::Suba => self.op_adda(insn, size),
            Op::Cmpa => self.op_cmpa(size),
            Op::Addx | Op::Subx => self.op_addx(insn, size),
            Op::Abcd | Op::Sbcd => self.op_bcd(insn),
            Op::Nbcd => self.op_nbcd(),

            Op::Neg | Op::Negx | Op::Not | Op::Clr => self.op_unary(insn, size),
            Op::Tst => self.op_tst(size),
            Op::Tas => self.op_tas(),
            Op::Ext => {
                let n = reg_lo(opcode);
                let value = if size == Size::Long {
                    i32::from(self.state.d[n] as i16) as u32
                } else {
                    let byte = self.state.d[n] as i8;
                    (self.state.d[n] & 0xffff_0000) | u32::from(byte as u16)
                };
                self.state.d[n] = value;
                self.set_logic_flags(value, size);
                self.settle()
            }
            Op::Swap => {
                let n = reg_lo(opcode);
                let value = self.state.d[n].rotate_left(16);
                self.state.d[n] = value;
                self.set_logic_flags(value, Size::Long);
                self.settle()
            }
            Op::Exg => self.op_exg(insn),

            Op::Muls | Op::Mulu => self.op_mul(insn),
            Op::Divs | Op::Divu => self.op_div(insn, pc0),
            Op::Chk => self.op_chk(size),

            Op::Btst | Op::Bchg | Op::Bclr | Op::Bset => self.op_bit(insn, size),
            Op::Asl | Op::Asr | Op::Lsl | Op::Lsr | Op::Rol | Op::Ror | Op::Roxl | Op::Roxr => {
                self.op_shift(insn, size)
            }

            Op::Lea => {
                let Loc::Mem(addr) = self.resolve_control(Arg::Ea, ExtraCycles::Control)? else {
                    return Err(Trap::at(vector::ILLEGAL, pc0));
                };
                self.state.a[reg_hi(opcode)] = addr;
                self.settle()
            }
            Op::Pea => self.op_pea(),
            Op::Jmp => {
                let target = self.jump_target()?;
                self.refill(target, 0)
            }
            Op::Jsr => self.op_jsr(pc0),
            Op::Bra | Op::Bsr | Op::Bcc => self.op_branch(insn),
            Op::Dbcc => self.op_dbcc(),
            Op::Scc => self.op_scc(),
            Op::Rts => {
                let sp = self.state.a[7];
                let target = self.read_long(sp)?;
                self.state.a[7] = sp.wrapping_add(4);
                self.refill(target, 0)
            }
            // RTE and RTR read their six-byte frame in an order that is
            // neither ascending nor a long access: the high half of the
            // program counter, then the status word, then the low half. The
            // microcode is holding the status word's slot open while it
            // decides what privilege state to return to.
            Op::Rtr => {
                let sp = self.state.a[7];
                let high = self.read_word(sp.wrapping_add(2))?;
                let ccr = self.read_word(sp)?;
                let low = self.read_word(sp.wrapping_add(4))?;
                self.state.a[7] = sp.wrapping_add(6);
                let sr = (self.state.sr & !flags::CCR) | (ccr & flags::CCR);
                self.state.set_sr(sr);
                self.refill((u32::from(high) << 16) | u32::from(low), 0)
            }
            Op::Rte => {
                if self.model.has_010() {
                    return self.op_rte_formatted();
                }
                let sp = self.state.a[7];
                let high = self.read_word(sp.wrapping_add(2))?;
                let sr = self.read_word(sp)?;
                let low = self.read_word(sp.wrapping_add(4))?;
                self.state.a[7] = sp.wrapping_add(6);
                self.state.set_sr(sr);
                self.refill((u32::from(high) << 16) | u32::from(low), 0)
            }
            Op::Rtd => {
                // RTS, then the displacement added to the stack pointer. The
                // displacement is already in the queue and is never fetched on
                // its own, so this costs what RTS does: MC68000UM Table 9-18
                // gives both 16(4/0).
                let disp = i32::from(self.queued()? as i16) as u32;
                let sp = self.state.a[7];
                let target = self.read_long(sp)?;
                self.state.a[7] = sp.wrapping_add(4).wrapping_add(disp);
                self.refill(target, 0)
            }
            Op::Trap => {
                let n = (opcode & 0xf) as u8;
                // A trap pushes the address of the *next* instruction. No
                // prefetch happens first, so that address is computed rather
                // than reached by sliding the queue.
                let next = self.state.pc.wrapping_add(2);
                Err(Trap::raised(vector::TRAP_BASE.wrapping_add(n), next))
            }
            Op::Trapv => {
                // Unlike TRAP, TRAPV finishes its prefetch first, so the
                // address it pushes is reached rather than computed.
                self.settle()?;
                if self.state.flag(flags::V) {
                    self.prologue = 0;
                    let pc = self.state.pc;
                    return Err(Trap::six(vector::TRAPV, pc, pc0));
                }
                Ok(())
            }
            Op::Link => self.op_link(size),
            Op::Unlk => {
                let n = reg_lo(opcode);
                let frame = self.state.a[n];
                let saved = self.read_long(frame)?;
                // The stack pointer is restored first and the register second,
                // so `UNLK A7` ends up holding the popped value rather than
                // the frame pointer plus four.
                self.state.a[7] = frame.wrapping_add(4);
                self.state.a[n] = saved;
                self.settle()
            }

            Op::MoveFromSr | Op::MoveFromCcr => self.op_move_from_sr(insn),
            Op::MoveToCcr | Op::MoveToSr => self.op_move_to_sr(insn),
            Op::MoveUsp => {
                let n = reg_lo(opcode);
                if insn.src == Arg::Usp {
                    self.state.a[n] = self.state.usp();
                } else {
                    let value = self.state.a[n];
                    self.state.set_usp(value);
                }
                self.settle()
            }
            Op::OriToCcr | Op::AndiToCcr | Op::EoriToCcr => self.op_imm_to_ccr(insn.op),
            Op::OriToSr | Op::AndiToSr | Op::EoriToSr => self.op_imm_to_sr(insn.op),
            Op::Movem => self.op_movem(insn, size),
            Op::Movep => self.op_movep(insn, size),

            // ---- the 68010's additions ------------------------------------
            Op::Bkpt => {
                // A breakpoint acknowledge cycle in CPU space, which nothing
                // here can answer — `MemAttrs` has no function code — so it
                // ends the way an unanswered one does, with the processor
                // taking an illegal-instruction exception through vector 4
                // and pushing the BKPT's own address (MC68020UM §6.1.10). The
                // acknowledge cycle is charged the four clocks the 68010
                // table assumes for it (MC68000UM Table 9-19).
                self.internal(4);
                Err(Trap::at(vector::ILLEGAL, pc0))
            }
            Op::Movec => self.op_movec(insn, pc0),
            Op::Moves => self.op_moves(size),

            // ---- the 68020's additions ------------------------------------
            Op::Bfchg
            | Op::Bfclr
            | Op::Bfexts
            | Op::Bfextu
            | Op::Bfffo
            | Op::Bfins
            | Op::Bfset
            | Op::Bftst => self.op_bitfield(insn.op),
            Op::Callm => self.op_callm(),
            Op::Rtm => self.op_rtm(),
            Op::Cas => self.op_cas(size),
            Op::Cas2 => self.op_cas2(size),
            Op::Cmp2 => self.op_cmp2(size),
            Op::Divl => self.op_divl(),
            Op::Mull => self.op_mull(),
            Op::Extb => self.op_extb(),
            Op::Pack => self.op_pack(),
            Op::Unpk => self.op_unpk(),
            Op::Trapcc => self.op_trapcc(),
            Op::Pgen => self.op_pgen(),
            Op::Fpgen => self.op_fpgen(),
            Op::Fbcc => self.op_fbcc(),
            Op::Fdbcc => self.op_fdbcc(),
            Op::Fscc => self.op_fscc(),
            Op::Ftrapcc => self.op_ftrapcc(),
            Op::Fsave => self.op_fsave(),
            Op::Frestore => self.op_frestore(),
            Op::Move16 => self.op_move16(),
            Op::Cinvl | Op::Cinvp | Op::Cinva | Op::Cpushl | Op::Cpushp | Op::Cpusha => {
                self.op_cache()
            }
            Op::Pflush | Op::Pflushn | Op::Pflusha | Op::Pflushan => self.op_pflush(insn.op),
            Op::Ptestr => self.op_ptest_040(true),
            Op::Ptestw => self.op_ptest_040(false),
        }
    }

    // ------------------------------------------------------------------
    // Operand resolution
    // ------------------------------------------------------------------

    /// Resolve an operand slot to a [`Loc`].
    fn resolve(&mut self, arg: Arg, size: Size) -> Result<Loc, Trap> {
        let opcode = self.opcode;
        match arg {
            Arg::None => Ok(Loc::Value(0)),
            Arg::DnHi => Ok(Loc::D(reg_hi(opcode) as u8)),
            Arg::DnLo => Ok(Loc::D(reg_lo(opcode) as u8)),
            Arg::AnHi => Ok(Loc::A(reg_hi(opcode) as u8)),
            Arg::AnLo => Ok(Loc::A(reg_lo(opcode) as u8)),
            Arg::Quick => {
                let q = (opcode >> 9) & 7;
                Ok(Loc::Value(if q == 0 { 8 } else { u32::from(q) }))
            }
            Arg::QuickByte => Ok(Loc::Value(i32::from(opcode as i8) as u32)),
            Arg::Vector => Ok(Loc::Value(u32::from(opcode & 0xf))),
            Arg::Imm => {
                let value = match size {
                    Size::Byte => u32::from(self.ext(0)? & 0xff),
                    Size::Word => u32::from(self.ext(0)?),
                    Size::Long => {
                        let hi = self.ext(0)?;
                        let lo = self.ext(0)?;
                        (u32::from(hi) << 16) | u32::from(lo)
                    }
                };
                Ok(Loc::Value(value))
            }
            Arg::Disp16 => {
                let word = self.ext(0)?;
                Ok(Loc::Value(i32::from(word as i16) as u32))
            }
            Arg::Ccr => Ok(Loc::Value(u32::from(self.state.ccr()))),
            Arg::Sr => Ok(Loc::Value(u32::from(self.state.sr))),
            Arg::Usp => Ok(Loc::Value(self.state.usp())),
            Arg::Ea | Arg::EaDst => self.resolve_ea(arg, size, ExtraCycles::Operand),
            // The register-pair, register-list, shift-count, branch-offset and
            // MOVEP slots are not values in an addressing sense: the
            // instruction bodies that use them read the opcode themselves,
            // because what they need is a register *number* or a direction
            // rather than an operand. Nothing routes them through here, and
            // the arm is spelled out so that adding an `Arg` cannot silently
            // start resolving to zero.
            Arg::RmLo
            | Arg::RmHi
            | Arg::PostLo
            | Arg::PostHi
            | Arg::ShiftCount
            | Arg::RegList
            | Arg::MovepEa
            | Arg::BitNumber
            | Arg::Disp8
            | Arg::Disp32
            | Arg::Vector3
            | Arg::Ctrl
            | Arg::ExtReg
            | Arg::TrapData => {
                debug_assert!(false, "{arg:?} is not resolved as an operand");
                Ok(Loc::Value(0))
            }
        }
    }

    /// Resolve an effective address for an instruction that only wants the
    /// address, never the operand — `LEA`, `PEA`, `JMP`, `JSR`.
    fn resolve_control(&mut self, arg: Arg, extra: ExtraCycles) -> Result<Loc, Trap> {
        self.resolve_ea(arg, Size::Long, extra)
    }

    fn resolve_ea(&mut self, arg: Arg, size: Size, extra: ExtraCycles) -> Result<Loc, Trap> {
        let opcode = self.opcode;
        let Some((mode, reg)) = ea_of(arg, opcode) else {
            // decode() rejected this already; reaching here means a caller
            // asked for an address the row does not have.
            debug_assert!(false, "{opcode:04x} has no effective address in {arg:?}");
            return Ok(Loc::Value(0));
        };
        let reg = reg as usize;
        // The indexed modes are classed when their extension word is read.
        if !matches!(mode, Mode::Index8 | Mode::PcIndex8) {
            self.facts.ea(ea_class(mode, size));
        }
        match mode {
            Mode::DataReg => Ok(Loc::D(reg as u8)),
            Mode::AddrReg => Ok(Loc::A(reg as u8)),
            Mode::Indirect => Ok(Loc::Mem(self.state.a[reg])),
            Mode::PostInc => {
                let addr = self.state.a[reg];
                // The register advances as the address is calculated, so an
                // access that faults still leaves it advanced — except as a
                // `MOVE` destination, where nothing has touched the address
                // register by the time the write is attempted.
                if extra == ExtraCycles::MoveDest {
                    self.deferred_postincrement = Some((reg as u8, step(size, reg)));
                } else {
                    self.state.a[reg] = addr.wrapping_add(step(size, reg));
                }
                Ok(Loc::Mem(addr))
            }
            Mode::PreDec => {
                // Two internal cycles for the decrement, before the access —
                // except as a `MOVE` destination, where the decrement overlaps
                // the prefetch that a move performs before its write and costs
                // nothing (MC68000UM Table 8-5 against Table 8-1).
                if extra != ExtraCycles::MoveDest {
                    self.internal(2);
                }
                let addr = self.state.a[reg].wrapping_sub(step(size, reg));
                // As a MOVE destination the register is stepped by the write
                // itself, a word at a time; anywhere else the whole decrement
                // happens here.
                if extra != ExtraCycles::MoveDest {
                    self.state.a[reg] = addr;
                }
                Ok(Loc::Mem(addr))
            }
            Mode::Disp16 => {
                let disp = i32::from(self.ext(0)? as i16) as u32;
                Ok(Loc::Mem(self.state.a[reg].wrapping_add(disp)))
            }
            Mode::Index8 => {
                if self.model.has_020() {
                    let base = self.state.a[reg];
                    return self.index_020(base, extra.index_delay()).map(Loc::Mem);
                }
                let word = self.ext(extra.index_delay())?;
                Ok(Loc::Mem(self.index_address(self.state.a[reg], word)))
            }
            Mode::AbsShort => {
                let word = self.ext(0)?;
                Ok(Loc::Mem(i32::from(word as i16) as u32))
            }
            Mode::AbsLong => {
                let hi = self.ext(0)?;
                // A `MOVE` to an absolute long address writes its operand
                // *before* the last of its own instruction fetches — but only
                // when the source came out of memory. With a register or
                // immediate source the microcode has a spare cycle and does
                // the fetch first. The difference is visible in the program
                // counter an address error pushes, so it is not a free choice.
                let defer = extra == ExtraCycles::MoveDest && self.source_was_memory;
                let lo = if defer {
                    self.ext_deferred()?
                } else {
                    self.ext(0)?
                };
                Ok(Loc::Mem((u32::from(hi) << 16) | u32::from(lo)))
            }
            Mode::PcDisp16 => {
                // The base is the address of the extension word itself, which
                // is `pc + 2` while the word is still in the queue.
                let base = self.state.pc.wrapping_add(2);
                let disp = i32::from(self.ext(0)? as i16) as u32;
                Ok(Loc::Mem(base.wrapping_add(disp)))
            }
            Mode::PcIndex8 => {
                let base = self.state.pc.wrapping_add(2);
                if self.model.has_020() {
                    return self.index_020(base, extra.index_delay()).map(Loc::Mem);
                }
                let word = self.ext(extra.index_delay())?;
                Ok(Loc::Mem(self.index_address(base, word)))
            }
            Mode::Imm => {
                let value = match size {
                    Size::Byte => u32::from(self.ext(0)? & 0xff),
                    Size::Word => u32::from(self.ext(0)?),
                    Size::Long => {
                        let hi = self.ext(0)?;
                        let lo = self.ext(0)?;
                        (u32::from(hi) << 16) | u32::from(lo)
                    }
                };
                Ok(Loc::Value(value))
            }
        }
    }

    /// Compute a `JMP` or `JSR` target.
    ///
    /// Not the same calculation as any other effective address, in two ways
    /// the bus shows. The **last extension word is taken straight out of the
    /// prefetch queue with no refill**: the queue is about to be reloaded from
    /// the target, so paying for a fetch that will be thrown away would be a
    /// wasted bus cycle and the 68000 does not make one — which is why
    /// `JMP (d16,An)` is ten cycles and not fourteen. And the microcode's
    /// address arithmetic is charged as a block up front rather than spread
    /// around a fetch (MC68000UM Table 8-13).
    fn jump_target(&mut self) -> Result<u32, Trap> {
        if self.model.has_020() {
            // The 68020's jump address is an ordinary control address,
            // including the memory-indirect modes, and its timing comes from
            // the tables rather than from the queue (MC68020UM §8.2.5).
            let Loc::Mem(target) = self.resolve_control(Arg::Ea, ExtraCycles::Control)? else {
                let pc0 = self.pc0;
                return Err(Trap::at(vector::ILLEGAL, pc0));
            };
            return Ok(target);
        }
        let Some((mode, reg)) = ea_of(Arg::Ea, self.opcode) else {
            // decode() only lets control modes reach here.
            debug_assert!(false, "{:04x} is not a jump", self.opcode);
            return Ok(self.state.pc);
        };
        let reg = reg as usize;
        let queued = self.state.prefetch[1];
        Ok(match mode {
            Mode::Indirect => self.state.a[reg],
            Mode::Disp16 => {
                self.internal(2);
                self.state.a[reg].wrapping_add(i32::from(queued as i16) as u32)
            }
            Mode::Index8 => {
                self.internal(6);
                self.index_address(self.state.a[reg], queued)
            }
            Mode::AbsShort => {
                self.internal(2);
                i32::from(queued as i16) as u32
            }
            Mode::AbsLong => {
                // Two words, and only the first of them is worth a refill.
                let hi = self.ext(0)?;
                (u32::from(hi) << 16) | u32::from(self.state.prefetch[1])
            }
            Mode::PcDisp16 => {
                self.internal(2);
                self.state
                    .pc
                    .wrapping_add(2)
                    .wrapping_add(i32::from(queued as i16) as u32)
            }
            Mode::PcIndex8 => {
                self.internal(6);
                let base = self.state.pc.wrapping_add(2);
                self.index_address(base, queued)
            }
            Mode::DataReg | Mode::AddrReg | Mode::PostInc | Mode::PreDec | Mode::Imm => {
                debug_assert!(false, "{:04x} jumps to a non-control mode", self.opcode);
                self.state.pc
            }
        })
    }

    /// Apply a brief extension word to a base address.
    ///
    /// Bit 15 selects the data or address file, bits 14–12 the register, bit
    /// 11 whether the index is the sign-extended low word or the whole
    /// register, and bits 7–0 are a signed displacement. Bits 10–8 are the
    /// 68020's scale and full-format bits and are ignored here, which is what
    /// a 68000 does with them (M68000PRM §2.1).
    fn index_address(&self, base: u32, ext: u16) -> u32 {
        debug_assert!(!self.model.has_020(), "a 68020 indexes through index_020");
        let reg = ((ext >> 12) & 7) as usize;
        let value = if ext & 0x8000 != 0 {
            self.state.a[reg]
        } else {
            self.state.d[reg]
        };
        let index = if ext & 0x0800 != 0 {
            value
        } else {
            i32::from(value as i16) as u32
        };
        let disp = i32::from(ext as i8) as u32;
        base.wrapping_add(index).wrapping_add(disp)
    }

    /// The value of an index register, sized and scaled.
    ///
    /// The 68020 multiplies the index by 1, 2, 4 or 8 from bits 10–9 of the
    /// extension word, in either format (M68000PRM §2.2.3); the scaled value is
    /// 32 bits and wraps.
    fn scaled_index(&self, addr: bool, reg: u8, long: bool, scale: u8) -> u32 {
        let reg = reg as usize;
        let value = if addr {
            self.state.a[reg]
        } else {
            self.state.d[reg]
        };
        let value = if long {
            value
        } else {
            i32::from(value as i16) as u32
        };
        value.wrapping_shl(u32::from(scale))
    }

    /// A 68020 indexed address: the brief format with its scale, or the full
    /// format with base and outer displacements, suppression and memory
    /// indirection (M68000PRM §2.2.3, Figure 2-2 and Table 2-2).
    ///
    /// `base` is `An`, or for the PC-relative modes the address of this
    /// extension word. The displacements follow the word in the instruction
    /// stream, base before outer, and are read before the indirection, which
    /// is a long data read — misaligned or not, since a 68020 does not care.
    ///
    /// A reserved full-format word is an illegal instruction: see
    /// [`FullExt::decode`].
    fn index_020(&mut self, base: u32, delay: u32) -> Result<u32, Trap> {
        let word = self.ext(delay)?;
        if !is_full_format(self.model, word) {
            self.facts.ea(timing::INDEX8);
            let index = self.scaled_index(
                word & 0x8000 != 0,
                ((word >> 12) & 7) as u8,
                word & 0x0800 != 0,
                ((word >> 9) & 3) as u8,
            );
            let disp = i32::from(word as i8) as u32;
            return Ok(base.wrapping_add(index).wrapping_add(disp));
        }
        let Some(full) = FullExt::decode(word) else {
            let pc0 = self.pc0;
            return Err(Trap::at(vector::ILLEGAL, pc0));
        };
        self.facts.ea(match full.indirect {
            Indirect::None => match full.bd_words {
                0 => timing::BASE,
                1 if !full.base_suppressed && !full.index_suppressed => timing::INDEX16,
                1 => timing::BASE16,
                _ => timing::BASE32,
            },
            _ => timing::INDIRECT + 3 * full.bd_words + full.od_words,
        });
        let bd = self.displacement(full.bd_words)?;
        let od = self.displacement(full.od_words)?;
        let base = if full.base_suppressed { 0 } else { base };
        let index = if full.index_suppressed {
            0
        } else {
            self.scaled_index(full.index_addr, full.index_reg, full.index_long, full.scale)
        };
        Ok(match full.indirect {
            Indirect::None => base.wrapping_add(bd).wrapping_add(index),
            Indirect::Pre => {
                let pointer = self.read_long(base.wrapping_add(bd).wrapping_add(index))?;
                pointer.wrapping_add(od)
            }
            Indirect::Post => {
                let pointer = self.read_long(base.wrapping_add(bd))?;
                pointer.wrapping_add(index).wrapping_add(od)
            }
        })
    }

    /// A base or outer displacement of `words` words, sign-extended.
    fn displacement(&mut self, words: u8) -> Result<u32, Trap> {
        Ok(match words {
            1 => i32::from(self.ext(0)? as i16) as u32,
            2 => {
                let hi = self.ext(0)?;
                let lo = self.ext(0)?;
                (u32::from(hi) << 16) | u32::from(lo)
            }
            _ => 0,
        })
    }

    /// Read a resolved operand.
    fn read_loc(&mut self, loc: Loc, size: Size) -> Result<u32, Trap> {
        Ok(match loc {
            Loc::D(n) => self.state.d[n as usize] & size.mask(),
            Loc::A(n) => self.state.a[n as usize] & size.mask(),
            Loc::Value(v) => v & size.mask(),
            Loc::Prefetched(_, value) => value & size.mask(),
            Loc::Mem(addr) => match size {
                Size::Byte => u32::from(self.read_byte(addr)?),
                Size::Word => u32::from(self.read_word(addr)?),
                Size::Long => self.read_long(addr)?,
            },
        })
    }

    /// Write a resolved operand.
    fn write_loc(&mut self, loc: Loc, size: Size, value: u32) -> Result<(), Trap> {
        match loc {
            Loc::D(n) => {
                let n = n as usize;
                self.state.d[n] = merge(self.state.d[n], value, size);
            }
            // Every write to an address register is 32 bits wide, whatever the
            // instruction's size says (M68000PRM §1.2).
            Loc::A(n) => self.state.a[n as usize] = value,
            Loc::Value(_) => {}
            Loc::Prefetched(addr, _) | Loc::Mem(addr) => match size {
                Size::Byte => self.write_byte(addr, value as u8)?,
                Size::Word => self.write_word(addr, value as u16)?,
                Size::Long => self.write_long(addr, value)?,
            },
        }
        Ok(())
    }

    /// Commit the result of a read-modify-write.
    ///
    /// Identical to [`Exec::write_loc`] except for a long memory destination,
    /// which goes out low word first — see [`Exec::write_long_low_first`].
    fn write_back(&mut self, loc: Loc, size: Size, value: u32) -> Result<(), Trap> {
        if let (Loc::Mem(addr) | Loc::Prefetched(addr, _), Size::Long) = (loc, size) {
            return self.write_long_low_first(addr, value);
        }
        self.write_loc(loc, size, value)
    }

    // ------------------------------------------------------------------
    // Flags
    // ------------------------------------------------------------------

    fn set_flag(&mut self, mask: u16, on: bool) {
        if on {
            self.state.sr |= mask;
        } else {
            self.state.sr &= !mask;
        }
    }

    /// `N` and `Z` from a result, `V` and `C` cleared — the logical and move
    /// rule, which is the only regular one in the instruction set.
    fn set_logic_flags(&mut self, value: u32, size: Size) {
        let value = value & size.mask();
        self.set_flag(flags::N, value & size.sign_bit() != 0);
        self.set_flag(flags::Z, value == 0);
        self.set_flag(flags::V, false);
        self.set_flag(flags::C, false);
    }

    // ------------------------------------------------------------------
    // Instruction bodies
    // ------------------------------------------------------------------

    fn op_move(&mut self, insn: Insn, size: Size) -> Result<(), Trap> {
        self.source_was_memory =
            ea_of(insn.src, self.opcode).is_some_and(|(mode, _)| mode.is_memory());
        let src = self.resolve(insn.src, size)?;
        let value = self.read_loc(src, size)?;
        if insn.op == Op::Movea {
            // MOVEA.W sign-extends into the whole register.
            let value = if size == Size::Word {
                i32::from(value as i16) as u32
            } else {
                value
            };
            self.state.a[reg_hi(self.opcode)] = value;
            return self.settle();
        }
        let dst = self.resolve_ea(insn.dst, size, ExtraCycles::MoveDest)?;
        self.set_logic_flags(value, size);
        // A predecrement destination is the one `MOVE` that prefetches before
        // it writes, and its long form puts the low word out first.
        if let Some((Mode::PreDec, reg)) = ea_of(insn.dst, self.opcode) {
            let reg = reg as usize;
            self.settle()?;
            if size == Size::Long {
                // Two word writes, low half first, with the register stepping
                // between them: a fault on the first leaves it two bytes down,
                // not four, and the handler can tell.
                let low = self.state.a[reg].wrapping_sub(2);
                self.state.a[reg] = low;
                self.write_word(low, value as u16)?;
                let high = low.wrapping_sub(2);
                self.state.a[reg] = high;
                self.write_word(high, (value >> 16) as u16)?;
            } else {
                let addr = self.state.a[reg].wrapping_sub(step(size, reg));
                self.state.a[reg] = addr;
                self.write_loc(Loc::Mem(addr), size, value)?;
            }
        } else {
            self.write_loc(dst, size, value)?;
            self.settle()?;
        }
        if let Some((reg, by)) = self.deferred_postincrement.take() {
            let reg = reg as usize;
            self.state.a[reg] = self.state.a[reg].wrapping_add(by);
        }
        Ok(())
    }

    fn op_binary(&mut self, insn: Insn, size: Size, kind: BinOp) -> Result<(), Trap> {
        let src = self.resolve(insn.src, size)?;
        let src_value = self.read_loc(src, size)?;
        let dst = self.resolve(insn.dst, size)?;
        // ADDQ/SUBQ on an address register is a full 32-bit add that touches
        // no flags at all (M68000PRM, ADDQ).
        if let Loc::A(n) = dst
            && matches!(insn.op, Op::Addq | Op::Subq)
        {
            let base = self.state.a[n as usize];
            self.state.a[n as usize] = if kind == BinOp::Add {
                base.wrapping_add(src_value)
            } else {
                base.wrapping_sub(src_value)
            };
            // A long quick add to an address register is two cycles cheaper
            // than a word one: the word form has to sign-extend first.
            self.internal(if size == Size::Long { 2 } else { 4 });
            return self.settle();
        }
        let dst_value = self.read_loc(dst, size)?;
        let result = match kind {
            BinOp::Add => {
                let r = dst_value.wrapping_add(src_value) & size.mask();
                self.set_add_flags(src_value, dst_value, r, size, true);
                r
            }
            BinOp::Sub => {
                let r = dst_value.wrapping_sub(src_value) & size.mask();
                self.set_sub_flags(src_value, dst_value, r, size, true);
                r
            }
            BinOp::And => {
                let r = dst_value & src_value & size.mask();
                self.set_logic_flags(r, size);
                r
            }
            BinOp::Or => {
                let r = (dst_value | src_value) & size.mask();
                self.set_logic_flags(r, size);
                r
            }
            BinOp::Eor => {
                let r = (dst_value ^ src_value) & size.mask();
                self.set_logic_flags(r, size);
                r
            }
        };
        self.arith_internal(insn, size, dst);
        if matches!(dst, Loc::D(_) | Loc::A(_)) {
            self.write_loc(dst, size, result)?;
            self.settle()
        } else {
            // A memory destination is written after the final prefetch on a
            // 68000; the write is the last bus cycle of a read-modify-write
            // only for TAS (MC68000UM Table 8-5).
            self.settle()?;
            self.write_back(dst, size, result)
        }
    }

    /// The long-operand penalty the manual marks with a double dagger.
    ///
    /// A 32-bit ALU operation into a data register costs two extra cycles, and
    /// four when the source needed no bus cycle of its own — a register or an
    /// immediate (MC68000UM Table 8-5, note **).
    fn arith_internal(&mut self, insn: Insn, size: Size, dst: Loc) {
        if size != Size::Long || !matches!(dst, Loc::D(_)) {
            return;
        }
        let cheap_source = match insn.src {
            // EOR is the one row whose source is named directly rather than
            // through an effective address, and it is still a register.
            Arg::Imm | Arg::Quick | Arg::DnHi => true,
            Arg::Ea => matches!(
                ea_of(Arg::Ea, self.opcode),
                Some((Mode::DataReg | Mode::AddrReg | Mode::Imm, _))
            ),
            _ => false,
        };
        self.internal(if cheap_source { 4 } else { 2 });
    }

    fn op_compare(&mut self, insn: Insn, size: Size) -> Result<(), Trap> {
        let src = self.resolve(insn.src, size)?;
        let src_value = self.read_loc(src, size)?;
        let dst = self.resolve(insn.dst, size)?;
        let dst_value = self.read_loc(dst, size)?;
        let result = dst_value.wrapping_sub(src_value) & size.mask();
        // CMP leaves X alone: it is a test, not an arithmetic step.
        self.set_sub_flags(src_value, dst_value, result, size, false);
        if size == Size::Long && matches!(dst, Loc::D(_)) {
            self.internal(2);
        }
        self.settle()
    }

    fn op_cmpm(&mut self, size: Size) -> Result<(), Trap> {
        let y = reg_lo(self.opcode);
        let x = reg_hi(self.opcode);
        let src_addr = self.state.a[y];
        self.state.a[y] = src_addr.wrapping_add(step(size, y));
        let src_value = self.read_loc(Loc::Mem(src_addr), size)?;
        let dst_addr = self.state.a[x];
        self.state.a[x] = dst_addr.wrapping_add(step(size, x));
        let dst_value = self.read_loc(Loc::Mem(dst_addr), size)?;
        let result = dst_value.wrapping_sub(src_value) & size.mask();
        self.set_sub_flags(src_value, dst_value, result, size, false);
        self.settle()
    }

    fn op_adda(&mut self, insn: Insn, size: Size) -> Result<(), Trap> {
        let src = self.resolve(insn.src, size)?;
        let raw = self.read_loc(src, size)?;
        // A word source is sign-extended to 32 bits before the add; the
        // operation itself is always long (M68000PRM, ADDA).
        let value = if size == Size::Word {
            i32::from(raw as i16) as u32
        } else {
            raw
        };
        let n = reg_hi(self.opcode);
        let base = self.state.a[n];
        self.state.a[n] = if insn.op == Op::Adda {
            base.wrapping_add(value)
        } else {
            base.wrapping_sub(value)
        };
        let cheap = matches!(
            ea_of(Arg::Ea, self.opcode),
            Some((Mode::DataReg | Mode::AddrReg | Mode::Imm, _))
        );
        self.internal(if size == Size::Word || cheap { 4 } else { 2 });
        self.settle()
    }

    fn op_cmpa(&mut self, size: Size) -> Result<(), Trap> {
        let src = self.resolve(Arg::Ea, size)?;
        let raw = self.read_loc(src, size)?;
        let value = if size == Size::Word {
            i32::from(raw as i16) as u32
        } else {
            raw
        };
        let dst_value = self.state.a[reg_hi(self.opcode)];
        let result = dst_value.wrapping_sub(value);
        self.set_sub_flags(value, dst_value, result, Size::Long, false);
        self.internal(2);
        self.settle()
    }

    fn op_addx(&mut self, insn: Insn, size: Size) -> Result<(), Trap> {
        let x = u32::from(self.state.flag(flags::X));
        let memory = self.opcode & 0x0008 != 0;
        let (src_value, dst) = if memory {
            let y = reg_lo(self.opcode);
            let xr = reg_hi(self.opcode);
            // One pair of internal cycles for the whole instruction, not one
            // per predecrement: the second address calculation overlaps the
            // first operand's fetch (MC68000UM Table 8-8).
            self.internal(2);
            let src_value = self.read_predecrement(y, size)?;
            let dst_value = self.read_predecrement(xr, size)?;
            (src_value, Loc::Prefetched(self.state.a[xr], dst_value))
        } else {
            let src_value = self.state.d[reg_lo(self.opcode)] & size.mask();
            (src_value, Loc::D(reg_hi(self.opcode) as u8))
        };
        let dst_value = self.read_loc(dst, size)?;
        // Z is only ever *cleared* by an extended operation: a multi-precision
        // sum is zero only if every step of it was, so a zero result leaves Z
        // exactly as the previous step left it (M68000PRM, ADDX).
        let was_zero = self.state.flag(flags::Z);
        let result = if insn.op == Op::Addx {
            let r = dst_value.wrapping_add(src_value).wrapping_add(x) & size.mask();
            self.set_add_flags(src_value, dst_value, r, size, true);
            r
        } else {
            let r = dst_value.wrapping_sub(src_value).wrapping_sub(x) & size.mask();
            self.set_sub_flags(src_value, dst_value, r, size, true);
            r
        };
        self.set_flag(flags::Z, result == 0 && was_zero);
        if size == Size::Long && matches!(dst, Loc::D(_)) {
            self.internal(4);
        }
        if let Loc::D(_) = dst {
            self.write_loc(dst, size, result)?;
            return self.settle();
        }
        // A long extended result puts its low word out, prefetches, and only
        // then writes the high word — the prefetch lands *between* the two
        // halves of the write, which no other instruction does.
        let (Loc::Mem(addr) | Loc::Prefetched(addr, _)) = dst else {
            return self.settle();
        };
        if size == Size::Long {
            self.write_word(addr.wrapping_add(2), result as u16)?;
            self.settle()?;
            return self.write_word(addr, (result >> 16) as u16);
        }
        self.settle()?;
        self.write_back(dst, size, result)
    }

    fn op_bcd(&mut self, insn: Insn) -> Result<(), Trap> {
        let size = Size::Byte;
        let memory = self.opcode & 0x0008 != 0;
        let (src_value, dst) = if memory {
            let y = reg_lo(self.opcode);
            let x = reg_hi(self.opcode);
            let src_value = self.read_predecrement(y, size)?;
            let dst_value = self.read_predecrement(x, size)?;
            (src_value, Loc::Prefetched(self.state.a[x], dst_value))
        } else {
            (
                self.state.d[reg_lo(self.opcode)] & 0xff,
                Loc::D(reg_hi(self.opcode) as u8),
            )
        };
        let dst_value = self.read_loc(dst, size)?;
        let result = if insn.op == Op::Abcd {
            self.bcd_add(src_value, dst_value)
        } else {
            self.bcd_sub(src_value, dst_value)
        };
        self.internal(2);
        if matches!(dst, Loc::D(_)) {
            self.write_loc(dst, size, result)?;
            self.settle()
        } else {
            self.settle()?;
            self.write_back(dst, size, result)
        }
    }

    fn op_nbcd(&mut self) -> Result<(), Trap> {
        let dst = self.resolve(Arg::Ea, Size::Byte)?;
        let value = self.read_loc(dst, Size::Byte)?;
        let result = self.bcd_sub(value, 0);
        if matches!(dst, Loc::D(_)) {
            self.internal(2);
            self.write_loc(dst, Size::Byte, result)?;
            self.settle()
        } else {
            self.settle()?;
            self.write_back(dst, Size::Byte, result)
        }
    }

    fn op_unary(&mut self, insn: Insn, size: Size) -> Result<(), Trap> {
        let dst = self.resolve(insn.dst, size)?;
        // CLR still reads its destination on a 68000 — the read is a real bus
        // cycle and a device can see it (MC68000UM Table 8-6, and the reason
        // CLR is not usable on a read-sensitive register). The 68010 dropped
        // the read: its CLR to memory is one read, the prefetch, and one write
        // (MC68000UM Table 9-10), and so is the 68020's (MC68020UM §8.2.11).
        let value = if insn.op == Op::Clr && self.model.has_010() {
            0
        } else {
            self.read_loc(dst, size)?
        };
        let x = u32::from(self.state.flag(flags::X));
        let result = match insn.op {
            Op::Clr => {
                self.set_flag(flags::N, false);
                self.set_flag(flags::Z, true);
                self.set_flag(flags::V, false);
                self.set_flag(flags::C, false);
                0
            }
            Op::Not => {
                let r = !value & size.mask();
                self.set_logic_flags(r, size);
                r
            }
            Op::Neg => {
                let r = 0u32.wrapping_sub(value) & size.mask();
                self.set_sub_flags(value, 0, r, size, true);
                r
            }
            _ => {
                let r = 0u32.wrapping_sub(value).wrapping_sub(x) & size.mask();
                let before = self.state.flag(flags::Z);
                self.set_sub_flags(value, 0, r, size, true);
                self.set_flag(flags::Z, if r == 0 { before } else { false });
                r
            }
        };
        if size == Size::Long && matches!(dst, Loc::D(_)) {
            self.internal(2);
        }
        if matches!(dst, Loc::D(_) | Loc::A(_)) {
            self.write_loc(dst, size, result)?;
            self.settle()
        } else {
            self.settle()?;
            self.write_back(dst, size, result)
        }
    }

    fn op_tst(&mut self, size: Size) -> Result<(), Trap> {
        let src = self.resolve(Arg::Ea, size)?;
        let value = self.read_loc(src, size)?;
        self.set_logic_flags(value, size);
        self.settle()
    }

    fn op_tas(&mut self) -> Result<(), Trap> {
        let dst = self.resolve(Arg::Ea, Size::Byte)?;
        let value = self.read_loc(dst, Size::Byte)?;
        self.set_logic_flags(value, Size::Byte);
        let result = value | 0x80;
        if matches!(dst, Loc::D(_)) {
            self.write_loc(dst, Size::Byte, result)?;
            self.settle()
        } else {
            // The read-modify-write cycle is indivisible: the write follows
            // the read immediately, before the prefetch (M68000PRM, TAS).
            self.internal(2);
            self.write_loc(dst, Size::Byte, result)?;
            self.settle()
        }
    }

    fn op_exg(&mut self, insn: Insn) -> Result<(), Trap> {
        let hi = reg_hi(self.opcode);
        let lo = reg_lo(self.opcode);
        match (insn.src, insn.dst) {
            (Arg::DnHi, Arg::DnLo) => self.state.d.swap(hi, lo),
            (Arg::AnHi, Arg::AnLo) => self.state.a.swap(hi, lo),
            _ => {
                core::mem::swap(&mut self.state.d[hi], &mut self.state.a[lo]);
            }
        }
        self.internal(2);
        self.settle()
    }

    fn op_mul(&mut self, insn: Insn) -> Result<(), Trap> {
        let src = self.resolve(Arg::Ea, Size::Word)?;
        let source = self.read_loc(src, Size::Word)? as u16;
        let n = reg_hi(self.opcode);
        let dest = self.state.d[n] as u16;
        let (result, extra) = if insn.op == Op::Mulu {
            // 38 cycles plus two per one bit in the source (MC68000UM
            // Table 8-6); four of the 38 are the final prefetch.
            let product = u32::from(source).wrapping_mul(u32::from(dest));
            (product, 34 + 2 * source.count_ones())
        } else {
            let product = (i32::from(source as i16)).wrapping_mul(i32::from(dest as i16)) as u32;
            // MULS counts the 01 and 10 pairs of the source with a zero
            // appended below it, which is the Booth encoding the microcode
            // steps through. Sixteen pairs, so bit 16 of the difference is not
            // one of them — counting it makes every negative multiplier two
            // cycles too slow.
            let pairs = (u32::from(source) << 1) ^ u32::from(source);
            (product, 34 + 2 * (pairs & 0xffff).count_ones())
        };
        self.internal(extra);
        self.state.d[n] = result;
        self.set_logic_flags(result, Size::Long);
        self.settle()
    }

    fn op_div(&mut self, insn: Insn, pc0: u32) -> Result<(), Trap> {
        let src = self.resolve(Arg::Ea, Size::Word)?;
        let divisor = self.read_loc(src, Size::Word)? as u16;
        let n = reg_hi(self.opcode);
        let dividend = self.state.d[n];
        if divisor == 0 {
            // Division by zero does not prefetch, does not advance the queue,
            // and pushes the address of the instruction itself. The condition
            // codes the manual leaves undefined are cleared, X excepted.
            //
            // The corpus contains exactly one divide-by-zero vector, so this
            // rests on a single measurement; it is recorded here rather than
            // guessed at from the manual, which says only "undefined".
            self.set_flag(flags::N, false);
            self.set_flag(flags::Z, false);
            self.set_flag(flags::V, false);
            self.set_flag(flags::C, false);
            self.prologue = 8;
            if self.model.has_020() {
                // The 68020's six-word frame pushes the next instruction and
                // the divide's own address (MC68020UM Table 6-5).
                let next = self.state.pc.wrapping_add(2);
                return Err(Trap::six(vector::DIVIDE_BY_ZERO, next, pc0));
            }
            return Err(Trap::at(vector::DIVIDE_BY_ZERO, pc0));
        }
        if insn.op == Op::Divu {
            let quotient = dividend / u32::from(divisor);
            let remainder = dividend % u32::from(divisor);
            self.internal(divu_cycles(dividend, divisor));
            if quotient > 0xffff {
                // Overflow leaves the destination untouched and sets V.
                self.set_flag(flags::V, true);
                self.set_flag(flags::C, false);
                return self.settle();
            }
            self.state.d[n] = (remainder << 16) | (quotient & 0xffff);
            self.set_logic_flags(quotient & 0xffff, Size::Word);
        } else {
            let dividend = dividend as i32;
            let divisor = i32::from(divisor as i16);
            self.internal(divs_cycles(dividend, divisor as i16));
            let quotient = dividend.wrapping_div(divisor);
            let remainder = dividend.wrapping_rem(divisor);
            if !(-0x8000..=0x7fff).contains(&quotient) {
                self.set_flag(flags::V, true);
                self.set_flag(flags::C, false);
                return self.settle();
            }
            self.state.d[n] = ((remainder as u32) << 16) | (quotient as u32 & 0xffff);
            self.set_logic_flags(quotient as u32 & 0xffff, Size::Word);
        }
        self.settle()
    }

    fn op_chk(&mut self, size: Size) -> Result<(), Trap> {
        let pc0 = self.pc0;
        let src = self.resolve(Arg::Ea, size)?;
        // CHK.L is the 68020's, and compares all 32 bits (M68000PRM, *CHK*).
        let (bound, value) = if size == Size::Long {
            (
                self.read_loc(src, size)? as i32,
                self.state.d[reg_hi(self.opcode)] as i32,
            )
        } else {
            (
                i32::from(self.read_loc(src, size)? as i16),
                i32::from(self.state.d[reg_hi(self.opcode)] as i16),
            )
        };
        // The manual defines N only for the two out-of-bounds cases and calls
        // Z, V and C undefined. The hardware clears those three and — this is
        // the part no document states — leaves **N alone** when the register
        // is in range. Only the two cases that trap write it, which is exactly
        // the two cases the manual defines.
        self.set_flag(flags::Z, false);
        self.set_flag(flags::V, false);
        self.set_flag(flags::C, false);
        // Unlike a trap, CHK completes its prefetch before vectoring, so the
        // address it pushes is the next instruction's by construction.
        self.settle()?;
        if value < 0 || value > bound {
            self.set_flag(flags::N, value < 0);
            // Two extra cycles of prologue unless the bound test is what
            // failed: a register above its bound is decided a test earlier
            // than a negative one.
            self.prologue = if value > bound { 4 } else { 6 };
            let pc = self.state.pc;
            return Err(Trap::six(vector::CHK, pc, pc0));
        }
        self.internal(6);
        Ok(())
    }

    fn op_bit(&mut self, insn: Insn, size: Size) -> Result<(), Trap> {
        let bit = match insn.src {
            Arg::BitNumber => u32::from(self.ext(0)? & 0xff),
            _ => self.state.d[reg_hi(self.opcode)],
        };
        let dst = self.resolve(insn.dst, size)?;
        let width = if size == Size::Long { 32 } else { 8 };
        let bit = bit % width;
        let value = self.read_loc(dst, size)?;
        let mask = 1u32 << bit;
        self.set_flag(flags::Z, value & mask == 0);
        let result = match insn.op {
            Op::Btst => {
                // A long test costs two more, and so does one whose operand is
                // an immediate: the bit number has to be reduced modulo the
                // operand size either way, and nothing else is happening.
                if size == Size::Long
                    || matches!(ea_of(insn.dst, self.opcode), Some((Mode::Imm, _)))
                {
                    self.internal(2);
                }
                return self.settle();
            }
            Op::Bchg => value ^ mask,
            Op::Bclr => value & !mask,
            _ => value | mask,
        };
        if matches!(dst, Loc::D(_)) {
            // A long bit operation on a register costs two more, and BCLR two
            // more again (MC68000UM Table 8-7).
            self.internal(if bit >= 16 { 4 } else { 2 });
            if insn.op == Op::Bclr {
                self.internal(2);
            }
            self.write_loc(dst, size, result)?;
            self.settle()
        } else {
            self.settle()?;
            self.write_back(dst, size, result)
        }
    }

    fn op_shift(&mut self, insn: Insn, size: Size) -> Result<(), Trap> {
        if insn.dst == Arg::Ea {
            // The memory form shifts one bit of one word.
            let dst = self.resolve(Arg::Ea, Size::Word)?;
            let value = self.read_loc(dst, Size::Word)?;
            let result = self.shift(insn.op, value, 1, Size::Word);
            self.settle()?;
            return self.write_back(dst, Size::Word, result);
        }
        let count = if self.opcode & 0x0020 == 0 {
            let q = (self.opcode >> 9) & 7;
            if q == 0 { 8 } else { u32::from(q) }
        } else {
            self.state.d[reg_hi(self.opcode)] % 64
        };
        let n = reg_lo(self.opcode);
        let value = self.state.d[n] & size.mask();
        let result = self.shift(insn.op, value, count, size);
        self.state.d[n] = merge(self.state.d[n], result, size);
        // Two cycles per bit, on top of the two (word) or four (long) the
        // instruction costs before it starts (MC68000UM Table 8-12).
        self.internal(if size == Size::Long { 4 } else { 2 } + 2 * count);
        self.settle()
    }

    /// One shift or rotate, setting the flags the manual gives it.
    ///
    /// The shift count is taken modulo 64, so it routinely exceeds the operand
    /// width, and what happens then is the part worth stating: **the carry runs
    /// out when the operand does.** The 68000 shifts one bit per cycle through
    /// a register of the operand's width, so a count larger than that width
    /// shifts in nothing but fill and leaves `C` and `X` clear — including for
    /// `ASR` of a negative value, whose *result* is all ones while its carry is
    /// zero. Shifting the sign bit back out again, which is what a naive loop
    /// does, is wrong at exactly the boundary a compiler's shift-by-register
    /// code lands on.
    fn shift(&mut self, op: Op, value: u32, count: u32, size: Size) -> u32 {
        let bits = size.bytes() * 8;
        let mask = size.mask();
        let sign = size.sign_bit();
        let mut result = value & mask;
        let mut carry = false;
        let mut overflow = false;
        // Past the operand's width the register holds only fill, so the loop
        // has nothing left to do.
        let steps = count.min(bits);
        let exhausted = count > bits;
        match op {
            Op::Asl => {
                for _ in 0..steps {
                    carry = result & sign != 0;
                    let next = (result << 1) & mask;
                    // V is set if the sign bit changed at *any* point in the
                    // shift, not just at the end (M68000PRM, ASL).
                    if (next ^ result) & sign != 0 {
                        overflow = true;
                    }
                    result = next;
                }
                if exhausted {
                    carry = false;
                }
            }
            Op::Asr => {
                for _ in 0..steps {
                    carry = result & 1 != 0;
                    result = (result >> 1) | (result & sign);
                }
                if exhausted {
                    carry = false;
                }
            }
            Op::Lsl => {
                for _ in 0..steps {
                    carry = result & sign != 0;
                    result = (result << 1) & mask;
                }
                if exhausted {
                    carry = false;
                }
            }
            Op::Lsr => {
                for _ in 0..steps {
                    carry = result & 1 != 0;
                    result >>= 1;
                }
                if exhausted {
                    carry = false;
                }
            }
            Op::Rol => {
                for _ in 0..count {
                    carry = result & sign != 0;
                    result = ((result << 1) | u32::from(carry)) & mask;
                }
            }
            Op::Ror => {
                for _ in 0..count {
                    carry = result & 1 != 0;
                    result = (result >> 1) | (if carry { sign } else { 0 });
                }
            }
            Op::Roxl => {
                let mut x = self.state.flag(flags::X);
                for _ in 0..count {
                    carry = result & sign != 0;
                    result = ((result << 1) | u32::from(x)) & mask;
                    x = carry;
                }
                if count == 0 {
                    carry = x;
                } else {
                    self.set_flag(flags::X, x);
                }
                self.set_flag(flags::N, result & sign != 0);
                self.set_flag(flags::Z, result == 0);
                self.set_flag(flags::V, false);
                self.set_flag(flags::C, carry);
                return result;
            }
            // ROXR, and the catch-all only because `Op` is non-exhaustive.
            _ => {
                let mut x = self.state.flag(flags::X);
                for _ in 0..count {
                    carry = result & 1 != 0;
                    result = (result >> 1) | (if x { sign } else { 0 });
                    x = carry;
                }
                if count == 0 {
                    carry = x;
                } else {
                    self.set_flag(flags::X, x);
                }
                self.set_flag(flags::N, result & sign != 0);
                self.set_flag(flags::Z, result == 0);
                self.set_flag(flags::V, false);
                self.set_flag(flags::C, carry);
                return result;
            }
        }
        self.set_flag(flags::N, result & sign != 0);
        self.set_flag(flags::Z, result == 0);
        self.set_flag(flags::V, overflow);
        self.set_flag(flags::C, count != 0 && carry);
        // A plain rotate does not touch X — only the shifts and the two
        // rotate-through-extend forms do, and the latter set it themselves.
        // Getting this wrong quietly breaks every multi-precision routine that
        // rotates a mask between `ADDX` steps.
        if count != 0 && matches!(op, Op::Asl | Op::Asr | Op::Lsl | Op::Lsr) {
            self.set_flag(flags::X, carry);
        }
        result
    }

    fn op_pea(&mut self) -> Result<(), Trap> {
        let Loc::Mem(addr) = self.resolve_control(Arg::Ea, ExtraCycles::Control)? else {
            let pc = self.state.pc;
            return Err(Trap::at(vector::ILLEGAL, pc));
        };
        let sp = self.state.a[7].wrapping_sub(4);
        self.state.a[7] = sp;
        // An absolute operand needs no address arithmetic, so the push happens
        // a bus cycle earlier — before the final prefetch rather than after.
        let absolute = matches!(
            ea_of(Arg::Ea, self.opcode),
            Some((Mode::AbsShort | Mode::AbsLong, _))
        );
        if absolute {
            self.write_long(sp, addr)?;
            return self.settle();
        }
        self.settle()?;
        self.write_long(sp, addr)
    }

    /// `JSR`, whose push lands *between* the two fetches of the reloaded
    /// queue.
    ///
    /// That ordering is not decoration: it is why a `JSR` to an odd address
    /// pushes nothing at all before its address-error frame, and so why the
    /// handler sees the stack pointer the caller had.
    fn op_jsr(&mut self, pc0: u32) -> Result<(), Trap> {
        let target = self.jump_target()?;
        if self.model.has_020() {
            // Every extension word has been consumed, so the next instruction
            // is the one after the word in the queue's first slot.
            let ret = self.state.pc.wrapping_add(2);
            let sp = self.state.a[7].wrapping_sub(4);
            self.state.a[7] = sp;
            self.write_long(sp, ret)?;
            return self.refill(target, 0);
        }
        // The return address is the byte after the whole instruction, which
        // the queue never slid to: the last extension word was read out of it
        // rather than fetched.
        let words =
            1 + ea_of(Arg::Ea, self.opcode).map_or(0, |(mode, _)| mode.ext_words(Size::Long));
        let ret = pc0.wrapping_add(2 * words);

        self.deferred_slides = 0;
        let fc = self.program_fc();
        self.state.pc = target.wrapping_sub(4);
        let first = self.read_word_fc(target, fc)?;
        self.state.pc = target.wrapping_sub(2);
        let sp = self.state.a[7].wrapping_sub(4);
        self.state.a[7] = sp;
        self.write_long(sp, ret)?;
        let second = self.read_word_fc(target.wrapping_add(2), fc)?;
        self.state.pc = target;
        self.state.prefetch = [first, second];
        Ok(())
    }

    fn op_branch(&mut self, insn: Insn) -> Result<(), Trap> {
        let byte = self.opcode as i8;
        let taken = match insn.op {
            Op::Bcc => self.state.test(Cond::from_opcode(self.opcode)),
            _ => true,
        };
        self.facts.taken = taken;
        // The base for a branch is the address of the word after the opcode.
        let base = self.state.pc.wrapping_add(2);
        if insn.src == Arg::Disp32 {
            // A 68020 displacement byte of $ff: the displacement is the long
            // that follows, still counted from the word after the opcode
            // (M68000PRM, *Bcc*).
            let hi = self.ext(0)?;
            let lo = self.ext(0)?;
            if !taken {
                return self.settle();
            }
            let target = base.wrapping_add((u32::from(hi) << 16) | u32::from(lo));
            if insn.op == Op::Bsr {
                let ret = base.wrapping_add(4);
                let sp = self.state.a[7].wrapping_sub(4);
                self.state.a[7] = sp;
                self.write_long(sp, ret)?;
            }
            return self.refill(target, 0);
        }
        if byte == 0 {
            let word = self.queued()?;
            if !taken {
                // A word displacement that is not taken still costs the fetch.
                self.internal(4);
                self.ext(0)?;
                return self.settle();
            }
            let target = base.wrapping_add(i32::from(word as i16) as u32);
            self.internal(2);
            if insn.op == Op::Bsr {
                let ret = base.wrapping_add(2);
                let sp = self.state.a[7].wrapping_sub(4);
                self.state.a[7] = sp;
                self.write_long(sp, ret)?;
            }
            return self.refill(target, 0);
        }
        if !taken {
            self.internal(4);
            return self.settle();
        }
        let target = base.wrapping_add(i32::from(byte) as u32);
        self.internal(2);
        if insn.op == Op::Bsr {
            let ret = base;
            let sp = self.state.a[7].wrapping_sub(4);
            self.state.a[7] = sp;
            self.write_long(sp, ret)?;
        }
        self.refill(target, 0)
    }

    fn op_dbcc(&mut self) -> Result<(), Trap> {
        let base = self.state.pc.wrapping_add(2);
        let n = reg_lo(self.opcode);
        if self.state.test(Cond::from_opcode(self.opcode)) {
            // Condition true: the loop is over, the counter is left alone.
            self.internal(4);
            self.ext(0)?;
            return self.settle();
        }
        let counter = (self.state.d[n] as u16).wrapping_sub(1);
        self.state.d[n] = merge(self.state.d[n], u32::from(counter), Size::Word);
        if counter == 0xffff {
            self.facts.expired = true;
            // The counter ran out: fall through, and pay for the two fetches.
            self.internal(6);
            self.ext(0)?;
            return self.settle();
        }
        let word = self.queued()?;
        self.internal(2);
        let target = base.wrapping_add(i32::from(word as i16) as u32);
        self.refill(target, 0)
    }

    fn op_scc(&mut self) -> Result<(), Trap> {
        let set = self.state.test(Cond::from_opcode(self.opcode));
        let dst = self.resolve(Arg::Ea, Size::Byte)?;
        let value = if set { 0xff } else { 0x00 };
        if !matches!(dst, Loc::D(_)) && !self.model.has_010() {
            // A memory destination is read before it is written, exactly as
            // CLR reads one: the 68000 has no write-only bus cycle, and a
            // read-sensitive register notices. The 68010 does not read it
            // (MC68000UM Table 9-9, "use nonfetching effective address
            // calculation time").
            self.read_loc(dst, Size::Byte)?;
        }
        if matches!(dst, Loc::D(_)) {
            // Two extra cycles when the byte is set, which is the one place a
            // 68000's timing depends on a condition (MC68000UM Table 8-11).
            // The 68010 takes four either way (Table 9-9).
            if set && !self.model.has_010() {
                self.internal(2);
            }
            self.write_loc(dst, Size::Byte, value)?;
            self.settle()
        } else {
            self.settle()?;
            self.write_back(dst, Size::Byte, value)
        }
    }

    fn op_link(&mut self, size: Size) -> Result<(), Trap> {
        let n = reg_lo(self.opcode);
        // LINK.L carries a 32-bit displacement (M68000PRM, *LINK*).
        let disp = if size == Size::Long {
            let hi = self.ext(0)?;
            let lo = self.ext(0)?;
            (u32::from(hi) << 16) | u32::from(lo)
        } else {
            i32::from(self.ext(0)? as i16) as u32
        };
        let sp = self.state.a[7].wrapping_sub(4);
        // The stack pointer is decremented *before* the register is read, so
        // `LINK A7,#d` pushes the new A7 rather than the old one — the one
        // case where the two differ, and the one the manual is explicit about.
        self.state.a[7] = sp;
        let value = self.state.a[n];
        self.write_long(sp, value)?;
        self.state.a[n] = sp;
        self.state.a[7] = sp.wrapping_add(disp);
        self.settle()
    }

    fn op_move_from_sr(&mut self, insn: Insn) -> Result<(), Trap> {
        let dst = self.resolve(Arg::Ea, Size::Word)?;
        let value = if insn.op == Op::MoveFromCcr {
            u16::from(self.state.ccr())
        } else {
            self.state.sr
        };
        if self.model.has_010() {
            // The 68010 neither reads the destination first nor spends the
            // two extra cycles on a register: MOVE from SR and MOVE from CCR
            // are 4(1/0) to a register and 8(1/1) plus a non-fetching address
            // calculation to memory (MC68000UM Table 9-18). The write keeps
            // the 68000's place after the final prefetch.
            if matches!(dst, Loc::D(_)) {
                self.write_loc(dst, Size::Word, u32::from(value))?;
                return self.settle();
            }
            self.settle()?;
            return self.write_loc(dst, Size::Word, u32::from(value));
        }
        // The 68000 reads the destination first, which is why MOVE from SR is
        // a read-modify-write and the 68010 replaced it (MC68000UM Table 8-6).
        let _ = self.read_loc(dst, Size::Word)?;
        if matches!(dst, Loc::D(_)) {
            self.internal(2);
            self.write_loc(dst, Size::Word, u32::from(value))?;
            return self.settle();
        }
        self.settle()?;
        self.write_loc(dst, Size::Word, u32::from(value))
    }

    fn op_move_to_sr(&mut self, insn: Insn) -> Result<(), Trap> {
        let src = self.resolve(Arg::Ea, Size::Word)?;
        let value = self.read_loc(src, Size::Word)? as u16;
        if insn.op == Op::MoveToCcr {
            let sr = (self.state.sr & !flags::CCR) | (value & flags::CCR);
            self.state.set_sr(sr);
        } else {
            self.state.set_sr(value);
        }
        self.internal(4);
        // Both forms *reload* the queue rather than sliding it. Writing SR can
        // change the privilege state, and the word already in the queue was
        // fetched with the old function code — so the 68000 fetches it again,
        // which is visible on FC0-FC2 as well as in the cycle count.
        let next = self.state.pc.wrapping_add(2);
        self.refill(next, 0)
    }

    fn op_imm_to_ccr(&mut self, op: Op) -> Result<(), Trap> {
        // An instruction that writes SR is a change of flow to a 68020
        // tracing with T0, since it refills its pipe (MC68020UM §6.1.7).
        self.flow = true;
        let value = self.ext(0)? & 0xff;
        let ccr = u16::from(self.state.ccr());
        let result = match op {
            Op::OriToCcr => ccr | value,
            Op::AndiToCcr => ccr & value,
            _ => ccr ^ value,
        };
        let sr = (self.state.sr & !flags::CCR) | (result & flags::CCR);
        self.state.set_sr(sr);
        self.internal(8);
        self.idle_fetch()?;
        self.settle()
    }

    fn op_imm_to_sr(&mut self, op: Op) -> Result<(), Trap> {
        self.flow = true;
        let value = self.ext(0)?;
        let sr = self.state.sr;
        let result = match op {
            Op::OriToSr => sr | value,
            Op::AndiToSr => sr & value,
            _ => sr ^ value,
        };
        self.state.set_sr(result);
        self.internal(8);
        self.idle_fetch()?;
        self.settle()
    }

    /// Re-read the word already sitting in `prefetch[1]`, discarding it.
    ///
    /// The immediate-to-`SR`/`CCR` instructions really do this: the microcode
    /// spends a bus cycle fetching a word it already has, which is why they
    /// cost twenty cycles for what looks like sixteen cycles of work. It is a
    /// visible access, so a device on the bus sees it.
    fn idle_fetch(&mut self) -> Result<(), Trap> {
        let addr = self.state.pc.wrapping_add(2);
        let fc = self.program_fc();
        self.read_word_fc(addr, fc)?;
        Ok(())
    }

    /// `MOVEM`, which is a **word engine**, not a register engine.
    ///
    /// The distinction is invisible until something goes wrong. A long
    /// transfer is two independent word accesses, and the address register a
    /// predecrement or postincrement form is walking is updated between them —
    /// so an address error half way through a `MOVEM.L` leaves that register
    /// two bytes into the element that failed, not four. Modelling it as "per
    /// register, then update" gets every ordinary case right and that one
    /// wrong, which is the sort of difference an exception handler notices.
    ///
    /// The other two things worth stating: a predecrement destination walks
    /// the mask backwards, `A7` first, and a memory-to-register form always
    /// reads one word past the last register it loads and throws it away
    /// (M68000PRM, *MOVEM*).
    fn op_movem(&mut self, insn: Insn, size: Size) -> Result<(), Trap> {
        let mask = self.ext(0)?;
        self.facts.registers = mask.count_ones();
        let to_memory = insn.dst == Arg::Ea;
        let Some((mode, reg)) = ea_of(Arg::Ea, self.opcode) else {
            let pc = self.state.pc;
            return Err(Trap::at(vector::ILLEGAL, pc));
        };
        let reg = reg as usize;
        let long = size == Size::Long;

        if matches!(mode, Mode::PreDec | Mode::PostInc) {
            // The walking forms never go through `resolve_ea`, which is where
            // an operand's timing class is otherwise noted.
            self.facts.ea(if mode == Mode::PreDec { 4 } else { 3 });
        }
        if to_memory && mode == Mode::PreDec {
            // The register being walked is stored with the value it had
            // *before* the instruction started, not the value it has reached
            // by the time its turn comes. That is a 68000 behaviour the 68020
            // changed — it stores the initial value less one operand
            // (M68000PRM, *MOVEM*) — and `MOVEM.L A7/D0-D7,-(A7)` depends on
            // which one it is running on.
            let initial = if self.model.has_020() {
                self.state.a[reg].wrapping_sub(if long { 4 } else { 2 })
            } else {
                self.state.a[reg]
            };
            for bit in 0..16u32 {
                if mask & (1 << bit) == 0 {
                    continue;
                }
                let index = 15 - bit;
                let value = if index as usize == reg + 8 {
                    initial
                } else {
                    self.register(index)
                };
                // The address register is updated *after* each word lands, so
                // a fault leaves it addressing the word before the one that
                // failed.
                let low = self.state.a[reg].wrapping_sub(2);
                self.write_word(low, value as u16)?;
                self.state.a[reg] = low;
                if long {
                    let high = low.wrapping_sub(2);
                    self.write_word(high, (value >> 16) as u16)?;
                    self.state.a[reg] = high;
                }
            }
            return self.settle();
        }

        let mut addr = if !to_memory && mode == Mode::PostInc {
            self.state.a[reg]
        } else {
            let Loc::Mem(addr) = self.resolve_control(Arg::Ea, ExtraCycles::Operand)? else {
                let pc = self.state.pc;
                return Err(Trap::at(vector::ILLEGAL, pc));
            };
            addr
        };
        let walking = !to_memory && mode == Mode::PostInc;

        for bit in 0..16u32 {
            if mask & (1 << bit) == 0 {
                continue;
            }
            if to_memory {
                let value = self.register(bit);
                if long {
                    self.write_word(addr, (value >> 16) as u16)?;
                    self.write_word(addr.wrapping_add(2), value as u16)?;
                } else {
                    self.write_word(addr, value as u16)?;
                }
                addr = addr.wrapping_add(if long { 4 } else { 2 });
            } else {
                if walking {
                    self.state.a[reg] = addr.wrapping_add(2);
                }
                let high = self.read_word(addr)?;
                let value = if long {
                    if walking {
                        self.state.a[reg] = addr.wrapping_add(4);
                    }
                    let low = self.read_word(addr.wrapping_add(2))?;
                    (u32::from(high) << 16) | u32::from(low)
                } else {
                    i32::from(high as i16) as u32
                };
                addr = addr.wrapping_add(if long { 4 } else { 2 });
                self.set_register(bit, value);
            }
        }
        if !to_memory && !self.model.has_020() {
            // One word past the end, read and discarded. It is a real bus
            // cycle, and a MOVEM that ends at the top of a mapped region can
            // fault on it. The 68020 does not make it: its table counts n
            // reads for n registers (MC68020UM §8.2.7).
            if walking {
                self.state.a[reg] = addr.wrapping_add(2);
            }
            self.read_word(addr)?;
            if walking {
                self.state.a[reg] = addr;
            }
        } else if walking {
            // A walking register the list also loads ends up holding the
            // incremented address, not what was read (M68000PRM, *MOVEM*).
            self.state.a[reg] = addr;
        }
        self.settle()
    }

    /// One of the sixteen registers a `MOVEM` mask can name: `D0`-`D7` then
    /// `A0`-`A7`.
    fn register(&self, index: u32) -> u32 {
        if index < 8 {
            self.state.d[index as usize]
        } else {
            self.state.a[(index - 8) as usize]
        }
    }

    /// Load one of those sixteen registers.
    fn set_register(&mut self, index: u32, value: u32) {
        if index < 8 {
            self.state.d[index as usize] = value;
        } else {
            self.state.a[(index - 8) as usize] = value;
        }
    }

    fn op_movep(&mut self, insn: Insn, size: Size) -> Result<(), Trap> {
        let n = reg_hi(self.opcode);
        let areg = reg_lo(self.opcode);
        let disp = i32::from(self.ext(0)? as i16) as u32;
        let base = self.state.a[areg].wrapping_add(disp);
        let to_memory = insn.dst == Arg::MovepEa;
        let count = if size == Size::Long { 4 } else { 2 };
        if to_memory {
            let value = self.state.d[n];
            for i in 0..count {
                let shift = 8 * (count - 1 - i);
                let byte = (value >> shift) as u8;
                self.write_byte(base.wrapping_add(i * 2), byte)?;
            }
        } else {
            let mut value = 0u32;
            for i in 0..count {
                let byte = self.read_byte(base.wrapping_add(i * 2))?;
                value = (value << 8) | u32::from(byte);
            }
            self.state.d[n] = merge(self.state.d[n], value, size);
        }
        self.settle()
    }

    // ------------------------------------------------------------------
    // The 68010's control instructions
    // ------------------------------------------------------------------

    /// The general register bits 15–12 of an extension word name: `D0`-`D7`
    /// then `A0`-`A7`.
    fn ext_register(&self, word: u16) -> u32 {
        self.register(u32::from(word >> 12))
    }

    /// `MOVEC`: a control register to or from a general one (M68000PRM,
    /// *MOVEC*). Always 32 bits; bits a register does not implement read as
    /// zero, and a code the model does not have is an illegal instruction.
    fn op_movec(&mut self, insn: Insn, pc0: u32) -> Result<(), Trap> {
        let word = self.ext(0)?;
        let code = word & 0x0fff;
        if !ctrl::exists(self.model, code) {
            return Err(Trap::at(vector::ILLEGAL, pc0));
        }
        let to_control = insn.dst == Arg::Ctrl;
        if to_control {
            let value = self.ext_register(word);
            match code {
                ctrl::SFC => self.state.sfc = (value & 7) as u8,
                ctrl::DFC => self.state.dfc = (value & 7) as u8,
                ctrl::USP => self.state.set_sp(Bank::User, value),
                ctrl::VBR => self.state.vbr = value,
                // Only E and F have storage. C and CE act on the cache's
                // contents, which are not modelled, and read as zero
                // (MC68020UM §4.3.1).
                ctrl::CACR => self.state.cacr = value & self.state.cacr_mask(),
                ctrl::CAAR => self.state.caar = value,
                ctrl::MSP => self.state.set_sp(Bank::Master, value),
                // The 68040's memory management registers. "The operating
                // system must flush the ATCs before enabling address
                // translation since the TCR accesses and reset do not flush
                // the ATCs" (M68040UM §3.1.2) — so, unlike the 68030's
                // `PMOVE`, none of these touches the cache.
                ctrl::TC => {
                    self.state.mmu040.tcr = (value as u16) & tcr::IMPLEMENTED;
                    self.mmu040_on = self.state.mmu040.active();
                }
                // "Bits 8–0 of an address loaded into the URP or the SRP must
                // be zero" (§3.1.1). The manual states it as a requirement on
                // software rather than as a register with nine dead bits, and
                // gives no exception for breaking it; the bits are dropped,
                // which is what a register drawn with nine zeros does.
                ctrl::URP => self.state.mmu040.urp = value & !0x1ff,
                ctrl::SRP => self.state.mmu040.srp = value & !0x1ff,
                ctrl::ITT0 | ctrl::ITT1 | ctrl::DTT0 | ctrl::DTT1 => {
                    let pair = if code < ctrl::DTT0 {
                        &mut self.state.mmu040.itt
                    } else {
                        &mut self.state.mmu040.dtt
                    };
                    pair[usize::from(code & 1 != 0)] = value & ttr::IMPLEMENTED;
                    self.mmu040_on = self.state.mmu040.active();
                }
                ctrl::MMUSR => self.state.mmu040.mmusr = value & mmusr040::IMPLEMENTED,
                _ => self.state.set_sp(Bank::Interrupt, value),
            }
            // 10(2/0) against the ext word and the prefetch (MC68000UM Table
            // 9-18, "Register → Destination").
            self.internal(2);
        } else {
            let value = match code {
                ctrl::SFC => u32::from(self.state.sfc),
                ctrl::DFC => u32::from(self.state.dfc),
                ctrl::USP => self.state.sp(Bank::User),
                ctrl::VBR => self.state.vbr,
                ctrl::CACR => self.state.cacr,
                ctrl::CAAR => self.state.caar,
                ctrl::MSP => self.state.sp(Bank::Master),
                ctrl::TC => u32::from(self.state.mmu040.tcr),
                ctrl::URP => self.state.mmu040.urp,
                ctrl::SRP => self.state.mmu040.srp,
                ctrl::ITT0 => self.state.mmu040.itt[0],
                ctrl::ITT1 => self.state.mmu040.itt[1],
                ctrl::DTT0 => self.state.mmu040.dtt[0],
                ctrl::DTT1 => self.state.mmu040.dtt[1],
                ctrl::MMUSR => self.state.mmu040.mmusr,
                _ => self.state.sp(Bank::Interrupt),
            };
            self.set_register(u32::from(word >> 12), value);
            // 12(2/0), "Source → Register".
            self.internal(4);
        }
        self.settle()
    }

    // ------------------------------------------------------------------
    // The 68040's own instructions
    // ------------------------------------------------------------------

    /// `MOVE16`: copy one aligned sixteen-byte line (M68000PRM §4,
    /// *MOVE16*).
    ///
    /// # What the manual settles and what it leaves to the bus
    ///
    /// Both addresses are used with their low four bits ignored — "the lines
    /// are aligned to 16-byte boundaries" — and the worked example is
    /// explicit that `A0 = $1400F` reads the line at `$14000`. An address
    /// register used in the postincrement mode steps by **16 from the value
    /// it held**, so that example leaves `$1401F` behind, not `$14010`.
    ///
    /// On hardware the transfer is a burst that starts at the long word the
    /// effective address actually names and wraps within the line. This core
    /// has no burst: it reads the four long words in ascending order from the
    /// aligned base and writes them the same way. The sixteen bytes that end
    /// up at the destination are the same either way; the order they cross
    /// the bus in is not, and that is in the ledger.
    ///
    /// The postincrement-to-postincrement form with one register named twice
    /// increments it **once** and "the line is copied over itself rather than
    /// to the next line" (M68040UM Table 1-4, note 7).
    fn op_move16(&mut self) -> Result<(), Trap> {
        let opcode = self.opcode;
        let spent_before = (self.used, self.table);
        let (src, dst, steps): (u32, u32, [Option<(usize, u32)>; 2]) = if opcode & 0x20 != 0 {
            // `MOVE16 (Ax)+,(Ay)+`: a second opcode word names the
            // destination register in bits 14-12.
            let word = self.ext(0)?;
            let x = reg_lo(opcode);
            let y = ((word >> 12) & 7) as usize;
            let src = self.state.a[x];
            let dst = self.state.a[y];
            // Wrapping: an address register that steps off the top of the
            // address space wraps, like every other postincrement.
            let steps = if x == y {
                [Some((x, src.wrapping_add(16))), None]
            } else {
                [
                    Some((x, src.wrapping_add(16))),
                    Some((y, dst.wrapping_add(16))),
                ]
            };
            (src, dst, steps)
        } else {
            // The absolute form: bits 4-3 say which side is `(xxx).L`, and
            // whether the register side postincrements.
            let hi = self.ext(0)?;
            let lo = self.ext(0)?;
            let absolute = (u32::from(hi) << 16) | u32::from(lo);
            let y = reg_lo(opcode);
            let reg = self.state.a[y];
            let post = matches!((opcode >> 3) & 3, 0 | 1);
            let step = post.then(|| (y, reg.wrapping_add(16)));
            match (opcode >> 3) & 3 {
                0 | 2 => (reg, absolute, [step, None]),
                _ => (absolute, reg, [step, None]),
            }
        };
        let src = src & !0xf;
        let dst = dst & !0xf;
        let mut line = [0u32; 4];
        for (i, word) in line.iter_mut().enumerate() {
            // Wrapping on the offset: the line is aligned, so `+12` cannot
            // leave it, but the base itself may sit at the top of the space.
            *word = self.read_long(src.wrapping_add(i as u32 * 4))?;
        }
        for (i, word) in line.into_iter().enumerate() {
            self.write_long(dst.wrapping_add(i as u32 * 4), word)?;
        }
        for (reg, value) in steps.into_iter().flatten() {
            // `a[7]` is the active bank and `banks` is reconciled when the
            // status register changes, so writing the array is enough — the
            // same thing every postincrement mode does.
            self.state.a[reg] = value;
        }
        let done = self.settle();
        self.charge_move16(spent_before);
        done
    }

    /// Charge `MOVE16` the accesses it actually drove, as a table entry.
    ///
    /// Neither the MC68020UM tables this core borrows nor M68040UM §10 has a
    /// `MOVE16` row, so the time is the bus cycles rather than a published
    /// number. Those cycles are already in `used` — but [`Exec::step`]
    /// *replaces* `used` with `table` whenever the table is non-zero, and a
    /// table search nested inside this instruction makes it so
    /// ([`Exec::search_cycles`]). Leaving the row at zero therefore threw
    /// the sixteen bytes' worth of transfers away exactly when the
    /// instruction had done the most work. So they go into the table too,
    /// less whatever a search has already put there.
    fn charge_move16(&mut self, before: (u64, u32)) {
        if !self.model.has_020() {
            return;
        }
        let spent = self.used.saturating_sub(before.0) as u32;
        let searched = self.table.saturating_sub(before.1);
        self.table = self.table.saturating_add(spent.saturating_sub(searched));
    }

    /// `CINV` and `CPUSH` (M68000PRM §6; M68040UM §4.2).
    ///
    /// # The cache model, stated rather than implied
    ///
    /// This core has no instruction or data cache: every access goes to
    /// memory, and nothing is ever held. That is a *copyback cache that never
    /// holds a dirty line*, which is a legal state for the hardware to be in
    /// and the one these two instructions are defined against:
    ///
    /// - `CINV` "invalidates selected cache lines"; there are none, so there
    ///   is nothing to invalidate. "Any dirty data in data cache lines that
    ///   invalidate are lost" — there is none to lose.
    /// - `CPUSH` pushes dirty lines and then invalidates; with no dirty line
    ///   anywhere, its best case in M68040UM Table 10-4 — "a cache containing
    ///   no dirty entries" — is what happens, every time.
    ///
    /// So both are no-ops that cost time, and the *guest-visible* behaviour
    /// is right: memory already holds everything a push would have written,
    /// and a later read already sees everything an invalidate would have
    /// exposed. What is not modelled is the cache's effect on *timing* and on
    /// the bus trace, which is the same thing already recorded for the
    /// 68020's and the 68030's caches.
    ///
    /// The address register a line or page operation names is read as a
    /// **physical** address and makes no bus cycle, so nothing here can
    /// fault.
    fn op_cache(&mut self) -> Result<(), Trap> {
        self.settle()
    }

    /// `PFLUSH`, `PFLUSHN`, `PFLUSHA` and `PFLUSHAN` (M68000PRM §6,
    /// *PFLUSH* (MC68040)).
    ///
    /// The function code comes from `DFC` and only its top bit is compared,
    /// because that is all an entry's tag holds: "destination function code
    /// values of 1 or 2 will result in flushing of user address translation
    /// cache entries ... whereas values of 5 or 6 will result in flushing of
    /// supervisor" ones, and the other four values are "undefined and may
    /// cause flushing of an unexpected entry". The address the page form
    /// names is the *contents* of the address register, not an effective
    /// address: the syntax is `PFLUSH (An)` and there is no other mode.
    ///
    /// "PFLUSH can be executed even if the E-bit is cleared" (§3.6.1), which
    /// is why this does not check it — an operating system is told to flush
    /// before enabling translation, so refusing here would break the
    /// sequence the manual prescribes.
    ///
    /// On an MC68EC040 there is no cache to flush. The manual's account of
    /// what the encoding does there is "suspends operation ... for an
    /// indefinite period of time and subsequently continues with no adverse
    /// effects", so it is a no-op that costs nothing.
    fn op_pflush(&mut self, op: Op) -> Result<(), Trap> {
        if self.model.has_mmu_040() {
            let supervisor = self.state.dfc & 4 != 0;
            match op {
                Op::Pflusha => self.state.mmu040.flush_all(),
                Op::Pflushan => self.state.mmu040.flush_non_global(),
                _ => {
                    let la = self.state.a[reg_lo(self.opcode)];
                    self.state
                        .mmu040
                        .flush_page(la, supervisor, op == Op::Pflush);
                }
            }
        }
        self.settle()
    }

    /// `PTESTR` and `PTESTW` (M68000PRM §6, *PTEST* (MC68040); M68040UM
    /// §3.1.4).
    ///
    /// "This instruction searches the translation tables for the page
    /// descriptor corresponding to the test address in An and sets the bits
    /// of the MMU status register according to the status of the descriptors
    /// ... PTESTR simulates a read access and sets the U-bit in each
    /// descriptor during table searches; PTESTW simulates a write access and
    /// also sets the M-bit". The search is the real one, on the real bus,
    /// with the real history write-backs.
    ///
    /// "A matching entry in the address translation cache ... will be flushed
    /// by PTEST. Completion of PTEST results in the creation of a new address
    /// translation cache entry."
    fn op_ptest_040(&mut self, read: bool) -> Result<(), Trap> {
        let la = self.state.a[reg_lo(self.opcode)];
        let supervisor = self.state.dfc & 4 != 0;
        // "Execution of the instruction continues until one of the following
        // conditions occurs: match with one of the two transparent
        // translation registers ..." — and then "the T-bit is set ... the
        // R-bit is set, and all other bits are zero" (§3.1.4, **T**). A
        // `PTEST` goes through the *data* unit, so it is the data pair.
        let pair = self.state.mmu040.dtt;
        if mmu040::transparent(&pair, la, supervisor).is_some() {
            self.state.mmu040.mmusr = mmusr040::T | mmusr040::R;
            return self.settle();
        }
        self.state.mmu040.flush_page(la, supervisor, true);
        let found = self.walk_040(la, supervisor, !read);
        self.state.mmu040.mmusr = found.mmusr();
        if !found.bus_error {
            self.state.mmu040.install(found.entry);
        }
        self.settle()
    }

    // ------------------------------------------------------------------
    // The memory management instructions
    // ------------------------------------------------------------------

    /// The logical address an MMU instruction names.
    ///
    /// "Only control-alterable addressing modes are allowed for MMU
    /// instructions on the MC68030" (MC68030UM §12.1.3), and the address is
    /// the one the mode *computes* rather than the operand at it — which is
    /// why `PFLUSH (SP)` flushes the stack's own page and the manual tells
    /// you to write `PFLUSH [(SP)]` when you meant the address on it.
    fn pmmu_address(&mut self) -> Result<u32, Trap> {
        match ea_of(Arg::Ea, self.opcode) {
            Some((mode, _)) if EaSet::CONTROL_ALT.contains(mode) => {}
            _ => return Err(Trap::at(vector::LINE_F, self.pc0)),
        }
        match self.resolve_control(Arg::Ea, ExtraCycles::Operand)? {
            Loc::Mem(addr) => Ok(addr),
            _ => Err(Trap::at(vector::LINE_F, self.pc0)),
        }
    }

    /// Resolve a function-code operand (M68000PRM §6, *PFLUSH*'s **FC**
    /// field).
    fn pmmu_fc(&self, source: pmmu::FcSource) -> u8 {
        match source {
            pmmu::FcSource::Immediate(fc) => fc & 7,
            pmmu::FcSource::DataReg(n) => (self.state.d[(n & 7) as usize] & 7) as u8,
            pmmu::FcSource::Sfc => self.state.sfc & 7,
            pmmu::FcSource::Dfc => self.state.dfc & 7,
        }
    }

    /// `PMOVE`, `PTEST`, `PLOAD` and `PFLUSH`, told apart by the command word
    /// (M68000PRM §6; MC68030UM §9.7).
    fn op_pgen(&mut self) -> Result<(), Trap> {
        let pc0 = self.pc0;
        let line_f = Trap::at(vector::LINE_F, pc0);
        let command = self.ext(0)?;
        let Some(class) = pmmu::decode(command) else {
            return Err(line_f);
        };
        // An MC68EC030 has the two transparent translation registers and the
        // status register and nothing else; everything the paged unit adds is
        // an unimplemented F-line instruction there (MC68EC030UM §9.4).
        let paged = self.model.has_mmu();
        match class {
            pmmu::Class::Move {
                reg,
                from_reg,
                no_flush,
            } => {
                if reg.needs_mmu() && !paged {
                    return Err(line_f);
                }
                self.op_pmove(reg, from_reg, no_flush, pc0)
            }
            pmmu::Class::Flush(what) => {
                if !paged {
                    return Err(line_f);
                }
                match what {
                    pmmu::Flush::All => self.state.mmu.flush_all(),
                    pmmu::Flush::ByFc(source, mask) => {
                        let fc = self.pmmu_fc(source);
                        self.state.mmu.flush_fc(fc, mask);
                    }
                    pmmu::Flush::ByFcAndAddress(source, mask) => {
                        let fc = self.pmmu_fc(source);
                        let la = self.pmmu_address()?;
                        self.state.mmu.flush_fc_address(fc, mask, la);
                    }
                }
                self.settle()
            }
            pmmu::Class::Load { fc, read } => {
                if !paged {
                    return Err(line_f);
                }
                let fc = self.pmmu_fc(fc);
                let la = self.pmmu_address()?;
                // "PLOAD performs a table search operation ... and loads the
                // entry into the ATC" — for a read or a write, which decides
                // whether the search sets the modified bit.
                let found = self.walk(la, fc, !read, 7);
                self.state.mmu.install(found.entry);
                self.settle()
            }
            pmmu::Class::Test {
                fc,
                level,
                read,
                areg,
            } => {
                // An MC68EC030's PTEST searches only the access control
                // registers, so a level above zero has nothing to search.
                if !paged && level != 0 {
                    return Err(line_f);
                }
                let fc = self.pmmu_fc(fc);
                let la = self.pmmu_address()?;
                self.op_ptest(la, fc, level, read, areg);
                self.settle()
            }
        }
    }

    /// `PMOVE` to or from one memory management register, with the side
    /// effects §9.7.5.1 and §9.7.5.3 give it.
    fn op_pmove(
        &mut self,
        reg: pmmu::PReg,
        from_reg: bool,
        no_flush: bool,
        pc0: u32,
    ) -> Result<(), Trap> {
        let at = self.pmmu_address()?;
        if from_reg {
            match reg {
                pmmu::PReg::Tc => self.write_long(at, self.state.mmu.tc)?,
                pmmu::PReg::Tt0 => self.write_long(at, self.state.mmu.tt[0])?,
                pmmu::PReg::Tt1 => self.write_long(at, self.state.mmu.tt[1])?,
                pmmu::PReg::Mmusr => self.write_word(at, self.state.mmu.mmusr)?,
                pmmu::PReg::Srp | pmmu::PReg::Crp => {
                    let value = if reg == pmmu::PReg::Srp {
                        self.state.mmu.srp
                    } else {
                        self.state.mmu.crp
                    };
                    self.write_long(at, (value >> 32) as u32)?;
                    self.write_long(at.wrapping_add(4), value as u32)?;
                }
            }
            return self.settle();
        }
        // Into the register. The flush comes first because a new mapping
        // makes the old entries wrong the instant it lands (§9.7.5.1).
        let mut misconfigured = false;
        match reg {
            pmmu::PReg::Mmusr => {
                let value = self.read_word(at)?;
                self.state.mmu.mmusr = value & mmusr::IMPLEMENTED;
            }
            pmmu::PReg::Tc => {
                let value = self.read_long(at)? & tc::IMPLEMENTED;
                if !no_flush {
                    self.state.mmu.flush_all();
                }
                // "When written with the E bit set ... a consistency check is
                // performed on the values of PS, IS, and Tlx ... If an MMU
                // configuration exception occurs, the TC register is updated
                // with the data, and the E bit is cleared" (§9.7.2).
                if value & tc::E != 0 && !Mmu::tc_is_consistent(value) {
                    self.state.mmu.tc = value & !tc::E;
                    misconfigured = true;
                } else {
                    self.state.mmu.tc = value;
                }
            }
            pmmu::PReg::Tt0 | pmmu::PReg::Tt1 => {
                let value = self.read_long(at)? & tt::IMPLEMENTED;
                if !no_flush {
                    self.state.mmu.flush_all();
                }
                self.state.mmu.tt[usize::from(reg == pmmu::PReg::Tt1)] = value;
            }
            pmmu::PReg::Srp | pmmu::PReg::Crp => {
                let hi = self.read_long(at)?;
                let lo = self.read_long(at.wrapping_add(4))?;
                let value = ((u64::from(hi) << 32) | u64::from(lo)) & ROOT_POINTER_BITS;
                if !no_flush {
                    self.state.mmu.flush_all();
                }
                if reg == pmmu::PReg::Srp {
                    self.state.mmu.srp = value;
                } else {
                    self.state.mmu.crp = value;
                }
                // "A PMOVE instruction that loads either the CRP or the SRP
                // causes an MMU configuration exception if the new value of
                // the DT field is zero (invalid). In this case, the register
                // is loaded with the new value before the exception is taken"
                // (§9.7.5.3).
                misconfigured = (hi & 3) == 0;
            }
        }
        self.mmu_on = self.model.has_mmu() && self.state.mmu.enabled();
        self.settle()?;
        if misconfigured {
            // Vector 56, and a format $2 frame carrying the address of the
            // PMOVE that did it (MC68030UM Table 8-1, Table 8-6).
            let pc = self.state.pc;
            return Err(Trap::six(vector::MMU_CONFIG, pc, pc0));
        }
        Ok(())
    }

    /// `PTEST`: report on one logical address through `MMUSR` (Table 9-3).
    fn op_ptest(&mut self, la: u32, fc: u8, level: u8, read: bool, areg: Option<u8>) {
        // "This bit is set if a match occurred in either (or both) of the
        // transparent translation registers. If the T bit is set, all
        // remaining MMUSR bits are undefined" — and for a level of one to
        // seven "this bit is set to zero".
        if level == 0 {
            if self.state.mmu.transparent(la, fc, !read).is_some() {
                self.state.mmu.mmusr = mmusr::T;
                return;
            }
            let mut out = 0u16;
            match self.state.mmu.lookup(la, fc) {
                None => out |= mmusr::I,
                Some(entry) => {
                    if entry.data & Entry::BERR != 0 {
                        out |= mmusr::B | mmusr::I;
                    }
                    if entry.data & Entry::WP != 0 {
                        out |= mmusr::W;
                    }
                    if entry.data & Entry::M != 0 {
                        out |= mmusr::M;
                    }
                }
            }
            self.state.mmu.mmusr = out;
            return;
        }
        let found = self.walk(la, fc, !read, level);
        self.state.mmu.mmusr = found.mmusr();
        // A table search creates an ATC entry whatever it found (Figure
        // 9-27), and `PTEST`'s operation is "logical address status → MMU
        // status register; entry → ATC" (M68000PRM §6). A search the level
        // field cut short reached no descriptor worth caching.
        if !found.capped {
            self.state.mmu.install(found.entry);
        }
        if let Some(reg) = areg {
            // "Return the address of the last descriptor searched in the
            // address register specified in the register field."
            self.set_register(8 + u32::from(reg & 7), found.last_descriptor);
        }
    }

    // ------------------------------------------------------------------
    // The floating-point coprocessor
    // ------------------------------------------------------------------

    /// The line-F exception, which is what a main processor takes when no
    /// coprocessor answers (MC68020UM §7.5.2) and therefore what this core
    /// takes for a command word it does not implement.
    fn fp_line_f(&self) -> Trap {
        Trap::at(vector::LINE_F, self.pc0)
    }

    /// What this unit does with an operation and a source format.
    ///
    /// The two parts answer differently, and the difference is the whole of
    /// the 68040's floating-point story: a 6888x implements everything and
    /// this core computes it, while a 68040 implements ten operations in
    /// hardware and traps for the rest so software can emulate them
    /// (M68040UM §9.6, Table 9-10).
    fn fp_availability(&self, op: FpOp, forced: Forced, fmt: Option<Fmt>) -> Availability {
        let packed = matches!(fmt, Some(Fmt::Packed));
        if self.cfg.fpu.is_onchip_040() {
            // "An unsupported data type exception occurs when ... either the
            // source or destination data format is packed decimal real"
            // (§9.6.2), and it takes precedence over nothing — the
            // unimplemented *instruction* exception does, for the
            // instructions that have one.
            if !fpu::implemented_040(op) {
                return Availability::Unimplemented;
            }
            if packed {
                return Availability::UnsupportedType;
            }
            return Availability::Hardware;
        }
        // The 68040 added `FSxxx`/`FDxxx` forms that force a rounding
        // precision; a 6888x has none, and the encoding is not a command
        // word it recognises.
        if forced != Forced::Control || !fpu::implemented(op) || packed {
            return Availability::LineF;
        }
        Availability::Hardware
    }

    /// Start a floating-point operation: the instruction address a trap
    /// handler reads, and a clean exception byte.
    ///
    /// "The 32-bit floating-point instruction address (FPIAR) register is
    /// loaded with the logical address of an instruction before the
    /// instruction is executed (unless all arithmetic exceptions are
    /// disabled)" (M68881UM §2.4). The parenthesis is honoured: with no trap
    /// enabled the register keeps whatever it held, which is what a handler
    /// that enables a trap and then reads it expects.
    fn fp_begin(&mut self) {
        if self.state.fpu.fpcr & 0x0000_ff00 != 0 {
            self.state.fpu.fpiar = self.pc0;
        }
        // "This byte is cleared by the FPCP at the start of most operations"
        // (§2.3.3).
        self.state.fpu.clear_exceptions();
    }

    /// Finish one: the condition codes, the exception byte, the accrued byte,
    /// the destination, and the trap if one is enabled.
    ///
    /// The destination register is withheld only for the three exceptions
    /// whose trap-enabled paragraph says so — "the destination floating-point
    /// data register is not modified" for `SNAN` (§6.1.2), `OPERR` (§6.1.3)
    /// and `DZ` (§6.1.6). An enabled `OVFL`, `UNFL` or `INEX` stores "the
    /// same as the result stored when the trap is disabled" (§6.1.4, §6.1.5,
    /// §6.1.7).
    fn fp_complete(&mut self, out: FpResult, dst: u8) -> Result<(), Trap> {
        let pc0 = self.pc0;
        let f = &mut self.state.fpu;
        f.raise(out.exc);
        if out.condition {
            f.set_condition(out.value);
        }
        f.accrue();
        let withheld = u32::from(fpbits::SNAN | fpbits::OPERR | fpbits::DZ);
        let blocked = f.fpsr & f.fpcr & withheld & 0x0000_ff00 != 0;
        if out.store && !blocked {
            f.fp[(dst & 7) as usize] = out.value;
        }
        f.null = false;
        let trap = f.pending_trap();
        self.settle()?;
        match trap {
            // A 68881 reports this as a *pre-instruction* exception on the
            // next floating-point instruction, because it runs concurrently
            // with the main processor and has not finished when the main
            // processor moves on (§6.1.3). Nothing runs concurrently here, so
            // it is reported as a post-instruction exception on the
            // instruction that caused it: the same vector, the same `FPIAR`,
            // the same `FPSR`, and a stacked program counter one instruction
            // earlier than hardware's. `docs/cpu/m68k.md` records it.
            Some(vector) => {
                let pc = self.state.pc;
                Err(Trap::six(vector, pc, pc0))
            }
            None => Ok(()),
        }
    }

    /// The general instruction class: every arithmetic and transcendental
    /// operation, `FMOVE` both ways, `FMOVECR` and the two `FMOVEM`s, told
    /// apart by the command word (M68881UM §4; `isa::fp`).
    fn op_fpgen(&mut self) -> Result<(), Trap> {
        let command = self.ext(0)?;
        let Some(class) = fp::decode(command) else {
            return Err(self.fp_line_f());
        };
        match class {
            fp::Class::Control { to_fpu, regs } => self.op_fmove_control(to_fpu, regs),
            fp::Class::MoveM { to_fpu, mode, list } => self.op_fmovem(to_fpu, mode, list),
            fp::Class::MoveCr { offset, dst } => {
                // M68040UM Table 9-10 lists `FMOVECR` among the monadic
                // operations the 68040 does not implement: the constant ROM
                // belongs to the software package, and the handler reads the
                // offset out of `CMDREG1B` for itself. There is no source
                // operand to put in `ETEMP`, and no effective address.
                if self.cfg.fpu.is_onchip_040() {
                    return self.fp_trap_040(
                        Availability::Unimplemented,
                        command,
                        FpOp::Move,
                        F80::ZERO,
                        dst,
                        0,
                    );
                }
                self.op_fmovecr(offset, dst)
            }
            fp::Class::Store {
                fmt, src, k, k_reg, ..
            } => self.op_fstore(fmt, src, k, k_reg, command),
            fp::Class::RegOp {
                src,
                dst,
                op,
                forced,
                cos,
            } => {
                let raw = self.state.fpu.fp[(src & 7) as usize];
                match self.fp_availability(op, forced, None) {
                    Availability::LineF => Err(self.fp_line_f()),
                    Availability::Hardware => {
                        self.fp_begin();
                        let value = fpu::canonical(raw);
                        self.fp_operate(op, value, Flags::NONE, dst, cos, forced)
                    }
                    // No effective address: "the effective address field
                    // contains the calculated effective address determined
                    // by the effective address field of the unimplemented
                    // instruction" (M68040UM §8.4.6.2, **CU**), and a
                    // register-to-register instruction has none.
                    // Raw, not canonicalised: "a denormalized or
                    // unnormalized extended-precision source or destination
                    // operand is copied directly **without modification** to
                    // ETEMP or FPTEMP" (M68040UM §9.6.2), because "the
                    // floating-point instruction emulation routine must
                    // detect the unsupported data type" (§9.6.1) and cannot
                    // if the unit has already normalized it away.
                    outcome => self.fp_trap_040(outcome, command, op, raw, dst, 0),
                }
            }
            fp::Class::MemOp {
                fmt,
                dst,
                op,
                forced,
                cos,
            } => {
                let outcome = self.fp_availability(op, forced, Some(fmt));
                if outcome == Availability::LineF {
                    return Err(self.fp_line_f());
                }
                // "Next, the instruction is partially decoded to allow
                // fetching of the memory source operand ... the fetched
                // source operand is passed to the FPU, which converts the
                // operand to extended precision and saves the intermediate
                // result" (§9.6.1). So the effective address is calculated
                // and the operand is read even when the instruction traps,
                // and a postincrement has happened by the time it does.
                self.fp_begin();
                let (value, flags) = self.fp_load(fmt)?;
                if outcome == Availability::Hardware {
                    return self.fp_operate(op, value, flags, dst, cos, forced);
                }
                let ea = self.fp_last_address;
                self.fp_trap_040(outcome, command, op, value, dst, ea)
            }
        }
    }

    /// Take one of the 68040's two non-arithmetic floating-point exceptions.
    ///
    /// Both leave the same thing behind for the handler's `FSAVE`: the
    /// instruction's command word and both operands, converted to extended
    /// precision (M68040UM Table 9-16). They differ in the vector and in the
    /// frame the *integer* unit stacks.
    ///
    /// - **Unimplemented instruction**, vector 11: "the processor creates a
    ///   format $2 stack frame and saves the vector offset, PC, internal copy
    ///   of the SR, and calculated effective address ... The saved PC value
    ///   is the logical address of the instruction that **follows** the
    ///   unimplemented floating-point instruction" (§9.6.1). That last part
    ///   is what lets an emulation handler `RTE` straight back into the
    ///   program once it has produced the result, and it is what separates
    ///   this from an F-line *illegal* instruction, which shares the vector
    ///   and stacks a format `$0` frame.
    /// - **Unsupported data type**, vector 55: "a format $0 (for the
    ///   pre-instruction exception) or format $3 (for the post-instruction
    ///   exception) stack frame is saved" (§9.6.2). Opclass 000 and 010 are
    ///   pre-instruction; only opclass 011, `FMOVE` out, is post-instruction,
    ///   and that one is raised from `op_fstore`.
    fn fp_trap_040(
        &mut self,
        outcome: Availability,
        command: u16,
        op: FpOp,
        src: F80,
        dst: u8,
        ea: u32,
    ) -> Result<(), Trap> {
        let dest = self.state.fpu.fp[(dst & 7) as usize];
        self.state.fpu.pending_040 = Some(fpu::State040 {
            command,
            etemp: src,
            stag: fpu::data_tag(src),
            // "Destination operand, if any, is converted to extended
            // precision" — there is one only for a dyadic operation.
            fptemp: if op.is_dyadic() { dest } else { F80::ZERO },
            dtag: if op.is_dyadic() {
                fpu::data_tag(dest)
            } else {
                0
            },
            post_instruction: false,
        });
        self.state.fpu.null = false;
        self.settle()?;
        let pc = self.state.pc;
        match outcome {
            Availability::Unimplemented => Err(Trap::six(vector::LINE_F, pc, ea)),
            _ => Err(Trap::raised(vector::FP_UNSUPPORTED_TYPE, self.pc0)),
        }
    }

    /// Compute one operation and finish it.
    fn fp_operate(
        &mut self,
        op: FpOp,
        src: F80,
        load_flags: Flags,
        dst: u8,
        cos: u8,
        forced: Forced,
    ) -> Result<(), Trap> {
        let dst = dst & 7;
        let env = self.state.fpu.env();
        // "FSADD and FDADD specify single- and double-precision rounding
        // regardless of the precision specified in the FPCR PREC bits"
        // (M68040UM §9.4.2). Like `PREC` itself, the forced precision
        // shortens the exponent range as well as the significand, because
        // the point of it is to "produce the same results as any other
        // device that conforms to the IEEE 754 standard but does not support
        // extended precision" (§9.4).
        let spec = match forced {
            Forced::Control => self.state.fpu.spec(),
            Forced::Single => Spec::interchange(24, 127),
            Forced::Double => Spec::interchange(53, 1023),
        };
        let dest = fpu::canonical(self.state.fpu.fp[dst as usize]);
        let snan = fpu::is_snan(src) || (op.is_dyadic() && fpu::is_snan(dest));

        let computed = fpu::operate(op, dest, src, spec, env);
        let mut out = FpResult {
            value: computed.value,
            store: computed.store && op.writes_destination(),
            condition: true,
            exc: computed.exc,
        };
        out.exc |= fpu::exceptions_from(load_flags, snan);
        if out.store && fpu::is_tiny(out.value, computed.tininess) {
            // `src/float` reports underflow only for a result that is both
            // tiny *and* inexact, which is IEEE's rule for the flag; the
            // 68881's exception bit is tininess alone and the AND with
            // `INEX2` happens on the way into the accrued byte (M68881UM
            // §6.1.5's note).
            out.exc |= fpbits::UNFL;
        }
        if snan {
            // "the SNAN is converted to a non-signaling NAN (by setting the
            // SNAN bit in the operand to a one), and the operation continues"
            // (§4.5.4.2) — which `fpu::operate` has already done.
            out.exc |= fpbits::SNAN;
        }
        if let Some((negative, magnitude)) = computed.quotient {
            self.state.fpu.set_quotient(negative, magnitude);
        }
        if let Some(cosine) = computed.second
            && out.store
        {
            // `FSINCOS` writes the cosine first, so "if FPc and FPs specify
            // the same floating-point data register, the sine result is
            // stored in the register and the cosine result is discarded"
            // (M68881UM §4, *FSINCOS*). The condition codes come from the
            // sine, which `fp_complete` sets.
            self.state.fpu.fp[(cos & 7) as usize] = cosine;
        }
        self.fp_complete(out, dst)
    }

    /// `FMOVECR`: one of the constants in the coprocessor's on-chip ROM.
    fn op_fmovecr(&mut self, offset: u8, dst: u8) -> Result<(), Trap> {
        self.fp_begin();
        let spec = self.state.fpu.spec();
        let env = self.state.fpu.env();
        let (value, flags) = x87::round_to(fpu::constant(offset), spec, env);
        let out = FpResult::stored(value, flags, spec);
        self.fp_complete(out, dst)
    }

    /// `FMOVE FPm,<ea>`: one register out, converted to `fmt`.
    ///
    /// "Condition Codes: Not affected" (M68881UM §4, *FMOVE*,
    /// register-to-memory), which §2.3.1 states as a rule: the register-to-
    /// memory `FMOVE`, `FMOVEM` and the control-register moves leave the
    /// `FPCC` alone.
    fn op_fstore(
        &mut self,
        fmt: Fmt,
        src: u8,
        k: i8,
        k_reg: Option<u8>,
        command: u16,
    ) -> Result<(), Trap> {
        if matches!(fmt, Fmt::Packed) {
            let _ = (k, k_reg);
            if !self.cfg.fpu.is_onchip_040() {
                // Packed decimal is not implemented; see `fpu.rs`.
                return Err(self.fp_line_f());
            }
            // "When an unsupported data type is detected for opclass 011
            // (register-to-memory) instructions, a post-instruction
            // exception is generated immediately. A format ... $3 (for the
            // post-instruction exception) stack frame is saved, and vector
            // number 55 is fetched" (M68040UM §9.6.2). The effective address
            // is calculated first, so the handler knows where to put the
            // digits it produces, and `T` is set in the state frame because
            // "only an opclass 3 instruction can indicate a post-instruction
            // exception" (§9.7, **T**).
            let value = self.state.fpu.fp[(src & 7) as usize];
            let Loc::Mem(addr) = self.fp_address(fmt.bytes())? else {
                return Err(self.fp_line_f());
            };
            self.state.fpu.pending_040 = Some(fpu::State040 {
                command,
                etemp: value,
                stag: fpu::data_tag(value),
                fptemp: F80::ZERO,
                dtag: 0,
                post_instruction: true,
            });
            self.state.fpu.null = false;
            self.settle()?;
            let pc = self.state.pc;
            return Err(Trap::post_instruction(
                vector::FP_UNSUPPORTED_TYPE,
                pc,
                addr,
            ));
        }
        self.fp_begin();
        let env = self.state.fpu.env();
        let value = fpu::canonical(self.state.fpu.fp[(src & 7) as usize]);
        let (raw, words, flags) = fpu::narrow(fmt, value, env);
        let snan = fpu::is_snan(value);
        let mut exc = fpu::exceptions_from(flags, snan);
        if snan {
            exc |= fpbits::SNAN;
        }
        match fmt {
            // "<fmt> is B, W, or L: OPERR — set if the source operand is
            // infinity, or if the destination size is exceeded after
            // conversion and rounding; OVFL cleared, UNFL cleared."
            Fmt::Byte | Fmt::Word | Fmt::Long => exc &= !(fpbits::OVFL | fpbits::UNFL),
            // "<fmt> is S, D, or X: OPERR cleared; OVFL and UNFL refer to
            // 6.1.4 and 6.1.5." Underflow is measured against the
            // *destination* format, not the rounding precision.
            _ => {
                exc &= !fpbits::OPERR;
                let target = match fmt {
                    Fmt::Single => Spec::interchange(24, 127),
                    Fmt::Double => Spec::interchange(53, 1023),
                    _ => F80::SPEC,
                };
                if fpu::is_tiny(value, target) {
                    exc |= fpbits::UNFL;
                }
            }
        }
        self.fp_store(fmt, raw, words)?;
        let out = FpResult {
            value,
            store: false,
            condition: false,
            exc,
        };
        self.fp_complete(out, 0)
    }

    /// `FMOVE`/`FMOVEM` for `FPCR`, `FPSR` and `FPIAR`.
    ///
    /// "Since the FPCP FMOVE to/from the FPCR, FPSR, or FPIAR and FMOVEM
    /// instructions cannot generate floating-point exceptions, these
    /// instructions do not modify the FPIAR" (§2.4), do not clear the
    /// exception byte (§2.3.3) and do not touch the condition codes (§2.3.1).
    fn op_fmove_control(&mut self, to_fpu: bool, regs: u8) -> Result<(), Trap> {
        const SELECTED: [u8; 3] = [fp::CTRL_FPCR, fp::CTRL_FPSR, fp::CTRL_FPIAR];
        let count = SELECTED.iter().filter(|bit| regs & *bit != 0).count();
        let Some((mode, reg)) = ea_of(Arg::Ea, self.opcode) else {
            return Err(self.fp_line_f());
        };
        let reg = reg as usize;
        // One register may go to or from a data register, and `FPIAR` may use
        // an address register because it holds an address. More than one needs
        // memory, since they do not all fit (M68000PRM §5, *FMOVE* to and from
        // the control registers).
        if matches!(mode, Mode::DataReg | Mode::AddrReg) {
            if count != 1 || (mode == Mode::AddrReg && regs != fp::CTRL_FPIAR) {
                return Err(self.fp_line_f());
            }
            let index = if mode == Mode::DataReg { reg } else { 8 + reg } as u32;
            self.facts.ea(timing::DN);
            if to_fpu {
                let value = self.register(index);
                self.fp_set_control(regs, value);
            } else {
                let value = self.fp_get_control(regs);
                self.set_register(index, value);
            }
            return self.settle();
        }
        if mode == Mode::Imm {
            if !to_fpu || count != 1 {
                return Err(self.fp_line_f());
            }
            self.facts.ea(timing::IMM_L);
            let hi = self.ext(0)?;
            let lo = self.ext(0)?;
            self.fp_set_control(regs, (u32::from(hi) << 16) | u32::from(lo));
            return self.settle();
        }
        let Loc::Mem(addr) = self.fp_address(4 * count as u32)? else {
            return Err(self.fp_line_f());
        };
        let mut at = addr;
        for bit in SELECTED {
            if regs & bit == 0 {
                continue;
            }
            if to_fpu {
                let value = self.read_long(at)?;
                self.fp_set_control(bit, value);
            } else {
                let value = self.fp_get_control(bit);
                self.write_long(at, value)?;
            }
            at = at.wrapping_add(4);
        }
        self.settle()
    }

    /// One control register's value.
    fn fp_get_control(&self, which: u8) -> u32 {
        match which {
            fp::CTRL_FPCR => self.state.fpu.fpcr,
            fp::CTRL_FPSR => self.state.fpu.fpsr,
            _ => self.state.fpu.fpiar,
        }
    }

    /// Load one control register.
    fn fp_set_control(&mut self, which: u8, value: u32) {
        match which {
            fp::CTRL_FPCR => self.state.fpu.fpcr = value & fpbits::FPCR_IMPLEMENTED,
            fp::CTRL_FPSR => self.state.fpu.fpsr = value & fpbits::FPSR_IMPLEMENTED,
            _ => self.state.fpu.fpiar = value,
        }
        self.state.fpu.null = false;
    }

    /// `FMOVEM` of the eight data registers.
    ///
    /// The mask's bit numbering is reversed between the two list formats —
    /// bit 7 is `FP7` in predecrement order and `FP0` in postincrement or
    /// control order (M68000PRM §5, *FMOVEM*, *Register List field*) — and
    /// the transfer runs from bit 0 in both, exactly as an integer `MOVEM`
    /// does. The two reversals cancel: whichever form was used, the selected
    /// registers appear in memory in decreasing register order, `FP7` at the
    /// lowest address.
    fn op_fmovem(&mut self, to_fpu: bool, mode: ListMode, list: u8) -> Result<(), Trap> {
        let Some((ea, _)) = ea_of(Arg::Ea, self.opcode) else {
            return Err(self.fp_line_f());
        };
        // "Only control addressing modes or the postincrement addressing
        // mode" into the unit, "only control alterable addressing modes or
        // the predecrement addressing mode" out of it.
        let allowed = if to_fpu {
            EaSet::MOVEM_TO_REG
        } else {
            EaSet::MOVEM_TO_MEM
        };
        if !allowed.contains(ea) {
            return Err(self.fp_line_f());
        }
        let mask = if mode.dynamic() {
            (self.state.d[(list & 7) as usize] & 0xff) as u8
        } else {
            list
        };
        let count = mask.count_ones();
        let Loc::Mem(addr) = self.fp_address(12 * count)? else {
            return Err(self.fp_line_f());
        };
        self.facts.registers = mask.count_ones();
        let mut at = addr;
        for index in (0..8u8).rev() {
            let bit = if mode.predecrement_order() {
                index
            } else {
                7 - index
            };
            if mask & (1 << bit) == 0 {
                continue;
            }
            if to_fpu {
                let hi = self.read_long(at)?;
                let mid = self.read_long(at.wrapping_add(4))?;
                let lo = self.read_long(at.wrapping_add(8))?;
                self.state.fpu.fp[index as usize] = fpu::read_extended(hi, mid, lo);
            } else {
                let words = fpu::write_extended(self.state.fpu.fp[index as usize]);
                self.write_long(at, words[0])?;
                self.write_long(at.wrapping_add(4), words[1])?;
                self.write_long(at.wrapping_add(8), words[2])?;
            }
            at = at.wrapping_add(12);
        }
        self.state.fpu.null = false;
        self.settle()
    }

    /// The effective address a floating-point operand of `bytes` bytes uses.
    ///
    /// Separate from [`Exec::resolve_ea`] because the auto-adjusting modes
    /// step by the *operand's* size, and a floating-point operand can be
    /// twelve bytes, which `Size` cannot name.
    ///
    /// Both walking modes are accepted in both directions: `FMOVE` reaches
    /// `(An)+` and `-(An)` either way round (M68881UM §4, *FMOVE*), and the
    /// instructions that do restrict them — `FMOVEM`, `FSAVE`, `FRESTORE` —
    /// check their own mode sets before they get here.
    fn fp_address(&mut self, bytes: u32) -> Result<Loc, Trap> {
        let Some((mode, reg)) = ea_of(Arg::Ea, self.opcode) else {
            return Err(self.fp_line_f());
        };
        let reg = reg as usize;
        match mode {
            Mode::PostInc => {
                let addr = self.state.a[reg];
                self.state.a[reg] = addr.wrapping_add(bytes);
                self.facts.ea(3);
                Ok(Loc::Mem(addr))
            }
            Mode::PreDec => {
                let addr = self.state.a[reg].wrapping_sub(bytes);
                self.state.a[reg] = addr;
                self.facts.ea(4);
                Ok(Loc::Mem(addr))
            }
            Mode::DataReg | Mode::AddrReg | Mode::Imm => Err(self.fp_line_f()),
            // Every other mode computes an address without a size, so the
            // ordinary resolver gives the right answer and reads the right
            // number of extension words.
            _ => self.resolve_ea(Arg::Ea, Size::Long, ExtraCycles::Operand),
        }
    }

    /// Read a floating-point source operand of `fmt`.
    fn fp_load(&mut self, fmt: Fmt) -> Result<(F80, Flags), Trap> {
        let env = self.state.fpu.env();
        let Some((mode, reg)) = ea_of(Arg::Ea, self.opcode) else {
            return Err(self.fp_line_f());
        };
        let reg = reg as usize;
        match mode {
            Mode::DataReg => {
                // "Only if <fmt> is byte, word, long, or single" — the
                // footnote under every operand table in M68881UM §4.
                if !fmt.fits_in_a_register() {
                    return Err(self.fp_line_f());
                }
                self.facts.ea(timing::DN);
                Ok(fpu::widen(
                    fmt,
                    u64::from(self.state.d[reg]),
                    F80::ZERO,
                    env,
                ))
            }
            Mode::AddrReg => Err(self.fp_line_f()),
            Mode::Imm => {
                // The operand follows the command word in the instruction
                // stream, a word at a time; a byte one occupies a whole word.
                let words = (fmt.bytes().max(2) / 2) as usize;
                self.facts.ea(if words > 2 {
                    timing::IMM_L
                } else {
                    timing::IMM_W
                });
                let mut buffer = [0u16; 6];
                for slot in buffer.iter_mut().take(words) {
                    *slot = self.ext(0)?;
                }
                let long = |a: u16, b: u16| (u32::from(a) << 16) | u32::from(b);
                let raw = match fmt {
                    Fmt::Byte => u64::from(buffer[0] & 0xff),
                    Fmt::Word => u64::from(buffer[0]),
                    Fmt::Long | Fmt::Single => u64::from(long(buffer[0], buffer[1])),
                    Fmt::Double => {
                        (u64::from(long(buffer[0], buffer[1])) << 32)
                            | u64::from(long(buffer[2], buffer[3]))
                    }
                    Fmt::Extended | Fmt::Packed => 0,
                };
                let extended = fpu::read_extended(
                    long(buffer[0], buffer[1]),
                    long(buffer[2], buffer[3]),
                    long(buffer[4], buffer[5]),
                );
                Ok(fpu::widen(fmt, raw, extended, env))
            }
            _ => {
                let Loc::Mem(addr) = self.fp_address(fmt.bytes())? else {
                    return Err(self.fp_line_f());
                };
                // Kept for the 68040's format $2 frame, whose address field
                // is "the calculated effective address determined by the
                // effective address field of the unimplemented instruction"
                // (M68040UM §9.6.1).
                self.fp_last_address = addr;
                let (raw, extended) = match fmt {
                    Fmt::Byte => (u64::from(self.read_byte(addr)?), F80::ZERO),
                    Fmt::Word => (u64::from(self.read_word(addr)?), F80::ZERO),
                    Fmt::Long | Fmt::Single => (u64::from(self.read_long(addr)?), F80::ZERO),
                    Fmt::Double => {
                        let hi = self.read_long(addr)?;
                        let lo = self.read_long(addr.wrapping_add(4))?;
                        ((u64::from(hi) << 32) | u64::from(lo), F80::ZERO)
                    }
                    Fmt::Extended | Fmt::Packed => {
                        let hi = self.read_long(addr)?;
                        let mid = self.read_long(addr.wrapping_add(4))?;
                        let lo = self.read_long(addr.wrapping_add(8))?;
                        (0, fpu::read_extended(hi, mid, lo))
                    }
                };
                Ok(fpu::widen(fmt, raw, extended, env))
            }
        }
    }

    /// Write a floating-point destination operand of `fmt`.
    fn fp_store(&mut self, fmt: Fmt, raw: u64, words: [u32; 3]) -> Result<(), Trap> {
        let Some((mode, reg)) = ea_of(Arg::Ea, self.opcode) else {
            return Err(self.fp_line_f());
        };
        let reg = reg as usize;
        if mode == Mode::DataReg {
            if !fmt.fits_in_a_register() {
                return Err(self.fp_line_f());
            }
            self.facts.ea(timing::DN);
            let size = match fmt {
                Fmt::Byte => Size::Byte,
                Fmt::Word => Size::Word,
                _ => Size::Long,
            };
            self.state.d[reg] = merge(self.state.d[reg], raw as u32, size);
            return Ok(());
        }
        let Loc::Mem(addr) = self.fp_address(fmt.bytes())? else {
            return Err(self.fp_line_f());
        };
        match fmt {
            Fmt::Byte => self.write_byte(addr, raw as u8),
            Fmt::Word => self.write_word(addr, raw as u16),
            Fmt::Long | Fmt::Single => self.write_long(addr, raw as u32),
            Fmt::Double => {
                self.write_long(addr, (raw >> 32) as u32)?;
                self.write_long(addr.wrapping_add(4), raw as u32)
            }
            Fmt::Extended | Fmt::Packed => {
                self.write_long(addr, words[0])?;
                self.write_long(addr.wrapping_add(4), words[1])?;
                self.write_long(addr.wrapping_add(8), words[2])
            }
        }
    }

    // ------------------------------------------------------------------
    // The conditional instructions
    // ------------------------------------------------------------------

    /// Evaluate a predicate, raising `BSUN` if it is one that signals and the
    /// condition codes say unordered.
    ///
    /// Returns `Err` when the `BSUN` trap is enabled, which is a
    /// **pre-instruction** exception: the stacked program counter is the
    /// conditional instruction's own address, so an `RTE` that changes
    /// nothing runs into it again — which is exactly what the note under
    /// every conditional's page warns about (M68881UM §4, *FBcc*).
    fn fp_test(&mut self, pred: Pred) -> Result<bool, Trap> {
        let (n, z, _, nan) = self.state.fpu.condition();
        if nan && pred.signals_unordered() {
            self.state.fpu.raise(fpbits::BSUN);
            self.state.fpu.accrue();
            self.state.fpu.null = false;
            if self.state.fpu.fpcr & u32::from(fpbits::BSUN) != 0 {
                let pc0 = self.pc0;
                return Err(Trap::at(vector::FP_BSUN, pc0));
            }
        }
        Ok(pred.test(n, z, nan))
    }

    /// The predicate an extension word carries, or the line-F exception for
    /// one of the thirty-two encodings the manual does not define.
    fn fp_predicate(&mut self, word: u16) -> Result<Pred, Trap> {
        let pred = Pred((word & 0x3f) as u8);
        if pred.name().is_none() || word & 0xffc0 != 0 {
            return Err(self.fp_line_f());
        }
        Ok(pred)
    }

    /// `FBcc`: the predicate is in the opcode and the displacement follows.
    ///
    /// `FNOP` is this instruction with the predicate `F` and a zero
    /// displacement (M68881UM §4, *FNOP*), so it needs no code of its own.
    fn op_fbcc(&mut self) -> Result<(), Trap> {
        let pred = Pred((self.opcode & 0x3f) as u8);
        if pred.name().is_none() {
            return Err(self.fp_line_f());
        }
        // The displacement is measured from the word after the opcode.
        let base = self.state.pc.wrapping_add(2);
        let long = self.opcode & 0x40 != 0;
        let taken = self.fp_test(pred)?;
        if long {
            let hi = self.ext(0)?;
            let lo = self.queued()?;
            if taken {
                self.facts.taken = true;
                let offset = (u32::from(hi) << 16) | u32::from(lo);
                return self.refill(base.wrapping_add(offset), 0);
            }
            self.ext(0)?;
            return self.settle();
        }
        let word = self.queued()?;
        if taken {
            self.facts.taken = true;
            let offset = i32::from(word as i16) as u32;
            return self.refill(base.wrapping_add(offset), 0);
        }
        self.ext(0)?;
        self.settle()
    }

    /// `FDBcc`: the predicate is in the extension word, then a displacement.
    fn op_fdbcc(&mut self) -> Result<(), Trap> {
        let word = self.ext(0)?;
        let pred = self.fp_predicate(word)?;
        let base = self.state.pc.wrapping_add(2);
        let n = reg_lo(self.opcode);
        if self.fp_test(pred)? {
            self.ext(0)?;
            return self.settle();
        }
        let counter = (self.state.d[n] as u16).wrapping_sub(1);
        self.state.d[n] = merge(self.state.d[n], u32::from(counter), Size::Word);
        if counter == 0xffff {
            self.facts.expired = true;
            self.ext(0)?;
            return self.settle();
        }
        let word = self.queued()?;
        self.facts.taken = true;
        let target = base.wrapping_add(i32::from(word as i16) as u32);
        self.refill(target, 0)
    }

    /// `FScc`: a byte of all ones or all zeros.
    fn op_fscc(&mut self) -> Result<(), Trap> {
        let word = self.ext(0)?;
        let pred = self.fp_predicate(word)?;
        let set = self.fp_test(pred)?;
        let dst = self.resolve(Arg::Ea, Size::Byte)?;
        let value = if set { 0xff } else { 0x00 };
        if matches!(dst, Loc::D(_)) {
            self.write_loc(dst, Size::Byte, value)?;
            self.settle()
        } else {
            self.settle()?;
            self.write_back(dst, Size::Byte, value)
        }
    }

    /// `FTRAPcc`: the `TRAPcc` exception on a floating-point condition, with
    /// an optional operand nothing reads.
    fn op_ftrapcc(&mut self) -> Result<(), Trap> {
        let word = self.ext(0)?;
        let pred = self.fp_predicate(word)?;
        let pc0 = self.pc0;
        let taken = self.fp_test(pred)?;
        match self.opcode & 7 {
            2 => {
                self.ext(0)?;
            }
            3 => {
                self.ext(0)?;
                self.ext(0)?;
            }
            _ => {}
        }
        self.settle()?;
        if taken {
            let pc = self.state.pc;
            return Err(Trap::six(vector::TRAPV, pc, pc0));
        }
        Ok(())
    }

    // ------------------------------------------------------------------
    // Context switching
    // ------------------------------------------------------------------

    /// `FSAVE`: the coprocessor's internal state.
    ///
    /// Two frames are written. A unit that has not been touched since its
    /// reset writes the **null** frame, four bytes of zeros, which is what
    /// tells an operating system there is nothing to save (M68881UM §4,
    /// *FSAVE*). Anything else writes the **idle** frame, whose length is the
    /// coprocessor's — twenty-eight bytes on a 68881, sixty on a 68882 — with
    /// a format word carrying a version number and the length of what follows
    /// it.
    ///
    /// The body of an idle frame is "the user invisible portion of the
    /// machine", which on hardware is microcode state. This core has none, so
    /// it writes the one thing the frame is documented to carry that is
    /// architectural — the pending exception byte, which `FSAVE` then clears
    /// internally — and zeros for the rest. The version number is therefore
    /// **this core's own**, and `FRESTORE` refuses a frame written by
    /// anything else rather than reading somebody's microcode as its own.
    fn op_fsave(&mut self) -> Result<(), Trap> {
        let coprocessor = self.cfg.fpu;
        if self.state.fpu.null {
            let Loc::Mem(addr) = self.fp_address(4)? else {
                return Err(self.fp_line_f());
            };
            self.write_long(addr, 0)?;
            return self.settle();
        }
        if let Some(state) = self.state.fpu.pending_040 {
            return self.fsave_unimplemented_040(state);
        }
        let size = coprocessor.idle_frame();
        if size == 4 {
            // A 68040's idle frame is the format long word and nothing else
            // (M68040UM Figure 9-10(c)), so there is no body to write and
            // no pending exception to clear out of one: this core reports
            // every arithmetic exception at the instruction that caused it
            // and never leaves the unit busy.
            let Loc::Mem(addr) = self.fp_address(size)? else {
                return Err(self.fp_line_f());
            };
            let format = (u32::from(coprocessor.state_version()) << 24) | ((size - 4) << 16);
            self.write_long(addr, format)?;
            return self.settle();
        }
        let Loc::Mem(addr) = self.fp_address(size)? else {
            return Err(self.fp_line_f());
        };
        let format = (u32::from(coprocessor.state_version()) << 24) | ((size - 4) << 16);
        self.write_long(addr, format)?;
        let pending = self.state.fpu.fpsr & 0x0000_ff00;
        self.write_long(addr.wrapping_add(4), pending)?;
        for offset in (8..size).step_by(4) {
            self.write_long(addr.wrapping_add(offset), 0)?;
        }
        // "Any exceptions that were pending are saved in the frame and are
        // then cleared internally."
        self.state.fpu.clear_exceptions();
        self.settle()
    }

    /// `FRESTORE`: the other half.
    ///
    /// A null frame "is equivalent to a hardware reset of the FPCP"; a format
    /// word this core did not write is a format error, which is what the main
    /// processor does with one the coprocessor rejects (M68881UM §4,
    /// *FRESTORE*).
    fn op_frestore(&mut self) -> Result<(), Trap> {
        let coprocessor = self.cfg.fpu;
        let Loc::Mem(addr) = self.fp_address(4)? else {
            return Err(self.fp_line_f());
        };
        let format = self.read_long(addr)?;
        if format >> 16 == 0 {
            self.state.fpu = Fpu::RESET;
            return self.settle();
        }
        let version = (format >> 24) as u8;
        let size = ((format >> 16) & 0xff) + 4;
        if version != coprocessor.state_version() {
            self.settle()?;
            let pc = self.state.pc;
            return Err(Trap::raised(vector::FORMAT_ERROR, pc));
        }
        if coprocessor.is_onchip_040() {
            return self.frestore_040(addr, size);
        }
        if size != coprocessor.idle_frame() {
            self.settle()?;
            let pc = self.state.pc;
            return Err(Trap::raised(vector::FORMAT_ERROR, pc));
        }
        let pending = self.read_long(addr.wrapping_add(4))?;
        for offset in (8..size).step_by(4) {
            self.read_long(addr.wrapping_add(offset))?;
        }
        self.state.fpu.raise((pending & 0x0000_ff00) as u16);
        self.state.fpu.null = false;
        // `(An)+` stepped by four for the format word; the rest of the frame
        // follows it.
        if let Some((Mode::PostInc, reg)) = ea_of(Arg::Ea, self.opcode) {
            let reg = reg as usize;
            self.state.a[reg] = self.state.a[reg].wrapping_add(size - 4);
        }
        self.settle()
    }

    /// The MC68040's twenty-six-word unimplemented-instruction state frame
    /// (M68040UM Figure 9-10(d)).
    ///
    /// This is the frame the emulation handler reads, and the reason the
    /// unimplemented-instruction exception is worth taking faithfully: "the
    /// exception handler uses the information provided in the state frame to
    /// determine the instruction that it needs to emulate and the input
    /// operands to that instruction" (§9.6.1). Table 9-16 names the fields
    /// that matter — `CMDREG1B`, `ETEMP`, `STAG`, `FPTEMP`, `DTAG`, `E1`
    /// and `T` — and every other field belongs to the arithmetic exceptions
    /// this core reports at the instruction that caused them, so it is
    /// written as zero.
    ///
    /// `ETEMP` and `FPTEMP` are the extended format laid out as the figure
    /// draws it: sign in bit 31 of the first long word, the fifteen-bit
    /// exponent in bits 30–16, then the sixty-four-bit significand. That is
    /// the 96-bit extended memory format with its reserved word in the
    /// middle, which is what an operand in memory already looks like.
    fn fsave_unimplemented_040(&mut self, state: fpu::State040) -> Result<(), Trap> {
        const SIZE: u32 = 0x34;
        let Loc::Mem(addr) = self.fp_address(SIZE)? else {
            return Err(self.fp_line_f());
        };
        let extended = |v: F80| -> (u32, u32, u32) {
            (
                u32::from(v.sign_exp) << 16,
                (v.sig >> 32) as u32,
                v.sig as u32,
            )
        };
        let (fpts_fpte, fptm_hi, fptm_lo) = extended(state.fptemp);
        let (ets_ete, etm_hi, etm_lo) = extended(state.etemp);
        let words: [(u32, u32); 13] = [
            // +$00 version $41 in bits 31-24, the length of the body in
            // 23-16.
            (0x00, (0x41 << 24) | ((SIZE - 4) << 16)),
            (0x04, 0), // CMDREG3B, bits 26-16: an E3 exception only
            (0x08, 0), // reserved in this frame
            (0x0c, u32::from(state.stag & 7) << 29), // STAG in bits 31-29
            (0x10, u32::from(state.command) << 16), // CMDREG1B in 31-16
            (0x14, u32::from(state.dtag & 7) << 29), // DTAG in bits 31-29
            // +$18 E1 is bit 26, E3 bit 25, T bit 20. "E1 — Always 1" and
            // "T — Always 0" for an unimplemented instruction; `T` is 1 for
            // the post-instruction case (Table 9-16).
            (0x18, (1 << 26) | (u32::from(state.post_instruction) << 20)),
            (0x1c, fpts_fpte),
            (0x20, fptm_hi),
            (0x24, fptm_lo),
            (0x28, ets_ete),
            (0x2c, etm_hi),
            (0x30, etm_lo),
        ];
        for (offset, value) in words {
            self.write_long(addr.wrapping_add(offset), value)?;
        }
        // The state has been handed over; the unit is idle again.
        self.state.fpu.pending_040 = None;
        self.settle()
    }

    /// `FRESTORE` on a 68040 (M68040UM §9.7).
    ///
    /// Three frames exist here: the null frame, handled by the caller; the
    /// idle frame, four bytes; and the twenty-six-word unimplemented
    /// instruction frame, which an emulation handler pops after it has
    /// produced the result. The fifty-word busy frame is pipeline state this
    /// core never produces, so a frame claiming to be one is a format error
    /// rather than a guess.
    fn frestore_040(&mut self, addr: u32, size: u32) -> Result<(), Trap> {
        match size {
            4 => {}
            0x34 => {
                // Touch the whole frame, so an unreadable one faults before
                // anything is committed, and drop it: this core has nothing
                // to put back, and the handler has already done the work the
                // frame described.
                for offset in (4..size).step_by(4) {
                    self.read_long(addr.wrapping_add(offset))?;
                }
            }
            _ => {
                self.settle()?;
                let pc = self.state.pc;
                return Err(Trap::raised(vector::FORMAT_ERROR, pc));
            }
        }
        self.state.fpu.pending_040 = None;
        self.state.fpu.null = false;
        if let Some((Mode::PostInc, reg)) = ea_of(Arg::Ea, self.opcode) {
            let reg = reg as usize;
            self.state.a[reg] = self.state.a[reg].wrapping_add(size - 4);
        }
        self.settle()
    }

    /// `MOVES`: an operand in the address space `SFC` or `DFC` names
    /// (M68000PRM, *MOVES*).
    ///
    /// The internal time makes the totals the 68010's table gives, 18 to 28
    /// clocks by mode (MC68000UM Table 9-16).
    fn op_moves(&mut self, size: Size) -> Result<(), Trap> {
        let word = self.ext(0)?;
        let to_memory = word & 0x0800 != 0;
        self.facts.to_memory = to_memory;
        let index = u32::from(word >> 12);
        let loc = self.resolve(Arg::Ea, size)?;
        let extra = match ea_of(Arg::Ea, self.opcode) {
            Some((Mode::PostInc, _)) => 8,
            Some((Mode::Disp16 | Mode::AbsShort | Mode::AbsLong, _)) => 4,
            _ => 6,
        };
        self.internal(extra);
        let fc = if to_memory {
            self.state.dfc
        } else {
            self.state.sfc
        };
        if to_memory {
            // Read after the address is resolved, so `MOVES An,(An)+` stores
            // the incremented value, as the note in the manual says every
            // implementation does.
            let value = self.register(index);
            self.fc_override = Some(fc);
            let done = self.write_loc(loc, size, value);
            self.fc_override = None;
            done?;
        } else {
            self.fc_override = Some(fc);
            let read = self.read_loc(loc, size);
            self.fc_override = None;
            let value = read?;
            if index >= 8 {
                // An address register takes the operand sign-extended.
                let value = match size {
                    Size::Byte => i32::from(value as i8) as u32,
                    Size::Word => i32::from(value as i16) as u32,
                    Size::Long => value,
                };
                self.set_register(index, value);
            } else {
                let n = index as usize;
                self.state.d[n] = merge(self.state.d[n], value, size);
            }
        }
        self.settle()
    }

    /// `RTE` on a processor with format words: read the format, then do what
    /// it says (MC68000UM §6.4; MC68020UM §6.1.12, Figure 6-7).
    ///
    /// A format this model does not define is a format error, vector 14, and
    /// the frame is left where it was — "the processor creates a normal
    /// four-word ... stack frame below the frame that it was attempting to
    /// use", so a handler can inspect the bad one.
    fn op_rte_formatted(&mut self) -> Result<(), Trap> {
        let pc0 = self.pc0;
        let mut throwaways = 0u32;
        loop {
            let sp = self.state.a[7];
            let high = self.read_word(sp.wrapping_add(2))?;
            let sr = self.read_word(sp)?;
            let low = self.read_word(sp.wrapping_add(4))?;
            let format = self.read_word(sp.wrapping_add(6))? >> 12;
            let pc = (u32::from(high) << 16) | u32::from(low);
            if self.model.has_020() {
                self.table += timing::rte(format);
            }
            let size = match (format, self.model.has_020()) {
                (0x0, _) => 8,
                (0x8, false) => return self.rte_long_010(sp, sr),
                (0x1, true) => {
                    // A throwaway frame: take its status word, which switches
                    // stacks, and start again with the frame on top of the
                    // new one. The cap is this core's, not the processor's:
                    // a chain of throwaway frames that long is a corrupt
                    // stack, and hardware would walk it for as long as memory
                    // lasts while holding the scheduler inside one step.
                    self.state.a[7] = sp.wrapping_add(8);
                    self.state.set_sr(sr);
                    throwaways += 1;
                    if throwaways > MAX_THROWAWAY_FRAMES {
                        return Err(Trap::raised(vector::FORMAT_ERROR, pc0));
                    }
                    continue;
                }
                (0x2, true) => 12,
                // The 68040's floating-point post-instruction frame, which
                // carries an effective address where format $2 carries an
                // instruction address; `RTE` pops both the same way
                // (M68040UM §8.4.4).
                (0x3, _) if self.model.has_040() => 12,
                (0x7, _) if self.model.has_040() => return self.rte_access_040(sp, sr, pc),
                // Formats $A and $B are the 68020's and the 68030's; a 68040
                // does not recognise either (M68040UM §8.4).
                (0xa, true) if !self.model.has_040() => 32,
                (0xb, true) if !self.model.has_040() => return self.rte_long_020(sp, sr, pc),
                // Format $9 is the coprocessor mid-instruction frame. With no
                // coprocessor on the bus there is nothing to resume the
                // instruction with, so it is treated as a format this
                // processor cannot use.
                _ => return Err(Trap::raised(vector::FORMAT_ERROR, pc0)),
            };
            self.state.a[7] = sp.wrapping_add(size);
            self.state.set_sr(sr);
            return self.refill(pc, 0);
        }
    }

    /// `RTE` from a 68010 format $8 frame (MC68000UM §6.4).
    fn rte_long_010(&mut self, sp: u32, sr: u16) -> Result<(), Trap> {
        let pc0 = self.pc0;
        // Validity first: the version number this processor wrote, else a
        // format error with the stack untouched.
        let version = self.read_word(sp.wrapping_add(0x1a))?;
        if (version >> 10) & 0xf != VERSION_68010 {
            return Err(Trap::raised(vector::FORMAT_ERROR, pc0));
        }
        // Then accessibility: the last word of the frame.
        self.read_word(sp.wrapping_add(0x38))?;
        let ssw = self.read_word(sp.wrapping_add(0x08))?;
        let addr = self.read_long(sp.wrapping_add(0x0a))?;
        let dib = self.read_word(sp.wrapping_add(0x14))?;
        let restart = self.read_long(sp.wrapping_add(0x1c))?;
        let undo = self.read_undo(sp.wrapping_add(0x20), 4)?;
        self.state.a[7] = sp.wrapping_add(58);
        self.state.set_sr(sr);
        self.apply_undo(&undo);
        // RR set: software ran the cycle (MC68000UM Figure 6-9).
        if ssw & 0x8000 != 0 {
            let byte = ssw & 0x0200 != 0;
            let read = ssw & 0x0100 != 0;
            // A read-modify-write the handler finished has finished the
            // instruction, and execution resumes after it.
            if ssw & 0x0800 != 0 {
                let next = restart.wrapping_add(self.instruction_length(restart));
                return self.refill(next, 0);
            }
            let data = if byte {
                if ssw & 0x0400 != 0 {
                    u32::from(dib >> 8)
                } else {
                    u32::from(dib & 0xff)
                }
            } else {
                u32::from(dib)
            };
            self.state.replay = Some(Replay {
                addr,
                read,
                width: if byte { 1 } else { 2 },
                data,
            });
        }
        self.refill(restart, 0)
    }

    /// `RTE` from a 68020 format $B frame (MC68020UM §6.1.12, §6.2.3).
    fn rte_long_020(&mut self, sp: u32, sr: u16, pc: u32) -> Result<(), Trap> {
        let pc0 = self.pc0;
        let version = self.read_word(sp.wrapping_add(0x36))?;
        if version >> 12 != VERSION_68020 {
            return Err(Trap::raised(vector::FORMAT_ERROR, pc0));
        }
        self.read_word(sp.wrapping_add(0x5a))?;
        let data_fault = self.read_word(sp.wrapping_add(0x08))? & 1 != 0;
        let ssw = self.read_word(sp.wrapping_add(0x0a))?;
        let addr = self.read_long(sp.wrapping_add(0x10))?;
        let dib = self.read_long(sp.wrapping_add(0x2c))?;
        let undo = self.read_undo(sp.wrapping_add(0x38), 6)?;
        self.state.a[7] = sp.wrapping_add(92);
        self.state.set_sr(sr);
        self.apply_undo(&undo);
        // DF cleared on a data fault: software did the cycle.
        if data_fault && ssw & SSW_DF == 0 {
            if ssw & 0x0080 != 0 {
                // RM: the whole CAS, CAS2 or TAS was emulated, condition
                // codes and all (MC68020UM §6.2.2).
                let next = pc.wrapping_add(self.instruction_length(pc));
                return self.refill(next, 0);
            }
            let width = if (ssw >> 4) & 3 == 0b01 { 1 } else { 2 };
            let data = if width == 1 { dib & 0xff } else { dib & 0xffff };
            self.state.replay = Some(Replay {
                addr,
                read: ssw & 0x0040 != 0,
                width,
                data,
            });
        }
        self.refill(pc, 0)
    }

    /// `RTE` from a 68040 format `$7` access error frame (M68040UM §8.4.6.7).
    ///
    /// "The processor restores the SR and PC values from the stack and checks
    /// the four continuation status bits in the SSW on the stack. If these
    /// bits are not set, the processor increments the active supervisor stack
    /// pointer by 30 words and resumes normal instruction execution."
    ///
    /// This core never sets a continuation bit, and a handler that sets one
    /// is documented to leave the processor undefined ("If the access error
    /// exception handler sets multiple bits, operation of the RTE instruction
    /// is undefined") — so the continuation cases are a format error here
    /// rather than a guess. What it does do is put back the address
    /// registers the faulted instruction had stepped, from the write-back
    /// fields the invalid status bytes told the handler to ignore, and
    /// restart the instruction the `PC` field names.
    ///
    /// The whole frame is touched before the stack pointer moves, so a frame
    /// that is not fully readable faults with the stack intact — "if a format
    /// error or access fault exception occurs during the frame validation
    /// sequence of the RTE instruction ... the illegal stack frame remains
    /// intact".
    fn rte_access_040(&mut self, sp: u32, sr: u16, pc: u32) -> Result<(), Trap> {
        let pc0 = self.pc0;
        let ssw = self.read_word(sp.wrapping_add(0x0c))?;
        // The last word of the frame, for accessibility.
        self.read_word(sp.wrapping_add(0x3a))?;
        if ssw & SSW_040_CONTINUE != 0 {
            return Err(Trap::raised(vector::FORMAT_ERROR, pc0));
        }
        let undo = self.read_undo(sp.wrapping_add(0x18), 6)?;
        self.state.a[7] = sp.wrapping_add(60);
        self.state.set_sr(sr);
        self.apply_undo(&undo);
        self.refill(pc, 0)
    }

    /// Read `slots` undo entries a long frame carries.
    fn read_undo(&mut self, at: u32, slots: u32) -> Result<UndoList, Trap> {
        let mut out = [(0u8, 0u32); 10];
        let mut n = 0;
        for i in 0..slots {
            let base = at.wrapping_add(6 * i);
            let code = self.read_word(base)?;
            let value = self.read_long(base.wrapping_add(2))?;
            // Anything but a register code this core writes is ignored rather
            // than trusted: a handler is entitled to scribble on words the
            // manual calls internal.
            if code < 10 && n < out.len() {
                out[n] = (code as u8, value);
                n += 1;
            }
        }
        Ok((out, n))
    }

    /// Put back the registers a faulted instruction had already moved.
    fn apply_undo(&mut self, undo: &UndoList) {
        let (entries, n) = undo;
        for &(code, value) in &entries[..*n] {
            match code {
                0..=6 => self.state.a[code as usize] = value,
                7 => self.state.set_sp(Bank::User, value),
                8 => self.state.set_sp(Bank::Interrupt, value),
                _ => self.state.set_sp(Bank::Master, value),
            }
        }
    }

    /// How long the instruction at `pc` is, read without side effects.
    ///
    /// Only an `RTE` that must step *over* an instruction a handler emulated
    /// needs this, which is why it goes through the disassembler — the one
    /// other place instruction lengths are computed, from the same table.
    fn instruction_length(&self, pc: u32) -> u32 {
        let mut words = [0u16; super::disasm::MAX_EXT_WORDS + 1];
        for (i, slot) in words.iter_mut().enumerate() {
            let at = pc.wrapping_add(2 * i as u32) & self.mask;
            *slot = self
                .space
                .read(u64::from(at), Width::U16, MemAttrs::DEBUG)
                .map_or(0, |v| v as u16);
        }
        u32::from(super::disasm::disassemble_with(self.model, self.copro, pc, &words).len)
    }

    // ------------------------------------------------------------------
    // The 68020's instructions
    // ------------------------------------------------------------------

    /// `MULS.L` and `MULU.L`: 32 × 32, keeping 32 bits or all 64 (M68000PRM,
    /// *MULS*, *MULU*).
    ///
    /// **V** reports that a 32-bit result lost bits — for the signed form,
    /// that the high long is not the sign extension of the low one; a 64-bit
    /// result cannot overflow and clears it. With `Dh` = `Dl` the manual calls
    /// the result undefined; this core writes the low long and then the high
    /// one, so the register ends up holding the high long.
    fn op_mull(&mut self) -> Result<(), Trap> {
        let word = self.ext(0)?;
        let src = self.resolve(Arg::Ea, Size::Long)?;
        let source = self.read_loc(src, Size::Long)?;
        let dl = usize::from((word >> 12) & 7);
        let dh = usize::from(word & 7);
        let signed = word & 0x0800 != 0;
        let quad = word & 0x0400 != 0;
        let multiplicand = self.state.d[dl];
        // Two 32-bit factors cannot overflow 64 bits; wrapping is spelled out
        // only so the arithmetic does not depend on the build profile.
        let product = if signed {
            i64::from(source as i32).wrapping_mul(i64::from(multiplicand as i32)) as u64
        } else {
            u64::from(source).wrapping_mul(u64::from(multiplicand))
        };
        let low = product as u32;
        if quad {
            self.state.d[dl] = low;
            self.state.d[dh] = (product >> 32) as u32;
            self.set_flag(flags::N, product >> 63 != 0);
            self.set_flag(flags::Z, product == 0);
            self.set_flag(flags::V, false);
        } else {
            self.state.d[dl] = low;
            let overflow = if signed {
                product as i64 != i64::from(low as i32)
            } else {
                product >> 32 != 0
            };
            self.set_flag(flags::N, low >> 31 != 0);
            self.set_flag(flags::Z, low == 0);
            self.set_flag(flags::V, overflow);
        }
        self.set_flag(flags::C, false);
        self.settle()
    }

    /// `DIVS.L`, `DIVU.L`, `DIVSL.L` and `DIVUL.L` (M68000PRM, *DIVS*,
    /// *DIVU*).
    ///
    /// Bit 10 of the extension word asks for a 64-bit dividend in `Dr:Dq`;
    /// without it the dividend is `Dq` and the remainder still goes to `Dr`
    /// unless `Dr` is `Dq`. The remainder takes the dividend's sign, which is
    /// what Rust's truncating division gives. A quotient that does not fit in
    /// 32 bits sets **V** and leaves both registers alone.
    fn op_divl(&mut self) -> Result<(), Trap> {
        let pc0 = self.pc0;
        let word = self.ext(0)?;
        let src = self.resolve(Arg::Ea, Size::Long)?;
        let divisor = self.read_loc(src, Size::Long)?;
        let dq = usize::from((word >> 12) & 7);
        let dr = usize::from(word & 7);
        let signed = word & 0x0800 != 0;
        let quad = word & 0x0400 != 0;
        self.facts.signed = signed;
        if divisor == 0 {
            // As the word form: the condition codes the manual leaves
            // undefined are cleared, X excepted. The 68020's frame says where
            // the next instruction is and which one divided (MC68020UM Table
            // 6-5), and nothing was prefetched.
            self.set_flag(flags::N, false);
            self.set_flag(flags::Z, false);
            self.set_flag(flags::V, false);
            self.set_flag(flags::C, false);
            let next = self.state.pc.wrapping_add(2);
            return Err(Trap::six(vector::DIVIDE_BY_ZERO, next, pc0));
        }
        let low = u64::from(self.state.d[dq]);
        let result = if signed {
            let dividend = if quad {
                ((u64::from(self.state.d[dr]) << 32) | low) as i64
            } else {
                i64::from(low as u32 as i32)
            };
            let divisor = i64::from(divisor as i32);
            // Only i64::MIN / -1 can overflow here, and it wraps to a
            // quotient the range check below rejects.
            let quotient = dividend.wrapping_div(divisor);
            let remainder = dividend.wrapping_rem(divisor);
            i32::try_from(quotient)
                .ok()
                .map(|q| (q as u32, remainder as u32))
        } else {
            let dividend = if quad {
                (u64::from(self.state.d[dr]) << 32) | low
            } else {
                low
            };
            let divisor = u64::from(divisor);
            u32::try_from(dividend / divisor)
                .ok()
                .map(|q| (q, (dividend % divisor) as u32))
        };
        match result {
            Some((quotient, remainder)) => {
                // Remainder first, so a `Dr` that is `Dq` ends up holding
                // only the quotient, as the manual says.
                self.state.d[dr] = remainder;
                self.state.d[dq] = quotient;
                self.set_flag(flags::N, quotient >> 31 != 0);
                self.set_flag(flags::Z, quotient == 0);
                self.set_flag(flags::V, false);
            }
            None => self.set_flag(flags::V, true),
        }
        self.set_flag(flags::C, false);
        self.settle()
    }

    /// `EXTB.L`: a byte sign-extended to a long (M68000PRM, *EXT, EXTB*).
    fn op_extb(&mut self) -> Result<(), Trap> {
        let n = reg_lo(self.opcode);
        let value = i32::from(self.state.d[n] as i8) as u32;
        self.state.d[n] = value;
        self.set_logic_flags(value, Size::Long);
        self.settle()
    }

    /// `TRAPcc`: trap through vector 7 if the condition holds, after skipping
    /// the operand words that are only there for a handler to read
    /// (M68000PRM, *TRAPcc*). The frame pushes the next instruction and the
    /// `TRAPcc`'s own address (MC68020UM Table 6-5).
    fn op_trapcc(&mut self) -> Result<(), Trap> {
        let pc0 = self.pc0;
        let words = match self.opcode & 7 {
            2 => 1,
            3 => 2,
            _ => 0,
        };
        for _ in 0..words {
            self.ext(0)?;
        }
        self.settle()?;
        if self.state.test(Cond::from_opcode(self.opcode)) {
            let pc = self.state.pc;
            return Err(Trap::six(vector::TRAPV, pc, pc0));
        }
        Ok(())
    }

    /// `CAS`: compare an operand with `Dc`, and write `Du` over it if they
    /// are equal or load it into `Dc` if not (M68000PRM, *CAS*). The flags are
    /// `CMP`'s, destination minus compare operand.
    ///
    /// The read and the write are one indivisible cycle on the bus; this bus
    /// model has no lock to assert, and nothing else runs between them.
    fn op_cas(&mut self, size: Size) -> Result<(), Trap> {
        let word = self.ext(0)?;
        let dc = usize::from(word & 7);
        let du = usize::from((word >> 6) & 7);
        let loc = self.resolve(Arg::Ea, size)?;
        let dest = self.read_loc(loc, size)?;
        let compare = self.state.d[dc] & size.mask();
        let result = dest.wrapping_sub(compare) & size.mask();
        self.set_sub_flags(compare, dest, result, size, false);
        self.facts.taken = result == 0;
        if result == 0 {
            let update = self.state.d[du];
            self.write_loc(loc, size, update)?;
        } else {
            self.state.d[dc] = merge(self.state.d[dc], dest, size);
        }
        self.settle()
    }

    /// `CAS2`: `CAS` on two operands at once, each addressed by a register
    /// (M68000PRM, *CAS2*). Both must match for either to be written; if
    /// either does not, both are loaded into their compare registers, operand
    /// 2 first, so that with `Dc1` = `Dc2` the register holds operand 1, as
    /// the manual specifies.
    fn op_cas2(&mut self, size: Size) -> Result<(), Trap> {
        let first = self.ext(0)?;
        let second = self.ext(0)?;
        let addr1 = self.register(u32::from(first >> 12));
        let addr2 = self.register(u32::from(second >> 12));
        let (dc1, du1) = (usize::from(first & 7), usize::from((first >> 6) & 7));
        let (dc2, du2) = (usize::from(second & 7), usize::from((second >> 6) & 7));
        let dest1 = self.read_loc(Loc::Mem(addr1), size)?;
        let dest2 = self.read_loc(Loc::Mem(addr2), size)?;
        let compare1 = self.state.d[dc1] & size.mask();
        let result1 = dest1.wrapping_sub(compare1) & size.mask();
        self.set_sub_flags(compare1, dest1, result1, size, false);
        let mut equal = result1 == 0;
        if equal {
            let compare2 = self.state.d[dc2] & size.mask();
            let result2 = dest2.wrapping_sub(compare2) & size.mask();
            self.set_sub_flags(compare2, dest2, result2, size, false);
            equal = result2 == 0;
        }
        self.facts.taken = equal;
        if equal {
            let (update1, update2) = (self.state.d[du1], self.state.d[du2]);
            self.write_loc(Loc::Mem(addr1), size, update1)?;
            self.write_loc(Loc::Mem(addr2), size, update2)?;
        } else {
            self.state.d[dc2] = merge(self.state.d[dc2], dest2, size);
            self.state.d[dc1] = merge(self.state.d[dc1], dest1, size);
        }
        self.settle()
    }

    /// `CMP2` and `CHK2`: is a register inside a bounds pair, lower bound
    /// first in memory (M68000PRM, *CMP2*, *CHK2*)?
    ///
    /// The manual defines the result for a signed pair whose lower bound is
    /// the arithmetically smaller and for an unsigned pair whose lower bound is
    /// the logically smaller, without saying which comparison the processor
    /// makes. One test answers both: the register is in range when its
    /// distance above the lower bound, modulo the operand size, is no more
    /// than the upper bound's. An address register is compared in all 32 bits
    /// against bounds sign-extended to 32. **Z** says the register equals a
    /// bound and **C** that it is outside; **N** and **V** are undefined and
    /// left alone. `CHK2` out of range traps through vector 6.
    fn op_cmp2(&mut self, size: Size) -> Result<(), Trap> {
        let pc0 = self.pc0;
        let word = self.ext(0)?;
        let index = u32::from(word >> 12);
        let check = word & 0x0800 != 0;
        let Loc::Mem(addr) = self.resolve(Arg::Ea, size)? else {
            return Err(Trap::at(vector::ILLEGAL, pc0));
        };
        let lower = self.read_loc(Loc::Mem(addr), size)?;
        let upper = self.read_loc(Loc::Mem(addr.wrapping_add(size.bytes())), size)?;
        let (value, lower, upper, mask) = if index >= 8 {
            let extend = |v: u32| match size {
                Size::Byte => i32::from(v as i8) as u32,
                Size::Word => i32::from(v as i16) as u32,
                Size::Long => v,
            };
            (self.register(index), extend(lower), extend(upper), u32::MAX)
        } else {
            (
                self.register(index) & size.mask(),
                lower,
                upper,
                size.mask(),
            )
        };
        let span = upper.wrapping_sub(lower) & mask;
        let above = value.wrapping_sub(lower) & mask;
        let outside = above > span;
        self.set_flag(flags::Z, value == lower || value == upper);
        self.set_flag(flags::C, outside);
        self.settle()?;
        if check && outside {
            let pc = self.state.pc;
            return Err(Trap::six(vector::CHK, pc, pc0));
        }
        Ok(())
    }

    /// `PACK`: two unpacked BCD digits, plus an adjustment, into one byte
    /// (M68000PRM, *PACK*). Condition codes are not affected.
    ///
    /// The memory form reads two bytes by predecrement, the low one first —
    /// the lower address holds the digit that ends up in the high nibble —
    /// and writes one.
    fn op_pack(&mut self) -> Result<(), Trap> {
        let adjust = self.ext(0)?;
        let (x, y) = (reg_lo(self.opcode), reg_hi(self.opcode));
        let packed = |word: u16| -> u8 {
            let v = word.wrapping_add(adjust);
            (((v >> 4) & 0xf0) | (v & 0x0f)) as u8
        };
        if self.opcode & 8 == 0 {
            let v = packed(self.state.d[x] as u16);
            self.state.d[y] = merge(self.state.d[y], u32::from(v), Size::Byte);
            return self.settle();
        }
        let low = self.predecrement(x, Size::Byte);
        let low = self.read_byte(low)?;
        let high = self.predecrement(x, Size::Byte);
        let high = self.read_byte(high)?;
        let v = packed((u16::from(high) << 8) | u16::from(low));
        let at = self.predecrement(y, Size::Byte);
        self.write_byte(at, v)?;
        self.settle()
    }

    /// `UNPK`: one packed BCD byte into two digits, plus an adjustment
    /// (M68000PRM, *UNPK*). The memory form writes the two bytes by
    /// predecrement, the low one first.
    fn op_unpk(&mut self) -> Result<(), Trap> {
        let adjust = self.ext(0)?;
        let (x, y) = (reg_lo(self.opcode), reg_hi(self.opcode));
        let unpacked = |byte: u8| -> u16 {
            let b = u16::from(byte);
            (((b & 0xf0) << 4) | (b & 0x0f)).wrapping_add(adjust)
        };
        if self.opcode & 8 == 0 {
            let v = unpacked(self.state.d[x] as u8);
            self.state.d[y] = merge(self.state.d[y], u32::from(v), Size::Word);
            return self.settle();
        }
        let at = self.predecrement(x, Size::Byte);
        let byte = self.read_byte(at)?;
        let v = unpacked(byte);
        let low = self.predecrement(y, Size::Byte);
        self.write_byte(low, v as u8)?;
        let high = self.predecrement(y, Size::Byte);
        self.write_byte(high, (v >> 8) as u8)?;
        self.settle()
    }

    /// A bit field's offset and width, from its extension word and the
    /// registers it names (M68000PRM, *BFTST*): the offset signed and as
    /// wide as a register, the width 1–32.
    fn field_spec(&self, word: u16) -> (i32, u32) {
        let spec = FieldSpec::decode(word);
        let offset = if spec.offset_reg {
            self.state.d[usize::from(spec.offset)] as i32
        } else {
            i32::from(spec.offset)
        };
        let width = if spec.width_reg {
            self.state.d[usize::from(spec.width)] & 31
        } else {
            u32::from(spec.width)
        };
        (offset, if width == 0 { 32 } else { width })
    }

    /// The bit-field instructions (M68000PRM, *BFCHG* to *BFTST*).
    ///
    /// Bit 0 of a field is its most significant bit, and the offset counts
    /// from the most significant bit of the base. In a data register the
    /// offset is taken modulo 32 and a field that runs off bit 0 wraps round
    /// to bit 31; in memory the offset is signed and addresses bytes either
    /// side of the base, and the field can straddle five of them. Every
    /// instruction sets **N** and **Z** from the field as it was, except
    /// `BFINS`, which sets them from what it inserts.
    fn op_bitfield(&mut self, op: Op) -> Result<(), Trap> {
        let word = self.ext(0)?;
        let (offset, width) = self.field_spec(word);
        let reg = usize::from((word >> 12) & 7);
        let top = if width == 32 {
            u32::MAX
        } else {
            !(u32::MAX >> width)
        };
        let loc = self.resolve(Arg::Ea, Size::Long)?;
        // The field, right-justified, and a way to put a new one back.
        let (field, memory) = match loc {
            Loc::D(n) => {
                let rotated = self.state.d[usize::from(n)].rotate_left(offset as u32 & 31);
                (rotated >> (32 - width), None)
            }
            Loc::Mem(base) => {
                // Arithmetic shift: a negative offset reaches below the base.
                let address = base.wrapping_add((offset >> 3) as u32);
                let bit = (offset & 7) as u32;
                let bytes = (bit + width).div_ceil(8);
                self.facts.five_bytes = bytes == 5;
                let span = self.read_span(address, bytes)?;
                let shift = 8 * bytes - bit - width;
                (
                    (span >> shift) as u32 & (top >> (32 - width)),
                    Some((address, bytes, span, shift)),
                )
            }
            _ => {
                let pc0 = self.pc0;
                return Err(Trap::at(vector::ILLEGAL, pc0));
            }
        };
        let sign = 1u32 << (width - 1);
        let inserted = if op == Op::Bfins {
            self.state.d[reg] & (top >> (32 - width))
        } else {
            field
        };
        let flagged = if op == Op::Bfins { inserted } else { field };
        self.set_flag(flags::N, flagged & sign != 0);
        self.set_flag(flags::Z, flagged == 0);
        self.set_flag(flags::V, false);
        self.set_flag(flags::C, false);
        let replacement = match op {
            Op::Bfchg => Some(!field),
            Op::Bfclr => Some(0),
            Op::Bfset => Some(u32::MAX),
            Op::Bfins => Some(inserted),
            _ => None,
        };
        match op {
            Op::Bfextu => self.state.d[reg] = field,
            Op::Bfexts => {
                self.state.d[reg] = if field & sign != 0 {
                    field | !(top >> (32 - width))
                } else {
                    field
                };
            }
            Op::Bfffo => {
                // The offset of the first set bit, counted from the field's
                // offset; offset plus width if there is none.
                let lead = (field << (32 - width)).leading_zeros().min(width);
                self.state.d[reg] = (offset as u32).wrapping_add(lead);
            }
            _ => {}
        }
        if let Some(value) = replacement {
            let value = value & (top >> (32 - width));
            match (loc, memory) {
                (Loc::D(n), _) => {
                    let n = usize::from(n);
                    let mask = top.rotate_right(offset as u32 & 31);
                    let placed = (value << (32 - width)).rotate_right(offset as u32 & 31);
                    self.state.d[n] = (self.state.d[n] & !mask) | placed;
                }
                (_, Some((address, bytes, span, shift))) => {
                    let mask = u64::from(top >> (32 - width)) << shift;
                    let span = (span & !mask) | (u64::from(value) << shift);
                    self.write_span(address, bytes, span)?;
                }
                _ => {}
            }
        }
        self.settle()
    }

    /// Read `bytes` (1–5) bytes as one right-justified value, in the operand
    /// cycles a 68020 on a 16-bit port would use: a long and then a byte for
    /// five (M68000PRM, *BFCHG*: "long word with byte (for a 5-byte access)").
    fn read_span(&mut self, address: u32, bytes: u32) -> Result<u64, Trap> {
        Ok(match bytes {
            1 => u64::from(self.read_byte(address)?),
            2 => u64::from(self.read_word(address)?),
            3 => {
                let hi = u64::from(self.read_word(address)?);
                (hi << 8) | u64::from(self.read_byte(address.wrapping_add(2))?)
            }
            4 => u64::from(self.read_long(address)?),
            _ => {
                let hi = u64::from(self.read_long(address)?);
                (hi << 8) | u64::from(self.read_byte(address.wrapping_add(4))?)
            }
        })
    }

    /// Write back what [`Exec::read_span`] read.
    fn write_span(&mut self, address: u32, bytes: u32, value: u64) -> Result<(), Trap> {
        match bytes {
            1 => self.write_byte(address, value as u8),
            2 => self.write_word(address, value as u16),
            3 => {
                self.write_word(address, (value >> 8) as u16)?;
                self.write_byte(address.wrapping_add(2), value as u8)
            }
            4 => self.write_long(address, value as u32),
            _ => {
                self.write_long(address, (value >> 8) as u32)?;
                self.write_byte(address.wrapping_add(4), value as u8)
            }
        }
    }

    /// `CALLM`: call a module through its descriptor (M68000PRM, *CALLM*;
    /// MC68020UM §9.7, §9.8.1).
    ///
    /// A type 0 descriptor — no change of access rights — is carried out in
    /// full: the 24-byte module frame on the stack, the module data pointer
    /// saved and reloaded through the register the entry word names, and
    /// execution from the word after it. A type 1 descriptor asks external
    /// access-control hardware, through CPU space, to change the access
    /// level; there is none on this bus, and "if the processor receives a bus
    /// error on any of these CPU space accesses ... the processor will take a
    /// format error exception" (§9.8), so that is what it does. A descriptor
    /// with an unknown option or type is a format error too.
    fn op_callm(&mut self) -> Result<(), Trap> {
        let pc0 = self.pc0;
        let word = self.ext(0)?;
        let Loc::Mem(descriptor) = self.resolve_control(Arg::Ea, ExtraCycles::Control)? else {
            return Err(Trap::at(vector::ILLEGAL, pc0));
        };
        let control = self.read_long(descriptor)?;
        let option = control >> 29;
        let kind = (control >> 24) & 0x1f;
        if (option != 0 && option != 4) || kind != 0 {
            return Err(Trap::raised(vector::FORMAT_ERROR, pc0));
        }
        let entry = self.read_long(descriptor.wrapping_add(4))?;
        let data = self.read_long(descriptor.wrapping_add(8))?;
        let entry_word = self.read_word(entry)?;
        let register = u32::from(entry_word >> 12);
        let ret = self.state.pc.wrapping_add(2);
        let sp0 = self.state.a[7];
        let frame = sp0.wrapping_sub(MODULE_FRAME);
        // Opt and type from the descriptor; a type 0 call changes no access
        // level, so the saved one is zero.
        self.write_word(frame, ((option as u16) << 13) | ((kind as u16) << 8))?;
        self.write_word(frame.wrapping_add(2), u16::from(self.state.ccr()))?;
        self.write_word(frame.wrapping_add(4), word & 0xff)?;
        self.write_word(frame.wrapping_add(6), 0)?;
        self.write_long(frame.wrapping_add(8), descriptor)?;
        self.write_long(frame.wrapping_add(12), ret)?;
        let saved = self.register(register);
        self.write_long(frame.wrapping_add(16), saved)?;
        // With arguments passed by pointer, the caller's stack pointer goes in
        // the frame for the callee to find them by (§9.7.1).
        if option == 4 {
            self.write_long(frame.wrapping_add(20), sp0)?;
        }
        self.state.a[7] = frame;
        self.set_register(register, data);
        if register == 15 {
            // "the loaded value will be overwritten with the correct stack
            // pointer value after the module stack frame is created"
            self.state.a[7] = frame;
        }
        self.refill(entry.wrapping_add(2), 0)
    }

    /// `RTM`: return from a module, undoing `CALLM` (M68000PRM, *RTM*;
    /// MC68020UM §9.8.2). A type 1 frame would need the access-control
    /// hardware again and is a format error, as is anything unrecognised.
    fn op_rtm(&mut self) -> Result<(), Trap> {
        let pc0 = self.pc0;
        let sp = self.state.a[7];
        let control = self.read_word(sp)?;
        let option = control >> 13;
        let kind = (control >> 8) & 0x1f;
        if (option != 0 && option != 4) || kind != 0 {
            return Err(Trap::raised(vector::FORMAT_ERROR, pc0));
        }
        let ccr = self.read_word(sp.wrapping_add(2))?;
        let count = u32::from(self.read_word(sp.wrapping_add(4))? & 0xff);
        let pc = self.read_long(sp.wrapping_add(12))?;
        let saved = self.read_long(sp.wrapping_add(16))?;
        let register = u32::from(self.opcode & 0xf);
        self.set_register(register, saved);
        // After the register, so `RTM A7` leaves the stack pointer the
        // frame's removal made, "and the saved module data area pointer is
        // lost".
        self.state.a[7] = sp.wrapping_add(MODULE_FRAME).wrapping_add(count);
        let sr = (self.state.sr & !flags::CCR) | (ccr & flags::CCR);
        self.state.set_sr(sr);
        self.refill(pc, 0)
    }

    // ------------------------------------------------------------------
    // Shared arithmetic
    // ------------------------------------------------------------------

    /// `-(An)` as `ADDX`, `SUBX`, `ABCD` and `SBCD` perform it: **one word at
    /// a time**.
    ///
    /// Those four are the only instructions whose predecrement is not a single
    /// address calculation. A long operand is two separate steps — decrement
    /// two, read a word, decrement two, read a word — which is why the low
    /// half comes off the bus first and why an address error part way through
    /// leaves the register two bytes further on than a `-(An)` anywhere else
    /// would. Multi-precision code that catches its own bus errors can see the
    /// difference.
    fn predecrement(&mut self, reg: usize, size: Size) -> u32 {
        let addr = self.state.a[reg].wrapping_sub(step(size, reg));
        self.state.a[reg] = addr;
        addr
    }

    /// Read the operand a predecrement step just addressed.
    fn read_predecrement(&mut self, reg: usize, size: Size) -> Result<u32, Trap> {
        if size != Size::Long {
            let addr = self.predecrement(reg, size);
            return self.read_loc(Loc::Mem(addr), size);
        }
        let low_at = self.state.a[reg].wrapping_sub(2);
        self.state.a[reg] = low_at;
        let low = self.read_word(low_at)?;
        let high_at = low_at.wrapping_sub(2);
        self.state.a[reg] = high_at;
        let high = self.read_word(high_at)?;
        Ok((u32::from(high) << 16) | u32::from(low))
    }

    /// `N`, `Z`, `V`, `C` (and `X` when `extend`) for an addition.
    fn set_add_flags(&mut self, src: u32, dst: u32, result: u32, size: Size, extend: bool) {
        let sign = size.sign_bit();
        let sm = src & sign != 0;
        let dm = dst & sign != 0;
        let rm = result & sign != 0;
        let carry = (sm && dm) || (!rm && (sm || dm));
        let overflow = (sm && dm && !rm) || (!sm && !dm && rm);
        self.set_flag(flags::N, rm);
        self.set_flag(flags::Z, result & size.mask() == 0);
        self.set_flag(flags::V, overflow);
        self.set_flag(flags::C, carry);
        if extend {
            self.set_flag(flags::X, carry);
        }
    }

    /// `N`, `Z`, `V`, `C` (and `X` when `extend`) for a subtraction.
    fn set_sub_flags(&mut self, src: u32, dst: u32, result: u32, size: Size, extend: bool) {
        let sign = size.sign_bit();
        let sm = src & sign != 0;
        let dm = dst & sign != 0;
        let rm = result & sign != 0;
        let borrow = (sm && !dm) || (rm && (sm || !dm));
        let overflow = (!sm && dm && !rm) || (sm && !dm && rm);
        self.set_flag(flags::N, rm);
        self.set_flag(flags::Z, result & size.mask() == 0);
        self.set_flag(flags::V, overflow);
        self.set_flag(flags::C, borrow);
        if extend {
            self.set_flag(flags::X, borrow);
        }
    }

    /// `ABCD`'s decimal addition.
    ///
    /// Binary addition with a nibble correction, which is what the hardware
    /// does — the difference matters because the operands are not required to
    /// be valid BCD and the result for invalid input is well defined even
    /// though the manual calls it undefined (M68000PRM, ABCD).
    fn bcd_add(&mut self, src: u32, dst: u32) -> u32 {
        let x = u32::from(self.state.flag(flags::X));
        let low = (src & 0x0f) + (dst & 0x0f) + x;
        let binary = (src & 0xff) + (dst & 0xff) + x;
        // The decimal carry is decided on the *binary* sum, before the low
        // nibble's correction is folded in. That distinction is only visible
        // for operands that are not valid BCD — `$2d + $69` corrects to `$9c`
        // with no carry, where testing after the correction would wrongly
        // carry — but the hardware is unambiguous about it and real code has
        // been known to rely on it.
        let carry = binary > 0x99;
        let mut result = binary;
        if low > 9 {
            result += 6;
        }
        if carry {
            result += 0x60;
        }
        let result = result & 0xff;
        self.set_flag(flags::C, carry);
        self.set_flag(flags::X, carry);
        self.set_flag(flags::N, result & 0x80 != 0);
        // V is documented as undefined and is not: it reports either decimal
        // correction carrying the result across the sign boundary, which is
        // the one thing the adder can cheaply notice.
        self.set_flag(flags::V, !binary & result & 0x80 != 0);
        if result != 0 {
            self.set_flag(flags::Z, false);
        }
        result
    }

    /// `SBCD`/`NBCD`'s decimal subtraction.
    fn bcd_sub(&mut self, src: u32, dst: u32) -> u32 {
        // Signed arithmetic, because two *different* borrows have to be told
        // apart and a `0x100` test conflates them once a value has gone more
        // than one place negative. The **binary** subtraction's borrow decides
        // whether the tens digit needs correcting; the borrow reported in C
        // and X is the one out of the units-corrected result. `$f0 - $ef`
        // borrows nothing in binary, needs no tens correction, and still sets
        // the carry — because subtracting the units correction is what takes
        // it below zero.
        let x = i32::from(self.state.flag(flags::X));
        let src = (src & 0xff) as i32;
        let dst = (dst & 0xff) as i32;
        let low = (dst & 0x0f) - (src & 0x0f) - x;
        let mut result = dst - src - x;
        let binary = (result as u32) & 0xff;
        let binary_borrow = result < 0;
        if low < 0 {
            result -= 6;
        }
        let borrow = result < 0;
        if binary_borrow {
            result -= 0x60;
        }
        let result = (result as u32) & 0xff;
        self.set_flag(flags::C, borrow);
        self.set_flag(flags::X, borrow);
        self.set_flag(flags::N, result & 0x80 != 0);
        // As for ABCD, against the uncorrected binary difference.
        self.set_flag(flags::V, binary & !result & 0x80 != 0);
        if result != 0 {
            self.set_flag(flags::Z, false);
        }
        result
    }
}

/// The fourteen-byte group-0 stack frame's extra fields.
#[derive(Debug, Clone, Copy)]
struct Group0 {
    ssw: u16,
    addr: u32,
    ir: u16,
}

/// This core's version number in a 68010 long frame, bits 13–10 of the word
/// at `SP + $1A`.
///
/// The manual says only that `RTE` compares it with the processor's own and
/// takes a format error if they differ (MC68000UM §6.4); the value is this
/// core's, and a frame another core wrote is not one it can restart from.
pub(super) const VERSION_68010: u16 = 0x1;

/// The same for a 68020 long bus fault frame, bits 15–12 of the word at
/// `SP + $36` (MC68020UM §6.1.12).
pub(super) const VERSION_68020: u16 = 0x1;

/// The 68040 special status word's `TT` and `TM` fields, from a function
/// code (M68040UM Tables 3-2, 5-2 and 5-3).
///
/// Two things happen here that a 68020's `FC2`–`FC0` field does not do.
///
/// First, "the integer unit translates `MOVES` accesses to instruction
/// address spaces (SFC/DFC = $6 or $2) into data references (SFC/DFC = $5 or
/// $1) ... the resulting access error stack frame contains the **converted**
/// function code in the TM field" (§3.2.5). So a `MOVES` to program space
/// reports as a data access.
///
/// Second, the function codes that are not one of the four ordinary ones —
/// `$0`, `$3`, `$4` and `$7` — are *alternate logical* accesses, `TT = 10`,
/// and carry the function code itself in `TM`. Only `MOVES` can produce one,
/// which is why `alternate` is the caller's "this access came from an
/// `SFC`/`DFC` override" rather than a property of `fc` alone.
const fn ssw_transfer(fc: u8, alternate: bool) -> (u8, u8) {
    if !alternate {
        // An ordinary access drives its own function code: Table 5-3 is the
        // function code numbering, so user code stays `$2` and supervisor
        // code stays `$6`. §8.4.6 leans on exactly that — "for user and
        // supervisor instruction faults, the TM field contains $2 and $6".
        return (0b00, fc & 7);
    }
    match fc & 7 {
        // Table 3-2's four ordinary rows, with program space folded onto
        // data because the data memory unit is what carries the access.
        1 | 2 => (0b00, 0b001),
        5 | 6 => (0b00, 0b101),
        // The other four are alternate accesses, and only reachable through
        // an `SFC`/`DFC` that names one.
        other => (0b10, other),
    }
}

/// 68020 special status word bits (MC68020UM Figure 6-8).
const SSW_FC: u16 = 0x8000;
/// Fault on pipe stage B.
const SSW_FB: u16 = 0x4000;
/// Rerun pipe stage C.
const SSW_RC: u16 = 0x2000;
/// Rerun pipe stage B.
const SSW_RB: u16 = 0x1000;
/// Data fault: rerun the data cycle.
const SSW_DF: u16 = 0x0100;

/// The 68040 special status word's four continuation flags: `CP`, `CU`, `CT`
/// and `CM`, bits 15–12 (M68040UM Figure 8-7).
const SSW_040_CONTINUE: u16 = 0xf000;

/// What the floating-point unit does with an instruction it has decoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Availability {
    /// It computes it.
    Hardware,
    /// Nothing answers the F line: the line-F exception, vector 11, with a
    /// format `$0` frame.
    LineF,
    /// The 68040 recognises it as a floating-point instruction but does not
    /// implement it: vector 11 with a format `$2` frame, which is how an
    /// emulation handler tells the two apart (M68040UM §9.6.1).
    Unimplemented,
    /// The operand's data format is one the 68040 leaves to software:
    /// vector 55 (§9.6.2).
    UnsupportedType,
}

/// What one floating-point operation produced.
struct FpResult {
    /// The value, which is also what sets the condition codes.
    value: F80,
    /// Whether it is written to the destination register.
    store: bool,
    /// Whether it sets the condition codes.
    condition: bool,
    /// The `FPSR` exception bits it raised.
    exc: u16,
}

impl FpResult {
    /// A result that is stored, with the exceptions `src/float` reported and
    /// the 68881's own tininess on top.
    fn stored(value: F80, flags: Flags, spec: Spec) -> FpResult {
        let mut exc = fpu::exceptions_from(flags, false);
        // `src/float` reports underflow only for a result that is both tiny
        // *and* inexact, which is IEEE's rule for the flag; the 68881's
        // exception bit is tininess alone and the AND with INEX2 happens on
        // the way into the accrued byte (M68881UM §6.1.5's note).
        if fpu::is_tiny(value, spec) {
            exc |= fpbits::UNFL;
        }
        FpResult {
            value,
            store: true,
            condition: true,
            exc,
        }
    }
}

/// The version number this core writes in an `FSAVE` state frame.
///
/// The frame's body is microcode state on hardware and this core's own here,
/// so it carries a version of its own and `FRESTORE` refuses anybody else's
/// — the same bargain the 68010's long bus-fault frame makes.
pub(super) const FP_STATE_VERSION: u8 = 0x40;

/// The bits a root pointer descriptor has storage for: **L/U** (63), the
/// fifteen-bit **LIMIT** (62–48), **DT** (33–32) and the table address
/// (31–4). "All other unused bits must always be zeros ... In the root
/// pointers, these bits are not alterable" (MC68030UM §9.5.1.1).
const ROOT_POINTER_BITS: u64 = 0xffff_0003_ffff_fff0;

/// The size of a `CALLM` module frame, arguments not included (MC68020UM
/// Figure 9-12).
const MODULE_FRAME: u32 = 24;

/// The 68020's `CACR` bits with storage: **E** and **F** (MC68020UM Figure
/// 4-2). **C** and **CE** act on contents and always read back as zero.
const CACR_STORED_020: u32 = 0x0003;

/// The 68030's: it has a data cache as well, so **EI** (0), **FI** (1),
/// **IBE** (4), **ED** (8), **FD** (9), **DBE** (12) and **WA** (13) have
/// storage, and the four clear bits — **CEI** (2), **CI** (3), **CED** (10)
/// and **CD** (11) — do not (MC68030UM §6.3.1, Figure 6-14).
const CACR_STORED_030: u32 = 0x3313;

/// The 68040's: two enable bits and nothing else. **DE** is bit 31 and **IE**
/// is bit 15; everything between and below them is drawn "UNDEFINED"
/// (M68040UM Figure 4-4). The 68020's and 68030's clear-the-cache bits are
/// gone — `CINV` and `CPUSH` do that job, and they are instructions.
const CACR_STORED_040: u32 = 0x8000_8000;

/// How many throwaway frames one `RTE` follows before calling the stack
/// corrupt. See `op_rte_formatted`.
const MAX_THROWAWAY_FRAMES: u32 = 256;

/// Undo entries a long frame carries: `(code, old value)`, the code 0–6 for
/// `A0`–`A6` and 7–9 for the user, interrupt and master stack pointers.
type UndoList = ([(u8, u32); 10], usize);

/// Which stack frame an exception builds.
#[derive(Debug, Clone, Copy)]
enum Frame {
    /// The 68000's own: six bytes, or fourteen with the group-0 fields.
    Classic(Option<Group0>),
    /// A format-word frame, 68010 on.
    Format(FrameImage),
}

impl Frame {
    /// A frame of `format` with nothing past the first four words.
    const fn format(format: u8) -> Frame {
        Frame::Format(FrameImage::new(format))
    }

    /// The same frame, followed by a throwaway frame on the interrupt stack.
    const fn throwaway(self) -> Frame {
        match self {
            Frame::Format(mut image) => {
                image.throwaway = true;
                Frame::Format(image)
            }
            other => other,
        }
    }
}

/// The words of a format-word frame beyond the four every format shares.
#[derive(Debug, Clone, Copy)]
struct FrameImage {
    /// The format code, bits 15–12 of the format word.
    format: u8,
    /// `words[0]` is the word at `SP + 8`.
    words: [u16; 42],
    len: u8,
    /// Words the processor reserves but does not write.
    skip: u64,
    /// Follow with a format $1 frame on the interrupt stack.
    throwaway: bool,
}

impl FrameImage {
    const fn new(format: u8) -> FrameImage {
        FrameImage {
            format,
            words: [0; 42],
            len: 0,
            skip: 0,
            throwaway: false,
        }
    }

    fn push(&mut self, word: u16) {
        self.words[self.len as usize] = word;
        self.len += 1;
    }

    fn skip_one(&mut self) {
        self.skip |= 1u64 << self.len;
        self.len += 1;
    }

    /// `slots` undo entries of three words each: the register code, then the
    /// value. An unused slot has code `$ffff`.
    fn push_undo(&mut self, undo: &UndoList, slots: usize) {
        let (entries, n) = undo;
        let used = (*n).min(slots);
        for &(code, value) in &entries[..used] {
            self.push(u16::from(code));
            self.push((value >> 16) as u16);
            self.push(value as u16);
        }
        for _ in used..slots {
            self.push(0xffff);
            self.push(0);
            self.push(0);
        }
    }
}

/// Which binary operation an ALU row performs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BinOp {
    Add,
    Sub,
    And,
    Or,
    Eor,
}

/// How much internal time an addressing mode costs beyond its bus cycles.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ExtraCycles {
    /// The mode is fetching an operand.
    Operand,
    /// The mode is `MOVE`'s destination, whose last absolute-long fetch is
    /// deferred past the write.
    MoveDest,
    /// The mode is only computing an address, as for `LEA` and `PEA`.
    Control,
}

impl ExtraCycles {
    /// The internal cycles an indexed mode costs before its refill.
    const fn index_delay(self) -> u32 {
        match self {
            ExtraCycles::Operand | ExtraCycles::MoveDest => 2,
            // LEA and PEA pay two more than an operand fetch does, because
            // nothing else in the instruction overlaps the adder.
            ExtraCycles::Control => 4,
        }
    }
}

/// A non-indexed mode's row in the 68020 effective-address tables.
const fn ea_class(mode: Mode, size: Size) -> timing::EaClass {
    match mode {
        Mode::DataReg => timing::DN,
        Mode::AddrReg => timing::AN,
        Mode::Indirect => 2,
        Mode::PostInc => 3,
        Mode::PreDec => 4,
        Mode::Disp16 | Mode::PcDisp16 => 5,
        Mode::AbsShort => 6,
        Mode::AbsLong => 7,
        Mode::Imm => {
            if matches!(size, Size::Long) {
                timing::IMM_L
            } else {
                timing::IMM_W
            }
        }
        Mode::Index8 | Mode::PcIndex8 => timing::INDEX8,
    }
}

/// The data register named by bits 11–9.
#[inline]
const fn reg_hi(opcode: u16) -> usize {
    ((opcode >> 9) & 7) as usize
}

/// The register named by bits 2–0.
#[inline]
const fn reg_lo(opcode: u16) -> usize {
    (opcode & 7) as usize
}

/// How far an autoincrement mode steps.
///
/// A byte access through `A7` steps by two, because the 68000 keeps the stack
/// pointer even; there is no such rule for `A0`–`A6` (M68000PRM §1.2).
#[inline]
const fn step(size: Size, reg: usize) -> u32 {
    match size {
        Size::Byte if reg == 7 => 2,
        other => other.bytes(),
    }
}

/// Replace the low `size` bits of `old` with `value`.
#[inline]
const fn merge(old: u32, value: u32, size: Size) -> u32 {
    (old & !size.mask()) | (value & size.mask())
}

/// `DIVU`'s data-dependent execution time.
///
/// The manual publishes only the maximum, 140 cycles (MC68000UM Table 8-6),
/// because the microcode's loop exits early — so this is derived from the
/// loop's shape instead, and checked against every division vector in the
/// conformance corpus.
///
/// The loop is restoring division, fifteen iterations, one quotient bit each.
/// What varies is how much work an iteration does: shifting a one out of the
/// top of the partial remainder means the subtraction is known to be needed
/// and costs four; otherwise the comparison costs six when the subtraction
/// happens and eight when it does not. An overflow — a quotient that will not
/// fit in sixteen bits — is detected before the loop starts and costs six.
fn divu_cycles(dividend: u32, divisor: u16) -> u32 {
    if divisor == 0 {
        return 0;
    }
    if (dividend >> 16) >= u32::from(divisor) {
        return 6;
    }
    let divisor = u32::from(divisor);
    let mut cycles = 12;
    let mut high = dividend >> 16;
    let mut low = dividend & 0xffff;
    for _ in 0..15 {
        let carried = high & 0x8000 != 0;
        high = ((high << 1) | (low >> 15)) & 0xffff;
        low = (low << 1) & 0xffff;
        if carried {
            high = high.wrapping_sub(divisor) & 0xffff;
            cycles += 4;
        } else if high >= divisor {
            high -= divisor;
            cycles += 6;
        } else {
            cycles += 8;
        }
    }
    cycles
}

/// `DIVS`'s data-dependent execution time, on the same footing.
///
/// The signed loop does not branch on the partial remainder the way `DIVU`'s
/// does, so its shape is simpler: a fixed cost that depends on the signs, two
/// cycles for every *zero* bit of the quotient's magnitude, and two more when
/// its lowest bit is set. Overflow is again decided before the loop.
fn divs_cycles(dividend: i32, divisor: i16) -> u32 {
    if divisor == 0 {
        return 0;
    }
    let magnitude = dividend.unsigned_abs() / u32::from(divisor.unsigned_abs());
    let negative = (dividend < 0) != (divisor < 0);
    let limit = if negative { 0x8000 } else { 0x7fff };
    if magnitude > limit {
        return 12 + if dividend < 0 { 2 } else { 0 };
    }
    // The overflow test above caps the magnitude at $8000, which has exactly
    // one bit set, so this subtraction has fifteen to give. Raising that cap
    // would underflow it.
    debug_assert!(magnitude <= 0x8000);
    let zeros = 15 - (magnitude & 0xffff).count_ones();
    116 + if dividend < 0 { 4 } else { 0 }
        + if negative { 2 } else { 0 }
        + 2 * zeros
        + 2 * (magnitude & 1)
}
