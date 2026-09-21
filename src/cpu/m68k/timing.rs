//! The 68020's instruction times, from the tables in MC68020UM Section 8.
//!
//! # And the 68030's, which are not its own
//!
//! The 68030 runs through this table too, and that is an approximation stated
//! as one. MC68030UM §11 gives the 68030 its own cache-case column, and it is
//! not the 68020's: the 68030 has a data cache and a wider internal bus, so a
//! `MOVE.L (An),(An)` that costs the 68020 ten clocks costs a 68030 fewer.
//! Charging the 68020's numbers keeps every instruction's *relative* cost and
//! overstates the absolute one; what it cannot do is claim to be measured.
//! A 68030 column belongs here, and until it is here the deviation is in the
//! conformance ledger rather than hidden.
//!
//! # Which column, and why
//!
//! The manual gives three times for everything (MC68020UM §8.2): *best case*,
//! with the instruction in the cache and overlapped as much as its neighbours
//! allow; *cache case*, in the cache but with no overlap; and *worst case*, not
//! in the cache and with no overlap. This model reports the **cache case**,
//! for three reasons:
//!
//! - It is a property of the instruction alone. The best case depends on how
//!   much of the *previous* instruction's tail and the next one's head can run
//!   under this one, which the tables do not give per pair; charging it per
//!   instruction would claim overlap that is not there.
//! - It needs no model of the cache's contents, which this core does not keep
//!   (see `mod.rs`): the worst case is what every instruction would cost with
//!   the cache off or missing, and software the 68020 was built for runs with
//!   it on.
//! - It contains no instruction fetches — its `p` count is zero throughout —
//!   so what is left is the operand cycles and the sequencer's own time, which
//!   is the part of the 68020 an emulator can describe honestly.
//!
//! The manual itself says calculating exact timing from these tables "is
//! impossible", and that they bound real timings rather than predict them
//! (§8.2, Tables 8-2 and 8-3). So this is an approximation, stated as one:
//! the time charged for an instruction is its cache-case entry, the same every
//! time, and the bus cycles the interpreter makes are *not* what the time is
//! built from — a 68020 on a 32-bit port does a long in one cycle where this
//! bus model does two words. That is the one place this crate charges a
//! table rather than counting accesses (CLAUDE.md, *CPU cores*), because the
//! 68020 has no published per-access timing to count.
//!
//! # How an entry is composed
//!
//! As the manual does it: the instruction's own row from §8.2.7–§8.2.18, plus
//! the effective-address time its footnote names — *fetch* (§8.2.1), *fetch
//! immediate* (§8.2.2), *calculate* (§8.2.3), *calculate immediate* (§8.2.4) or
//! *jump* (§8.2.5) — except `MOVE`, whose table (§8.2.6) is complete for every
//! source and destination.
//!
//! # Where the tables are silent
//!
//! - `NBCD` to memory is not listed; it is charged `NBCD Dn`'s six plus the
//!   fetch-effective-address time.
//! - The calculate-immediate table has no `-(An)` row; `(An)+`'s is used.
//! - Exceptions the table does not list — `CHK`, `CHK2` and a zero divide
//!   taking their trap — are charged the time up to the trap plus the listed
//!   six-word-frame exception (`TRAPcc`'s, 25); a format error is charged as a
//!   four-word-frame exception, 20.
//! - The reset sequence is not listed and keeps the per-access count.
//! - The two `(d16,An,Xn)` and `(d16,B)` rows differ, although the note under
//!   the table says the base's form does not affect timing. This model reads
//!   `(d16,An,Xn)` as a full-format word with both a base and an index and a
//!   word displacement, and `(d16,B)` as the same with either suppressed.

use super::isa::{Arg, Insn, Op, Size};

/// A row of the effective-address tables, in the order of §8.2.1.
///
/// 0 `Dn`, 1 `An`, 2 `(An)`, 3 `(An)+`, 4 `-(An)`, 5 `(d16,An)` or `(d16,PC)`,
/// 6 `(xxx).W`, 7 `(xxx).L`, 8 `#<data>.B/.W`, 9 `#<data>.L`,
/// 10 `(d8,An,Xn)`, 11 `(d16,An,Xn)`, 12 `(B)`, 13 `(d16,B)`, 14 `(d32,B)`,
/// and 15–23 the memory-indirect modes, `15 + 3 × bd + od` with each
/// displacement 0 null, 1 word, 2 long.
pub(super) type EaClass = u8;

/// `Dn`.
pub(super) const DN: EaClass = 0;
/// `An`.
pub(super) const AN: EaClass = 1;
/// `#<data>.B` or `.W`.
pub(super) const IMM_W: EaClass = 8;
/// `#<data>.L`.
pub(super) const IMM_L: EaClass = 9;
/// The brief-format indexed modes.
pub(super) const INDEX8: EaClass = 10;
/// A full-format word with a word base displacement, a base and an index.
pub(super) const INDEX16: EaClass = 11;
/// A full-format word with a null base displacement.
pub(super) const BASE: EaClass = 12;
/// A full-format word with a word base displacement and something
/// suppressed.
pub(super) const BASE16: EaClass = 13;
/// A full-format word with a long base displacement.
pub(super) const BASE32: EaClass = 14;
/// The first memory-indirect row.
pub(super) const INDIRECT: EaClass = 15;

/// Fetch effective address, cache case (MC68020UM §8.2.1).
const FETCH: [u8; 24] = [
    0, 0, 4, 4, 5, 5, 4, 4, 2, 4, 7, 7, 7, 9, 13, 12, 14, 14, 14, 16, 16, 18, 20, 20,
];

/// Calculate effective address, cache case (§8.2.3). No immediate rows.
const CALC: [u8; 24] = [
    0, 0, 2, 2, 2, 2, 2, 4, 0, 0, 4, 6, 6, 8, 12, 11, 13, 13, 13, 15, 15, 17, 19, 19,
];

/// Jump effective address, cache case (§8.2.5). Only the control modes.
const JUMP: [u8; 24] = [
    0, 0, 2, 0, 0, 4, 2, 2, 0, 0, 6, 6, 6, 8, 12, 11, 13, 13, 13, 15, 15, 17, 19, 19,
];

/// Fetch immediate effective address, cache case (§8.2.2): `(#.W, #.L)` by
/// destination. `An` has no row and takes `Dn`'s.
const FETCH_IMM: [(u8, u8); 24] = [
    (2, 4),
    (2, 4),
    (4, 4),
    (6, 8),
    (5, 7),
    (5, 7),
    (5, 7),
    (6, 8),
    (4, 6),
    (6, 8),
    (9, 11),
    (9, 11),
    (9, 11),
    (11, 13),
    (15, 17),
    (14, 16),
    (16, 18),
    (16, 18),
    (16, 18),
    (18, 20),
    (17, 19),
    (20, 22),
    (22, 24),
    (22, 24),
];

/// Calculate immediate effective address, cache case (§8.2.4): `(#.W, #.L)`
/// by destination; `-(An)` has no row and takes `(An)+`'s.
const CALC_IMM: [(u8, u8); 24] = [
    (2, 4),
    (2, 4),
    (2, 4),
    (4, 6),
    (4, 6),
    (4, 6),
    (4, 6),
    (4, 8),
    (0, 0),
    (0, 0),
    (6, 8),
    (8, 10),
    (8, 10),
    (10, 12),
    (14, 16),
    (13, 15),
    (15, 17),
    (15, 17),
    (15, 17),
    (17, 19),
    (17, 19),
    (19, 21),
    (21, 23),
    (21, 23),
];

/// `MOVE`, cache case (§8.2.6): 23 source rows by 22 destination columns,
/// transcribed from the three parts of the table.
///
/// Rows: `Rn`, `#.B/.W`, `#.L`, `(An)`, `(An)+`, `-(An)`, `(d16,An)`,
/// `(xxx).W`, `(xxx).L`, `(d8,An,Xn)`, `(d16,An,Xn)`, `(B)`, `(d16,B)`,
/// `(d32,B)`, then the nine memory-indirect modes. Columns: `An`, `Dn`,
/// `(An)`, `(An)+`, `-(An)`, `(d16,An)`, `(xxx).W`, `(xxx).L`, `(d8,An,Xn)`,
/// `(d16,An,Xn)`, `(B)`, `(d16,B)`, `(d32,B)`, then the nine memory-indirect
/// modes.
const MOVE: [[u8; 22]; 23] = [
    [
        2, 2, 4, 4, 5, 5, 4, 6, 7, 9, 8, 10, 14, 12, 14, 15, 14, 16, 17, 18, 20, 21,
    ],
    [
        4, 4, 6, 6, 7, 7, 6, 8, 7, 9, 8, 10, 14, 12, 14, 15, 14, 16, 17, 18, 20, 21,
    ],
    [
        6, 6, 8, 8, 9, 9, 8, 10, 9, 11, 10, 12, 16, 14, 16, 17, 16, 18, 19, 20, 22, 23,
    ],
    [
        6, 6, 7, 7, 7, 7, 7, 9, 9, 11, 10, 12, 16, 14, 16, 17, 16, 18, 19, 20, 22, 23,
    ],
    [
        6, 6, 7, 7, 7, 7, 7, 9, 9, 11, 10, 12, 16, 14, 16, 17, 16, 18, 19, 20, 22, 23,
    ],
    [
        7, 7, 8, 8, 8, 8, 8, 10, 10, 12, 11, 13, 17, 15, 17, 18, 17, 19, 20, 21, 23, 24,
    ],
    [
        7, 7, 8, 8, 8, 8, 8, 10, 10, 12, 11, 13, 17, 15, 17, 18, 17, 19, 20, 21, 23, 24,
    ],
    [
        6, 6, 7, 7, 7, 7, 7, 9, 9, 11, 10, 12, 16, 14, 16, 17, 16, 18, 19, 20, 22, 23,
    ],
    [
        6, 6, 7, 7, 7, 7, 7, 9, 9, 11, 10, 12, 16, 14, 16, 17, 16, 18, 19, 20, 22, 23,
    ],
    [
        9, 9, 10, 10, 10, 10, 10, 12, 12, 14, 13, 15, 19, 17, 19, 20, 19, 21, 22, 23, 25, 26,
    ],
    [
        9, 9, 10, 10, 10, 10, 10, 12, 12, 14, 13, 15, 19, 17, 19, 20, 19, 21, 22, 23, 25, 26,
    ],
    [
        9, 9, 10, 10, 10, 10, 10, 12, 12, 14, 13, 15, 19, 17, 19, 20, 19, 21, 22, 23, 25, 26,
    ],
    [
        11, 11, 12, 12, 12, 12, 12, 14, 14, 16, 15, 17, 21, 19, 21, 22, 21, 23, 24, 25, 27, 28,
    ],
    [
        15, 15, 16, 16, 16, 16, 16, 18, 18, 20, 19, 21, 25, 23, 25, 26, 25, 27, 28, 29, 31, 32,
    ],
    [
        14, 14, 15, 15, 15, 15, 15, 17, 17, 19, 18, 20, 24, 22, 24, 25, 24, 26, 27, 28, 30, 31,
    ],
    [
        16, 16, 17, 17, 17, 17, 17, 19, 19, 21, 20, 22, 26, 24, 26, 27, 26, 28, 29, 30, 32, 33,
    ],
    [
        16, 16, 17, 17, 17, 17, 17, 19, 19, 21, 20, 22, 26, 24, 26, 27, 26, 28, 29, 30, 32, 33,
    ],
    [
        16, 16, 17, 17, 17, 17, 17, 19, 19, 21, 20, 22, 26, 24, 26, 27, 26, 28, 29, 30, 32, 33,
    ],
    [
        18, 18, 19, 19, 19, 19, 19, 21, 21, 23, 22, 24, 28, 26, 28, 29, 28, 30, 31, 32, 34, 35,
    ],
    [
        18, 18, 19, 19, 19, 19, 19, 21, 21, 23, 22, 24, 28, 26, 28, 29, 28, 30, 31, 32, 34, 35,
    ],
    [
        20, 20, 21, 21, 21, 21, 21, 23, 23, 25, 24, 26, 30, 28, 30, 31, 30, 32, 33, 34, 36, 37,
    ],
    [
        22, 22, 23, 23, 23, 23, 23, 25, 25, 27, 26, 28, 32, 30, 32, 33, 32, 34, 35, 36, 38, 39,
    ],
    [
        22, 22, 23, 23, 23, 23, 23, 25, 25, 27, 26, 28, 32, 30, 32, 33, 32, 34, 35, 36, 38, 39,
    ],
];

/// The `MOVE` table's row for a source class.
const fn move_row(class: EaClass) -> usize {
    match class {
        DN | AN => 0,
        IMM_W => 1,
        IMM_L => 2,
        c if c < IMM_W => c as usize + 1,
        c => c as usize - 1,
    }
}

/// The `MOVE` table's column for a destination class.
const fn move_column(class: EaClass) -> usize {
    match class {
        AN => 0,
        DN => 1,
        c if c < IMM_W => c as usize,
        c => c as usize - 2,
    }
}

/// What an instruction did that its time depends on, gathered as it ran.
#[derive(Debug, Clone, Copy, Default)]
pub(super) struct Facts {
    /// The effective addresses resolved, in order: the source, then the
    /// destination.
    pub eas: [EaClass; 2],
    /// How many of `eas` are set.
    pub ea_count: u8,
    /// A branch taken, a trap condition true, a `CAS` that matched.
    pub taken: bool,
    /// A `DBcc` whose count ran out.
    pub expired: bool,
    /// A bit field that needed five bytes.
    pub five_bytes: bool,
    /// The registers a `MOVEM` moved.
    pub registers: u32,
    /// A signed `DIVS.L`, which has its own row.
    pub signed: bool,
    /// A `MOVES` to memory rather than from it.
    pub to_memory: bool,
}

impl Facts {
    /// Note an effective address as it is resolved.
    pub(super) fn ea(&mut self, class: EaClass) {
        if let Some(slot) = self.eas.get_mut(self.ea_count as usize) {
            *slot = class;
            self.ea_count += 1;
        }
    }

    /// The first effective address resolved, or `Dn` if none was.
    fn first(&self) -> usize {
        if self.ea_count > 0 {
            self.eas[0] as usize
        } else {
            DN as usize
        }
    }

    /// The last effective address resolved.
    fn last(&self) -> usize {
        match self.ea_count {
            0 => DN as usize,
            n => self.eas[n as usize - 1] as usize,
        }
    }
}

fn fetch(class: usize) -> u32 {
    u32::from(FETCH[class])
}

fn fetch_imm(class: usize, long: bool) -> u32 {
    let (w, l) = FETCH_IMM[class];
    u32::from(if long { l } else { w })
}

fn calc(class: usize) -> u32 {
    u32::from(CALC[class])
}

fn calc_imm(class: usize, long: bool) -> u32 {
    let (w, l) = CALC_IMM[class];
    u32::from(if long { l } else { w })
}

/// Whether a class names a register rather than memory.
const fn register(class: usize) -> bool {
    class <= AN as usize
}

/// The cache-case time of an instruction that ran to completion (MC68020UM
/// §8.2.6–§8.2.18). `RTE` and exceptions are charged where they happen, not
/// here.
#[allow(clippy::too_many_lines)]
pub(super) fn instruction(insn: Insn, opcode: u16, size: Size, f: &Facts) -> u32 {
    let long = size == Size::Long;
    let src = f.first();
    let dst = f.last();
    match insn.op {
        Op::Move | Op::Movea => {
            let to = if insn.op == Op::Movea { AN } else { f.eas[1] };
            u32::from(MOVE[move_row(f.eas[0])][move_column(to)])
        }
        Op::Moveq => 2,
        Op::Add | Op::Sub | Op::And | Op::Or | Op::Eor | Op::Cmp => {
            if insn.dst == Arg::DnHi || register(dst) {
                // `op EA,Dn`, and `EOR Dn,Dn`.
                2 + fetch(src)
            } else {
                4 + fetch(dst)
            }
        }
        Op::Adda | Op::Suba => 2 + fetch(src),
        Op::Cmpa => 4 + fetch(src),
        Op::Addq | Op::Subq => {
            if register(dst) {
                2
            } else {
                4 + fetch(dst)
            }
        }
        Op::Addi | Op::Subi | Op::Andi | Op::Ori | Op::Eori => {
            if register(dst) {
                2 + fetch_imm(DN as usize, long)
            } else {
                4 + fetch_imm(dst, long)
            }
        }
        Op::Cmpi => 2 + fetch_imm(dst, long),
        Op::Abcd | Op::Sbcd => {
            if opcode & 8 == 0 {
                4
            } else {
                16
            }
        }
        Op::Addx | Op::Subx => {
            if opcode & 8 == 0 {
                2
            } else {
                12
            }
        }
        Op::Cmpm => 9,
        Op::Pack => {
            if opcode & 8 == 0 {
                6
            } else {
                13
            }
        }
        Op::Unpk => {
            if opcode & 8 == 0 {
                8
            } else {
                13
            }
        }
        Op::Clr => {
            if register(dst) {
                2
            } else {
                4 + calc(dst)
            }
        }
        Op::Neg | Op::Negx | Op::Not => {
            if register(dst) {
                2
            } else {
                4 + fetch(dst)
            }
        }
        Op::Nbcd => 6 + fetch(dst),
        Op::Ext | Op::Extb | Op::Swap => 4,
        Op::Exg | Op::MoveUsp | Op::Nop => 2,
        Op::Scc => {
            if register(dst) {
                4
            } else {
                6 + calc(dst)
            }
        }
        Op::Tas => {
            if register(dst) {
                4
            } else {
                12 + calc(dst)
            }
        }
        Op::Tst => 2 + fetch(src),
        Op::Asl | Op::Asr | Op::Lsl | Op::Lsr | Op::Rol | Op::Ror | Op::Roxl | Op::Roxr => {
            let memory = insn.dst == Arg::Ea;
            let dynamic = opcode & 0x0020 != 0;
            match (insn.op, memory) {
                (Op::Lsl | Op::Lsr, false) => {
                    if dynamic {
                        6
                    } else {
                        4
                    }
                }
                (Op::Lsl | Op::Lsr | Op::Asr | Op::Roxl | Op::Roxr, true) => 5 + fetch(dst),
                (Op::Asl, false) | (Op::Rol | Op::Ror, false) => 8,
                (Op::Asr, false) => 6,
                (Op::Asl, true) => 6 + fetch(dst),
                (Op::Rol | Op::Ror, true) => 7 + fetch(dst),
                _ => 12,
            }
        }
        Op::Btst | Op::Bchg | Op::Bclr | Op::Bset => {
            if register(dst) {
                4
            } else if insn.src == Arg::BitNumber {
                4 + fetch_imm(dst, false)
            } else {
                4 + fetch(dst)
            }
        }
        Op::Bftst
        | Op::Bfchg
        | Op::Bfclr
        | Op::Bfset
        | Op::Bfexts
        | Op::Bfextu
        | Op::Bfins
        | Op::Bfffo => {
            let (reg, short, five) = match insn.op {
                Op::Bftst => (6, 11, 15),
                Op::Bfchg | Op::Bfclr | Op::Bfset => (12, 16, 24),
                Op::Bfexts | Op::Bfextu => (8, 13, 18),
                Op::Bfins => (10, 14, 20),
                _ => (18, 24, 32),
            };
            if register(src) {
                reg
            } else {
                (if f.five_bytes { five } else { short }) + calc_imm(src, false)
            }
        }
        Op::Mulu | Op::Muls => 27 + fetch(src),
        Op::Mull => 43 + fetch_imm(src, false),
        Op::Divu => 44 + fetch(src),
        Op::Divs => 56 + fetch(src),
        Op::Divl => {
            // The signed and unsigned forms have their own rows.
            (if f.signed { 90 } else { 78 }) + fetch_imm(src, false)
        }
        Op::Chk => 8 + fetch(src),
        Op::Cmp2 => 18 + fetch_imm(src, false),
        Op::Lea => 2 + calc(src),
        Op::Pea => 5 + calc(src),
        Op::Jmp => 4 + u32::from(JUMP[src]),
        Op::Jsr => 5 + u32::from(JUMP[src]),
        Op::Bra | Op::Bcc => {
            if f.taken || insn.op == Op::Bra {
                6
            } else if insn.src == Arg::Disp8 && opcode & 0xff != 0 {
                4
            } else {
                6
            }
        }
        Op::Bsr => 7,
        Op::Dbcc => {
            if f.expired {
                10
            } else {
                6
            }
        }
        Op::Rts | Op::Rtd => 10,
        Op::Rtr => 14,
        // Charged frame by frame as it reads them.
        Op::Rte => 0,
        Op::Link => {
            if long {
                6
            } else {
                5
            }
        }
        Op::Unlk => 6,
        Op::Reset => 518,
        Op::Stop => 8,
        Op::Movec => {
            if insn.dst == Arg::Ctrl {
                12
            } else {
                6
            }
        }
        Op::MoveFromSr | Op::MoveFromCcr => {
            if register(dst) {
                4
            } else {
                5 + calc(dst)
            }
        }
        Op::MoveToSr => 8 + fetch(src),
        Op::MoveToCcr => 4 + fetch(src),
        Op::OriToCcr
        | Op::AndiToCcr
        | Op::EoriToCcr
        | Op::OriToSr
        | Op::AndiToSr
        | Op::EoriToSr => 12,
        Op::Movem => {
            let n = f.registers;
            if insn.dst == Arg::Ea {
                4 + 3 * n + calc_imm(src, false)
            } else {
                8 + 4 * n + calc_imm(src, false)
            }
        }
        Op::Movep => match (insn.dst == Arg::MovepEa, long) {
            (true, false) => 11,
            (true, true) => 17,
            (false, false) => 12,
            (false, true) => 18,
        },
        Op::Moves => (if f.to_memory { 5 } else { 7 }) + calc_imm(src, long),
        Op::Cas => (if f.taken { 15 } else { 12 }) + calc_imm(src, false),
        Op::Cas2 => {
            if f.taken {
                25
            } else {
                22
            }
        }
        Op::Callm => 30 + fetch_imm(src, false),
        Op::Rtm => 19,
        // The memory management instructions. MC68030UM §11 tables them;
        // this charges the effective-address calculation the operand needs
        // plus eight clocks for the unit itself, and a table search's own
        // bus cycles are added where they happen (`Exec::search_cycles`).
        // Recorded in the conformance ledger as the approximation it is.
        Op::Pgen => calc(f.eas[0] as usize) + 8,
        // The coprocessor's instructions. MC68020UM §8.2 has no rows for
        // them — a coprocessor's execution time is the coprocessor's, and
        // M68881UM §8 tables it separately in *its* clocks, which need not be
        // the main processor's. What is charged is the main processor's own
        // work: the effective-address time the operand needs plus the
        // transfer. In the conformance ledger.
        Op::Fpgen => fetch(f.eas[0] as usize) + 4 * f.registers.max(1),
        Op::Fbcc | Op::Fdbcc => {
            if f.taken {
                10
            } else {
                6
            }
        }
        Op::Fscc => fetch(f.eas[0] as usize) + 6,
        Op::Ftrapcc => 6,
        Op::Fsave | Op::Frestore => calc(f.eas[0] as usize) + 8,
        Op::Trapcc => match opcode & 7 {
            2 => 6,
            3 => 8,
            _ => 4,
        },
        Op::Trapv => 4,
        Op::Bkpt => 10,
        Op::Trap | Op::Illegal | Op::LineA | Op::LineF => 0,
        // The 68040's own instructions, and the one place this file charges
        // an M68040UM number rather than an MC68020UM one — there is no
        // 68020 row to borrow, because there is no 68020 instruction.
        //
        // `MOVE16` is eight long-word bus cycles here rather than the two
        // bursts hardware makes; M68040UM §10 gives no row for it at all, so
        // what is charged is the accesses this core actually drives, counted
        // where they happen, plus nothing.
        Op::Move16 => 0,
        // M68040UM Tables 10-3 and 10-4, with `Idle` zero — nothing is
        // pending, because this core has no write-back pipeline — and the
        // CPUSH *best* case, which is the manual's "cache containing no dirty
        // entries" and is the only state this core's cache is ever in.
        Op::Cinvl | Op::Cinva => 9,
        Op::Cinvp => 266,
        Op::Cpushl => 6,
        Op::Cpushp | Op::Cpusha => 267,
    }
}

/// The cache-case time an instruction spent before the exception it raised
/// itself; the exception's own time is added when it is taken. Only the
/// instructions whose trap time the table does not already include have any.
pub(super) fn before_exception(insn: Insn, f: &Facts) -> u32 {
    match insn.op {
        Op::Chk | Op::Divu | Op::Divs => fetch(f.first()),
        Op::Cmp2 | Op::Divl => fetch_imm(f.first(), false),
        Op::Bkpt => 10,
        _ => 0,
    }
}

/// An exception's cache-case time (MC68020UM §8.2.17, §8.2.18), by the frame
/// it builds: format $0 20, format $2 25, an interrupt 26 — 41 with the
/// throwaway frame — and the short and long bus fault frames 43 and 79.
pub(super) const fn exception(format: u8, interrupt: bool, throwaway: bool) -> u32 {
    match (format, interrupt, throwaway) {
        (_, true, true) => 41,
        (_, true, false) => 26,
        (0x2, _, _) => 25,
        (0xa, _, _) => 43,
        (0xb, _, _) => 79,
        _ => 20,
    }
}

/// What `RTE` spends on one frame (§8.2.18): 21 for a normal or six-word
/// frame, 16 for a throwaway frame and the next one's on top, 43 and 92 for
/// the short and long bus fault frames.
pub(super) const fn rte(format: u16) -> u32 {
    match format {
        0x1 => 16,
        0xa => 43,
        0xb => 92,
        _ => 21,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_move_table_rows_and_columns_line_up_with_the_manual() {
        // Spot checks against MC68020UM §8.2.6, cache case.
        let row = |c| move_row(c);
        let col = |c| move_column(c);
        // Rn to Dn: 2(0/0/0); Rn to (xxx).L: 6(0/0/1).
        assert_eq!(MOVE[row(DN)][col(DN)], 2);
        assert_eq!(MOVE[row(DN)][col(7)], 6);
        // (An) to (An): 7(1/0/1). -(An) to Dn: 7(1/0/0).
        assert_eq!(MOVE[row(2)][col(2)], 7);
        assert_eq!(MOVE[row(4)][col(DN)], 7);
        // #.L to An: 6. (d8,An,Xn) to (d8,An,Xn): 12.
        assert_eq!(MOVE[row(IMM_L)][col(AN)], 6);
        assert_eq!(MOVE[row(INDEX8)][col(INDEX8)], 12);
        // ([d32,B],I,d32) to ([d32,B],I,d32): 39, the table's last cell.
        assert_eq!(MOVE[row(INDIRECT + 8)][col(INDIRECT + 8)], 39);
        // (d16,B) source into (B): 15.
        assert_eq!(MOVE[row(BASE16)][col(BASE)], 15);
    }

    #[test]
    fn the_manuals_worked_example_adds_up() {
        // MC68020UM §8.2: MULU.L (D7),D1:D2 is 2 + 43; BFCLR $6000{0:8} is
        // 5 + 16 — but that 5 is the *fetch* immediate row, where the bit
        // field footnote says calculate; DIVS.L #$10000,D3:D4 is 6 + 90.
        assert_eq!(fetch_imm(DN as usize, false) + 43, 45);
        assert_eq!(fetch_imm(IMM_L as usize, false) + 90, 96);
    }
}
