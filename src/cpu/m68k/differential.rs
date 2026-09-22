//! The differential harness: the m68k lifter against the m68k interpreter,
//! forever.
//!
//! CLAUDE.md, "CPU cores": *the IR frontend comes later and is differentially
//! tested against the interpreter forever. **The interpreter is the oracle.***
//! This module is that harness.
//!
//! # The comparison
//!
//! One program, two cores built the same way on two identical address spaces,
//! and everything either of them can be seen to do:
//!
//! | | oracle | subject |
//! | --- | --- | --- |
//! | engine | `engine = "interp"` | `engine = "ir"` |
//! | registers | `D0`-`D7`, `A0`-`A7`, `USP`, `SSP` | the same |
//! | `SR` | including the **X** bit that `CMP` leaves alone | the same |
//! | the program counter | `Regs::pc` | the same |
//! | **the prefetch queue** | `Regs::prefetch`, both words | the same |
//! | cycles | `M68k::cycles` | the same |
//! | the scheduler's debt | `M68k::cycle_debt` | the same |
//! | memory | every byte of RAM | every byte of RAM |
//! | halted / stopped | `M68k::is_halted`, `is_stopped` | the same |
//!
//! Every column catches a different class of bug and only the first is
//! obvious. A miscounted [`Opcode::CHARGE`](crate::ir::Opcode::CHARGE) is
//! invisible in the registers and fails the phase-5 state-hash gate a million
//! cycles later (`src/ir/mod.rs`, decision 2). A store lifted with the wrong
//! width writes the right register and the wrong memory. A missed **X** write
//! shows up nowhere until the third `ADDX` of a multi-precision add. And the
//! **prefetch queue** is the column no other core in this tree has: it is
//! architectural state on a 68000, it moves once per instruction *word*, and a
//! frontend that charged the fetches without performing them would pass every
//! other column and fail this one.
//!
//! # Why this compares whole cores rather than one block
//!
//! `cpu::riscv::differential` lifts a block, runs it against its own
//! [`IrHost`](crate::ir::IrHost), and compares that to `Hart::step`. That
//! shape needs a second implementation of the memory path — the harness's own
//! host — and on this core the memory path is where most of the difficulty
//! lives: four cycles per bus cycle, an address error before the cycle rather
//! than a fault during it, a long access that is two word cycles, and the
//! *order* of those cycles. A second implementation of that would be a second
//! thing to get wrong, and a bug in it would look like a frontend bug.
//!
//! So the subject here is the **engine**, through the ordinary public API:
//! [`M68k`] with `engine = "ir"` against [`M68k`] with `engine = "interp"`.
//! That costs the ability to compare a single block in isolation and buys
//! three things the block-level shape cannot reach at all:
//!
//! * **the fallback path**, which is most of the instruction set and has to
//!   leave the guest in exactly the state a block would have;
//! * **the fault path**, which on this core hands the instruction back to the
//!   interpreter and must not pay for it twice;
//! * **the cache**, including a block whose bytes the guest has rewritten.
//!
//! The unit of comparison is one call to [`M68k::step`], which on the subject
//! is one *block*. [`engine::Stats::steps`] is what says how many interpreter
//! steps that block was worth, so the oracle is stepped exactly that far and
//! the two are compared with both standing at an instruction boundary.
//!
//! # The corpus
//!
//! Three sources, and the third is the one that matters:
//!
//! * [`Case`]s written by hand, in the tests below — one per rule worth
//!   naming.
//! * [`synthesize`], which turns a pair of numbers into one encoding from
//!   *anywhere* in the instruction set, for a seeded pseudo-random stream or a
//!   fuzzer's bytes.
//! * **every opcode word there is.** A 68000 opcode is sixteen bits, so the
//!   whole decode space is 65 536 cases and [`sweep`] runs them. Nothing is
//!   skipped: an encoding the frontend declines is *still* compared, because
//!   the fallback has to be right too, and an illegal one is compared through
//!   its exception.
//!
//! # What this harness does not cover
//!
//! * **Interrupts and reset pulses**, which arrive from outside the machine
//!   and are the record/replay seam's business rather than the frontend's. The
//!   engine refuses to run a block while one is pending ([`engine`]'s
//!   `liftable`), and `tests/` is where a board asserts that.
//! * **Any model but a 68000**, because [`lift`] refuses one and
//!   `from_props` refuses the configuration. A 68010 or 68020 with
//!   `engine = "ir"` is a configuration error, not a divergence.
//! * **A second bus master rewriting the code a block is running**, which is
//!   the one prefetch skew `engine`'s module docs record.

use alloc::format;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::space::{AddressSpace, RamStore, Region};
use crate::core::value::Endian;

use super::isa::Model;
use super::{Config, Engine, M68k, engine, lift};

/// Where a case's exception vector table lives: address zero, because a 68000
/// has no vector base register and cannot move it (MC68000UM §6.1).
pub const VECTORS: u32 = 0;

/// Where a case's program is loaded.
///
/// Page-aligned and in a [`lift::WINDOW`] of its own, so the data window a
/// case's stores are aimed at cannot be mistaken for self-modifying code.
pub const CODE: u32 = 0x1000;

/// Where the data window starts. A separate window from [`CODE`].
pub const DATA: u32 = 0x2000;

/// The initial stack pointer, inside the data window and well away from where
/// a case's loads and stores land.
pub const STACK: u32 = 0x3000;

/// How much RAM a case gets.
pub const RAM_SIZE: u32 = 0x4000;

/// One differential case: a program and the registers it starts with.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Case {
    /// The instruction words, loaded big-endian at [`CODE`].
    pub program: Vec<u16>,
    /// The initial data registers.
    pub d: [u32; 8],
    /// The initial address registers. `a[7]` is the supervisor stack pointer,
    /// which the reset sequence loads from vector 0 and this overrides.
    pub a: [u32; 8],
    /// The initial status register.
    ///
    /// Supervisor state with interrupts masked, as reset leaves it, plus
    /// whatever condition codes a case wants — the **X** bit especially, since
    /// `ADDX`, `SUBX`, `NEGX`, `ROXL` and `ROXR` all read it.
    pub sr: u16,
    /// How many units of the subject's engine to run.
    ///
    /// One call to [`M68k::step`] each, which on the subject is one block.
    pub units: usize,
}

impl Case {
    /// A case that runs `program` from a zeroed register file in supervisor
    /// state with interrupts masked.
    #[must_use]
    pub fn new(program: Vec<u16>) -> Case {
        let mut a = [0u32; 8];
        a[7] = STACK;
        Case {
            program,
            d: [0; 8],
            a,
            sr: super::flags::S | super::flags::IPL,
            units: 4,
        }
    }

    /// The same case with `A1`..`A4` pointing into the data window, spread so
    /// that a small signed displacement from any of them stays inside RAM.
    ///
    /// The companion to [`synthesize`], which takes a memory operand's base
    /// from exactly those four registers. `A1` is deliberately **odd**, so the
    /// address-error path is reachable without the displacement having to
    /// supply the misalignment — that is where the tick accounting and the
    /// fault path part company if anything is wrong.
    #[must_use]
    pub fn seeded(program: Vec<u16>) -> Case {
        let mut case = Case::new(program);
        case.a[1] = DATA + 0x101;
        case.a[2] = DATA + 0x400;
        case.a[3] = DATA + 0x800;
        case.a[4] = DATA + 0xc00;
        case.d[0] = 0x1234_5678;
        case.d[1] = 0xffff_ffff;
        case.d[2] = 0x0000_0001;
        case.d[3] = 0x8000_0000;
        case.d[4] = 0x0000_00ff;
        case.d[5] = 0x7fff_ffff;
        case.d[6] = 0x0000_8000;
        case.d[7] = 0x0000_0000;
        case
    }

    /// The same case with `X` set in `SR`.
    #[must_use]
    pub fn with_extend(mut self) -> Case {
        self.sr |= super::flags::X;
        self
    }

    /// The same case with `SR`'s condition codes set to `ccr`.
    #[must_use]
    pub fn with_ccr(mut self, ccr: u16) -> Case {
        self.sr = (self.sr & !super::flags::CCR) | (ccr & super::flags::CCR);
        self
    }

    /// The same case run for `units` units.
    #[must_use]
    pub fn with_units(mut self, units: usize) -> Case {
        self.units = units;
        self
    }

    /// The same case with `Dn` starting at `value`.
    #[must_use]
    pub fn with_d(mut self, n: usize, value: u32) -> Case {
        if n < 8 {
            self.d[n] = value;
        }
        self
    }

    /// The same case with `An` starting at `value`.
    #[must_use]
    pub fn with_a(mut self, n: usize, value: u32) -> Case {
        if n < 8 {
            self.a[n] = value;
        }
        self
    }
}

/// What comparing one case established.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Verdict {
    /// They agreed on every column, for every unit.
    Agreed {
        /// How many units of the subject's engine ran.
        units: usize,
        /// How many interpreter steps those units were worth.
        steps: u64,
        /// How many of those steps ran inside a lifted block.
        lifted: u64,
        /// How many cycles both engines charged.
        cycles: u64,
    },
}

/// The oracle and the subject disagreed.
///
/// Carries the program disassembled, because a fuzzer's finding is useless
/// without the bytes that produced it and the words are not readable.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Divergence {
    /// Which column disagreed, and how.
    pub what: String,
    /// The program, disassembled.
    pub program: String,
}

impl core::fmt::Display for Divergence {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}\n{}", self.what, self.program)
    }
}

/// The address space a case runs in, and the RAM behind it.
///
/// Public because `engine`'s own tests assert the *shape* of a run — that no
/// block runs with an interrupt pending, that a budget leaves a block
/// part-way through — and they need a case's world without a second core
/// beside it. A test that wants agreement wants [`compare`] instead.
#[must_use]
pub fn space_for(case: &Case) -> (Arc<AddressSpace>, Arc<RamStore>) {
    machine(case)
}

/// Build the address space a case runs in, and the RAM behind it.
///
/// Big-endian and twenty-four bits, which is what a 68000 board is: the core
/// makes word accesses and the *space* decides their byte order, so a
/// little-endian space would swap every operand and no frontend could fix it.
fn machine(case: &Case) -> (Arc<AddressSpace>, Arc<RamStore>) {
    let ram = Arc::new(RamStore::new(u64::from(RAM_SIZE)));
    let space = AddressSpace::new("m68k-diff", 24).with_endian(Endian::Big);
    {
        let mut topo = space.topology();
        // The *region* carries the byte order as well as the space: a 68000
        // board is big-endian end to end, and a little-endian region would
        // swap every operand with no frontend able to fix it.
        topo.map(
            Region::ram("ram", Arc::clone(&ram)).with_endian(Endian::Big),
            0,
        )
        .expect("the case's RAM maps at zero");
    }

    // Vector 0 is the initial supervisor stack pointer and vector 1 the
    // initial program counter: "a reset reads them in that order"
    // (MC68000UM §6.2.6). Every other vector points at a `STOP #$2700`, so a
    // case whose program raises an exception stops the core rather than
    // running off into whatever happens to be in RAM — and "stopped" is a
    // column this harness compares.
    let halt = CODE + 0x800;
    write_word(&ram, halt, 0x4e72);
    write_word(&ram, halt + 2, 0x2700);
    for vector in 0..256u32 {
        let at = VECTORS + 4 * vector;
        match vector {
            0 => write_long(&ram, at, STACK),
            1 => write_long(&ram, at, CODE),
            _ => write_long(&ram, at, halt),
        }
    }
    for (i, word) in case.program.iter().enumerate() {
        write_word(&ram, CODE + 2 * i as u32, *word);
    }
    (Arc::new(space), ram)
}

fn write_word(ram: &RamStore, at: u32, word: u16) {
    // Byte at a time and high byte first, which is what a big-endian region
    // holds — the same way `cpu::m68k::tests`'s `poke_word` does it.
    ram.write_u8(u64::from(at), (word >> 8) as u8)
        .expect("the case's RAM is writable");
    ram.write_u8(u64::from(at) + 1, word as u8)
        .expect("the case's RAM is writable");
}

/// The word at `at`, read out of a case's RAM in the order the region holds
/// it.
fn read_word(ram: &RamStore, at: u64) -> u16 {
    (u16::from(ram.read_u8(at).unwrap_or(0)) << 8) | u16::from(ram.read_u8(at + 1).unwrap_or(0))
}

fn write_long(ram: &RamStore, at: u32, value: u32) {
    write_word(ram, at, (value >> 16) as u16);
    write_word(ram, at + 2, value as u16);
}

/// Build one core on `space`, past its reset sequence, with `case`'s
/// registers.
fn core(case: &Case, space: Arc<AddressSpace>, engine: Engine) -> M68k {
    let cpu = M68k::new(Config::default().with_model(Model::M68000)).with_engine(engine);
    cpu.attach_space(space);
    // The reset sequence is one step and it is what fills the prefetch queue,
    // so the registers are set *after* it — otherwise `PC` and `prefetch`
    // would be whatever the case asked for rather than a consistent pair.
    cpu.step();
    let mut regs = cpu.regs();
    regs.d = case.d;
    regs.a = case.a;
    regs.pc = CODE;
    regs.sr = case.sr;
    regs.ssp = case.a[7];
    // `prefetch` has to hold the words at `pc` and `pc + 2`, which is the
    // queue's whole invariant (`exec.rs`); a pair that does not is a state no
    // hardware can be in and neither engine would know what to do with.
    regs.prefetch = [
        case.program.first().copied().unwrap_or(0),
        case.program.get(1).copied().unwrap_or(0),
    ];
    cpu.set_regs(regs);
    cpu
}

/// Compare the translated engine against the interpreter for one case.
///
/// # Errors
///
/// [`Divergence`] naming the first column that disagreed, with the program
/// disassembled.
pub fn compare(case: &Case) -> Result<Verdict, Divergence> {
    let (oracle_space, oracle_ram) = machine(case);
    let (subject_space, subject_ram) = machine(case);
    let oracle = core(case, oracle_space, Engine::Interp);
    let subject = core(case, subject_space, Engine::Ir);

    // Before anything runs: the two cores must start from the same place, or
    // every column after this compares two different guests.
    agree(case, usize::MAX, &oracle, &subject)?;

    let mut steps = 0u64;
    let mut lifted = 0u64;
    for unit in 0..case.units {
        let before = subject.ir_stats().unwrap_or_default();
        let subject_cycles = subject.step();
        let after = subject.ir_stats().unwrap_or_default();
        let unit_steps = after.steps.wrapping_sub(before.steps);
        lifted += after.retired.wrapping_sub(before.retired);
        steps += unit_steps;

        // Zero would be a spin: a unit must retire at least one interpreter
        // step, or the run loop cannot make progress. The one legal zero is a
        // halted core, and then both engines report it.
        if unit_steps == 0 {
            if subject.is_halted() {
                break;
            }
            return Err(diverged(
                case,
                format!(
                    "unit {unit} retired nothing and the core is not halted \
                     (stats {after:?})"
                ),
            ));
        }
        let mut oracle_cycles = 0u64;
        for _ in 0..unit_steps {
            oracle_cycles += oracle.step();
        }
        if oracle_cycles != subject_cycles {
            return Err(diverged(
                case,
                format!(
                    "unit {unit}: the interpreter charged {oracle_cycles} cycles over \
                     {unit_steps} instructions and the lifted engine charged \
                     {subject_cycles}"
                ),
            ));
        }
        agree(case, unit, &oracle, &subject)?;
        memory(case, &oracle_ram, &subject_ram, unit)?;
        if subject.is_halted() {
            break;
        }
    }

    Ok(Verdict::Agreed {
        units: case.units,
        steps,
        lifted,
        cycles: subject.cycles(),
    })
}

/// Compare every architectural column of two cores.
fn agree(case: &Case, unit: usize, oracle: &M68k, subject: &M68k) -> Result<(), Divergence> {
    let a = oracle.regs();
    let b = subject.regs();
    let at = |what: &str, want: &dyn core::fmt::Debug, got: &dyn core::fmt::Debug| {
        diverged(
            case,
            format!("unit {unit}: {what}: the interpreter has {want:?}, the block {got:?}"),
        )
    };
    for n in 0..8 {
        if a.d[n] != b.d[n] {
            return Err(at(&format!("D{n}"), &a.d[n], &b.d[n]));
        }
        if a.a[n] != b.a[n] {
            return Err(at(&format!("A{n}"), &a.a[n], &b.a[n]));
        }
    }
    if a.sr != b.sr {
        return Err(at(
            &format!("SR (ccr {} vs {})", ccr_text(a.sr), ccr_text(b.sr)),
            &a.sr,
            &b.sr,
        ));
    }
    if a.pc != b.pc {
        return Err(at("PC", &a.pc, &b.pc));
    }
    if a.prefetch != b.prefetch {
        return Err(at("the prefetch queue", &a.prefetch, &b.prefetch));
    }
    if a.usp != b.usp {
        return Err(at("USP", &a.usp, &b.usp));
    }
    if a.ssp != b.ssp {
        return Err(at("SSP", &a.ssp, &b.ssp));
    }
    if oracle.cycles() != subject.cycles() {
        return Err(at("the cycle counter", &oracle.cycles(), &subject.cycles()));
    }
    if oracle.cycle_debt() != subject.cycle_debt() {
        return Err(at(
            "the scheduler's debt",
            &oracle.cycle_debt(),
            &subject.cycle_debt(),
        ));
    }
    if oracle.is_halted() != subject.is_halted() {
        return Err(at("halted", &oracle.is_halted(), &subject.is_halted()));
    }
    if oracle.is_stopped() != subject.is_stopped() {
        return Err(at("stopped", &oracle.is_stopped(), &subject.is_stopped()));
    }
    if oracle.bus_faults() != subject.bus_faults() {
        return Err(at(
            "the bus-fault count and its address",
            &oracle.bus_faults(),
            &subject.bus_faults(),
        ));
    }
    Ok(())
}

/// The condition codes, spelled, for a divergence report.
fn ccr_text(sr: u16) -> String {
    let mut s = String::new();
    for (mask, name) in [
        (super::flags::X, 'X'),
        (super::flags::N, 'N'),
        (super::flags::Z, 'Z'),
        (super::flags::V, 'V'),
        (super::flags::C, 'C'),
    ] {
        s.push(if sr & mask != 0 { name } else { '-' });
    }
    s
}

/// Compare every byte of the two cores' RAM.
fn memory(
    case: &Case,
    oracle: &RamStore,
    subject: &RamStore,
    unit: usize,
) -> Result<(), Divergence> {
    for at in (0..u64::from(RAM_SIZE)).step_by(2) {
        let want = read_word(oracle, at);
        let got = read_word(subject, at);
        if want != got {
            return Err(diverged(
                case,
                format!(
                    "unit {unit}: memory at {at:#06x}: the interpreter has {want:#06x}, \
                     the block {got:#06x}"
                ),
            ));
        }
    }
    Ok(())
}

fn diverged(case: &Case, what: String) -> Divergence {
    Divergence {
        what,
        program: disassembly(case),
    }
}

/// A case's program, disassembled, for a divergence report.
fn disassembly(case: &Case) -> String {
    use core::fmt::Write as _;
    let mut out = String::new();
    let mut at = 0usize;
    while at < case.program.len() && at < 16 {
        let words = &case.program[at..];
        let d = super::disasm::disassemble_for(Model::M68000, CODE + 2 * at as u32, words);
        let _ = writeln!(
            out,
            "  {:06x}: {:04x}  {}",
            CODE + 2 * at as u32,
            words[0],
            d
        );
        at += (usize::from(d.len) / 2).max(1);
    }
    let _ = write!(
        out,
        "  d={:08x?}\n  a={:08x?}\n  sr={:04x}",
        case.d, case.a, case.sr
    );
    out
}

/// Encode one instruction, from anywhere in the instruction set.
///
/// `form` picks the shape and `fields` supplies the register numbers, the
/// effective-address field and the immediate, so a generator — a fuzzer's byte
/// stream, a seeded pseudo-random sequence — produces programs that *decode*
/// rather than programs that are all illegal. Both numbers are reduced, so
/// every pair encodes something.
///
/// The choices that are not arbitrary:
///
/// * A memory operand's base register is `A1`..`A4`, which [`Case::seeded`]
///   points into the data window. A generator that picked base registers
///   uniformly would fault on nearly every case and measure the fallback path
///   instead of the lifter.
/// * A displacement is small and signed, so an access lands near its base —
///   and often at an odd address, which is where the address-error path is.
/// * A branch displacement is small and word-granular, so a target is a real
///   instruction in the same window rather than a wild jump.
///
/// Returns the instruction's words, opcode first.
#[must_use]
#[allow(clippy::too_many_lines)]
pub fn synthesize(form: u32, fields: u32) -> Vec<u16> {
    let dn = (fields & 7) as u16;
    let dm = ((fields >> 3) & 7) as u16;
    // A base register the seeded case points into the data window.
    let base = 1 + ((fields >> 6) & 3) as u16;
    // An effective-address field, biased towards the modes that reach memory
    // through one of those base registers.
    let ea: u16 = match (fields >> 8) & 15 {
        0 => dm,                  // Dn
        1 => 0x08 | dm,           // An
        2 => 0x10 | base,         // (An)
        3 => 0x18 | base,         // (An)+
        4 => 0x20 | base,         // -(An)
        5 => 0x28 | base,         // (d16,An)
        6 => 0x30 | base,         // (d8,An,Xn)
        7 => 0x38,                // (xxx).W
        8 => 0x39,                // (xxx).L
        9 => 0x3a,                // (d16,PC)
        10 => 0x3b,               // (d8,PC,Xn)
        11 => 0x3c,               // #imm
        other => (other as u16) & 0x3f,
    };
    let size = ((fields >> 12) & 3) as u16;
    let disp = (((fields >> 14) & 0x1f) as i32 - 16) as u16;
    let imm = (fields >> 16) as u16;

    // The extension words an effective-address field needs, in order.
    let ext = |ea: u16, size: u16| -> Vec<u16> {
        match ea & 0x38 {
            0x28 => alloc::vec![disp],
            0x30 => alloc::vec![(dn << 12) | (disp & 0xff)],
            0x38 => match ea & 7 {
                0 => alloc::vec![(DATA as u16).wrapping_add(disp)],
                1 => alloc::vec![0, (DATA as u16).wrapping_add(disp)],
                2 => alloc::vec![disp],
                3 => alloc::vec![(dn << 12) | (disp & 0xff)],
                4 => {
                    if size == 2 {
                        alloc::vec![imm, imm.rotate_left(5)]
                    } else {
                        alloc::vec![imm]
                    }
                }
                _ => Vec::new(),
            },
            _ => Vec::new(),
        }
    };

    let one = |op: u16| alloc::vec![op];
    let with_ea = |op: u16, ea: u16, size: u16| {
        let mut v = alloc::vec![op];
        v.extend(ext(ea, size));
        v
    };

    match form % 44 {
        // MOVE and MOVEA, in all three sizes, memory to memory included.
        0 => {
            let dst_ea = match (fields >> 20) & 3 {
                0 => dn,
                1 => 0x10 | (1 + ((fields >> 22) & 3) as u16),
                2 => 0x18 | (1 + ((fields >> 22) & 3) as u16),
                _ => 0x20 | (1 + ((fields >> 22) & 3) as u16),
            };
            let szbits = match size {
                0 => 1u16,
                1 => 3,
                _ => 2,
            };
            let mut v = alloc::vec![
                (szbits << 12) | ((dst_ea & 7) << 9) | ((dst_ea & 0x38) << 3) | ea
            ];
            v.extend(ext(ea, size));
            v.extend(ext(dst_ea, size));
            v
        }
        1 => with_ea(0x2040 | (dn << 9) | ea, ea, 2), // MOVEA.L
        2 => with_ea(0x3040 | (dn << 9) | ea, ea, 1), // MOVEA.W
        3 => one(0x7000 | (dn << 9) | (imm & 0xff)),  // MOVEQ
        // the ALU groups, <ea> -> Dn and Dn -> <ea>
        4 => with_ea(0xd000 | (dn << 9) | (size << 6) | ea, ea, size), // ADD <ea>,Dn
        5 => with_ea(0xd100 | (dn << 9) | (size << 6) | ea, ea, size), // ADD Dn,<ea>
        6 => with_ea(0x9000 | (dn << 9) | (size << 6) | ea, ea, size), // SUB <ea>,Dn
        7 => with_ea(0x9100 | (dn << 9) | (size << 6) | ea, ea, size), // SUB Dn,<ea>
        8 => with_ea(0xc000 | (dn << 9) | (size << 6) | ea, ea, size), // AND <ea>,Dn
        9 => with_ea(0xc100 | (dn << 9) | (size << 6) | ea, ea, size), // AND Dn,<ea>
        10 => with_ea(0x8000 | (dn << 9) | (size << 6) | ea, ea, size), // OR <ea>,Dn
        11 => with_ea(0x8100 | (dn << 9) | (size << 6) | ea, ea, size), // OR Dn,<ea>
        12 => with_ea(0xb100 | (dn << 9) | (size << 6) | ea, ea, size), // EOR Dn,<ea>
        13 => with_ea(0xb000 | (dn << 9) | (size << 6) | ea, ea, size), // CMP <ea>,Dn
        // the immediate group: one or two words of immediate, then the ea
        14..=18 => {
            let base_op = match form % 44 {
                14 => 0x0600u16, // ADDI
                15 => 0x0400,    // SUBI
                16 => 0x0200,    // ANDI
                17 => 0x0000,    // ORI
                _ => 0x0a00,     // EORI
            };
            let mut v = alloc::vec![base_op | (size << 6) | ea];
            if size == 2 {
                v.push(imm);
                v.push(imm.rotate_left(3));
            } else {
                v.push(imm);
            }
            v.extend(ext(ea, size));
            v
        }
        19 => {
            // CMPI
            let mut v = alloc::vec![0x0c00 | (size << 6) | ea];
            if size == 2 {
                v.push(imm);
                v.push(imm.rotate_left(3));
            } else {
                v.push(imm);
            }
            v.extend(ext(ea, size));
            v
        }
        // ADDQ / SUBQ
        20 => with_ea(0x5000 | (dn << 9) | (size << 6) | ea, ea, size),
        21 => with_ea(0x5100 | (dn << 9) | (size << 6) | ea, ea, size),
        // ADDA / SUBA / CMPA
        22 => with_ea(0xd0c0 | (dn << 9) | ea, ea, 2),
        23 => with_ea(0x90c0 | (dn << 9) | ea, ea, 2),
        24 => with_ea(0xb0c0 | (dn << 9) | ea, ea, 2),
        25 => with_ea(0xd0c0 | (dn << 9) | ea, ea, 1),
        // ADDX / SUBX, register and memory forms
        26 => one(0xd100 | (dn << 9) | (size << 6) | ((fields >> 5) & 8) as u16 | dm),
        27 => one(0x9100 | (dn << 9) | (size << 6) | ((fields >> 5) & 8) as u16 | dm),
        // CMPM
        28 => one(0xb108 | (dn << 9) | (size << 6) | dm),
        // the unary group
        29 => with_ea(0x4200 | (size << 6) | ea, ea, size), // CLR
        30 => with_ea(0x4600 | (size << 6) | ea, ea, size), // NOT
        31 => with_ea(0x4400 | (size << 6) | ea, ea, size), // NEG
        32 => with_ea(0x4000 | (size << 6) | ea, ea, size), // NEGX
        33 => with_ea(0x4a00 | (size << 6) | ea, ea, size), // TST
        // EXT / SWAP / EXG
        34 => one(0x4880 | ((fields >> 4) & 0x40) as u16 | dn),
        35 => one(0x4840 | dn),
        36 => one(0xc140 | (dn << 9) | dm),
        // the shifts and rotates: a quick count, register and memory forms
        37 => one(0xe000 | (dn << 9) | (size << 6) | ((fields >> 3) & 0x38) as u16 | dm),
        38 => with_ea(0xe0c0 | ((fields >> 2) & 0x0700) as u16 | ea, ea, 1),
        // the bit instructions, static and dynamic
        39 => {
            let mut v = alloc::vec![0x0800 | (((fields >> 20) & 3) as u16) << 6 | ea];
            v.push(imm & 0x1f);
            v.extend(ext(ea, 0));
            v
        }
        40 => with_ea(
            0x0100 | (dn << 9) | (((fields >> 20) & 3) as u16) << 6 | ea,
            ea,
            0,
        ),
        // the branches: a byte displacement, and a word one
        41 => one(0x6000 | ((((fields >> 16) & 15) as u16) << 8) | ((disp << 1) & 0xfe)),
        42 => alloc::vec![0x6000 | ((((fields >> 16) & 15) as u16) << 8), disp << 1],
        // DBcc, Scc, LEA, JMP, MOVEM and RTS, rotated through by `fields`
        _ => match (fields >> 20) & 7 {
            0 => alloc::vec![0x50c8 | ((((fields >> 16) & 15) as u16) << 8) | dn, disp << 1],
            1 => with_ea(0x50c0 | ((((fields >> 16) & 15) as u16) << 8) | ea, ea, 0),
            2 => with_ea(0x41c0 | (dn << 9) | ea, ea, 2),
            3 => with_ea(0x4ec0 | ea, ea, 2),
            4 => {
                let mut v = alloc::vec![0x4c80 | (((fields >> 23) & 1) as u16) << 6 | ea];
                v.push(imm);
                v.extend(ext(ea, 1));
                v
            }
            5 => one(0x4e75), // RTS
            6 => one(0x4e71), // NOP
            _ => with_ea(0x40c0 | ea, ea, 1), // MOVE from SR
        },
    }
}

/// Run a seeded pseudo-random sweep, and report the rate.
///
/// `seed` starts an `xorshift64*` sequence — no `HashMap`, no wall clock, no
/// float, so a failing case is reproducible from the seed alone (CLAUDE.md,
/// "Determinism"). Each case is a short program of [`synthesize`]d
/// instructions followed by a `STOP`, so a case that runs off the end stops
/// rather than executing whatever is next.
///
/// Returns `(cases, divergences)` with the first divergence, so a caller can
/// report a **rate** rather than a verdict: "N cases, 0 disagreements" is a
/// measurement and "green" is not.
///
/// # Errors
///
/// Never as such: a divergence comes back in the tuple, because a sweep's
/// value is the count beside it.
#[must_use]
pub fn sweep(seed: u64, cases: usize, per_case: usize) -> (usize, Option<Divergence>) {
    let mut rng = Rng::new(seed);
    let mut found = None;
    for _ in 0..cases {
        let mut program = Vec::new();
        for _ in 0..per_case {
            program.extend(synthesize(rng.draw() as u32, rng.draw() as u32));
        }
        // `STOP #$2700` — supervisor state, interrupts masked — so a program
        // that falls off its own end halts instead of running into the data
        // window. "Stopped" is a column this harness compares.
        program.push(0x4e72);
        program.push(0x2700);
        let case = Case::seeded(program)
            .with_ccr((rng.draw() & 0x1f) as u16)
            .with_units(per_case + 2);
        if let Err(d) = compare(&case) {
            found = Some(d);
            break;
        }
    }
    (cases, found)
}

/// A deterministic `xorshift64*`.
///
/// Not a quality generator and not meant to be: it has to be reproducible from
/// a seed and to have no dependency, which rules out everything else here.
#[derive(Debug)]
pub struct Rng(u64);

impl Rng {
    /// A generator seeded with `seed` (zero is replaced, since the sequence
    /// has a fixed point there).
    #[must_use]
    pub fn new(seed: u64) -> Rng {
        Rng(if seed == 0 { 0x9e37_79b9_7f4a_7c15 } else { seed })
    }

    /// The next value.
    pub fn draw(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_f491_4f6c_dd1d)
    }
}

/// Compare **every opcode word there is**, one case each.
///
/// A 68000 opcode is sixteen bits, so this is the whole decode space: 65 536
/// cases, of which a few hundred lift and the rest exercise the fallback. It
/// is the corpus this core has and the other three frontends do not — an
/// exhaustive one — and nothing is skipped, because an encoding the frontend
/// declines still has to leave the guest in the state the interpreter leaves
/// it in.
///
/// `stride` runs every `stride`-th word, so a fast test can take a slice of it
/// and a slow one the lot. `extensions` supplies the words after the opcode.
///
/// Returns `(cases, divergences)`, with the first divergence.
#[must_use]
pub fn opcode_sweep(stride: u32, extensions: &[u16]) -> (usize, Option<Divergence>) {
    let mut cases = 0usize;
    let mut word = 0u32;
    while word < 0x1_0000 {
        let mut program = alloc::vec![word as u16];
        program.extend_from_slice(extensions);
        program.push(0x4e72);
        program.push(0x2700);
        let case = Case::seeded(program).with_extend().with_units(3);
        cases += 1;
        if let Err(d) = compare(&case) {
            return (cases, Some(d));
        }
        word += stride;
    }
    (cases, None)
}

/// Whether the lifter claims this program's first instruction at all.
///
/// Exposed because a test that wants to *prove the fallback is exercised*
/// needs to be able to say which side of the line an encoding is on, and
/// because `lift`'s subset is a documented list rather than a guess.
#[must_use]
pub fn lifts(program: &[u16]) -> bool {
    struct Words<'a>(&'a [u16], u32);
    impl lift::InsnSource for Words<'_> {
        fn word(&mut self, addr: u32) -> Option<u16> {
            let off = addr.checked_sub(self.1)? / 2;
            self.0.get(off as usize).copied()
        }
    }
    let mut src = Words(program, CODE);
    lift::lift(Model::M68000, CODE, &mut src, 1).is_ok_and(|l| l.insns > 0)
}

/// What a case's subject engine did, for a test that wants to assert the
/// *shape* of a run rather than its agreement.
#[must_use]
pub fn stats_for(case: &Case) -> Option<engine::Stats> {
    let (space, _ram) = machine(case);
    let subject = core(case, space, Engine::Ir);
    for _ in 0..case.units {
        if subject.step() == 0 {
            break;
        }
    }
    subject.ir_stats()
}

#[cfg(test)]
mod tests;
