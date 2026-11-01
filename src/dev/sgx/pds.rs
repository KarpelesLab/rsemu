//! The PDS — the SGX543's programmable data sequencer: decoder, disassembler
//! and interpreter.
//!
//! The PDS is the small processor in front of the shader engine. Every USSE
//! task starts as a PDS program: it is launched by an event (a host kick, a
//! hardware completion, the microkernel's own `EMIT`), receives up to three
//! input words, moves data into the new task's attribute registers, and ends
//! by handing the USSE an entry point. On this machine it is what boots the
//! microkernel and what routes every event to the right microkernel handler.
//!
//! # Provenance
//!
//! No PDS documentation is public and none was consulted: no DDK source, no
//! Mesa, no community reverse-engineering, no web search. Everything here was
//! read off the guest's own binaries: the PDS code generators in the navi's
//! `libGLES_CM.so`, run as a black box and diffed while one input changed at a
//! time; the 48 prebuilt programs in the user-space libraries; and the
//! firmware programs `libsrv_init.so` installs, checked against the
//! microkernel entries they must reach. Our notes on that study
//! (`PDS_ENCODING.md`, since lost; the later additions are in
//! `docs/devices/sgx-usse-encoding.md` §13) grade each claim exact / strong
//! / guess; the guesses are marked `[G]` below.
//!
//! # Programs
//!
//! A program is one block of GPU memory: a data segment of `rows` 16-byte
//! rows, then 32-bit code words. Row `r` holds `DS0[2r]`, `DS0[2r+1]`,
//! `DS1[2r]`, `DS1[2r+1]` — two data stores interleaved two words at a time.
//! How many rows there are is not in the program: whatever launches it says
//! (an event register, an `EMIT` control word).
//!
//! Each store is 64 words: `0x00–0x2f` the data segment, `0x30–0x37`
//! temporaries, `0x38–0x3f` temporaries that persist from one program to the
//! next \[strong\]. The inputs `IR0–IR2` sit beside them.
//!
//! # Shape
//!
//! As in [`usse`](super::usse): one declarative table, `FORMS`, describes each
//! instruction — match bits, mnemonic, operand encodings, execution class —
//! and [`decode`], [`disassemble`] and [`Pds::step`] all walk it.

use alloc::string::String;
use core::fmt::{self, Write as _};

use crate::core::error::BusError;
use crate::core::space::MemResult;

/// Words in each data store.
pub const STORE_WORDS: usize = 64;
/// The first temporary.
pub const TEMPS: usize = 0x30;
/// The first temporary that survives from one program to the next.
pub const PERSISTENT: usize = 0x38;
/// The most data rows a program can have: the rest of the store is temps.
pub const MAX_ROWS: u32 = (TEMPS / 2) as u32;

// ---------------------------------------------------------------------------
// The instruction word

/// What a word does: `[31:28]` op and `[27]` sub-op select it (§3).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Op {
    /// Data out: a 4-word window to the USSE, the DMA engine or an attribute.
    Dout {
        /// The `.add` form: `W0 += W3` \[strong\].
        add: bool,
    },
    /// `dst = src` (MOV-form operands, §2.4).
    Mov,
    /// `dst = src0 + src1`.
    Add,
    /// `dst = src0 - src1`.
    Sub,
    /// `dst = src0.half * src1.half`, 16 × 16 → 32.
    Mul,
    /// `P0 = (operand == 0)`.
    Tstz,
    /// Branch to a code index.
    Bra,
    /// End of program.
    Halt,
    /// `dst = src0 | src1`.
    Or,
    /// `dst = src0 & src1`.
    And,
    /// `dst = src0 ^ src1`.
    Xor,
    /// `dst = src << imm`.
    Shl,
    /// `dst = src >> imm`, logical.
    Shr,
}

/// How an operand is encoded (§2).
#[derive(Clone, Copy, Debug)]
enum Opd {
    /// An ALU destination, `[7:0]`: a word address, bit 6 the bank.
    AluDst,
    /// `src0`, `[23:16]`: a DS0-side half-word address, `≥ 0x80` an input.
    Src0,
    /// `src1`, `[15:8]`: the same for the DS1 side.
    Src1,
    /// A MOV destination, `[7:0]`: a half-word address, bit 7 the bank.
    MovDst,
    /// A MOV source, `[15:8]`: DS0-side half-word address.
    MovSrc,
    /// A shift source, `[23:16]`, word address as [`Opd::AluDst`].
    ShiftSrc,
    /// A shift amount, `[15:8]`.
    ShiftImm,
    /// TSTZ's operand: `[8]` clear → `src0`; set → DS1-side half `[15:9]`.
    Tst,
    /// A branch target, `[15:0]`.
    Target,
}

/// One instruction form.
#[derive(Debug)]
struct Form {
    mask: u32,
    value: u32,
    /// The mnemonic; a DOUT's comes from its type instead.
    name: &'static str,
    ops: &'static [Opd],
    exec: Op,
}

const TOP: u32 = 0xf800_0000;

const fn top(op: u32, b27: u32) -> u32 {
    op << 28 | b27 << 27
}

const O_ALU: &[Opd] = &[Opd::AluDst, Opd::Src0, Opd::Src1];
const O_SHIFT: &[Opd] = &[Opd::AluDst, Opd::ShiftSrc, Opd::ShiftImm];

/// The instruction forms (§3); first match wins.
static FORMS: &[Form] = &[
    Form {
        mask: TOP,
        value: top(0, 0),
        name: "dout",
        ops: &[],
        exec: Op::Dout { add: false },
    },
    Form {
        mask: TOP,
        value: top(2, 1),
        name: "dout",
        ops: &[],
        exec: Op::Dout { add: true },
    },
    Form {
        mask: TOP,
        value: top(1, 0),
        name: "mov",
        ops: &[Opd::MovDst, Opd::MovSrc],
        exec: Op::Mov,
    },
    Form {
        mask: TOP,
        value: top(4, 0),
        name: "add",
        ops: O_ALU,
        exec: Op::Add,
    },
    Form {
        mask: TOP,
        value: top(4, 1),
        name: "sub",
        ops: O_ALU,
        exec: Op::Sub,
    },
    Form {
        mask: TOP,
        value: top(6, 0),
        name: "mul",
        ops: O_ALU,
        exec: Op::Mul,
    },
    Form {
        mask: TOP,
        value: top(8, 0),
        name: "tstz",
        ops: &[Opd::Tst],
        exec: Op::Tstz,
    },
    Form {
        mask: TOP,
        value: top(9, 0),
        name: "bra",
        ops: &[Opd::Target],
        exec: Op::Bra,
    },
    // Only `af000000` has ever been seen; the low bits are not checked.
    Form {
        mask: TOP,
        value: top(0xa, 1),
        name: "halt",
        ops: &[],
        exec: Op::Halt,
    },
    Form {
        mask: TOP,
        value: top(0xc, 0),
        name: "or",
        ops: O_ALU,
        exec: Op::Or,
    },
    Form {
        mask: TOP,
        value: top(0xc, 1),
        name: "and",
        ops: O_ALU,
        exec: Op::And,
    },
    Form {
        mask: TOP,
        value: top(0xd, 0),
        name: "xor",
        ops: O_ALU,
        exec: Op::Xor,
    },
    Form {
        mask: TOP,
        value: top(0xf, 0),
        name: "shl",
        ops: O_SHIFT,
        exec: Op::Shl,
    },
    Form {
        mask: TOP,
        value: top(0xf, 1),
        name: "shr",
        ops: O_SHIFT,
        exec: Op::Shr,
    },
];

/// One decoded operand.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Operand {
    /// A data-store word, or one 16-bit half of it.
    Ds {
        /// 0 or 1.
        bank: u8,
        /// The word, `0–63`.
        word: u8,
        /// `None` for the whole word, else whether the high half.
        half: Option<bool>,
    },
    /// An input register, or one half of it.
    Ir {
        /// `IR0–IR2` (a larger number reads zero \[G\]).
        n: u8,
        /// As for [`Operand::Ds`].
        half: Option<bool>,
    },
    /// An immediate.
    Imm(u32),
    /// A code index.
    Target(u16),
}

/// The condition field, `[26:24]` (§3.1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cond {
    /// 7: always.
    Always,
    /// 0: when P0 is set \[strong\].
    P0,
    /// Anything else: a hardware flag whose meaning is not known; the
    /// interpreter asks [`PdsBus::condition`]. Where they are seen in the
    /// navi's programs: `3` on the `DOUTU` that ends every vertex PDS
    /// program, which the driver's generator ORs in unconditionally
    /// (`0x0300_0005 | ...`), so it is presumably true whenever a vertex
    /// task should run \[G\]; `3` and `4` on the two branches of the
    /// event-dispatch programs in the pixel PDS heap, which pick one of
    /// three `DOUTU`s \[G: hardware event flags\]; `1` and `2` only in words
    /// that are data, not code.
    Other(u8),
}

impl Cond {
    fn of(w: u32) -> Cond {
        match w >> 24 & 7 {
            7 => Cond::Always,
            0 => Cond::P0,
            c => Cond::Other(c as u8),
        }
    }
}

/// A DOUT's type, `[3:0]` (§4). Extensible: only three are executed.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct DoutType(pub u8);

impl DoutType {
    /// Iterator set-up \[G\]: two window words, both from the data
    /// segment. Every pixel PDS program in the navi's UI issues one, after
    /// its `DOUTU` and before its [`DoutType::TEXTURE`]. Passed to the bus
    /// unchanged when it [accepts](PdsBus::accepts_dout) the type.
    pub const ITER: DoutType = DoutType(2);
    /// DMA from memory into attribute registers: `W0` address, `W1`
    /// control `(dwords - 1) | first_register << 8`.
    pub const DMA: DoutType = DoutType(3);
    /// Texture fetch \[G\]: one window word from the data segment, the
    /// last DOUT of every pixel PDS program in the navi's UI — the
    /// non-dependent texture reads that land in the pixel task's primary
    /// attributes before it starts. Passed to the bus unchanged when it
    /// [accepts](PdsBus::accepts_dout) the type.
    pub const TEXTURE: DoutType = DoutType(4);
    /// Start a USSE task: `W0..W2` (§6).
    pub const USSE: DoutType = DoutType(5);
    /// Write one attribute register: `W0` value, `W1` `register << 8`.
    pub const ATTR: DoutType = DoutType(6);

    fn mnemonic(self) -> Option<&'static str> {
        Some(match self {
            DoutType::ITER => "dout2",
            DoutType::DMA => "doutd",
            DoutType::TEXTURE => "dout4",
            DoutType::USSE => "doutu",
            DoutType::ATTR => "douta",
            _ => return None,
        })
    }

    /// How many window words the type uses.
    fn words(self) -> usize {
        match self {
            DoutType::USSE => 3,
            DoutType::DMA | DoutType::ATTR | DoutType::ITER => 2,
            DoutType::TEXTURE => 4,
            _ => 4,
        }
    }
}

/// A DOUT's 4-word window (§4): which data word each of `W0..W3` reads.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Window {
    /// The type.
    pub kind: DoutType,
    /// The four words' sources.
    pub words: [Operand; 4],
    /// `b17`: the DS1-side selectors read `IR[rowB]` instead.
    pub ir_source: bool,
}

/// A decoded instruction.
#[derive(Clone, Copy, Debug)]
pub struct Insn {
    /// The word.
    pub word: u32,
    /// Its condition.
    pub cond: Cond,
    form: &'static Form,
    args: [Operand; 3],
    nargs: u8,
    window: Option<Window>,
}

impl Insn {
    /// What it does.
    pub fn op(&self) -> Op {
        self.form.exec
    }

    /// Its operands, in disassembly order (a DOUT's are in [`Insn::window`]).
    pub fn args(&self) -> &[Operand] {
        &self.args[..usize::from(self.nargs)]
    }

    /// A DOUT's window.
    pub fn window(&self) -> Option<&Window> {
        self.window.as_ref()
    }
}

/// A half-word address (§2.1, §2.2): below `0x80` a store word and half,
/// above it an input register and half.
fn half_addr(bank: u8, h: u32) -> Operand {
    let half = Some(h & 1 != 0);
    if h >= 0x80 {
        Operand::Ir {
            n: ((h - 0x80) >> 1) as u8,
            half,
        }
    } else {
        Operand::Ds {
            bank,
            word: (h >> 1) as u8,
            half,
        }
    }
}

/// A word address (§2.3): bit 6 the bank, `[5:0]` the word.
fn word_addr(f: u32) -> Operand {
    Operand::Ds {
        bank: (f >> 6 & 1) as u8,
        word: (f & 0x3f) as u8,
        half: None,
    }
}

fn decode_opd(opd: Opd, w: u32) -> Operand {
    let (a, b, c) = (w >> 16 & 0xff, w >> 8 & 0xff, w & 0xff);
    match opd {
        Opd::AluDst => word_addr(c & 0x7f),
        Opd::Src0 => half_addr(0, a),
        Opd::Src1 => half_addr(1, b),
        // §2.4: bit 7 the bank, [6:0] a half address within it.
        Opd::MovDst => Operand::Ds {
            bank: (c >> 7) as u8,
            word: ((c & 0x7f) >> 1) as u8,
            half: Some(c & 1 != 0),
        },
        Opd::MovSrc => half_addr(0, b),
        Opd::ShiftSrc => word_addr(a & 0x7f),
        Opd::ShiftImm => Operand::Imm(b),
        Opd::Tst if b & 1 != 0 => half_addr(1, b >> 1),
        Opd::Tst => half_addr(0, a),
        Opd::Target => Operand::Target(w as u16),
    }
}

/// The DOUT window (§4): `[23:18]` rowA, `b17` IR source, `{b16, [15:12]}`
/// rowB, `[11:4]` four 2-bit selectors, `[3:0]` type.
fn decode_window(w: u32) -> Window {
    let row_a = w >> 18 & 0x3f;
    let ir_source = w >> 17 & 1 != 0;
    let row_b = (w >> 16 & 1) << 4 | (w >> 12 & 0xf);
    let pick = |s: u32| -> Operand {
        if s < 2 {
            if row_a >= 0x20 {
                Operand::Ir {
                    n: ((row_a - 0x20) * 2 + s) as u8,
                    half: None,
                }
            } else {
                Operand::Ds {
                    bank: 0,
                    word: (row_a * 2 + s) as u8,
                    half: None,
                }
            }
        } else if ir_source {
            Operand::Ir {
                n: row_b as u8,
                half: None,
            }
        } else {
            Operand::Ds {
                bank: 1,
                word: (row_b * 2 + (s - 2)) as u8,
                half: None,
            }
        }
    };
    let kind = DoutType((w & 0xf) as u8);
    // A texture DOUT sends four words: rowA's pair from DS0 and rowB's
    // from DS1, whatever the selectors hold \[strong\]. Every textured UI
    // program keeps a four-word texture descriptor in exactly those words
    // (address third) and leaves the selectors at 0.
    let sel = if kind == DoutType::TEXTURE {
        [0, 1, 2, 3]
    } else {
        [w >> 10 & 3, w >> 8 & 3, w >> 6 & 3, w >> 4 & 3]
    };
    Window {
        kind,
        words: sel.map(pick),
        ir_source,
    }
}

/// Decodes one word; `None` when no form matches.
pub fn decode(word: u32) -> Option<Insn> {
    let form = FORMS.iter().find(|f| word & f.mask == f.value)?;
    let mut args = [Operand::Imm(0); 3];
    for (slot, opd) in args.iter_mut().zip(form.ops) {
        *slot = decode_opd(*opd, word);
    }
    Some(Insn {
        word,
        cond: Cond::of(word),
        form,
        args,
        nargs: form.ops.len() as u8,
        window: matches!(form.exec, Op::Dout { .. }).then(|| decode_window(word)),
    })
}

// ---------------------------------------------------------------------------
// Disassembly

/// Names a store word the way the reference disassembler does: `d<n>` for a
/// data-segment word (its index in the segment as laid out in memory),
/// `ds<bank>.t<n>` for a temporary.
fn store_name(f: &mut impl fmt::Write, bank: u8, word: u8) -> fmt::Result {
    let word = u32::from(word);
    if word >= TEMPS as u32 {
        write!(f, "ds{bank}.t{}", word - TEMPS as u32)
    } else {
        write!(
            f,
            "d{}",
            (word >> 1) * 4 + (word & 1) + if bank == 1 { 2 } else { 0 }
        )
    }
}

impl fmt::Display for Operand {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let hi = |f: &mut fmt::Formatter<'_>, half: Option<bool>| {
            if half == Some(true) {
                f.write_str(".hi")
            } else {
                Ok(())
            }
        };
        match *self {
            Operand::Ds { bank, word, half } => {
                store_name(f, bank, word)?;
                hi(f, half)
            }
            Operand::Ir { n, half } => {
                write!(f, "ir{n}")?;
                hi(f, half)
            }
            Operand::Imm(v) => write!(f, "#{v}"),
            Operand::Target(t) => write!(f, "{t}"),
        }
    }
}

impl fmt::Display for Insn {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.cond {
            Cond::Always => {}
            Cond::P0 => f.write_str("(p0) ")?,
            Cond::Other(c) => write!(f, "(cc{c}) ")?,
        }
        if let Some(win) = &self.window {
            match win.kind.mnemonic() {
                Some(m) => f.write_str(m)?,
                None => write!(f, "dout{}", win.kind.0)?,
            }
            let add = matches!(self.form.exec, Op::Dout { add: true });
            if add {
                f.write_str(".add")?;
            }
            for (i, w) in win.words[..win.kind.words()].iter().enumerate() {
                f.write_str(if i == 0 { " " } else { ", " })?;
                write!(f, "{w}")?;
            }
            if add {
                write!(f, " + {}", win.words[3])?;
            }
            if win.ir_source {
                f.write_str("   ; irsrc")?;
            }
            return Ok(());
        }
        f.write_str(self.form.name)?;
        if self.form.exec == Op::Tstz {
            f.write_str(" p0,")?;
        }
        for (i, a) in self.args().iter().enumerate() {
            f.write_str(if i == 0 { " " } else { ", " })?;
            write!(f, "{a}")?;
        }
        Ok(())
    }
}

/// Disassembles one word.
pub fn disassemble(word: u32) -> String {
    let mut s = String::new();
    match decode(word) {
        Some(insn) => {
            let _ = write!(s, "{insn}");
        }
        None => {
            let _ = write!(s, ".word {word:#010x}");
        }
    }
    s
}

// ---------------------------------------------------------------------------
// The interpreter

/// A DOUT that leaves the PDS: the type and its window's values.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Dout {
    /// The type.
    pub kind: DoutType,
    /// `W0..W3` (unused words are whatever the window selected).
    pub words: [u32; 4],
}

/// Everything outside the PDS: GPU memory (for its own code) and the units
/// its DOUTs feed.
pub trait PdsBus {
    /// Reads the word at GPU virtual address `addr`.
    fn load(&mut self, addr: u32) -> MemResult<u32>;

    /// Takes a DOUT of a type [`PdsBus::accepts_dout`] accepts.
    fn dout(&mut self, out: &Dout) -> MemResult;

    /// Whether [`PdsBus::dout`] takes DOUTs of type `kind`. A DOUT of a type
    /// it does not take stops the PDS with [`Stop::Unimplemented`] instead.
    /// The default is the three types this module knows the meaning of:
    /// DMA, attribute and USSE. A bus that models the iterators and the
    /// texture unit adds [`DoutType::ITER`] and [`DoutType::TEXTURE`]: their
    /// window words reach it unchanged (§4.2).
    fn accepts_dout(&self, kind: DoutType) -> bool {
        matches!(kind, DoutType::DMA | DoutType::ATTR | DoutType::USSE)
    }

    /// Evaluates condition code `cc` (`[26:24]` of the word: neither 7,
    /// always, nor 0, `P0`) for the instruction at `pc` (§3.1). Their
    /// meanings are not known; `None`, the default, stops the PDS with
    /// [`Stop::Unimplemented`]. See [`Cond::Other`] for where each is seen.
    fn condition(&mut self, cc: u8, pc: u32) -> Option<bool> {
        let _ = (cc, pc);
        None
    }
}

/// Why the PDS stopped. `pc` counts code words from the first.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stop {
    /// `HALT`.
    Halt {
        /// The HALT.
        pc: u32,
    },
    /// The step budget ran out.
    Budget {
        /// The next instruction.
        pc: u32,
    },
    /// No form matched.
    Undecodable {
        /// Where.
        pc: u32,
        /// The word.
        word: u32,
    },
    /// Decoded, but not executed.
    Unimplemented {
        /// Where.
        pc: u32,
        /// The word.
        word: u32,
        /// What is missing.
        why: &'static str,
    },
    /// A fetch faulted, or the bus refused a DOUT.
    Fault {
        /// Where.
        pc: u32,
        /// The address (0 for a DOUT).
        addr: u32,
        /// The error.
        error: BusError,
    },
}

/// One PDS's state.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Pds {
    /// DS0 and DS1.
    pub ds: [[u32; STORE_WORDS]; 2],
    /// `IR0–IR2`.
    pub ir: [u32; 3],
    /// The predicate.
    pub p0: bool,
    /// The next instruction.
    pub pc: u32,
    /// The GPU address of code word 0.
    pub code: u32,
    /// Instructions retired.
    pub retired: u64,
}

impl Default for Pds {
    fn default() -> Pds {
        Pds {
            ds: [[0; STORE_WORDS]; 2],
            ir: [0; 3],
            p0: false,
            pc: 0,
            code: 0,
            retired: 0,
        }
    }
}

impl Pds {
    /// Starts the program at `base` with `rows` data rows and inputs `ir`:
    /// loads the data segment, clears the per-program temporaries and keeps
    /// the persistent ones.
    ///
    /// # Errors
    ///
    /// A fault reading the data segment, or more rows than the stores hold.
    pub fn launch(
        &mut self,
        bus: &mut dyn PdsBus,
        base: u32,
        rows: u32,
        ir: [u32; 3],
    ) -> Result<(), Stop> {
        if rows > MAX_ROWS {
            return Err(Stop::Unimplemented {
                pc: 0,
                word: rows,
                why: "data segment larger than the stores",
            });
        }
        for bank in &mut self.ds {
            bank[..PERSISTENT].fill(0);
        }
        for r in 0..rows {
            for i in 0..4 {
                let addr = base.wrapping_add(16 * r + 4 * i);
                let v = bus
                    .load(addr)
                    .map_err(|error| Stop::Fault { pc: 0, addr, error })?;
                let (bank, word) = ((i >> 1) as usize, (2 * r + (i & 1)) as usize);
                self.ds[bank][word] = v;
            }
        }
        self.ir = ir;
        self.p0 = false;
        self.pc = 0;
        self.code = base.wrapping_add(16 * rows);
        Ok(())
    }

    /// Runs until something stops it or `budget` instructions have retired.
    pub fn run(&mut self, bus: &mut dyn PdsBus, budget: u64) -> Stop {
        for _ in 0..budget {
            if let Err(stop) = self.step(bus) {
                return stop;
            }
        }
        Stop::Budget { pc: self.pc }
    }

    fn get(&self, op: Operand) -> u32 {
        let (v, half) = match op {
            Operand::Ds { bank, word, half } => (
                self.ds[usize::from(bank & 1)][usize::from(word & 0x3f)],
                half,
            ),
            Operand::Ir { n, half } => (self.ir.get(usize::from(n)).copied().unwrap_or(0), half),
            Operand::Imm(v) => (v, None),
            Operand::Target(t) => (u32::from(t), None),
        };
        // [G] A 32-bit operation on an odd (high-half) address sees that
        // half; nothing observed uses one.
        if half == Some(true) { v >> 16 } else { v }
    }

    /// A MUL operand: always a 16-bit half.
    fn get16(&self, op: Operand) -> u32 {
        self.get(op) & 0xffff
    }

    fn set(&mut self, op: Operand, v: u32) {
        if let Operand::Ds { bank, word, half } = op {
            let slot = &mut self.ds[usize::from(bank & 1)][usize::from(word & 0x3f)];
            *slot = match half {
                // [G] A MOV to an odd address writes the high half.
                Some(true) => *slot & 0xffff | v << 16,
                _ => v,
            };
        }
    }

    /// Executes one instruction.
    ///
    /// # Errors
    ///
    /// Whatever stops the program, including `HALT`.
    pub fn step(&mut self, bus: &mut dyn PdsBus) -> Result<(), Stop> {
        let pc = self.pc;
        let addr = self.code.wrapping_add(pc.wrapping_mul(4));
        let word = bus
            .load(addr)
            .map_err(|error| Stop::Fault { pc, addr, error })?;
        let insn = decode(word).ok_or(Stop::Undecodable { pc, word })?;
        let enabled = match insn.cond {
            Cond::Always => true,
            Cond::P0 => self.p0,
            Cond::Other(cc) => match bus.condition(cc, pc) {
                Some(taken) => taken,
                None => {
                    return Err(Stop::Unimplemented {
                        pc,
                        word,
                        why: "condition",
                    });
                }
            },
        };
        self.pc = pc.wrapping_add(1);
        self.retired = self.retired.wrapping_add(1);
        if !enabled {
            return Ok(());
        }
        let a = insn.args();
        let alu = |pds: &Pds, f: fn(u32, u32) -> u32| f(pds.get(a[1]), pds.get(a[2]));
        let r = match insn.op() {
            Op::Halt => {
                self.pc = pc;
                return Err(Stop::Halt { pc });
            }
            Op::Bra => {
                self.pc = self.get(a[0]);
                return Ok(());
            }
            Op::Tstz => {
                self.p0 = self.get(a[0]) == 0;
                return Ok(());
            }
            Op::Dout { add } => {
                let win = insn.window.expect("a DOUT decodes a window");
                let mut words = win.words.map(|w| self.get(w));
                if add {
                    words[0] = words[0].wrapping_add(words[3]);
                }
                if !bus.accepts_dout(win.kind) {
                    self.pc = pc;
                    return Err(Stop::Unimplemented {
                        pc,
                        word,
                        why: "DOUT type",
                    });
                }
                return bus
                    .dout(&Dout {
                        kind: win.kind,
                        words,
                    })
                    .map_err(|error| {
                        self.pc = pc;
                        Stop::Fault { pc, addr: 0, error }
                    });
            }
            Op::Mov => {
                let v = self.get(a[1]);
                self.set(a[0], v);
                return Ok(());
            }
            Op::Add => alu(self, u32::wrapping_add),
            Op::Sub => alu(self, u32::wrapping_sub),
            Op::Or => alu(self, |x, y| x | y),
            Op::And => alu(self, |x, y| x & y),
            Op::Xor => alu(self, |x, y| x ^ y),
            Op::Mul => self.get16(a[1]).wrapping_mul(self.get16(a[2])),
            // [G] Shift amounts are taken modulo 32.
            Op::Shl => self.get(a[1]) << (self.get(a[2]) & 31),
            Op::Shr => self.get(a[1]) >> (self.get(a[2]) & 31),
        };
        self.set(a[0], r);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::collections::BTreeMap;
    use alloc::vec::Vec;

    // A tiny assembler from the field positions above; no word here is
    // copied out of a guest binary.

    fn w(op: u32, b27: u32, cond: u32, low: u32) -> u32 {
        op << 28 | b27 << 27 | cond << 24 | low
    }
    fn alu(op: u32, b27: u32, dst: u32, s0: u32, s1: u32) -> u32 {
        w(op, b27, 7, s0 << 16 | s1 << 8 | dst)
    }
    /// A DOUT: rowA, IR-source, rowB, selectors, type.
    fn dout(
        add: bool,
        cond: u32,
        row_a: u32,
        irsrc: bool,
        row_b: u32,
        sel: [u32; 4],
        t: u32,
    ) -> u32 {
        let (op, b27) = if add { (2, 1) } else { (0, 0) };
        w(
            op,
            b27,
            cond,
            row_a << 18
                | u32::from(irsrc) << 17
                | (row_b >> 4) << 16
                | (row_b & 0xf) << 12
                | sel[0] << 10
                | sel[1] << 8
                | sel[2] << 6
                | sel[3] << 4
                | t,
        )
    }
    fn bra(cond: u32, t: u32) -> u32 {
        w(9, 0, cond, t)
    }
    fn tstz0(cond: u32, h: u32) -> u32 {
        w(8, 0, cond, h << 16)
    }
    fn tstz1(cond: u32, h: u32) -> u32 {
        w(8, 0, cond, h << 9 | 1 << 8)
    }
    const HALT: u32 = 0xa << 28 | 1 << 27 | 7 << 24;
    const T0: u32 = 0x30; // ds0.t0, word address
    const DS1_T0: u32 = 0x70;

    #[derive(Default)]
    struct Bus {
        mem: BTreeMap<u32, u32>,
        outs: Vec<Dout>,
        /// Condition codes this bus answers, and whether it takes every
        /// DOUT type.
        conds: BTreeMap<u8, bool>,
        all_types: bool,
    }

    impl Bus {
        /// A program at 0x1000: `data` rows, then `code`.
        fn program(data: &[[u32; 4]], code: &[u32]) -> Bus {
            let mut bus = Bus::default();
            let mut a = 0x1000;
            for v in data.iter().flatten().chain(code) {
                bus.mem.insert(a, *v);
                a += 4;
            }
            bus
        }
    }

    impl PdsBus for Bus {
        fn load(&mut self, addr: u32) -> MemResult<u32> {
            self.mem.get(&addr).copied().ok_or(BusError::Unassigned)
        }

        fn dout(&mut self, out: &Dout) -> MemResult {
            self.outs.push(*out);
            Ok(())
        }

        fn accepts_dout(&self, kind: DoutType) -> bool {
            self.all_types || matches!(kind, DoutType::DMA | DoutType::ATTR | DoutType::USSE)
        }

        fn condition(&mut self, cc: u8, _pc: u32) -> Option<bool> {
            self.conds.get(&cc).copied()
        }
    }

    fn run(data: &[[u32; 4]], code: &[u32], ir: [u32; 3]) -> (Pds, Bus, Stop) {
        let mut bus = Bus::program(data, code);
        let mut pds = Pds::default();
        pds.launch(&mut bus, 0x1000, data.len() as u32, ir).unwrap();
        let stop = pds.run(&mut bus, 1000);
        (pds, bus, stop)
    }

    #[test]
    fn the_data_segment_interleaves_the_stores() {
        let (pds, _, stop) = run(&[[1, 2, 3, 4], [5, 6, 7, 8]], &[HALT], [0; 3]);
        assert_eq!(stop, Stop::Halt { pc: 0 });
        assert_eq!(pds.ds[0][..4], [1, 2, 5, 6]);
        assert_eq!(pds.ds[1][..4], [3, 4, 7, 8]);
        assert_eq!(pds.code, 0x1020);
    }

    #[test]
    fn disassembly_names_operands_as_the_reference_does() {
        assert_eq!(
            disassemble(dout(false, 7, 0, true, 0, [2, 0, 0, 0], 6)),
            "douta ir0, d0   ; irsrc"
        );
        assert_eq!(
            disassemble(dout(false, 7, 2, false, 0, [0, 1, 2, 0], 5)),
            "doutu d8, d9, d2"
        );
        assert_eq!(
            disassemble(dout(false, 7, 0x18, false, 0x18, [0, 1, 0, 0], 3)),
            "doutd ds0.t0, ds0.t1"
        );
        assert_eq!(
            disassemble(dout(true, 7, 0, false, 0x19, [0, 1, 2, 2], 3)),
            "doutd.add d0, d1 + ds1.t2"
        );
        assert_eq!(
            disassemble(alu(0xc, 1, T0, 0x82, 0x0a)),
            "and ds0.t0, ir1, d11"
        );
        assert_eq!(
            disassemble(alu(0xd, 0, DS1_T0, 0x80, 0x60) & !(7 << 24)),
            "(p0) xor ds1.t0, ir0, ds1.t0"
        );
        assert_eq!(disassemble(w(1, 0, 7, 0x0a << 8 | 0xe0)), "mov ds1.t0, d9");
        assert_eq!(
            disassemble(alu(0xf, 0, 0x71, 0x70, 4)),
            "shl ds1.t1, ds1.t0, #4"
        );
        assert_eq!(disassemble(tstz1(7, 0x60)), "tstz p0, ds1.t0");
        assert_eq!(disassemble(tstz0(7, 0x0e)), "tstz p0, d13");
        assert_eq!(disassemble(bra(0, 7)), "(p0) bra 7");
        assert_eq!(disassemble(HALT), "halt");
        assert_eq!(disassemble(0x3 << 28), ".word 0x30000000");
    }

    #[test]
    fn an_event_program_routes_on_its_inputs() {
        // The shape of an event router: pass IR0/IR1 on as attributes, then
        // pick an entry by whether IR1 has bit 6.
        let data = [
            [0, 0x100, 0x40, 0], // d0, d1 (attr regs), d2 (mask)
            [0xa5, 1, 0, 0],     // d4, d5: entry A
            [0xb5, 1, 0, 0],     // d8, d9: entry B
        ];
        let code = [
            dout(false, 7, 0, true, 0, [2, 0, 0, 0], 6), // douta ir0 -> attr 0
            dout(false, 7, 0, true, 1, [2, 1, 0, 0], 6), // douta ir1 -> attr 1
            alu(0xc, 1, T0, 0x82, 0x00),                 // and t0, ir1, d2
            tstz0(7, 0x60),                              // tstz t0
            bra(0, 7),
            dout(false, 7, 1, false, 0, [0, 1, 2, 0], 5), // doutu d4, d5, d2
            HALT,
            dout(false, 7, 2, false, 0, [0, 1, 2, 0], 5), // doutu d8, d9, d2
            HALT,
        ];
        let (_, bus, stop) = run(&data, &code, [0x11, 0x40, 0]);
        assert_eq!(stop, Stop::Halt { pc: 6 });
        assert_eq!(bus.outs[0].kind, DoutType::ATTR);
        assert_eq!(bus.outs[0].words[..2], [0x11, 0]);
        assert_eq!(bus.outs[1].words[..2], [0x40, 0x100]);
        assert_eq!(bus.outs[2].kind, DoutType::USSE);
        assert_eq!(bus.outs[2].words[..3], [0xa5, 1, 0x40]);
        let (_, bus, stop) = run(&data, &code, [0x11, 0, 0]);
        assert_eq!(stop, Stop::Halt { pc: 8 });
        assert_eq!(bus.outs[2].words[0], 0xb5);
    }

    #[test]
    fn a_dma_loop_counts_down() {
        // d0 = address, d1 = count, d4 = byte step, d6 = 1.
        let data = [[0x4000, 3, 0, 0], [0x400, 0, 1, 0]];
        let code = [
            w(1, 0, 7, 0x60),                                   // mov ds0.t0, d0
            w(1, 0, 7, 0x02 << 8 | 0x66),                       // mov ds0.t3, d1
            w(1, 0, 7, 0x04 << 8 | 0xe0),                       // mov ds1.t0, d4
            tstz0(7, 0x66),                                     // loop: tstz p0, ds0.t3
            bra(0, 9),                                          // (p0) bra end
            dout(false, 7, 0x18, false, 0x18, [0, 1, 0, 0], 3), // doutd ds0.t0, ds0.t1
            alu(4, 0, T0, 0x60, 0x60),                          // add ds0.t0, ds0.t0, ds1.t0
            alu(4, 1, 0x33, 0x66, 0x04),                        // sub ds0.t3, ds0.t3, d6
            bra(7, 3),                                          // bra loop
            HALT,
        ];
        assert_eq!(disassemble(code[7]), "sub ds0.t3, ds0.t3, d6");
        let (_, bus, stop) = run(&data, &code, [0; 3]);
        assert_eq!(stop, Stop::Halt { pc: 9 });
        let addrs: Vec<u32> = bus.outs.iter().map(|o| o.words[0]).collect();
        assert_eq!(addrs, [0x4000, 0x4400, 0x4800]);
        assert!(bus.outs.iter().all(|o| o.kind == DoutType::DMA));
    }

    #[test]
    fn mul_shift_and_the_add_form() {
        let data = [[0x0003_0007, 0x10, 0x20, 0]];
        let code = [
            alu(6, 0, T0, 0x80, 0x01), // mul ds0.t0, ir0, d2.hi? (DS1 word 0 hi) -> 0x7 * 0
            alu(6, 0, 0x31, 0x80, 0x00), // mul ds0.t1, ir0, d2 (DS1 word 0 = 0x20)
            alu(0xf, 0, 0x32, 0x31, 4), // shl ds0.t2, ds0.t1, #4
            alu(0xf, 1, 0x33, 0x00, 16), // shr ds0.t3, d0, #16
            // doutd.add d0(t-window), with W3 from ds1.t1 set below
            dout(true, 7, 0x18, false, 0x18, [0, 1, 2, 3], 3),
            HALT,
        ];
        let mut bus = Bus::program(&data, &code);
        let mut pds = Pds::default();
        pds.launch(&mut bus, 0x1000, 1, [5, 0, 0]).unwrap();
        pds.ds[1][0x31] = 0x1000;
        let stop = pds.run(&mut bus, 100);
        assert_eq!(stop, Stop::Halt { pc: 5 });
        assert_eq!(pds.ds[0][0x30], 0);
        assert_eq!(pds.ds[0][0x31], 5 * 0x20);
        assert_eq!(pds.ds[0][0x32], (5 * 0x20) << 4);
        assert_eq!(pds.ds[0][0x33], 3);
        // W0 = ds0.t0 + W3 (ds1.t1).
        assert_eq!(bus.outs[0].words[0], 0x1000);
    }

    #[test]
    fn persistent_temps_survive_a_launch() {
        let mut bus = Bus::program(&[[9, 9, 9, 9]], &[HALT]);
        let mut pds = Pds::default();
        pds.ds[0][PERSISTENT] = 0x77;
        pds.ds[1][TEMPS] = 0x55;
        pds.launch(&mut bus, 0x1000, 1, [0; 3]).unwrap();
        assert_eq!(pds.ds[0][PERSISTENT], 0x77);
        assert_eq!(pds.ds[1][TEMPS], 0);
    }

    #[test]
    fn the_bus_answers_conditions_and_takes_iterator_and_texture_douts() {
        // `(cc3) bra 2; halt; (cc4) dout2 d0, d1; dout4 d2; halt`.
        let code = [
            bra(3, 2),
            HALT,
            dout(false, 4, 0, false, 0, [0, 1, 0, 0], 2),
            dout(false, 7, 1, false, 0, [0, 0, 0, 0], 4),
            HALT,
        ];
        let data = [[10, 11, 0, 0], [12, 13, 0, 0]];
        let mut bus = Bus::program(&data, &code);
        bus.conds.insert(3, true);
        bus.conds.insert(4, false);
        bus.all_types = true;
        let mut pds = Pds::default();
        pds.launch(&mut bus, 0x1000, 2, [0; 3]).unwrap();
        assert_eq!(pds.run(&mut bus, 100), Stop::Halt { pc: 4 });
        assert_eq!(bus.outs.len(), 1, "cc4 false skips the dout2");
        assert_eq!(bus.outs[0].kind, DoutType::TEXTURE);
        assert_eq!(bus.outs[0].words[0], 12);
        bus.conds.insert(4, true);
        bus.outs.clear();
        pds.launch(&mut bus, 0x1000, 2, [0; 3]).unwrap();
        pds.run(&mut bus, 100);
        assert_eq!(bus.outs[0].kind, DoutType::ITER);
        assert_eq!(bus.outs[0].words[..2], [10, 11]);
    }

    #[test]
    fn unknown_conditions_and_types_stop() {
        let (_, _, stop) = run(&[], &[bra(3, 0)], [0; 3]);
        assert!(matches!(
            stop,
            Stop::Unimplemented {
                why: "condition",
                ..
            }
        ));
        let (pds, _, stop) = run(&[], &[dout(false, 7, 0, false, 0, [0; 4], 2)], [0; 3]);
        assert!(matches!(
            stop,
            Stop::Unimplemented {
                pc: 0,
                why: "DOUT type",
                ..
            }
        ));
        assert_eq!(pds.pc, 0);
        let (_, _, stop) = run(&[], &[0x3000_0000], [0; 3]);
        assert_eq!(
            stop,
            Stop::Undecodable {
                pc: 0,
                word: 0x3000_0000
            }
        );
        // Running off the end of what exists faults.
        let (_, _, stop) = run(&[], &[], [0; 3]);
        assert!(matches!(stop, Stop::Fault { pc: 0, .. }));
    }

    /// The firmware's own PDS programs against our reference listings, when
    /// `RSEMU_SGX_FW` names a directory holding `pds_<name>.loaded.bin` and
    /// `pds_<name>.dis`. Skips without it: the programs are the guest's.
    #[test]
    fn firmware_programs_match_the_reference() {
        extern crate std;
        let Some(dir) = std::env::var_os("RSEMU_SGX_FW").map(std::path::PathBuf::from) else {
            return;
        };
        let mut checked = 0;
        for entry in std::fs::read_dir(&dir).expect("RSEMU_SGX_FW") {
            let path = entry.expect("entry").path();
            let name = path.file_name().and_then(|n| n.to_str()).unwrap_or("");
            let Some(stem) = name
                .strip_suffix(".loaded.bin")
                .filter(|s| s.starts_with("pds_"))
            else {
                continue;
            };
            let Ok(listing) = std::fs::read_to_string(dir.join(alloc::format!("{stem}.dis")))
            else {
                continue;
            };
            let image = std::fs::read(&path).expect("image");
            let rows = listing.lines().filter(|l| l.contains("data row")).count();
            for line in listing.lines().filter(|l| l.contains("] ")) {
                let (_, rest) = line.split_once("] ").unwrap();
                let (hex, text) = rest.split_once("  ").unwrap();
                let word = u32::from_str_radix(hex, 16).unwrap();
                let at = line[line.find('[').unwrap() + 1..line.find(']').unwrap()].trim();
                let index: usize = at.parse().unwrap();
                let off = 16 * rows + 4 * index;
                let raw = u32::from_le_bytes(image[off..off + 4].try_into().unwrap());
                assert_eq!(raw, word, "{stem}[{index}]");
                let want = text.split("   ; use@").next().unwrap().trim_end();
                assert_eq!(disassemble(word), want, "{stem}[{index}]");
                checked += 1;
            }
        }
        std::println!("{checked} PDS words match");
    }
}
