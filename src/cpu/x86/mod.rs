//! The Intel 8086 and 8088 — a real-mode interpreter with a hardware-checked
//! instruction set.
//!
//! `ROADMAP.md` §6 calls x86 "the hard one" and schedules it for phase 6. This
//! is the first, deliberately finishable slice of it: the 8086/8088, which is
//! the whole architecture before protection, paging, 32-bit operands and the
//! two decades of extensions that follow. Everything here is real mode —
//! `segment:offset`, a 20-bit physical address, and a wraparound at 1 MiB that
//! real software depends on.
//!
//! What that buys is a *measurable* milestone. `SingleStepTests/8088` is a
//! hardware-generated corpus: ten thousand vectors per opcode, captured from a
//! real AMD D8088 with an Arduino interposer, complete with the bus trace. The
//! conformance module runs it. `ROADMAP.md` §0 asks for accuracy to be
//! measured rather than asserted, and on this architecture there is no other
//! honest way to claim anything.
//!
//! # What is modelled
//!
//! - Every 8086 instruction, the undocumented encodings included, because the
//!   8086 has no invalid-opcode exception and the corpus tests all of them:
//!   `SALC`, `SETMO`, `POP CS`, the `60`-`6F` jump aliases, the second `RET`
//!   encodings, and the group extensions Intel left blank.
//! - Segmentation: `segment:offset` through a 20-bit adder, with the address
//!   wrapping at 1 MiB — `0xffff:0x0010` is physical `0x00000`, which is the
//!   behaviour the A20 gate was later invented to suppress.
//! - The undefined-flag results. The 8086 leaves flags undefined after a dozen
//!   instructions; real silicon is nevertheless deterministic, and each case
//!   here was matched against the corpus rather than guessed. The private
//!   `exec` module documents them one by one.
//! - A separate I/O address space for `IN`/`OUT`, supplied by the machine as a
//!   second [`AddressSpace`] — not a corner of memory, which is what the
//!   architecture says and what a PC's chipset relies on.
//! - The prefetch queue, four bytes on an 8088 and six on an 8086, with the
//!   flush semantics a control transfer needs.
//! - Interrupts: the vector table at `0000:0000`, NMI, maskable `INTR` with an
//!   acknowledge that fetches the vector, the single-step trap, the divide
//!   error, and the one-instruction interrupt shadow after `MOV SS` and
//!   `POP SS` without which no 8086 stack switch is safe.
//!
//! # What is not
//!
//! Cycle *counts* are documented timing, not measured timing: this core
//! charges four clocks per bus cycle plus the manual's internal execution
//! figures, and does not simulate the overlap between the bus interface unit's
//! prefetching and the execution unit's work. The corpus's per-cycle bus
//! traces are therefore not used as a gate; its *data* accesses are. See the
//! private `conformance` module and `docs/cpu/x86.md`.
//!
//! Also out of scope by design: protected mode, paging, 32-bit operands, and
//! the 80186's extra instructions. Those are i386 work, and i386 is a
//! different milestone.
//!
//! # Assembling one
//!
//! ```
//! use std::sync::Arc;
//! use rsemu::core::space::{AddressSpace, RamStore, Region};
//! use rsemu::cpu::x86::{Config, I8086};
//!
//! // A megabyte of RAM, and `mov ax, 0x1234` at the reset vector.
//! let ram = Arc::new(RamStore::new(0x10_0000));
//! for (i, b) in [0xb8u8, 0x34, 0x12].into_iter().enumerate() {
//!     ram.write_u8(0xffff0 + i as u64, b).unwrap();
//! }
//!
//! let mem = AddressSpace::new("mem", 20);
//! mem.topology().map(Region::ram("ram", ram), 0).unwrap();
//!
//! let cpu = I8086::new(Config::default());
//! cpu.attach_space(Arc::new(mem));
//! cpu.step();                       // the reset sequence
//! assert_eq!(cpu.regs().cs, 0xffff);
//! cpu.step();                       // mov ax, 0x1234
//! assert_eq!(cpu.regs().ax, 0x1234);
//! ```
//!
//! # Modules
//!
//! | Module | Holds |
//! | --- | --- |
//! | [`isa`] | the one declarative instruction table, and the stream decoder both the interpreter and the disassembler use |
//! | [`disasm`] | the disassembler generated from that table |
//! | `exec` (private) | the interpreter, its flag rules, and the undefined-flag notes |
//!
//! # Sources
//!
//! Hardware documentation only (`ROADMAP.md` §1): Intel's *iAPX 86/88,
//! 186/188 User's Manual* and *8086 Family User's Manual* (bitsavers has the
//! originals), the instruction-set summary and timing tables therein, and
//! sandpile.org's encoding tables for what the manual leaves out. Undefined
//! behaviour was measured against `SingleStepTests/8088` (MIT), which is
//! hardware output rather than anyone's emulator. **No copyleft emulator was
//! consulted** — `docs/cpu/x86.md` names the three that people reach for when
//! x86 gets hard and records that all three are forbidden.

pub mod disasm;
mod exec;
pub mod isa;

#[cfg(test)]
mod tests;

// The conformance runner reads a downloaded corpus off the filesystem, so it
// exists only where there is one (`ROADMAP.md` §12).
#[cfg(all(test, feature = "std"))]
mod conformance;

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, Initiator, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::registry::Registry;
use crate::core::space::{AddressSpace, MemAttrs, RequesterId};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{self, AtomicBool, AtomicU32, LockRank, Ordering};
use crate::core::value::Width;
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink};

use exec::{Exec, State};

/// The flags register.
///
/// Bits 1, 12, 13, 14 and 15 have no storage on an 8086: they read as one.
/// Bits 3 and 5 read as zero. That is not a convention this core invented —
/// it is what the corpus's captured flag words show on every vector, and
/// [`Regs::normalise_flags`] is where it is enforced.
pub mod flags {
    /// Carry.
    pub const CF: u16 = 0x0001;
    /// Parity of the low eight bits of the result.
    pub const PF: u16 = 0x0004;
    /// Auxiliary (BCD) carry out of bit 3.
    pub const AF: u16 = 0x0010;
    /// Zero.
    pub const ZF: u16 = 0x0040;
    /// Sign — a copy of the result's most significant bit.
    pub const SF: u16 = 0x0080;
    /// Trap: take a type-1 interrupt after every instruction.
    pub const TF: u16 = 0x0100;
    /// Interrupt enable, for the maskable `INTR` input only.
    pub const IF: u16 = 0x0200;
    /// Direction: string operations count down when set.
    pub const DF: u16 = 0x0400;
    /// Signed overflow.
    pub const OF: u16 = 0x0800;

    /// Every bit that has storage.
    pub const DEFINED: u16 = CF | PF | AF | ZF | SF | TF | IF | DF | OF;

    /// The bits that always read as one on an 8086.
    pub const RESERVED_SET: u16 = 0xf002;

    /// The status bits `SAHF` and `LAHF` move between `AH` and the flags.
    pub const LOW_BYTE: u16 = CF | PF | AF | ZF | SF;
}

/// Which part this is.
///
/// The 8086 and 8088 are the same processor with different bus interface
/// units: the 8088 fetches a byte at a time into a four-byte queue, the 8086 a
/// word at a time into a six-byte one. Nothing architectural differs, which is
/// why this is a construction property rather than a separate core.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Model {
    /// 16-bit external bus, six-byte prefetch queue.
    I8086,
    /// 8-bit external bus, four-byte prefetch queue. The IBM PC's processor,
    /// and the one `SingleStepTests/8088` was captured from.
    I8088,
}

impl Model {
    /// How many bytes the prefetch queue holds.
    #[must_use]
    pub const fn queue_bytes(self) -> u8 {
        match self {
            Model::I8086 => 6,
            Model::I8088 => 4,
        }
    }

    /// How many bytes one bus cycle can transfer.
    #[must_use]
    pub const fn bus_bytes(self) -> u8 {
        match self {
            Model::I8086 => 2,
            Model::I8088 => 1,
        }
    }

    /// The part's name, as a machine description writes it.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Model::I8086 => "8086",
            Model::I8088 => "8088",
        }
    }
}

impl fmt::Display for Model {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// How this particular part is configured.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// 8086 or 8088 — the bus width and the prefetch queue depth.
    pub model: Model,
    /// This core's identity in `MemAttrs::requester`, for an IOMMU or a
    /// per-master filter.
    pub requester: RequesterId,
}

impl Config {
    /// An 8088: 8-bit bus, four-byte queue.
    pub const I8088: Config = Config {
        model: Model::I8088,
        requester: RequesterId::ANONYMOUS,
    };

    /// An 8086: 16-bit bus, six-byte queue.
    pub const I8086: Config = Config {
        model: Model::I8086,
        ..Config::I8088
    };

    /// Same configuration, with a different requester id.
    #[must_use]
    pub const fn with_requester(mut self, id: RequesterId) -> Self {
        self.requester = id;
        self
    }
}

impl Default for Config {
    /// An 8088, because that is the part the conformance corpus was captured
    /// from and therefore the one whose behaviour here is checked.
    fn default() -> Self {
        Config::I8088
    }
}

/// The physical address a `segment:offset` pair names.
///
/// The 8086 has no address translation: the segment is shifted left four bits
/// and added to the offset in a 20-bit adder. The mask is the whole point —
/// `0xffff:0x0010` is `0x100000`, which wraps to `0x00000`, and DOS software
/// used that wrap deliberately. The IBM AT's A20 gate exists because the
/// 80286 stopped wrapping and broke those programs.
#[inline]
#[must_use]
pub const fn linear(segment: u16, offset: u16) -> u32 {
    (((segment as u32) << 4).wrapping_add(offset as u32)) & 0xf_ffff
}

/// The architectural register file.
///
/// Public and `Copy` because a debugger, a tracer and a test all want to read
/// it out and put it back — this is the surface a future gdbstub serialises
/// (`ROADMAP.md` §9's debug story), and [`Reg`] enumerates it by name.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Regs {
    /// Accumulator.
    pub ax: u16,
    /// Count — the implicit operand of `LOOP`, the string repeats, and the
    /// variable shifts.
    pub cx: u16,
    /// Data — the implicit high half of a multiply or divide, and the I/O port
    /// register.
    pub dx: u16,
    /// Base.
    pub bx: u16,
    /// Stack pointer, in `SS`.
    pub sp: u16,
    /// Base pointer; addressing modes that use it default to `SS`.
    pub bp: u16,
    /// Source index, for the string instructions.
    pub si: u16,
    /// Destination index, for the string instructions.
    pub di: u16,
    /// Extra segment — always the destination of a string instruction.
    pub es: u16,
    /// Code segment.
    pub cs: u16,
    /// Stack segment.
    pub ss: u16,
    /// Data segment.
    pub ds: u16,
    /// Instruction pointer, an offset within `CS`.
    pub ip: u16,
    /// The flags register. See [`flags`].
    pub flags: u16,
}

/// The 16-bit register order used by the ModRM `reg` and `rm` fields.
const WORD_ORDER: [Reg; 8] = [
    Reg::Ax,
    Reg::Cx,
    Reg::Dx,
    Reg::Bx,
    Reg::Sp,
    Reg::Bp,
    Reg::Si,
    Reg::Di,
];

impl Regs {
    /// The state a power-on reset leaves behind.
    ///
    /// `CS:IP` is `ffff:0000`, sixteen bytes below the top of memory, which is
    /// why every PC has a far jump there. Every other segment register is
    /// zero, and the flags hold only their hard-wired bits.
    #[must_use]
    pub const fn new() -> Regs {
        Regs {
            ax: 0,
            cx: 0,
            dx: 0,
            bx: 0,
            sp: 0,
            bp: 0,
            si: 0,
            di: 0,
            es: 0,
            cs: 0xffff,
            ss: 0,
            ds: 0,
            ip: 0,
            flags: flags::RESERVED_SET,
        }
    }

    /// Force the flags word's hard-wired bits into shape.
    ///
    /// Applied on every write to the flags register, not only on
    /// `POPF`/`IRET`: the bits have no storage, so a value that has been
    /// through this is the only value the register can ever hold.
    #[inline]
    #[must_use]
    pub const fn normalise_flags(value: u16) -> u16 {
        (value & flags::DEFINED) | flags::RESERVED_SET
    }

    /// Whether a flag is set.
    #[inline]
    #[must_use]
    pub const fn flag(&self, mask: u16) -> bool {
        self.flags & mask != 0
    }

    /// Read one of the eight 16-bit registers by ModRM number.
    #[inline]
    #[must_use]
    pub const fn word(&self, index: u8) -> u16 {
        match index & 7 {
            0 => self.ax,
            1 => self.cx,
            2 => self.dx,
            3 => self.bx,
            4 => self.sp,
            5 => self.bp,
            6 => self.si,
            _ => self.di,
        }
    }

    /// Write one of the eight 16-bit registers by ModRM number.
    #[inline]
    pub const fn set_word(&mut self, index: u8, value: u16) {
        match index & 7 {
            0 => self.ax = value,
            1 => self.cx = value,
            2 => self.dx = value,
            3 => self.bx = value,
            4 => self.sp = value,
            5 => self.bp = value,
            6 => self.si = value,
            _ => self.di = value,
        }
    }

    /// Read one of the eight 8-bit registers by ModRM number.
    ///
    /// Numbers 0-3 are the low halves of `AX`-`BX` and 4-7 the high halves, in
    /// the same register order — which is why `AH` is 4 and not 1.
    #[inline]
    #[must_use]
    pub const fn byte(&self, index: u8) -> u8 {
        let word = self.word(index & 3);
        if index & 4 == 0 {
            word as u8
        } else {
            (word >> 8) as u8
        }
    }

    /// Write one of the eight 8-bit registers by ModRM number.
    #[inline]
    pub const fn set_byte(&mut self, index: u8, value: u8) {
        let word = self.word(index & 3);
        let merged = if index & 4 == 0 {
            (word & 0xff00) | value as u16
        } else {
            (word & 0x00ff) | ((value as u16) << 8)
        };
        self.set_word(index & 3, merged);
    }

    /// Read a segment register by number: `ES`, `CS`, `SS`, `DS`.
    #[inline]
    #[must_use]
    pub const fn segment(&self, index: u8) -> u16 {
        match index & 3 {
            0 => self.es,
            1 => self.cs,
            2 => self.ss,
            _ => self.ds,
        }
    }

    /// Write a segment register by number.
    #[inline]
    pub const fn set_segment(&mut self, index: u8, value: u16) {
        match index & 3 {
            0 => self.es = value,
            1 => self.cs = value,
            2 => self.ss = value,
            _ => self.ds = value,
        }
    }
}

impl Default for Regs {
    fn default() -> Self {
        Regs::new()
    }
}

impl fmt::Display for Regs {
    /// The two-line form a trace log wants.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "AX:{:04x} BX:{:04x} CX:{:04x} DX:{:04x} SP:{:04x} BP:{:04x} SI:{:04x} DI:{:04x} \
             ES:{:04x} CS:{:04x} SS:{:04x} DS:{:04x} IP:{:04x} F:{:04x}",
            self.ax,
            self.bx,
            self.cx,
            self.dx,
            self.sp,
            self.bp,
            self.si,
            self.di,
            self.es,
            self.cs,
            self.ss,
            self.ds,
            self.ip,
            self.flags
        )
    }
}

/// One named register, for a debugger that works by name or index.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Reg {
    /// Accumulator.
    Ax,
    /// Count.
    Cx,
    /// Data.
    Dx,
    /// Base.
    Bx,
    /// Stack pointer.
    Sp,
    /// Base pointer.
    Bp,
    /// Source index.
    Si,
    /// Destination index.
    Di,
    /// Extra segment.
    Es,
    /// Code segment.
    Cs,
    /// Stack segment.
    Ss,
    /// Data segment.
    Ds,
    /// Instruction pointer.
    Ip,
    /// Flags.
    Flags,
}

impl Reg {
    /// Every register, in the order a debugger should list them.
    pub const ALL: &'static [Reg] = &[
        Reg::Ax,
        Reg::Cx,
        Reg::Dx,
        Reg::Bx,
        Reg::Sp,
        Reg::Bp,
        Reg::Si,
        Reg::Di,
        Reg::Es,
        Reg::Cs,
        Reg::Ss,
        Reg::Ds,
        Reg::Ip,
        Reg::Flags,
    ];

    /// The register's name, lower case, as gdb and the monitor spell it.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Reg::Ax => "ax",
            Reg::Cx => "cx",
            Reg::Dx => "dx",
            Reg::Bx => "bx",
            Reg::Sp => "sp",
            Reg::Bp => "bp",
            Reg::Si => "si",
            Reg::Di => "di",
            Reg::Es => "es",
            Reg::Cs => "cs",
            Reg::Ss => "ss",
            Reg::Ds => "ds",
            Reg::Ip => "ip",
            Reg::Flags => "flags",
        }
    }

    /// How wide the register is. Every 8086 register is 16 bits.
    #[must_use]
    pub const fn width(self) -> Width {
        Width::U16
    }

    /// Read this register out of a register file.
    #[must_use]
    pub const fn get(self, regs: &Regs) -> u16 {
        match self {
            Reg::Ax => regs.ax,
            Reg::Cx => regs.cx,
            Reg::Dx => regs.dx,
            Reg::Bx => regs.bx,
            Reg::Sp => regs.sp,
            Reg::Bp => regs.bp,
            Reg::Si => regs.si,
            Reg::Di => regs.di,
            Reg::Es => regs.es,
            Reg::Cs => regs.cs,
            Reg::Ss => regs.ss,
            Reg::Ds => regs.ds,
            Reg::Ip => regs.ip,
            Reg::Flags => regs.flags,
        }
    }

    /// Write this register into a register file.
    ///
    /// A write to `flags` goes through [`Regs::normalise_flags`], because the
    /// hard-wired bits cannot be written on hardware either.
    pub const fn set(self, regs: &mut Regs, value: u16) {
        match self {
            Reg::Ax => regs.ax = value,
            Reg::Cx => regs.cx = value,
            Reg::Dx => regs.dx = value,
            Reg::Bx => regs.bx = value,
            Reg::Sp => regs.sp = value,
            Reg::Bp => regs.bp = value,
            Reg::Si => regs.si = value,
            Reg::Di => regs.di = value,
            Reg::Es => regs.es = value,
            Reg::Cs => regs.cs = value,
            Reg::Ss => regs.ss = value,
            Reg::Ds => regs.ds = value,
            Reg::Ip => regs.ip = value,
            Reg::Flags => regs.flags = Regs::normalise_flags(value),
        }
    }

    /// Look a register up by name.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Reg> {
        Reg::ALL.iter().copied().find(|r| r.name() == name)
    }

    /// The register the ModRM `reg`/`rm` field selects for a word operand.
    #[must_use]
    pub const fn from_word_index(index: u8) -> Reg {
        WORD_ORDER[(index & 7) as usize]
    }
}

impl fmt::Display for Reg {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// Which interrupt a poll latched.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Interrupt {
    /// Maskable, level-sensitive, and vectored by whatever answers the
    /// acknowledge cycle. Taken only while `IF` is set.
    Intr,
    /// Non-maskable, edge-sensitive, vectored through entry 2. `IF` does not
    /// gate it.
    Nmi,
}

/// The interrupt input pins, kept outside the execution lock.
///
/// Deliberately atomics rather than fields under the mutex: a device asserting
/// `INTR` from inside a write the CPU itself issued would otherwise re-enter
/// the CPU's own critical section, which is a deadlock under `native-std` and
/// a panic under `single`. The re-entrancy contract says mutate your own state
/// in a short critical section and act outward afterwards; a pin that is one
/// atomic store needs no critical section at all (`ROADMAP.md` §4.7).
#[derive(Debug, Default)]
pub(crate) struct Lines {
    /// `INTR` is level-sensitive: it is taken whenever it is asserted and `IF`
    /// is set.
    intr: AtomicBool,
    /// The vector the acknowledge cycle would return.
    ///
    /// On a real PC the 8259A drives it onto the data bus during the second
    /// `INTA` cycle. Modelling it as a latched byte rather than a bus
    /// transaction keeps the interrupt controller out of the CPU's type
    /// signature (`ROADMAP.md` §15, invariant 1); a controller that wants the
    /// real handshake sets this from its own acknowledge path.
    intr_vector: AtomicU32,
    /// The last level seen on NMI, for edge detection.
    nmi_level: AtomicBool,
    /// NMI is edge-sensitive: a rising edge sets this latch, which stays set
    /// until the interrupt is serviced, however long that takes.
    nmi_latch: AtomicBool,
}

impl Lines {
    fn set_intr(&self, asserted: bool) {
        self.intr.store(asserted, Ordering::Release);
    }

    fn intr_asserted(&self) -> bool {
        self.intr.load(Ordering::Acquire)
    }

    fn set_intr_vector(&self, vector: u8) {
        self.intr_vector.store(u32::from(vector), Ordering::Release);
    }

    pub(crate) fn intr_vector(&self) -> u8 {
        self.intr_vector.load(Ordering::Acquire) as u8
    }

    /// Drive the NMI pin, latching a rising edge.
    fn set_nmi(&self, asserted: bool) {
        let previous = self.nmi_level.swap(asserted, Ordering::AcqRel);
        if asserted && !previous {
            self.nmi_latch.store(true, Ordering::Release);
        }
    }

    pub(crate) fn nmi_pending(&self) -> bool {
        self.nmi_latch.load(Ordering::Acquire)
    }

    /// Consume the NMI latch, reporting whether it was set.
    pub(crate) fn take_nmi_pending(&self) -> bool {
        self.nmi_latch.swap(false, Ordering::AcqRel)
    }

    pub(crate) fn intr_pending(&self) -> bool {
        self.intr_asserted()
    }

    fn clear_nmi_latch(&self) {
        self.nmi_latch.store(false, Ordering::Release);
    }

    fn snapshot(&self) -> (bool, bool, bool, u8) {
        (
            self.intr_asserted(),
            self.nmi_level.load(Ordering::Acquire),
            self.nmi_pending(),
            self.intr_vector(),
        )
    }

    fn restore(&self, (intr, level, latch, vector): (bool, bool, bool, u8)) {
        self.intr.store(intr, Ordering::Release);
        self.nmi_level.store(level, Ordering::Release);
        self.nmi_latch.store(latch, Ordering::Release);
        self.intr_vector.store(u32::from(vector), Ordering::Release);
    }
}

/// Everything the interpreter needs to mutate, behind one lock.
#[derive(Debug)]
struct Session {
    state: State,
    memory: Option<Arc<AddressSpace>>,
    io: Option<Arc<AddressSpace>>,
}

/// An Intel 8086 or 8088 core.
///
/// # Locking
///
/// Execution state sits behind one [`sync::Mutex`] at [`LockRank::BUS`]. That
/// rank, rather than `DEVICE`, because a CPU is a bus master: it holds this
/// lock while calling into device models, which take their own
/// `DEVICE`-ranked locks, which drive `WIRE`-ranked lines. The ladder runs in
/// the direction calls travel, so the debug order check passes for the real
/// call graph and fires on an inverted one.
///
/// The interrupt pins are *not* under that lock: they are atomics, so a device
/// asserting `INTR` from inside a write the CPU itself issued cannot re-enter
/// the CPU's own critical section.
#[derive(Debug)]
pub struct I8086 {
    cfg: Config,
    lines: Lines,
    session: sync::Mutex<Session>,
}

impl I8086 {
    /// A core in its power-on state, with no address space yet.
    ///
    /// Two-phase construction (`ROADMAP.md` §4.4): nothing observable happens
    /// until [`attach_space`](I8086::attach_space) and [`Device::realize`].
    /// The first [`step`](I8086::step) runs the reset sequence.
    #[must_use]
    pub fn new(cfg: Config) -> I8086 {
        I8086 {
            cfg,
            lines: Lines::default(),
            session: sync::Mutex::with_rank(
                LockRank::BUS,
                Session {
                    state: State::new(cfg.model),
                    memory: None,
                    io: None,
                },
            ),
        }
    }

    /// Build one from machine-description properties.
    ///
    /// # Errors
    ///
    /// If a property has the wrong type, `model` is not `8086` or `8088`, or a
    /// property nothing here accepts was given — a typo'd property that was
    /// silently ignored is an afternoon lost.
    pub fn from_props(props: &Props) -> Result<I8086> {
        let mut r = props.reader();
        let model = r.or_enum("model", "8088", &["8086", "8088"])?;
        let model = if model == "8086" {
            Model::I8086
        } else {
            Model::I8088
        };
        r.finish()?;
        Ok(I8086::new(Config {
            model,
            requester: RequesterId::ANONYMOUS,
        }))
    }

    /// This core's configuration.
    #[must_use]
    pub fn config(&self) -> Config {
        self.cfg
    }

    /// Give the core the memory space it executes from.
    ///
    /// Separate from construction because the space is built by the machine
    /// assembly layer; when `RealizeCtx` grows space accessors this moves into
    /// [`Device::realize`] and the method stays as the way a test wires one up.
    pub fn attach_space(&self, space: Arc<AddressSpace>) {
        self.session.lock().memory = Some(space);
    }

    /// Give the core the **separate** I/O address space `IN` and `OUT` reach.
    ///
    /// Not a window into memory: the 8086 drives a distinct status line for an
    /// I/O cycle, and a PC's chipset decodes it differently. A core with no
    /// I/O space reads ones and discards writes, which is what an unpopulated
    /// bus does.
    pub fn attach_io_space(&self, space: Arc<AddressSpace>) {
        self.session.lock().io = Some(space);
    }

    /// The memory space this core executes from, if one is attached.
    #[must_use]
    pub fn space(&self) -> Option<Arc<AddressSpace>> {
        self.session.lock().memory.clone()
    }

    /// The I/O space, if one is attached.
    #[must_use]
    pub fn io_space(&self) -> Option<Arc<AddressSpace>> {
        self.session.lock().io.clone()
    }

    /// The register file.
    #[must_use]
    pub fn regs(&self) -> Regs {
        self.session.lock().state.regs
    }

    /// Overwrite the register file — a debugger, a test vector, a snapshot.
    ///
    /// The prefetch queue is flushed, because the bytes in it belonged to the
    /// old `CS:IP`.
    pub fn set_regs(&self, regs: Regs) {
        let mut session = self.session.lock();
        session.state.regs = regs;
        session.state.regs.flags = Regs::normalise_flags(regs.flags);
        session.state.queue.flush();
    }

    /// Read one register by name.
    #[must_use]
    pub fn reg(&self, reg: Reg) -> u16 {
        reg.get(&self.session.lock().state.regs)
    }

    /// Write one register by name.
    pub fn set_reg(&self, reg: Reg, value: u16) {
        let mut session = self.session.lock();
        reg.set(&mut session.state.regs, value);
        if matches!(reg, Reg::Cs | Reg::Ip) {
            session.state.queue.flush();
        }
    }

    /// Clock cycles executed since power-on.
    ///
    /// Four per bus cycle plus the manual's internal execution figures. This
    /// is documented timing rather than measured timing: the bus interface
    /// unit's prefetching is not overlapped with execution here, so a count
    /// taken over a long run is an upper bound rather than the number a
    /// logic analyser would show. The module documentation says why that
    /// trade was made.
    #[must_use]
    pub fn cycles(&self) -> u64 {
        self.session.lock().state.cycles
    }

    /// Whether a `HLT` has stopped the core.
    ///
    /// A halted 8086 restarts on any interrupt, so this is not the terminal
    /// state a `JAM` is on a 6502 — but a scheduler still has to notice it
    /// rather than spin, because [`step`](I8086::step) charges nothing while
    /// it holds.
    #[must_use]
    pub fn is_halted(&self) -> bool {
        self.session.lock().state.halted
    }

    /// Whether a reset sequence is still owed.
    #[must_use]
    pub fn reset_pending(&self) -> bool {
        self.session.lock().state.reset_pending
    }

    /// The prefetch queue's current contents, oldest byte first.
    ///
    /// Exposed because it is genuinely observable on an 8088 — the queue
    /// status lines are pins — and because the conformance corpus specifies
    /// it as part of a vector's initial state.
    #[must_use]
    pub fn prefetch_queue(&self) -> Vec<u8> {
        self.session.lock().state.queue.contents()
    }

    /// Install a prefetch queue, as if the bus interface unit had already
    /// fetched these bytes from `CS:IP` onwards.
    ///
    /// # Errors
    ///
    /// If more bytes are supplied than the part's queue holds.
    pub fn set_prefetch_queue(&self, bytes: &[u8]) -> Result<()> {
        let mut session = self.session.lock();
        session.state.queue.install(bytes).map_err(|()| {
            Error::Property(alloc::format!(
                "the {} prefetch queue holds {} bytes, not {}",
                self.cfg.model,
                self.cfg.model.queue_bytes(),
                bytes.len()
            ))
        })
    }

    /// How many accesses an address space refused, and where the last one was.
    ///
    /// The 8086 has no bus-error input, so a refused access cannot raise an
    /// exception: a read returns whatever was last on the data bus, which is
    /// what an unterminated bus does. This counter is how that becomes visible
    /// instead of silent — a machine whose memory map has a hole will show it
    /// climbing.
    #[must_use]
    pub fn bus_faults(&self) -> (u64, u32) {
        let s = self.session.lock();
        (s.state.faults, s.state.last_fault)
    }

    /// Drive the `INTR` pin. Level-sensitive: it is taken while asserted and
    /// `IF` is set.
    ///
    /// `asserted` is the logical level, not the pin's polarity; inverting a
    /// real signal belongs to whatever models the wire.
    pub fn set_intr(&self, asserted: bool) {
        self.lines.set_intr(asserted);
    }

    /// Whether `INTR` is currently asserted.
    #[must_use]
    pub fn intr_asserted(&self) -> bool {
        self.lines.intr_asserted()
    }

    /// Set the vector the next `INTR` acknowledge will read.
    ///
    /// A PC's 8259A drives this onto the data bus during the second `INTA`
    /// cycle. Setting it before asserting `INTR` is the whole handshake as far
    /// as the CPU is concerned.
    pub fn set_intr_vector(&self, vector: u8) {
        self.lines.set_intr_vector(vector);
    }

    /// The vector the next acknowledge would read.
    #[must_use]
    pub fn intr_vector(&self) -> u8 {
        self.lines.intr_vector()
    }

    /// Drive the NMI pin. Edge-sensitive: a rising edge latches, and the latch
    /// survives until the interrupt is taken.
    pub fn set_nmi(&self, asserted: bool) {
        self.lines.set_nmi(asserted);
    }

    /// A complete NMI pulse, for a caller that does not model the pin's level.
    pub fn pulse_nmi(&self) {
        self.lines.set_nmi(true);
        self.lines.set_nmi(false);
    }

    /// Whether an NMI edge is latched and not yet serviced.
    #[must_use]
    pub fn nmi_pending(&self) -> bool {
        self.lines.nmi_pending()
    }

    /// Whether the next instruction runs with interrupts inhibited.
    ///
    /// Set for exactly one instruction after `MOV SS,x` and `POP SS`, so that
    /// the `SS:SP` pair can be reloaded without an interrupt landing on the
    /// half-changed stack. On an 8086 the shadow covers NMI as well as
    /// `INTR`, which later parts changed.
    #[must_use]
    pub fn interrupt_shadow(&self) -> bool {
        self.session.lock().state.int_shadow
    }

    /// Request a reset sequence without changing any register.
    ///
    /// The sequence runs on the next [`step`](I8086::step): a reset is a
    /// signal, not a method call.
    pub fn request_reset(&self) {
        self.session.lock().state.reset_pending = true;
    }

    /// Execute one reset sequence, interrupt sequence, or instruction.
    ///
    /// Returns the clock cycles charged: zero if the core is halted with no
    /// interrupt pending, or has no address space, which the caller must treat
    /// as "stop", not "retry".
    pub fn step(&self) -> u64 {
        let mut session = self.session.lock();
        let Session { state, memory, io } = &mut *session;
        let Some(memory) = memory.clone() else {
            return 0;
        };
        let io = io.clone();
        Exec::new(state, &memory, io.as_deref(), &self.cfg, &self.lines).step()
    }

    /// Execute until at least `budget` cycles have been charged.
    ///
    /// Returns the cycles actually used, which overshoots by at most one
    /// instruction — an 8086 cannot be stopped mid-instruction, and pretending
    /// otherwise is how a scheduler ends up with a CPU in an impossible state.
    /// Stops early if the core halts.
    pub fn run(&self, budget: u64) -> u64 {
        let mut used = 0;
        while used < budget {
            let n = self.step();
            if n == 0 {
                break;
            }
            used += n;
        }
        used
    }

    /// Disassemble `count` instructions starting at `cs:ip`, reading guest
    /// memory with debug attributes.
    ///
    /// Debug attributes are the point: a monitor listing the code around `IP`
    /// must not pop a FIFO or clear a status bit on the way (`ROADMAP.md`
    /// §15, invariant 5).
    #[must_use]
    pub fn disassemble(&self, cs: u16, ip: u16, count: usize) -> Vec<disasm::Disassembled> {
        let Some(space) = self.space() else {
            return Vec::new();
        };
        disasm::disassemble_run(cs, ip, count, |addr| {
            space
                .read(u64::from(addr), Width::U8, MemAttrs::DEBUG)
                .ok()
                .map(|v| v as u8)
        })
    }
}

/// The `cpu.i8086` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: "cpu.i8086",
    version: 1,
    summary: "Intel 8086 / 8088 16-bit CPU core, real mode, hardware-checked interpreter",
    properties: &[PropertySpec {
        name: "model",
        kind: ValueKind::Str,
        required: false,
        summary: "\"8086\" (16-bit bus, 6-byte queue) or \"8088\" (8-bit bus, 4-byte queue)",
    }],
    construct: |props| Ok(Box::new(I8086::from_props(props)?)),
};

/// Add this core's class to a registry.
///
/// Registration is explicit per feature rather than link-time magic
/// (`ROADMAP.md` §4.4), so the machine assembly layer calls this from its own
/// `#[cfg(feature = "cpu-x86")]` arm.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut Registry) -> Result<()> {
    reg.add(&CLASS)
}

impl Device for I8086 {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // A CPU with no address space cannot fetch, and failing here is the
        // difference between a config error and a machine that runs zero
        // instructions and says nothing.
        if self.session.lock().memory.is_none() {
            return Err(ctx.error("no memory address space attached to this core"));
        }
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        let mut session = self.session.lock();
        if kind == ResetKind::Cold {
            session.state = State::new(self.cfg.model);
        } else {
            // A warm reset is a pulse on the RESET pin, and on an 8086 that is
            // not subtle: the sequence loads CS:IP itself, so what survives is
            // only the general registers.
            session.state.reset_pending = true;
            session.state.halted = false;
            session.state.int_shadow = false;
            session.state.queue.flush();
        }
        drop(session);
        if kind == ResetKind::Cold {
            self.lines.restore((false, false, false, 0));
        } else {
            // The input *levels* belong to whatever drives them, not to the
            // CPU — clearing them here would make a reset lie about the
            // machine. The edge latch is internal, so it goes.
            self.lines.clear_nmi_latch();
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.session.lock().state;
        for reg in Reg::ALL {
            w.write_u16(reg.get(&state.regs))?;
        }
        w.write_u64(state.cycles)?;
        w.write_bool(state.halted)?;
        w.write_bool(state.reset_pending)?;
        w.write_bool(state.int_shadow)?;
        w.write_u8(state.open_bus)?;
        w.write_u64(state.faults)?;
        w.write_u32(state.last_fault)?;
        let queue = state.queue.contents();
        w.write_u8(queue.len() as u8)?;
        for byte in queue {
            w.write_u8(byte)?;
        }
        let (intr, nmi_level, nmi_latch, vector) = self.lines.snapshot();
        w.write_bool(intr)?;
        w.write_bool(nmi_level)?;
        w.write_bool(nmi_latch)?;
        w.write_u8(vector)?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::new(self.cfg.model);
        for reg in Reg::ALL {
            let value = r.read_u16()?;
            reg.set(&mut state.regs, value);
        }
        state.cycles = r.read_u64()?;
        state.halted = r.read_bool()?;
        state.reset_pending = r.read_bool()?;
        state.int_shadow = r.read_bool()?;
        state.open_bus = r.read_u8()?;
        state.faults = r.read_u64()?;
        state.last_fault = r.read_u32()?;
        let len = r.read_u8()?;
        let mut queue = Vec::with_capacity(usize::from(len));
        for _ in 0..len {
            queue.push(r.read_u8()?);
        }
        state.queue.install(&queue).map_err(|()| {
            Error::State(alloc::format!(
                "snapshot has a {len}-byte prefetch queue; an {} holds {}",
                self.cfg.model,
                self.cfg.model.queue_bytes()
            ))
        })?;
        let intr = r.read_bool()?;
        let nmi_level = r.read_bool()?;
        let nmi_latch = r.read_bool()?;
        let vector = r.read_u8()?;
        self.session.lock().state = state;
        self.lines.restore((intr, nmi_level, nmi_latch, vector));
        Ok(())
    }
}

impl Initiator for I8086 {
    fn requester(&self) -> RequesterId {
        self.cfg.requester
    }
}

/// One of the CPU's two interrupt inputs, as something a [`Wire`] can drive.
///
/// A wire hands each sink the level of the *driver that changed*, not the
/// resolved level of the net, because a net with several drivers is resolved
/// by whoever cares. A PC's `INTR` line comes from one 8259A, but its NMI is
/// wire-ORed from the parity checker and the coprocessor, so this keeps a
/// [`FanIn`].
///
/// [`Wire`]: crate::core::wire::Wire
#[derive(Debug)]
pub struct InterruptPin {
    cpu: Arc<I8086>,
    which: Interrupt,
    inputs: FanIn,
    resolve: Resolve,
}

impl InterruptPin {
    /// Connect `which` pin of `cpu` to a net driven by `sources`.
    ///
    /// Wire-OR by default: any source asserting asserts the pin, which is how
    /// an open-collector interrupt line behaves.
    #[must_use]
    pub fn new(cpu: Arc<I8086>, which: Interrupt, sources: &[WireId]) -> InterruptPin {
        InterruptPin {
            cpu,
            which,
            inputs: FanIn::new(sources),
            resolve: Resolve::Or,
        }
    }

    /// The same pin with an explicit resolution rule.
    #[must_use]
    pub fn with_resolve(mut self, resolve: Resolve) -> Self {
        self.resolve = resolve;
        self
    }

    /// Which pin this is.
    #[must_use]
    pub fn which(&self) -> Interrupt {
        self.which
    }

    /// The per-source levels currently seen.
    #[must_use]
    pub fn inputs(&self) -> &FanIn {
        &self.inputs
    }
}

impl WireSink for InterruptPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let asserted = self.inputs.resolve(self.resolve).is_high();
        match self.which {
            Interrupt::Intr => self.cpu.set_intr(asserted),
            Interrupt::Nmi => self.cpu.set_nmi(asserted),
        }
    }
}

/// A description of this core's opcode map, for `rsemu describe cpu.i8086`.
///
/// Built from [`isa::TABLE`], so it cannot drift from what the interpreter
/// implements. Group opcodes are expanded one extension per line, because a
/// map that hides `F7 /6` hides the divide.
#[must_use]
pub fn describe_isa() -> String {
    use core::fmt::Write as _;
    let mut out = String::new();
    for opcode in 0..=255u8 {
        let insn = isa::decode(opcode);
        let mark = |class: isa::Class| match class {
            isa::Class::Documented => ' ',
            isa::Class::Alias => '=',
            isa::Class::Undocumented => '*',
            isa::Class::Undefined => '?',
            isa::Class::Prefix => ':',
            isa::Class::Escape => '~',
        };
        if insn.group == isa::Grp::None {
            let _ = writeln!(
                out,
                "{opcode:02x}    {}{:<6} {}",
                mark(insn.class),
                insn.op.mnemonic(),
                insn.op.summary()
            );
        } else {
            for reg in 0..8u8 {
                let row = isa::resolve(insn, reg);
                let _ = writeln!(
                    out,
                    "{opcode:02x}/{reg} {}{:<6} {}",
                    mark(row.class),
                    row.op.mnemonic(),
                    row.op.summary()
                );
            }
        }
    }
    out
}
