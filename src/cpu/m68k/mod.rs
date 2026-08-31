//! The Motorola MC68000 — a bus-accurate interpreter with a modelled prefetch
//! queue.
//!
//! The plain 68000, as fitted to the Amiga, the Atari ST, the Mega Drive and
//! the first Macintoshes: 32-bit registers, a 16-bit data bus, 24 address
//! pins, two stack pointers and a supervisor/user split. Not the 68010 or the
//! 68020 — no `MOVEC`, no `BFEXTU`, no 32-bit multiply, and the 68000's own
//! group-0 exception frame rather than the 68010's format words.
//!
//! # What "bus-accurate" means here
//!
//! A 68000 bus cycle is four clocks, and every published instruction time is a
//! sum of bus cycles and microcode idle cycles (MC68000UM §8). This
//! interpreter has no per-instruction cycle table: each access it makes
//! charges four, and the idle time is charged where the manual says it is
//! spent. A device watching the bus sees the same reads and writes real
//! hardware would, in the same order — including the extra word `MOVEM` reads
//! past the end of its register list, and the destination read `CLR` performs
//! before writing zero.
//!
//! # The prefetch queue
//!
//! The 68000 holds two instruction words and refills them a word at a time,
//! and that is *observable*: it decides the program counter an address-error
//! frame pushes and the order a `MOVE` to an absolute long address puts its
//! write in. So it is modelled, not approximated. The invariant is that
//! [`Regs::prefetch`]`[0]` is the word at [`Regs::pc`] and `prefetch[1]` is
//! the word at `pc + 2`; executing one instruction slides the queue once per
//! instruction word. The module documentation on `exec.rs` has the long form.
//!
//! # Big-endian, and only 24 address pins
//!
//! The 68000 is big-endian, so **every region this core reaches must declare
//! big-endian byte order** — `Region::ram(..).with_endian(Endian::Big)`, and
//! `AddressSpace::with_endian(Endian::Big)` for the unmapped fallback. Byte
//! order is a property of the region rather than of the master
//! (`ROADMAP.md` §4.1), which is what lets a little-endian device sit on the
//! same bus; the core does not byte-swap behind the framework's back.
//!
//! Addresses reach the bus modulo 16 MiB, because A24–A31 are not brought out
//! of the package. `(xxx).L` with a high byte set therefore aliases into the
//! low 16 MiB, which is how the Amiga's mirrors and the Mac's 24-bit mode
//! work, and the core masks every access accordingly.
//!
//! # Assembling one
//!
//! ```
//! use std::sync::Arc;
//! use rsemu::core::space::{AddressSpace, RamStore, Region};
//! use rsemu::core::value::Endian;
//! use rsemu::cpu::m68k::{Config, M68k};
//!
//! let ram = Arc::new(RamStore::new(0x1_0000));
//! // Reset vector: SSP = $2000, PC = $400.
//! for (offset, byte) in [(3, 0x20u8), (6, 0x04), (7, 0x00)] {
//!     ram.write_u8(offset, byte).unwrap();
//! }
//! // MOVEQ #$42,D0 at $400.
//! ram.write_u8(0x400, 0x70).unwrap();
//! ram.write_u8(0x401, 0x42).unwrap();
//!
//! let space = AddressSpace::new("cpu", 24).with_endian(Endian::Big);
//! let region = Region::ram("ram", ram).with_endian(Endian::Big);
//! space.topology().map(region, 0).unwrap();
//!
//! let cpu = M68k::new(Config::default());
//! cpu.attach_space(Arc::new(space));
//! cpu.step();                       // the reset sequence
//! assert_eq!(cpu.regs().pc, 0x400);
//! cpu.step();                       // MOVEQ
//! assert_eq!(cpu.regs().d[0], 0x42);
//! ```
//!
//! # How accurate, measured
//!
//! `ROADMAP.md` §0: accuracy is measured, never asserted. Against
//! `SingleStepTests/680x0`'s 68000 corpus — 124 instruction files, 1 000 058
//! vectors — this core reproduces **every** vector's final registers, both
//! stack pointers, prefetch queue and memory, **and** every vector's cycle
//! count, **and** every vector's complete bus trace, access for access in
//! order. Two vectors are skipped as corpus errors and are named and argued
//! for in the runner, and the known-failures ledger carries nothing the corpus
//! covers.
//!
//! The corpus has no licence file, so it is fetched and run, never vendored.
//! `src/cpu/m68k/conformance.rs` has the command.
//!
//! What the corpus does *not* reach, because every vector runs in supervisor
//! state with the interrupt mask at seven and tracing off: reset, interrupts,
//! `STOP`, tracing, user mode and the privilege violation. Those are covered
//! by the hand-written tests beside it.
//!
//! # What is not modelled
//!
//! Two things, both because they need something the framework does not carry
//! yet, and both stated here rather than discovered later:
//!
//! - **The interrupt-acknowledge bus cycle.** It is CPU space — function code
//!   7 — and `MemAttrs` has no function code, so the cycle is charged but not
//!   driven. A vectoring controller arms its vector through
//!   [`M68k::set_interrupt_vector`] instead of answering an access, and there
//!   is consequently no spurious-interrupt path.
//! - **`STOP`'s bus behaviour.** It settles the prefetch queue before
//!   stopping, which costs two bus cycles hardware makes on the way out
//!   instead. The state is identical; the trace and the four-cycle published
//!   time are not.
//!
//! # Modules
//!
//! | Module | Holds |
//! | --- | --- |
//! | [`isa`] | the one declarative instruction description; decode and disassembly both read it |
//! | [`disasm`] | the disassembler generated from that description |
//! | `exec` (private) | the interpreter, the prefetch queue and exception processing |
//!
//! # Sources
//!
//! Hardware documentation only (`ROADMAP.md` §1): the *M68000 Family
//! Programmer's Reference Manual* (Motorola M68000PRM/AD) for the instruction
//! set, encodings and condition codes, and the *MC68000 8-/16-/32-Bit
//! Microprocessors User's Manual* (MC68000UM) for exception processing, the
//! stack frames, the signal description and the timing tables. Both are listed
//! in `docs/cpu/other.md`. No copyleft emulator was consulted, and no emulator
//! source of any licence was used for the instruction semantics.

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
use core::fmt::{self, Write as _};

use crate::core::device::{Device, DeviceClass, Initiator, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::registry::Registry;
use crate::core::space::{AddressSpace, MemAttrs, RequesterId};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{self, AtomicBool, AtomicU8, AtomicU16, AtomicU32, LockRank, Ordering};
use crate::core::value::Width;
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink};

use exec::{Exec, State};

/// The 24 address pins.
///
/// The 68000 has 32-bit registers and 24 address lines: A0 is not brought out
/// (the two byte-select strobes replace it) and A24–A31 do not exist. Every
/// address therefore reaches the bus modulo 16 MiB, which is why an Amiga sees
/// its chip RAM mirrored and why `(xxx).L` with a high byte set still lands in
/// the low 16 MiB (MC68000UM §3, *Signal Description*).
pub const ADDRESS_MASK: u32 = 0x00ff_ffff;

/// The status register's bits.
///
/// The low byte is the condition code register, which user code may write; the
/// high byte is the *system byte* — trace, supervisor state and the interrupt
/// mask — and writing it requires supervisor state (M68000PRM §1.3).
pub mod flags {
    /// Carry.
    pub const C: u16 = 0x0001;
    /// Overflow.
    pub const V: u16 = 0x0002;
    /// Zero.
    pub const Z: u16 = 0x0004;
    /// Negative.
    pub const N: u16 = 0x0008;
    /// Extend — the carry a multi-precision operation propagates.
    ///
    /// Separate from **C** on purpose: `CMP` sets carry without disturbing the
    /// extend of an `ADDX` chain in progress.
    pub const X: u16 = 0x0010;
    /// Every condition code bit.
    pub const CCR: u16 = 0x001f;
    /// The interrupt priority mask, bits 10–8.
    pub const IPL: u16 = 0x0700;
    /// Supervisor state.
    pub const S: u16 = 0x2000;
    /// Trace: take a trace exception after each instruction.
    pub const T: u16 = 0x8000;
    /// Every bit the 68000 implements.
    ///
    /// Bits 11, 12 and 14 have no storage and read as zero; the 68020's **M**
    /// bit is one of them.
    pub const IMPLEMENTED: u16 = T | S | IPL | CCR;
}

/// The exception vector numbers a 68000 defines.
///
/// A vector's address is four times its number, and the table starts at zero —
/// which on a 68000 cannot be moved, because there is no vector base register
/// (MC68000UM §6.1).
pub mod vector {
    /// Vector 0: the initial supervisor stack pointer.
    pub const RESET_SSP: u8 = 0;
    /// Vector 1: the initial program counter.
    pub const RESET_PC: u8 = 1;
    /// Vector 2: bus error — an access the hardware refused.
    pub const BUS_ERROR: u8 = 2;
    /// Vector 3: address error — a word or long access to an odd address.
    pub const ADDRESS_ERROR: u8 = 3;
    /// Vector 4: illegal instruction.
    pub const ILLEGAL: u8 = 4;
    /// Vector 5: divide by zero.
    pub const DIVIDE_BY_ZERO: u8 = 5;
    /// Vector 6: `CHK` found the register outside its bounds.
    pub const CHK: u8 = 6;
    /// Vector 7: `TRAPV` with **V** set.
    pub const TRAPV: u8 = 7;
    /// Vector 8: privilege violation.
    pub const PRIVILEGE: u8 = 8;
    /// Vector 9: trace.
    pub const TRACE: u8 = 9;
    /// Vector 10: an unimplemented `$Axxx` instruction.
    pub const LINE_A: u8 = 10;
    /// Vector 11: an unimplemented `$Fxxx` instruction.
    pub const LINE_F: u8 = 11;
    /// Vector 15: uninitialized interrupt vector.
    pub const UNINITIALIZED: u8 = 15;
    /// Vector 24: spurious interrupt — no device answered the acknowledge.
    pub const SPURIOUS: u8 = 24;
    /// Vectors 25–31: the autovectors, one per interrupt level.
    ///
    /// The level is added: level 1 uses vector 25. Vector 24 is the spurious
    /// slot immediately below.
    pub const AUTOVECTOR_BASE: u8 = 24;
    /// Vectors 32–47: the `TRAP #0`–`TRAP #15` family.
    pub const TRAP_BASE: u8 = 32;
}

/// The architectural register file, as a debugger or a test vector sees it.
///
/// `a[7]` is whichever stack pointer the **S** bit currently selects, and is
/// always equal to [`Regs::ssp`] in supervisor state or [`Regs::usp`] in user
/// state — the 68000 has two physical `A7`s and one name for them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Regs {
    /// The eight data registers.
    pub d: [u32; 8],
    /// The eight address registers; `a[7]` is the active stack pointer.
    pub a: [u32; 8],
    /// The user stack pointer.
    pub usp: u32,
    /// The supervisor stack pointer.
    pub ssp: u32,
    /// The program counter: the address of the word in `prefetch[0]`.
    pub pc: u32,
    /// The status register. See [`flags`].
    pub sr: u16,
    /// The two-word instruction prefetch queue.
    pub prefetch: [u16; 2],
}

impl Regs {
    /// Whether the core is in supervisor state.
    #[must_use]
    pub const fn supervisor(&self) -> bool {
        self.sr & flags::S != 0
    }

    /// Whether a status flag is set.
    #[inline]
    #[must_use]
    pub const fn flag(&self, mask: u16) -> bool {
        self.sr & mask != 0
    }

    /// The condition code register: the low byte of `SR`.
    #[must_use]
    pub const fn ccr(&self) -> u8 {
        (self.sr & flags::CCR) as u8
    }

    /// The interrupt priority mask, 0–7.
    #[must_use]
    pub const fn ipl_mask(&self) -> u8 {
        ((self.sr & flags::IPL) >> 8) as u8
    }
}

impl fmt::Display for Regs {
    /// The shape a trace log wants: the two register files, then `SR` decoded.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for (i, value) in self.d.iter().enumerate() {
            write!(f, "D{i}:{value:08x} ")?;
        }
        for (i, value) in self.a.iter().enumerate() {
            write!(f, "A{i}:{value:08x} ")?;
        }
        write!(f, "PC:{:08x} SR:{:04x} [", self.pc, self.sr)?;
        for (mask, name) in [
            (flags::T, 'T'),
            (flags::S, 'S'),
            (flags::X, 'X'),
            (flags::N, 'N'),
            (flags::Z, 'Z'),
            (flags::V, 'V'),
            (flags::C, 'C'),
        ] {
            f.write_char(if self.flag(mask) { name } else { '-' })?;
        }
        write!(f, "] I{}", self.ipl_mask())
    }
}

/// One named register, for a debugger that works by name or index.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Reg {
    /// A data register, `D0`–`D7`.
    D(u8),
    /// An address register, `A0`–`A7`.
    A(u8),
    /// The user stack pointer.
    Usp,
    /// The supervisor stack pointer.
    Ssp,
    /// The program counter.
    Pc,
    /// The status register.
    Sr,
}

impl Reg {
    /// Every register, in the order a debugger should list them.
    pub const ALL: &'static [Reg] = &[
        Reg::D(0),
        Reg::D(1),
        Reg::D(2),
        Reg::D(3),
        Reg::D(4),
        Reg::D(5),
        Reg::D(6),
        Reg::D(7),
        Reg::A(0),
        Reg::A(1),
        Reg::A(2),
        Reg::A(3),
        Reg::A(4),
        Reg::A(5),
        Reg::A(6),
        Reg::A(7),
        Reg::Usp,
        Reg::Ssp,
        Reg::Pc,
        Reg::Sr,
    ];

    /// How wide the register is.
    #[must_use]
    pub const fn width(self) -> Width {
        match self {
            Reg::Sr => Width::U16,
            _ => Width::U32,
        }
    }

    /// Read this register out of a register file.
    #[must_use]
    pub const fn get(self, regs: &Regs) -> u32 {
        match self {
            Reg::D(n) => regs.d[(n & 7) as usize],
            Reg::A(n) => regs.a[(n & 7) as usize],
            Reg::Usp => regs.usp,
            Reg::Ssp => regs.ssp,
            Reg::Pc => regs.pc,
            Reg::Sr => regs.sr as u32,
        }
    }

    /// Write this register into a register file, truncating to its width.
    ///
    /// Writing `A7` writes whichever bank is active, and writing `USP` or
    /// `SSP` writes that bank whether or not it is active — which is what a
    /// debugger showing both of them needs.
    pub const fn set(self, regs: &mut Regs, value: u32) {
        match self {
            Reg::D(n) => regs.d[(n & 7) as usize] = value,
            Reg::A(n) => {
                let n = (n & 7) as usize;
                regs.a[n] = value;
                if n == 7 {
                    if regs.supervisor() {
                        regs.ssp = value;
                    } else {
                        regs.usp = value;
                    }
                }
            }
            Reg::Usp => {
                regs.usp = value;
                if !regs.supervisor() {
                    regs.a[7] = value;
                }
            }
            Reg::Ssp => {
                regs.ssp = value;
                if regs.supervisor() {
                    regs.a[7] = value;
                }
            }
            Reg::Pc => regs.pc = value,
            Reg::Sr => regs.sr = value as u16,
        }
    }

    /// Look a register up by name, as gdb and the monitor spell it.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Reg> {
        let bytes = name.as_bytes();
        match (bytes.first(), bytes.len()) {
            (Some(b'd' | b'D'), 2) if bytes[1].is_ascii_digit() && bytes[1] <= b'7' => {
                Some(Reg::D(bytes[1] - b'0'))
            }
            (Some(b'a' | b'A'), 2) if bytes[1].is_ascii_digit() && bytes[1] <= b'7' => {
                Some(Reg::A(bytes[1] - b'0'))
            }
            _ => match name {
                "usp" => Some(Reg::Usp),
                "ssp" | "sp" => Some(Reg::Ssp),
                "pc" => Some(Reg::Pc),
                "sr" => Some(Reg::Sr),
                _ => None,
            },
        }
    }
}

impl fmt::Display for Reg {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Reg::D(n) => write!(f, "d{n}"),
            Reg::A(n) => write!(f, "a{n}"),
            Reg::Usp => f.write_str("usp"),
            Reg::Ssp => f.write_str("ssp"),
            Reg::Pc => f.write_str("pc"),
            Reg::Sr => f.write_str("sr"),
        }
    }
}

/// How this particular part differs from the generic 68000.
///
/// Construction properties, never `#[cfg]`: one build of rsemu has to be able
/// to run an Amiga and a Mega Drive at the same time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// This core's identity in `MemAttrs::requester`, for an IOMMU or a
    /// per-master filter.
    pub requester: RequesterId,
}

impl Config {
    /// A plain MC68000.
    pub const MC68000: Config = Config {
        requester: RequesterId::ANONYMOUS,
    };

    /// Same configuration, with a different requester id.
    #[must_use]
    pub const fn with_requester(mut self, id: RequesterId) -> Self {
        self.requester = id;
        self
    }
}

impl Default for Config {
    fn default() -> Self {
        Config::MC68000
    }
}

/// No interrupt vector has been supplied by an acknowledging device.
const NO_VECTOR: u16 = 0x100;

/// The interrupt and reset pins, kept outside the execution lock.
///
/// Deliberately atomics rather than fields under the mutex: a device raising
/// an interrupt from inside a write the CPU itself issued would otherwise
/// re-enter the CPU's own critical section, which is a deadlock under
/// `native-std` and a panic under `single`. A pin that is one atomic store
/// needs no critical section at all (`ROADMAP.md` §4.7).
#[derive(Debug)]
pub(crate) struct Lines {
    /// The encoded level on IPL0–IPL2, 0 (none) to 7 (non-maskable).
    ipl: AtomicU8,
    /// A vector supplied by an interrupt controller, or [`NO_VECTOR`] for the
    /// autovector the 68000 uses when `VPA` is asserted.
    vector: AtomicU16,
    /// A transition to level seven, latched until it is serviced.
    ///
    /// Level seven is edge-triggered, so the level alone is not enough to know
    /// whether to take it.
    level_seven: AtomicBool,
    /// How many times the `RESET` instruction has pulsed the reset line.
    ///
    /// A counter rather than a wire because `RESET` resets *peripherals*, not
    /// the processor, and what is on the other end is the machine's business.
    resets: AtomicU32,
}

impl Default for Lines {
    fn default() -> Lines {
        Lines {
            ipl: AtomicU8::new(0),
            // Autovectoring, not vector 0 — which is the reset stack pointer,
            // and would send the first interrupt somewhere very strange.
            vector: AtomicU16::new(NO_VECTOR),
            level_seven: AtomicBool::new(false),
            resets: AtomicU32::new(0),
        }
    }
}

impl Lines {
    fn set_ipl(&self, level: u8) {
        let level = level.min(7);
        let previous = self.ipl.swap(level, Ordering::AcqRel);
        if level == 7 && previous != 7 {
            self.level_seven.store(true, Ordering::Release);
        }
    }

    /// Consume a latched transition to level seven, reporting whether there
    /// was one.
    pub(crate) fn take_level_seven(&self) -> bool {
        self.level_seven.swap(false, Ordering::AcqRel)
    }

    pub(crate) fn ipl(&self) -> u8 {
        self.ipl.load(Ordering::Acquire)
    }

    fn set_vector(&self, vector: Option<u8>) {
        self.vector
            .store(vector.map_or(NO_VECTOR, u16::from), Ordering::Release);
    }

    pub(crate) fn take_vector(&self) -> Option<u8> {
        match self.vector.swap(NO_VECTOR, Ordering::AcqRel) {
            NO_VECTOR => None,
            other => Some(other as u8),
        }
    }

    pub(crate) fn pulse_reset(&self) {
        self.resets.fetch_add(1, Ordering::AcqRel);
    }

    fn resets(&self) -> u32 {
        self.resets.load(Ordering::Acquire)
    }

    fn snapshot(&self) -> (u8, u16, bool, u32) {
        (
            self.ipl(),
            self.vector.load(Ordering::Acquire),
            self.level_seven.load(Ordering::Acquire),
            self.resets(),
        )
    }

    fn restore(&self, (ipl, vector, level_seven, resets): (u8, u16, bool, u32)) {
        self.ipl.store(ipl, Ordering::Release);
        self.vector.store(vector, Ordering::Release);
        self.level_seven.store(level_seven, Ordering::Release);
        self.resets.store(resets, Ordering::Release);
    }
}

/// Everything the interpreter needs to mutate, behind one lock.
#[derive(Debug)]
struct Session {
    state: State,
    space: Option<Arc<AddressSpace>>,
}

/// An MC68000 core.
///
/// # Locking
///
/// Execution state sits behind one [`sync::Mutex`] at [`LockRank::BUS`]. That
/// rank, rather than `DEVICE`, because a CPU is a bus master: it holds this
/// lock while calling into device models, which take their own `DEVICE`-ranked
/// locks, which drive `WIRE`-ranked lines. The ladder runs in the direction
/// calls travel.
///
/// The interrupt pins are *not* under that lock: they are atomics, so a device
/// raising an interrupt from inside a write the CPU itself issued cannot
/// re-enter the CPU's own critical section.
#[derive(Debug)]
pub struct M68k {
    cfg: Config,
    lines: Lines,
    session: sync::Mutex<Session>,
}

impl M68k {
    /// A core in its power-on state, with no address space yet.
    ///
    /// Two-phase construction (`ROADMAP.md` §4.4): nothing observable happens
    /// until [`attach_space`](M68k::attach_space) and [`Device::realize`]. The
    /// first [`step`](M68k::step) runs the reset sequence, which is where
    /// vectors 0 and 1 are read.
    #[must_use]
    pub fn new(cfg: Config) -> M68k {
        M68k {
            cfg,
            lines: Lines::default(),
            session: sync::Mutex::with_rank(
                LockRank::BUS,
                Session {
                    state: State::new(),
                    space: None,
                },
            ),
        }
    }

    /// Build one from machine-description properties.
    ///
    /// # Errors
    ///
    /// If a property nothing here accepts was given — a typo'd property that
    /// was silently ignored is an afternoon lost.
    pub fn from_props(props: &Props) -> Result<M68k> {
        let mut r = props.reader();
        let requester = r.or_range("requester", 0u64, 0..=u64::from(u32::MAX))?;
        r.finish()?;
        Ok(M68k::new(
            Config::default().with_requester(RequesterId(requester as u32)),
        ))
    }

    /// This core's configuration.
    #[must_use]
    pub fn config(&self) -> Config {
        self.cfg
    }

    /// Give the core the address space it executes from.
    ///
    /// The space must be big-endian, or every word the core reads is
    /// byte-swapped — see the module documentation.
    pub fn attach_space(&self, space: Arc<AddressSpace>) {
        self.session.lock().space = Some(space);
    }

    /// The address space this core executes from, if one is attached.
    #[must_use]
    pub fn space(&self) -> Option<Arc<AddressSpace>> {
        self.session.lock().space.clone()
    }

    /// The register file.
    #[must_use]
    pub fn regs(&self) -> Regs {
        let state = self.session.lock().state;
        Regs {
            d: state.d,
            a: state.a,
            usp: state.usp(),
            ssp: state.ssp(),
            pc: state.pc,
            sr: state.sr,
            prefetch: state.prefetch,
        }
    }

    /// Overwrite the register file — a debugger, a test vector, a snapshot.
    ///
    /// [`Regs::usp`] and [`Regs::ssp`] are authoritative: `a[7]` is set from
    /// whichever the **S** bit in `regs.sr` selects, so a caller cannot leave
    /// the two banks disagreeing.
    pub fn set_regs(&self, regs: Regs) {
        let mut session = self.session.lock();
        let state = &mut session.state;
        state.d = regs.d;
        state.a = regs.a;
        state.sr = regs.sr & flags::IMPLEMENTED;
        if state.supervisor() {
            state.a[7] = regs.ssp;
            state.other_sp = regs.usp;
        } else {
            state.a[7] = regs.usp;
            state.other_sp = regs.ssp;
        }
        state.pc = regs.pc;
        state.prefetch = regs.prefetch;
    }

    /// Read one register by name.
    #[must_use]
    pub fn reg(&self, reg: Reg) -> u32 {
        reg.get(&self.regs())
    }

    /// Write one register by name.
    pub fn set_reg(&self, reg: Reg, value: u32) {
        let mut regs = self.regs();
        reg.set(&mut regs, value);
        self.set_regs(regs);
    }

    /// Cycles executed since power-on.
    #[must_use]
    pub fn cycles(&self) -> u64 {
        self.session.lock().state.cycles
    }

    /// Whether a double bus fault has halted the core.
    ///
    /// A 68000 that faults while taking an exception asserts `HALT` and stops
    /// until a reset. [`step`](M68k::step) returns zero cycles once this is
    /// true, so a scheduler must notice it rather than spin.
    #[must_use]
    pub fn is_halted(&self) -> bool {
        self.session.lock().state.halted
    }

    /// Whether `STOP` has suspended the core until an interrupt.
    #[must_use]
    pub fn is_stopped(&self) -> bool {
        self.session.lock().state.stopped
    }

    /// Whether a reset sequence is still owed.
    #[must_use]
    pub fn reset_pending(&self) -> bool {
        self.session.lock().state.reset_pending
    }

    /// How many accesses the address space refused, and where the last one
    /// was.
    ///
    /// A refused access becomes a bus-error exception, so unlike the 6502 this
    /// is not the whole story — but a machine whose memory map has a hole will
    /// still show it climbing.
    #[must_use]
    pub fn bus_faults(&self) -> (u64, u32) {
        let s = self.session.lock();
        (s.state.faults, s.state.last_fault)
    }

    /// How many times the `RESET` instruction has pulsed the reset line.
    ///
    /// `RESET` resets peripherals, not the processor; what hangs off the pin
    /// is the machine's business, so the core only counts.
    #[must_use]
    pub fn reset_pulses(&self) -> u32 {
        self.lines.resets()
    }

    /// Drive IPL0–IPL2 with an encoded interrupt level, 0 (none) to 7.
    ///
    /// Levels 1–6 are level-sensitive: they are taken, and re-taken, while
    /// they exceed the mask in `SR`. **Level 7 is edge-triggered** — the
    /// transition to it is what the processor recognises — so holding the pins
    /// at 7 raises exactly one non-maskable interrupt, and raising another
    /// means dropping the level and driving 7 again.
    pub fn set_ipl(&self, level: u8) {
        self.lines.set_ipl(level);
    }

    /// The level currently encoded on the interrupt pins.
    #[must_use]
    pub fn ipl(&self) -> u8 {
        self.lines.ipl()
    }

    /// Supply the vector number the *next* interrupt acknowledge will fetch.
    ///
    /// **Consumed by that acknowledge**, exactly as a device answering the
    /// cycle would be: a controller arms a vector per interrupt, and anything
    /// that does not arm one autovectors, which is what asserting `VPA` means
    /// and what most 68000 machines do. `None` disarms it again.
    ///
    /// The acknowledge cycle itself does not reach the bus — it is CPU space,
    /// and `MemAttrs` carries no function code — so this is how a vectoring
    /// controller talks to the core. See `exec.rs`'s `take_interrupt`.
    pub fn set_interrupt_vector(&self, vector: Option<u8>) {
        self.lines.set_vector(vector);
    }

    /// The vector armed for the next acknowledge, if any.
    #[must_use]
    pub fn interrupt_vector(&self) -> Option<u8> {
        match self.lines.vector.load(Ordering::Acquire) {
            NO_VECTOR => None,
            other => Some(other as u8),
        }
    }

    /// Say whether the core still owes a reset sequence.
    ///
    /// A fresh core owes one, so a register file written before the first
    /// [`step`](M68k::step) would be thrown away by it. Anything that places a
    /// core mid-program — a debugger, a test vector, a machine resuming a
    /// loaded image, the differential tester the IR frontend will need — turns
    /// it off first. [`request_reset`](M68k::request_reset) is the same switch
    /// the other way round, named for the common case.
    pub fn set_reset_pending(&self, pending: bool) {
        self.session.lock().state.reset_pending = pending;
    }

    /// Bring a halted or stopped core back to life without resetting it.
    ///
    /// A double bus fault halts the processor and only a reset restarts it on
    /// real hardware; this is the debugger's override, and the way a test
    /// places a core that a previous vector left halted.
    pub fn resume(&self) {
        let mut session = self.session.lock();
        session.state.halted = false;
        session.state.stopped = false;
    }

    /// Request a reset sequence without changing any register.
    ///
    /// The sequence runs on the next [`step`](M68k::step), because that is
    /// when the CPU can read vectors 0 and 1 — a reset is a signal, not a
    /// method call.
    pub fn request_reset(&self) {
        self.session.lock().state.reset_pending = true;
    }

    /// Execute one reset sequence, exception sequence, or instruction.
    ///
    /// Returns the cycles charged: zero if the core is halted or has no
    /// address space, which the caller must treat as "stop", not "retry".
    pub fn step(&self) -> u64 {
        let mut session = self.session.lock();
        let Session { state, space } = &mut *session;
        let Some(space) = space.clone() else {
            return 0;
        };
        Exec::new(state, &space, &self.cfg, &self.lines).step()
    }

    /// Execute until at least `budget` cycles have been charged.
    ///
    /// Returns the cycles actually used, which overshoots by at most one
    /// instruction — a 68000 cannot be stopped mid-instruction, and pretending
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

    /// Disassemble `count` instructions starting at `pc`, reading guest memory
    /// with debug attributes.
    ///
    /// Debug attributes are the point: a monitor listing the code around PC
    /// must not pop a FIFO or clear a status bit on the way (`ROADMAP.md`
    /// §15, invariant 5).
    #[must_use]
    pub fn disassemble(&self, pc: u32, count: usize) -> Vec<disasm::Disassembled> {
        let Some(space) = self.space() else {
            return Vec::new();
        };
        disasm::disassemble_run(pc, count, |addr| {
            space
                .read(u64::from(addr & ADDRESS_MASK), Width::U16, MemAttrs::DEBUG)
                .ok()
                .map(|v| v as u16)
        })
    }
}

/// The `cpu.m68k` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: "cpu.m68k",
    version: 1,
    summary: "Motorola MC68000 32-bit CPU core, bus-accurate interpreter",
    properties: &[PropertySpec {
        name: "requester",
        kind: ValueKind::Uint,
        required: false,
        summary: "this core's requester id in MemAttrs, for an IOMMU or a per-master filter",
    }],
    construct: |props| Ok(Box::new(M68k::from_props(props)?)),
};

/// Add this core's class to a registry.
///
/// Registration is explicit per feature rather than link-time magic
/// (`ROADMAP.md` §4.4), so the machine assembly layer calls this from its own
/// `#[cfg(feature = "cpu-m68k")]` arm.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut Registry) -> Result<()> {
    reg.add(&CLASS)
}

impl Device for M68k {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // A CPU with no address space cannot fetch, and failing here is the
        // difference between a config error and a machine that runs zero
        // instructions and says nothing.
        if self.session.lock().space.is_none() {
            return Err(ctx.error("no address space attached to this core"));
        }
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        let mut session = self.session.lock();
        if kind == ResetKind::Cold {
            // A cold start has no defined register contents on real hardware;
            // zeroing them is the reproducible choice, and determinism is a
            // first-class mode (`ROADMAP.md` §0).
            session.state = State::new();
        } else {
            // A warm reset is a pulse on the RESET pin: the register file
            // keeps its values and only the sequence's own effects apply.
            session.state.reset_pending = true;
            session.state.halted = false;
            session.state.stopped = false;
        }
        drop(session);
        if kind == ResetKind::Cold {
            self.lines.restore((0, NO_VECTOR, false, 0));
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.session.lock().state;
        for value in state.d {
            w.write_u32(value)?;
        }
        for value in state.a {
            w.write_u32(value)?;
        }
        w.write_u32(state.other_sp)?;
        w.write_u32(state.pc)?;
        w.write_u16(state.sr)?;
        w.write_u16(state.prefetch[0])?;
        w.write_u16(state.prefetch[1])?;
        w.write_u64(state.cycles)?;
        w.write_bool(state.halted)?;
        w.write_bool(state.stopped)?;
        w.write_bool(state.reset_pending)?;
        w.write_u64(state.faults)?;
        w.write_u32(state.last_fault)?;
        let (ipl, vector, level_seven, resets) = self.lines.snapshot();
        w.write_u8(ipl)?;
        w.write_u16(vector)?;
        w.write_bool(level_seven)?;
        w.write_u32(resets)?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::new();
        for slot in &mut state.d {
            *slot = r.read_u32()?;
        }
        for slot in &mut state.a {
            *slot = r.read_u32()?;
        }
        state.other_sp = r.read_u32()?;
        state.pc = r.read_u32()?;
        state.sr = r.read_u16()?;
        if state.sr & !flags::IMPLEMENTED != 0 {
            return Err(Error::State(alloc::format!(
                "status register 0x{:04x} sets bits a 68000 does not implement",
                state.sr
            )));
        }
        state.prefetch[0] = r.read_u16()?;
        state.prefetch[1] = r.read_u16()?;
        state.cycles = r.read_u64()?;
        state.halted = r.read_bool()?;
        state.stopped = r.read_bool()?;
        state.reset_pending = r.read_bool()?;
        state.faults = r.read_u64()?;
        state.last_fault = r.read_u32()?;
        let ipl = r.read_u8()?;
        if ipl > 7 {
            return Err(Error::State(alloc::format!(
                "interrupt level {ipl} does not fit on three pins"
            )));
        }
        let vector = r.read_u16()?;
        if vector != NO_VECTOR && vector > 0xff {
            return Err(Error::State(alloc::format!(
                "interrupt vector 0x{vector:04x} is not a vector number"
            )));
        }
        let level_seven = r.read_bool()?;
        let resets = r.read_u32()?;
        self.session.lock().state = state;
        self.lines.restore((ipl, vector, level_seven, resets));
        Ok(())
    }
}

impl Initiator for M68k {
    fn requester(&self) -> RequesterId {
        self.cfg.requester
    }
}

/// The three interrupt priority inputs, as something a [`Wire`] can drive.
///
/// IPL0–IPL2 carry an encoded *level*, not three independent requests, so a
/// net per line would be the wrong model: this sink keeps a [`FanIn`] per line
/// and recomputes the level whenever any of them changes. A machine with a
/// single interrupt source can drive one line and get level 1, 2 or 4; a
/// machine with a priority encoder drives all three.
///
/// The pins are active-low on real hardware; inverting them belongs to
/// whatever models the wire, so a high level here means "asserted".
///
/// [`Wire`]: crate::core::wire::Wire
#[derive(Debug)]
pub struct InterruptPins {
    cpu: Arc<M68k>,
    inputs: [FanIn; 3],
    resolve: Resolve,
}

impl InterruptPins {
    /// Connect `cpu`'s three interrupt inputs to nets driven by `sources`.
    ///
    /// `sources[i]` is every id that drives IPL`i`. Wire-OR by default, which
    /// is how an open-collector interrupt line behaves.
    #[must_use]
    pub fn new(cpu: Arc<M68k>, sources: [&[WireId]; 3]) -> InterruptPins {
        InterruptPins {
            cpu,
            inputs: [
                FanIn::new(sources[0]),
                FanIn::new(sources[1]),
                FanIn::new(sources[2]),
            ],
            resolve: Resolve::Or,
        }
    }

    /// The same pins with an explicit resolution rule.
    #[must_use]
    pub fn with_resolve(mut self, resolve: Resolve) -> Self {
        self.resolve = resolve;
        self
    }

    /// The per-source levels currently seen on one line.
    #[must_use]
    pub fn inputs(&self, line: usize) -> &FanIn {
        &self.inputs[line.min(2)]
    }
}

impl WireSink for InterruptPins {
    fn set_level(&self, src: WireId, line: u32, level: Level) {
        let index = (line as usize).min(2);
        self.inputs[index].set(src, level);
        let mut encoded = 0u8;
        for (bit, input) in self.inputs.iter().enumerate() {
            if input.resolve(self.resolve).is_high() {
                encoded |= 1 << bit;
            }
        }
        self.cpu.set_ipl(encoded);
    }
}

/// A description of this core's instruction set for `rsemu describe cpu.m68k`.
///
/// Built from [`isa::TABLE`], so it cannot drift from what the interpreter
/// implements.
#[must_use]
pub fn describe_isa() -> String {
    use core::fmt::Write as _;
    let mut out = String::new();
    for pattern in isa::TABLE {
        let insn = pattern.insn;
        let mark = if insn.privileged { '!' } else { ' ' };
        let _ = writeln!(
            out,
            "{:04x}/{:04x} {mark}{:<8} {}",
            pattern.mask,
            pattern.value,
            insn.op.mnemonic(),
            insn.op.summary()
        );
    }
    out
}
