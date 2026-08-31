//! The Zilog Z80 — a cycle-accurate interpreter.
//!
//! Covers the documented instruction set, all five prefix pages, and the
//! undocumented behaviour real software depends on: the `IX`/`IY` half
//! registers, `SLL`, the duplicate `ED` encodings, and the `DDCB`/`FDCB` forms
//! that write a register as well as memory. The internal `WZ` (`MEMPTR`)
//! register and flag bits 3 and 5 are modelled from the first commit rather
//! than retrofitted, because `BIT n,(HL)`, `SCF`/`CCF` and `IN r,(C)` make
//! them observable and no serious test suite passes without them.
//!
//! # What "cycle-accurate" means here
//!
//! Not "the instruction took eleven T-states". Every T-state belongs to an
//! M-cycle, and an M-cycle is a fetch, a read, a write, an I/O transfer, or an
//! internal operation that requests nothing — so `INC (HL)` is eleven
//! T-states *because* it fetches, reads, thinks and writes, not because a
//! table says so. [`Z80::last_cycles`] hands back that sequence, which is what
//! a bus-level trace and the conformance runner both compare against hardware.
//!
//! # Two address spaces
//!
//! The Z80 has a separate I/O address space, and it is not an afterthought:
//! `IN`/`OUT` drive `IORQ` instead of `MREQ`, the port address is sixteen bits
//! wide with `B` or `A` on the high half, and every access carries one
//! automatic wait state. So the core takes two
//! [`AddressSpace`]s — [`Z80::attach_space`] for memory and
//! [`Z80::attach_io_space`] for ports — and a machine that wires only the
//! first gets a floating bus on every port rather than a fault storm.
//!
//! # Assembling one
//!
//! ```
//! use std::sync::Arc;
//! use rsemu::core::space::{AddressSpace, RamStore, Region};
//! use rsemu::cpu::z80::{Config, Z80};
//!
//! // 64 KiB of RAM with `LD A,$42` at the reset address.
//! let ram = Arc::new(RamStore::new(0x1_0000));
//! ram.write_u8(0x0000, 0x3e).unwrap();
//! ram.write_u8(0x0001, 0x42).unwrap();
//!
//! let space = AddressSpace::new("cpu", 16);
//! space.topology().map(Region::ram("ram", ram), 0).unwrap();
//!
//! let cpu = Z80::new(Config::default());
//! cpu.attach_space(Arc::new(space));
//! cpu.step();              // the reset sequence
//! cpu.step();              // LD A,$42
//! assert_eq!(cpu.regs().a, 0x42);
//! assert_eq!(cpu.cycles(), 10);   // 3 T-states of reset, then 7
//! ```
//!
//! # Modules
//!
//! | Module | Holds |
//! | --- | --- |
//! | [`isa`] | the three declarative opcode tables, and the rules that derive the index pages from them |
//! | [`disasm`] | the disassembler generated from those tables |
//! | `exec` (private) | the interpreter: one bus access per M-cycle |
//!
//! # Accuracy
//!
//! Measured, not asserted (`ROADMAP.md` §0). The core passes all **1 604 000**
//! vectors of `SingleStepTests/z80` — every encoding on every page, compared
//! register by register, `WZ` and the flag latches included, against the full
//! T-state bus trace — and both `zexdoc` and `zexall` run clean, `zexall`
//! being the one that does *not* mask the undocumented flag bits. The
//! known-failures ledger in `conformance.rs` is empty.
//!
//! # Sources
//!
//! Hardware documentation only (`ROADMAP.md` §1): Zilog **UM0080**, the World
//! of Spectrum Z80 reference, Sean Young's *Undocumented Z80 Documented*
//! v0.91, and the *MEMPTR* write-up for the `WZ` rules
//! (`docs/cpu/z80-sm83.md`). A handful of undocumented flag rules — the
//! block-I/O repeat behaviour above all — were pinned down against
//! `SingleStepTests/z80` (MIT, © 2024 SingleStepTests), which is measured
//! hardware behaviour rather than anyone's implementation of it. No copyleft
//! emulator was consulted.

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
use alloc::string::{String, ToString};
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{
    Device, DeviceClass, Initiator, PropertySpec, RealizeCtx, ResetKind, SinkPin,
};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::registry::Registry;
use crate::core::sched::{Budget, Consumed};
use crate::core::space::{AddressSpace, MemAttrs, RequesterId};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{self, AtomicBool, AtomicU8, AtomicU32, LockRank, Ordering};
use crate::core::value::Width;
use crate::core::wire::{
    FanIn, IntAck, IntAckCycle, IntAckHandlers, IntAckResponse, Level, Resolve, WireId, WireSink,
};

use exec::{Exec, State};

/// The flag register's bits.
///
/// Bits 3 and 5 have no name in Zilog's manual and no defined meaning — they
/// are whatever the last operation left in the flag latch. That makes them
/// *observable*, and real software (and every conformance suite) depends on
/// them, so they are modelled rather than masked off.
pub mod flags {
    /// Carry.
    pub const C: u8 = 0x01;
    /// Add/subtract — set by the subtracting operations, and read by `DAA`.
    pub const N: u8 = 0x02;
    /// Parity / overflow, depending on the operation.
    pub const PV: u8 = 0x04;
    /// Undocumented bit 3, often written `F3` or `XF`.
    pub const XF: u8 = 0x08;
    /// Half carry — carry out of bit 3, which `DAA` needs.
    pub const H: u8 = 0x10;
    /// Undocumented bit 5, often written `F5` or `YF`.
    pub const YF: u8 = 0x20;
    /// Zero.
    pub const Z: u8 = 0x40;
    /// Sign — a copy of bit 7 of the result.
    pub const S: u8 = 0x80;
    /// Both undocumented bits, which almost always move together.
    pub const XY: u8 = XF | YF;
}

/// The architectural register file, shadow set and `WZ` included.
///
/// Public and `Copy` because a debugger, a tracer and a test all want to read
/// it out and put it back — this is the surface a future gdbstub serialises
/// (`ROADMAP.md` §9's debug story), and [`Reg`] enumerates it by name.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Regs {
    /// Accumulator.
    pub a: u8,
    /// Flags. See [`flags`].
    pub f: u8,
    /// General purpose `B`, and the counter `DJNZ` and block I/O decrement.
    pub b: u8,
    /// General purpose `C`, and the low half of the `(C)` port address.
    pub c: u8,
    /// General purpose `D`.
    pub d: u8,
    /// General purpose `E`.
    pub e: u8,
    /// General purpose `H`.
    pub h: u8,
    /// General purpose `L`.
    pub l: u8,
    /// Index register `IX`.
    pub ix: u16,
    /// Index register `IY`.
    pub iy: u16,
    /// Stack pointer. The stack grows downwards and `PUSH` writes the high
    /// byte first.
    pub sp: u16,
    /// Program counter.
    pub pc: u16,
    /// Interrupt vector base: the high half of the mode 2 table address.
    pub i: u8,
    /// Memory refresh counter. Only the low seven bits count; bit 7 is a latch
    /// the program owns and the hardware increment never carries into it.
    pub r: u8,
    /// The internal address latch, `WZ` — usually called `MEMPTR`.
    ///
    /// Not in any Zilog document, and not optional: `BIT n,(HL)` copies bits 3
    /// and 5 of `W` into the flags, so a core that does not model this fails
    /// on real software.
    pub wz: u16,
    /// The shadow `AF'`, which only `EX AF,AF'` reaches.
    pub af_alt: u16,
    /// The shadow `BC'`.
    pub bc_alt: u16,
    /// The shadow `DE'`.
    pub de_alt: u16,
    /// The shadow `HL'`.
    pub hl_alt: u16,
}

macro_rules! pair {
    ($get:ident, $set:ident, $hi:ident, $lo:ident, $name:literal) => {
        #[doc = concat!("The ", $name, " pair.")]
        #[inline]
        #[must_use]
        pub const fn $get(&self) -> u16 {
            ((self.$hi as u16) << 8) | self.$lo as u16
        }

        #[doc = concat!("Overwrite the ", $name, " pair.")]
        #[inline]
        pub const fn $set(&mut self, value: u16) {
            self.$hi = (value >> 8) as u8;
            self.$lo = value as u8;
        }
    };
}

impl Regs {
    /// The state a cold power-on leaves behind, *before* the reset sequence.
    ///
    /// Zeroed rather than randomised: a real Z80 comes up with undefined
    /// registers, and determinism is a first-class mode (`ROADMAP.md` §0).
    #[must_use]
    pub const fn new() -> Regs {
        Regs {
            a: 0,
            f: 0,
            b: 0,
            c: 0,
            d: 0,
            e: 0,
            h: 0,
            l: 0,
            ix: 0,
            iy: 0,
            sp: 0,
            pc: 0,
            i: 0,
            r: 0,
            wz: 0,
            af_alt: 0,
            bc_alt: 0,
            de_alt: 0,
            hl_alt: 0,
        }
    }

    pair!(bc, set_bc, b, c, "`BC`");
    pair!(de, set_de, d, e, "`DE`");
    pair!(hl, set_hl, h, l, "`HL`");

    /// The `AF` pair.
    #[inline]
    #[must_use]
    pub const fn af(&self) -> u16 {
        ((self.a as u16) << 8) | self.f as u16
    }

    /// Overwrite the `AF` pair.
    #[inline]
    pub const fn set_af(&mut self, value: u16) {
        self.a = (value >> 8) as u8;
        self.f = value as u8;
    }

    /// Whether a flag is set.
    #[inline]
    #[must_use]
    pub const fn flag(&self, mask: u8) -> bool {
        self.f & mask != 0
    }
}

impl fmt::Display for Regs {
    /// The one-line form a trace log wants.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "AF:{:04x} BC:{:04x} DE:{:04x} HL:{:04x} IX:{:04x} IY:{:04x} \
             SP:{:04x} PC:{:04x} I:{:02x} R:{:02x} WZ:{:04x}",
            self.af(),
            self.bc(),
            self.de(),
            self.hl(),
            self.ix,
            self.iy,
            self.sp,
            self.pc,
            self.i,
            self.r,
            self.wz
        )
    }
}

/// One named register, for a debugger that works by name or index.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Reg {
    /// Accumulator and flags.
    Af,
    /// The `BC` pair.
    Bc,
    /// The `DE` pair.
    De,
    /// The `HL` pair.
    Hl,
    /// Index register `IX`.
    Ix,
    /// Index register `IY`.
    Iy,
    /// Stack pointer.
    Sp,
    /// Program counter.
    Pc,
    /// Interrupt vector base.
    I,
    /// Memory refresh counter.
    R,
    /// The internal address latch.
    Wz,
    /// Shadow `AF'`.
    AfAlt,
    /// Shadow `BC'`.
    BcAlt,
    /// Shadow `DE'`.
    DeAlt,
    /// Shadow `HL'`.
    HlAlt,
}

impl Reg {
    /// Every register, in the order a debugger should list them.
    pub const ALL: &'static [Reg] = &[
        Reg::Af,
        Reg::Bc,
        Reg::De,
        Reg::Hl,
        Reg::Ix,
        Reg::Iy,
        Reg::Sp,
        Reg::Pc,
        Reg::I,
        Reg::R,
        Reg::Wz,
        Reg::AfAlt,
        Reg::BcAlt,
        Reg::DeAlt,
        Reg::HlAlt,
    ];

    /// The register's name, lowercase, as gdb and the monitor spell it.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Reg::Af => "af",
            Reg::Bc => "bc",
            Reg::De => "de",
            Reg::Hl => "hl",
            Reg::Ix => "ix",
            Reg::Iy => "iy",
            Reg::Sp => "sp",
            Reg::Pc => "pc",
            Reg::I => "i",
            Reg::R => "r",
            Reg::Wz => "wz",
            Reg::AfAlt => "af'",
            Reg::BcAlt => "bc'",
            Reg::DeAlt => "de'",
            Reg::HlAlt => "hl'",
        }
    }

    /// How wide the register is.
    #[must_use]
    pub const fn width(self) -> Width {
        match self {
            Reg::I | Reg::R => Width::U8,
            _ => Width::U16,
        }
    }

    /// Read this register out of a register file.
    #[must_use]
    pub const fn get(self, regs: &Regs) -> u16 {
        match self {
            Reg::Af => regs.af(),
            Reg::Bc => regs.bc(),
            Reg::De => regs.de(),
            Reg::Hl => regs.hl(),
            Reg::Ix => regs.ix,
            Reg::Iy => regs.iy,
            Reg::Sp => regs.sp,
            Reg::Pc => regs.pc,
            Reg::I => regs.i as u16,
            Reg::R => regs.r as u16,
            Reg::Wz => regs.wz,
            Reg::AfAlt => regs.af_alt,
            Reg::BcAlt => regs.bc_alt,
            Reg::DeAlt => regs.de_alt,
            Reg::HlAlt => regs.hl_alt,
        }
    }

    /// Write this register into a register file, truncating to its width.
    pub const fn set(self, regs: &mut Regs, value: u16) {
        match self {
            Reg::Af => regs.set_af(value),
            Reg::Bc => regs.set_bc(value),
            Reg::De => regs.set_de(value),
            Reg::Hl => regs.set_hl(value),
            Reg::Ix => regs.ix = value,
            Reg::Iy => regs.iy = value,
            Reg::Sp => regs.sp = value,
            Reg::Pc => regs.pc = value,
            Reg::I => regs.i = value as u8,
            Reg::R => regs.r = value as u8,
            Reg::Wz => regs.wz = value,
            Reg::AfAlt => regs.af_alt = value,
            Reg::BcAlt => regs.bc_alt = value,
            Reg::DeAlt => regs.de_alt = value,
            Reg::HlAlt => regs.hl_alt = value,
        }
    }

    /// Look a register up by name.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Reg> {
        Reg::ALL.iter().copied().find(|r| r.name() == name)
    }
}

impl fmt::Display for Reg {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// What one M-cycle asked of the bus.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[non_exhaustive]
pub enum MCycle {
    /// An `M1` opcode fetch: a read at `PC`, then two T-states with the
    /// refresh address on the pins.
    Fetch,
    /// A memory read.
    Read,
    /// A memory write.
    Write,
    /// An I/O port read. Four T-states, because the Z80 inserts one wait
    /// state so peripherals need no `WAIT` logic of their own.
    PortRead,
    /// An I/O port write.
    PortWrite,
    /// An interrupt acknowledge: an `M1` whose byte comes from the
    /// interrupting device rather than from memory.
    Ack,
    /// An internal operation. No bus request; the address pins keep whatever
    /// the previous M-cycle left on them.
    ///
    /// The default, because a zeroed [`BusCycle`] describes nothing happening.
    #[default]
    Internal,
}

/// One M-cycle of bus activity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct BusCycle {
    /// What kind of M-cycle this was.
    pub kind: MCycle,
    /// The address driven for the request. For [`MCycle::Internal`] this is
    /// the stale address the pins were still holding.
    pub addr: u16,
    /// The byte transferred. Meaningless for [`MCycle::Internal`].
    pub value: u8,
    /// The refresh address driven during the second half of a fetch or
    /// acknowledge; zero otherwise.
    pub refresh: u16,
    /// How many T-states this M-cycle occupied.
    pub tstates: u8,
}

/// How many M-cycles [`CycleLog`] records before it gives up.
///
/// The longest single instruction is 23 T-states across eight M-cycles, so the
/// only way to overflow this is a run of redundant `$dd`/`$fd` prefixes —
/// legal, pointless, and not worth a heap allocation in the hot path.
pub const CYCLE_LOG_LEN: usize = 16;

/// The bus activity of one step.
///
/// Recorded unconditionally rather than behind a debug flag: it costs one
/// array store per M-cycle, and it is the only way to check a core's timing
/// against hardware instead of asserting it (`ROADMAP.md` §0).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CycleLog {
    cycles: [BusCycle; CYCLE_LOG_LEN],
    len: u8,
    truncated: bool,
}

impl CycleLog {
    /// An empty log.
    #[must_use]
    pub const fn new() -> CycleLog {
        CycleLog {
            cycles: [BusCycle {
                kind: MCycle::Internal,
                addr: 0,
                value: 0,
                refresh: 0,
                tstates: 0,
            }; CYCLE_LOG_LEN],
            len: 0,
            truncated: false,
        }
    }

    /// The M-cycles recorded, in the order they happened.
    #[inline]
    #[must_use]
    pub fn cycles(&self) -> &[BusCycle] {
        &self.cycles[..self.len as usize]
    }

    /// Whether the step performed more M-cycles than the log can hold.
    #[inline]
    #[must_use]
    pub const fn truncated(&self) -> bool {
        self.truncated
    }

    /// Total T-states across the recorded M-cycles.
    #[must_use]
    pub fn tstates(&self) -> u32 {
        self.cycles().iter().map(|c| u32::from(c.tstates)).sum()
    }

    #[inline]
    pub(crate) fn clear(&mut self) {
        self.len = 0;
        self.truncated = false;
    }

    #[inline]
    pub(crate) fn push(&mut self, cycle: BusCycle) {
        match self.cycles.get_mut(self.len as usize) {
            Some(slot) => {
                *slot = cycle;
                self.len += 1;
            }
            None => self.truncated = true,
        }
    }
}

impl Default for CycleLog {
    fn default() -> Self {
        CycleLog::new()
    }
}

/// Which interrupt input was taken.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Interrupt {
    /// Maskable, level-sensitive, gated by `IFF1` and vectored by the mode.
    Int,
    /// Non-maskable, edge-sensitive, always vectored through `$0066`.
    Nmi,
}

/// How this particular part and board differ from the generic Z80.
///
/// Construction properties, never `#[cfg]`: one build of rsemu has to be able
/// to run a Master System *and* a CP/M box.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// The byte the undocumented `OUT (C),0` writes.
    ///
    /// `$00` on the NMOS Z80, `$ff` on the CMOS parts — the one place the two
    /// families visibly disagree, and the reason this is a property rather
    /// than a constant.
    pub out_c_zero: u8,
    /// What a read of an address or port nothing answers returns.
    ///
    /// A Z80 has no bus-error input, so a refused access cannot raise an
    /// exception: the data pins float and the CPU latches whatever is there.
    /// `$ff` matches a bus with pull-ups, which is the common case.
    pub floating_bus: u8,
    /// This core's identity in `MemAttrs::requester`, for an IOMMU or a
    /// per-master filter.
    pub requester: RequesterId,
}

impl Config {
    /// A plain NMOS Z80.
    pub const NMOS: Config = Config {
        out_c_zero: 0x00,
        floating_bus: 0xff,
        requester: RequesterId::ANONYMOUS,
    };

    /// A CMOS Z80 (Z84C00 and relatives): `OUT (C),0` writes `$ff` instead.
    pub const CMOS: Config = Config {
        out_c_zero: 0xff,
        ..Config::NMOS
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
        Config::NMOS
    }
}

/// The interrupt input pins and the acknowledge data bus, kept outside the
/// execution lock.
///
/// Deliberately atomics rather than fields under the mutex: a device asserting
/// `INT` from inside a write the CPU itself issued would otherwise re-enter
/// the CPU's own critical section, which is a deadlock under `native-std` and
/// a panic under `single` (`ROADMAP.md` §4.7).
#[derive(Debug)]
pub(crate) struct Lines {
    /// `INT` is level-sensitive: it is taken whenever it is asserted, `IFF1`
    /// is set, and the previous instruction was not `EI`.
    int: AtomicBool,
    /// The last level seen on `NMI`, for edge detection.
    nmi_level: AtomicBool,
    /// `NMI` is edge-sensitive: a high-going edge sets this latch, which stays
    /// set until the interrupt is serviced, however long that takes.
    nmi_latch: AtomicBool,
    /// The byte the interrupting device puts on the data bus during the
    /// acknowledge cycle: the mode 2 vector, or the `RST` opcode in mode 0.
    ///
    /// The fallback for a machine that has no acknowledging device, which is
    /// most of them: a Master System's VDP drives `INT` and nothing answers,
    /// so the value latched here is what the CPU reads. A board with a real
    /// vectored peripheral attaches an [`IntAck`] instead.
    vector: AtomicU8,
    /// A reset asked for by the `reset` pin, latched until the next step folds
    /// it into the execution state.
    ///
    /// A latch rather than a write into `State::reset_pending`, because a wire
    /// is driven from inside whatever device changed it — often from inside an
    /// access this very core issued — and reaching for the session lock there
    /// would re-enter the core's own critical section (`ROADMAP.md` §4.7).
    reset: AtomicBool,
    /// What answers the acknowledge cycle, if devices on the `INT` net do.
    ///
    /// A list, because a Z80 machine's peripherals sit in a **daisy chain**:
    /// `IEI`/`IEO` gives them a priority order, and the acknowledge belongs to
    /// the highest-priority one that has something pending. Attach order stands
    /// in for chain order, and a peripheral with nothing to say declines
    /// ([`IntAckResponse::Declined`]) and passes the cycle down the chain.
    ///
    /// Weak references, behind a leaf lock released before each outward call:
    /// the machine owns both devices, a CPU that kept its peripheral alive
    /// would close a cycle nothing could drop (§4.3), and the peripheral is
    /// free to take its own locks.
    acks: IntAckHandlers,
}

impl Default for Lines {
    fn default() -> Self {
        Lines {
            int: AtomicBool::new(false),
            nmi_level: AtomicBool::new(false),
            nmi_latch: AtomicBool::new(false),
            // An idle bus with pull-ups reads as $ff, which in mode 0 is
            // `RST 38` — the historical default a machine gets for free.
            vector: AtomicU8::new(0xff),
            reset: AtomicBool::new(false),
            acks: IntAckHandlers::new(),
        }
    }
}

impl Lines {
    fn set_int(&self, asserted: bool) {
        self.int.store(asserted, Ordering::Release);
    }

    fn irq_asserted(&self) -> bool {
        self.int.load(Ordering::Acquire)
    }

    /// Drive the `NMI` pin, latching a high-going edge.
    fn set_nmi(&self, asserted: bool) {
        let previous = self.nmi_level.swap(asserted, Ordering::AcqRel);
        if asserted && !previous {
            self.nmi_latch.store(true, Ordering::Release);
        }
    }

    fn nmi_pending(&self) -> bool {
        self.nmi_latch.load(Ordering::Acquire)
    }

    /// Consume the `NMI` latch, reporting whether it was set.
    fn take_nmi_pending(&self) -> bool {
        self.nmi_latch.swap(false, Ordering::AcqRel)
    }

    fn clear_nmi_latch(&self) {
        self.nmi_latch.store(false, Ordering::Release);
    }

    fn vector(&self) -> u8 {
        self.vector.load(Ordering::Acquire)
    }

    fn set_vector(&self, value: u8) {
        self.vector.store(value, Ordering::Release);
    }

    /// Latch a reset request. Cleared by whoever folds it into the state.
    fn request_reset(&self) {
        self.reset.store(true, Ordering::Release);
    }

    /// Consume the latch, reporting whether a reset was owed.
    fn take_reset_request(&self) -> bool {
        self.reset.swap(false, Ordering::AcqRel)
    }

    /// Add a device to those that answer the acknowledge cycle on `INT`.
    fn attach_ack(&self, ack: Weak<dyn IntAck>) {
        self.acks.attach(ack);
    }

    /// Run the acknowledge cycle in interrupt mode `mode` and report the byte
    /// the device drove.
    ///
    /// The mode travels with the cycle: what the CPU does with the byte is
    /// wholly different in modes 0, 1 and 2, and a peripheral is entitled to
    /// know which it is answering into. With nothing attached — or nothing that
    /// claims the chain — the latched byte is the answer, which is what a
    /// machine with one fixed source sets once and what an idle bus with
    /// pull-ups reads as.
    ///
    /// No lock is held across the outward call: the re-entrancy contract
    /// forbids holding one across a call into another device (§4.7).
    pub(crate) fn acknowledge(&self, mode: u8) -> u8 {
        match self.acks.run(IntAckCycle::data_bus(mode)) {
            IntAckResponse::Vector(byte) => byte as u8,
            // A Z80 has no `VPA`: a cycle nobody drives leaves the data pins
            // floating, and the latch is what models that floating bus.
            IntAckResponse::Autovector | IntAckResponse::Declined => self.vector(),
        }
    }

    fn snapshot(&self) -> (bool, bool, bool, u8) {
        (
            self.irq_asserted(),
            self.nmi_level.load(Ordering::Acquire),
            self.nmi_pending(),
            self.vector(),
        )
    }

    fn restore(&self, (int, level, latch, vector): (bool, bool, bool, u8)) {
        self.int.store(int, Ordering::Release);
        self.nmi_level.store(level, Ordering::Release);
        self.nmi_latch.store(latch, Ordering::Release);
        self.vector.store(vector, Ordering::Release);
    }
}

/// Everything the interpreter needs to mutate, behind one lock.
#[derive(Debug)]
struct Session {
    state: State,
    space: Option<Arc<AddressSpace>>,
    io: Option<Arc<AddressSpace>>,
}

/// A Zilog Z80 core.
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
/// asserting `INT` from inside a write the CPU itself issued cannot re-enter
/// the CPU's own critical section.
#[derive(Debug)]
pub struct Z80 {
    cfg: Config,
    lines: Arc<Lines>,
    /// This core's identity in `MemAttrs::requester`, assigned at bind time.
    ///
    /// Separate from [`Config::requester`] because a machine file names no
    /// requester: the machine layer allocates one per initiator and hands it
    /// over in [`Instance::bind`](crate::machine::Instance::bind), which is
    /// after `new` (`ROADMAP.md` §4.4).
    requester: AtomicU32,
    /// The name of the address space `IN` and `OUT` reach, from the `iospace`
    /// property. Resolved in `bind`, because spaces do not exist before then.
    iospace: String,
    session: sync::Mutex<Session>,
    /// The strong end of every pin this core has handed to a wire.
    ///
    /// A net holds its sinks weakly — the machine owns devices and a wire
    /// merely refers to them (§4.3) — so a pin nothing else kept alive would
    /// die on the way out of [`Device::sink`] and the wire would silently
    /// deliver to nothing.
    pins: sync::Mutex<Pins>,
}

/// The pins [`Device::sink`] has built, kept alive by the core that owns them.
#[derive(Debug, Default)]
struct Pins {
    int: Option<Arc<InterruptPin>>,
    nmi: Option<Arc<InterruptPin>>,
    reset: Option<Arc<ResetPin>>,
}

impl Z80 {
    /// A core in its power-on state, with no address space yet.
    ///
    /// Two-phase construction (`ROADMAP.md` §4.4): nothing observable happens
    /// until [`attach_space`](Z80::attach_space) and [`Device::realize`]. The
    /// first [`step`](Z80::step) runs the reset sequence.
    #[must_use]
    pub fn new(cfg: Config) -> Z80 {
        Z80 {
            cfg,
            lines: Arc::new(Lines::default()),
            requester: AtomicU32::new(cfg.requester.0),
            iospace: String::new(),
            session: sync::Mutex::with_rank(
                LockRank::BUS,
                Session {
                    state: State::new(),
                    space: None,
                    io: None,
                },
            ),
            pins: sync::Mutex::new(Pins::default()),
        }
    }

    /// Build one from machine-description properties.
    ///
    /// # Errors
    ///
    /// If a property has the wrong type or is out of range, or a property
    /// nothing here accepts was given — a typo'd property that was silently
    /// ignored is an afternoon lost.
    pub fn from_props(props: &Props) -> Result<Z80> {
        let mut r = props.reader();
        let cmos = r.or("cmos", false)?;
        let default = if cmos { Config::CMOS } else { Config::NMOS };
        let out_c_zero = r.or_range("out-c-zero", u64::from(default.out_c_zero), 0..=0xff)?;
        let floating = r.or_range("floating-bus", u64::from(default.floating_bus), 0..=0xff)?;
        // Accepted and ignored: there is one engine until phase 5, and a
        // machine file that names it should not need editing when the second
        // one lands.
        let _engine = r.or_enum("engine", "interp", &["interp"])?;
        // `space =` is structural and there is exactly one of it, so the Z80's
        // *second* space — the 64 KiB of ports `IN` and `OUT` reach, which no
        // memory-mapped core has — is named by an ordinary string property and
        // looked up with `BindCtx::space_named`.
        let iospace = r.optional_str("iospace")?.unwrap_or("").to_string();
        r.finish()?;
        let mut cpu = Z80::new(Config {
            out_c_zero: out_c_zero as u8,
            floating_bus: floating as u8,
            requester: RequesterId::ANONYMOUS,
        });
        cpu.iospace = iospace;
        Ok(cpu)
    }

    /// Which address space `IN` and `OUT` reach, by name, or `""` for none.
    #[must_use]
    pub fn io_space_name(&self) -> &str {
        &self.iospace
    }

    /// This core's configuration, with the bind-time requester folded in.
    #[must_use]
    pub fn config(&self) -> Config {
        Config {
            requester: RequesterId(self.requester.load(Ordering::Relaxed)),
            ..self.cfg
        }
    }

    /// Give the core the identity its accesses travel under.
    ///
    /// The machine layer calls this from `bind`; a crate driving the core
    /// directly usually sets [`Config::requester`] at construction instead.
    pub fn set_requester(&self, id: RequesterId) {
        self.requester.store(id.0, Ordering::Relaxed);
    }

    /// Give the core the memory address space it executes from.
    pub fn attach_space(&self, space: Arc<AddressSpace>) {
        self.session.lock().space = Some(space);
    }

    /// Give the core its **I/O** address space, which `IN` and `OUT` reach and
    /// nothing else does.
    ///
    /// Optional: a machine with no ports simply never calls this, and reads
    /// return [`Config::floating_bus`] rather than faulting.
    pub fn attach_io_space(&self, space: Arc<AddressSpace>) {
        self.session.lock().io = Some(space);
    }

    /// The memory address space this core executes from, if one is attached.
    #[must_use]
    pub fn space(&self) -> Option<Arc<AddressSpace>> {
        self.session.lock().space.clone()
    }

    /// The I/O address space, if one is attached.
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
    pub fn set_regs(&self, regs: Regs) {
        self.session.lock().state.regs = regs;
    }

    /// Read one register by name.
    #[must_use]
    pub fn reg(&self, reg: Reg) -> u16 {
        reg.get(&self.session.lock().state.regs)
    }

    /// Write one register by name.
    pub fn set_reg(&self, reg: Reg, value: u16) {
        reg.set(&mut self.session.lock().state.regs, value);
    }

    /// T-states executed since power-on.
    #[must_use]
    pub fn cycles(&self) -> u64 {
        self.session.lock().state.cycles
    }

    /// The bus activity of the most recent [`step`](Z80::step).
    ///
    /// One entry per M-cycle, in order. This is what a bus-level trace, a
    /// logic-analyser view and the conformance runner all read; it is also the
    /// only honest way to check the core's timing rather than assert it.
    #[must_use]
    pub fn last_cycles(&self) -> CycleLog {
        self.session.lock().state.trace
    }

    /// Whether `HALT` has suspended the core.
    ///
    /// A halted Z80 is not stopped: it keeps issuing `M1` cycles so dynamic
    /// RAM stays refreshed, and [`step`](Z80::step) charges four T-states for
    /// each of them. Only an interrupt or a reset ends it.
    #[must_use]
    pub fn is_halted(&self) -> bool {
        self.session.lock().state.halted
    }

    /// Whether a reset sequence is still owed.
    #[must_use]
    pub fn reset_pending(&self) -> bool {
        self.session.lock().state.reset_pending
    }

    /// The two interrupt enable flip-flops, `IFF1` first.
    ///
    /// `IFF2` is `IFF1`'s backup across an `NMI`, and it is what `LD A,I`
    /// copies into the parity flag — which is the only way a program can read
    /// either of them.
    #[must_use]
    pub fn iff(&self) -> (bool, bool) {
        let s = self.session.lock();
        (s.state.iff1, s.state.iff2)
    }

    /// Set both interrupt enable flip-flops.
    pub fn set_iff(&self, iff1: bool, iff2: bool) {
        let mut s = self.session.lock();
        s.state.iff1 = iff1;
        s.state.iff2 = iff2;
    }

    /// The selected interrupt mode, 0 to 2.
    #[must_use]
    pub fn interrupt_mode(&self) -> u8 {
        self.session.lock().state.im
    }

    /// Select the interrupt mode, as `IM n` would.
    ///
    /// # Errors
    ///
    /// If `mode` is not 0, 1 or 2.
    pub fn set_interrupt_mode(&self, mode: u8) -> Result<()> {
        if mode > 2 {
            return Err(Error::Property(alloc::format!(
                "interrupt mode {mode} does not exist; the Z80 has modes 0, 1 and 2"
            )));
        }
        self.session.lock().state.im = mode;
        Ok(())
    }

    /// How many accesses the address spaces refused, and where the last one
    /// was.
    ///
    /// A Z80 has no bus-error input, so a refused access cannot raise an
    /// exception: the read returns [`Config::floating_bus`], which is what a
    /// bus with pull-ups does. This counter is how that becomes visible
    /// instead of silent.
    #[must_use]
    pub fn bus_faults(&self) -> (u64, u16) {
        let s = self.session.lock();
        (s.state.faults, s.state.last_fault)
    }

    /// Drive the `INT` pin. Level-sensitive: it is taken while asserted,
    /// `IFF1` is set, and the previous instruction was not `EI`.
    ///
    /// `asserted` is the logical level, not the pin's: a real `/INT` is
    /// active-low, and inverting it belongs to whatever models the wire.
    pub fn set_int(&self, asserted: bool) {
        self.lines.set_int(asserted);
    }

    /// Whether `INT` is currently asserted.
    #[must_use]
    pub fn int_asserted(&self) -> bool {
        self.lines.irq_asserted()
    }

    /// Drive the `NMI` pin. Edge-sensitive: a high-going edge latches, and the
    /// latch survives until the interrupt is taken.
    pub fn set_nmi(&self, asserted: bool) {
        self.lines.set_nmi(asserted);
    }

    /// A complete `NMI` pulse, for a caller that does not model the pin's
    /// level.
    pub fn pulse_nmi(&self) {
        self.lines.set_nmi(true);
        self.lines.set_nmi(false);
    }

    /// Whether an `NMI` edge is latched and not yet serviced.
    #[must_use]
    pub fn nmi_pending(&self) -> bool {
        self.lines.nmi_pending()
    }

    /// Set the byte the interrupting device puts on the data bus during the
    /// acknowledge cycle.
    ///
    /// In mode 2 this is the low half of the vector-table address; in mode 0
    /// it is an opcode, conventionally an `RST`. Defaults to `$ff`, which is
    /// what an undriven bus with pull-ups reads as — and `RST 38` in mode 0.
    pub fn set_interrupt_vector(&self, vector: u8) {
        self.lines.set_vector(vector);
    }

    /// The byte the acknowledge cycle will read.
    #[must_use]
    pub fn interrupt_vector(&self) -> u8 {
        self.lines.vector()
    }

    /// Request a reset sequence without changing any register.
    ///
    /// The sequence runs on the next [`step`](Z80::step), because a reset is a
    /// signal rather than a method call.
    pub fn request_reset(&self) {
        self.session.lock().state.reset_pending = true;
    }

    /// Execute one reset sequence, interrupt sequence, halt cycle or
    /// instruction.
    ///
    /// Returns the T-states charged: zero only if no address space is
    /// attached, which the caller must treat as "stop", not "retry". A halted
    /// core still returns four, because it is still refreshing.
    pub fn step(&self) -> u64 {
        let reset = self.lines.take_reset_request();
        let cfg = self.config();
        let mut session = self.session.lock();
        // Destructured so the two spaces can be borrowed while `state` is
        // borrowed mutably: they are different fields. Cloning the `Arc`s
        // instead would put two atomic refcount updates on the path of every
        // instruction, for a lifetime the lock already guarantees.
        let Session { state, space, io } = &mut *session;
        // The `reset` pin latches outside the lock; this is where the latch
        // becomes execution state, before the step, so a pulse is honoured at
        // the very next instruction boundary.
        state.reset_pending |= reset;
        let io = io.as_deref();
        let Some(space) = space.as_deref() else {
            return 0;
        };
        Exec::new(state, space, io, &cfg, &self.lines).step()
    }

    /// Execute until at least `budget` T-states have been charged.
    ///
    /// Returns the T-states actually used, which overshoots by at most one
    /// instruction — a Z80 cannot be stopped mid-instruction, and pretending
    /// otherwise is how a scheduler ends up with a CPU in an impossible state.
    ///
    /// [`run_budget`](Z80::run_budget) is the same loop with the overshoot
    /// carried forward instead, which is what the scheduler needs.
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

    /// Execute for at most `ticks`, carrying any overshoot into the next call.
    ///
    /// The scheduler hands out a budget and refuses a report larger than it, so
    /// the instruction that ran past the end is paid for by the *following*
    /// budget through `State::debt` — which keeps the core's T-state count
    /// exact while never letting its clock domain run ahead of the timeline.
    ///
    /// A core with no address space consumes only the debt it owed: a halted
    /// one is still refreshing and still charges.
    pub fn run_budget(&self, ticks: u64) -> u64 {
        let owed = self.session.lock().state.debt;
        if owed >= ticks {
            // The last instruction was longer than this whole budget: charge
            // the budget against the debt and execute nothing.
            self.session.lock().state.debt = owed - ticks;
            return ticks;
        }
        let allowance = ticks - owed;
        let mut used = 0u64;
        while used < allowance {
            let n = self.step();
            if n == 0 {
                // No address space. Stop — retrying would spin.
                break;
            }
            used += n;
        }
        if used >= allowance {
            self.session.lock().state.debt = used - allowance;
            ticks
        } else {
            self.session.lock().state.debt = 0;
            owed + used
        }
    }

    /// T-states owed to the next budget — see [`run_budget`](Z80::run_budget).
    #[must_use]
    pub fn cycle_debt(&self) -> u64 {
        self.session.lock().state.debt
    }

    /// Disassemble `count` instructions starting at `pc`, reading guest memory
    /// with debug attributes.
    ///
    /// Debug attributes are the point: a monitor listing the code around PC
    /// must not pop a FIFO or clear a status bit on the way (`ROADMAP.md`
    /// §15, invariant 5).
    #[must_use]
    pub fn disassemble(&self, pc: u16, count: usize) -> Vec<disasm::Disassembled> {
        let Some(space) = self.space() else {
            return Vec::new();
        };
        disasm::disassemble_run(pc, count, |addr| {
            space
                .read(u64::from(addr), Width::U8, MemAttrs::DEBUG)
                .ok()
                .map(|v| v as u8)
        })
    }
}

/// The `cpu.z80` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: "cpu.z80",
    // 2: the chunk gained the scheduler debt, without which a restored core
    // runs one instruction free.
    version: 2,
    summary: "Zilog Z80 8-bit CPU core, cycle-accurate interpreter",
    properties: &[
        PropertySpec {
            name: "cmos",
            kind: ValueKind::Bool,
            required: false,
            summary: "select the CMOS part, whose OUT (C),0 writes $ff instead of $00",
        },
        PropertySpec {
            name: "out-c-zero",
            kind: ValueKind::Uint,
            required: false,
            summary: "the byte the undocumented OUT (C),0 writes, overriding the part default",
        },
        PropertySpec {
            name: "floating-bus",
            kind: ValueKind::Uint,
            required: false,
            summary: "what a read nothing answers returns; $ff is a bus with pull-ups",
        },
        PropertySpec {
            name: "engine",
            kind: ValueKind::Str,
            required: false,
            summary: "which execution engine; only `interp` exists until phase 5",
        },
        PropertySpec {
            name: "iospace",
            kind: ValueKind::Str,
            required: false,
            summary: "the name of the separate 64 KiB address space IN and OUT reach",
        },
    ],
    construct: |props| Ok(Box::new(Z80::from_props(props)?)),
};

/// Add this core's class to a registry.
///
/// Registration is explicit per feature rather than link-time magic
/// (`ROADMAP.md` §4.4), so the machine assembly layer calls this from its own
/// `#[cfg(feature = "cpu-z80")]` arm.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut Registry) -> Result<()> {
    reg.add(&CLASS)
}

impl Device for Z80 {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward. A CPU with no address space cannot fetch, but
        // realize runs *before* the machine binds one — that check belongs to
        // `Instance::bind`, which is where the space arrives.
        Ok(())
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        // The fan-in can only be built now: it is told its sources at
        // construction and no `WireId` existed when this core was made.
        //
        // Every pin is named the way the package names it, minus the bar:
        // `/INT`, `/NMI` and `/RESET` are asserted low on real silicon, and
        // inverting a level belongs to whatever models the wire.
        let mut pins = self.pins.lock();
        let sink: Arc<dyn WireSink> = match port {
            "int" => {
                let pin = Arc::new(InterruptPin::from_lines(
                    Arc::clone(&self.lines),
                    Interrupt::Int,
                    sources,
                ));
                pins.int = Some(Arc::clone(&pin));
                pin
            }
            "nmi" => {
                let pin = Arc::new(InterruptPin::from_lines(
                    Arc::clone(&self.lines),
                    Interrupt::Nmi,
                    sources,
                ));
                pins.nmi = Some(Arc::clone(&pin));
                pin
            }
            "reset" => {
                let pin = Arc::new(ResetPin::new(Arc::clone(&self.lines), sources));
                pins.reset = Some(Arc::clone(&pin));
                pin
            }
            _ => return None,
        };
        Some(SinkPin { sink, line: 0 })
    }

    fn attach_int_ack(&self, port: &str, ack: Weak<dyn IntAck>) {
        // Only `INT` has an acknowledge cycle. `NMI` is vectored through
        // $0066 by the architecture and no device drives a byte for it.
        if port == "int" {
            self.lines.attach_ack(ack);
        }
    }

    fn is_runnable(&self) -> bool {
        true
    }

    fn run(&self, budget: Budget) -> Consumed {
        Consumed::new(self.run_budget(budget.ticks))
    }

    fn reset(&self, kind: ResetKind) {
        let mut session = self.session.lock();
        if kind == ResetKind::Cold {
            session.state = State::new();
        } else {
            // A warm reset is a pulse on the RESET pin: the sequence itself
            // clears PC, I, R and both flip-flops, and everything else keeps
            // its value.
            session.state.reset_pending = true;
            session.state.halted = false;
        }
        drop(session);
        if kind == ResetKind::Cold {
            self.lines.restore((false, false, false, 0xff));
        } else {
            // The input *levels* belong to whatever drives them, not to the
            // CPU — clearing them here would make a reset lie about the
            // machine. The edge latch is internal, so it goes.
            self.lines.clear_nmi_latch();
        }
        // The sequence the machine just asked for is the one the pin owed.
        self.lines.take_reset_request();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        // Fold the `reset` pin's latch in first. It is not a field of its own
        // in the chunk: `reset_pending` is where it was always going, and a
        // snapshot taken between an assertion and the next step would otherwise
        // lose the reset entirely.
        let reset = self.lines.take_reset_request();
        let state = {
            let mut session = self.session.lock();
            session.state.reset_pending |= reset;
            session.state
        };
        let r = state.regs;
        for value in [
            r.af(),
            r.bc(),
            r.de(),
            r.hl(),
            r.ix,
            r.iy,
            r.sp,
            r.pc,
            r.wz,
            r.af_alt,
            r.bc_alt,
            r.de_alt,
            r.hl_alt,
        ] {
            w.write_u16(value)?;
        }
        w.write_u8(r.i)?;
        w.write_u8(r.r)?;
        w.write_bool(state.iff1)?;
        w.write_bool(state.iff2)?;
        w.write_u8(state.im)?;
        w.write_bool(state.halted)?;
        w.write_bool(state.ei_pending)?;
        w.write_bool(state.after_ld_ir)?;
        w.write_u8(state.q)?;
        w.write_u64(state.cycles)?;
        w.write_bool(state.reset_pending)?;
        w.write_u64(state.faults)?;
        w.write_u16(state.last_fault)?;
        w.write_u64(state.debt)?;
        let (int, nmi_level, nmi_latch, vector) = self.lines.snapshot();
        w.write_bool(int)?;
        w.write_bool(nmi_level)?;
        w.write_bool(nmi_latch)?;
        w.write_u8(vector)?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        // Derived state is never serialized (invariant 3): the cycle log
        // describes the step that is already over.
        let mut state = State::new();
        let regs = &mut state.regs;
        regs.set_af(r.read_u16()?);
        regs.set_bc(r.read_u16()?);
        regs.set_de(r.read_u16()?);
        regs.set_hl(r.read_u16()?);
        regs.ix = r.read_u16()?;
        regs.iy = r.read_u16()?;
        regs.sp = r.read_u16()?;
        regs.pc = r.read_u16()?;
        regs.wz = r.read_u16()?;
        regs.af_alt = r.read_u16()?;
        regs.bc_alt = r.read_u16()?;
        regs.de_alt = r.read_u16()?;
        regs.hl_alt = r.read_u16()?;
        regs.i = r.read_u8()?;
        regs.r = r.read_u8()?;
        state.iff1 = r.read_bool()?;
        state.iff2 = r.read_bool()?;
        state.im = r.read_u8()?;
        if state.im > 2 {
            return Err(Error::State(alloc::format!(
                "snapshot names interrupt mode {}, which does not exist",
                state.im
            )));
        }
        state.halted = r.read_bool()?;
        state.ei_pending = r.read_bool()?;
        state.after_ld_ir = r.read_bool()?;
        state.q = r.read_u8()?;
        state.cycles = r.read_u64()?;
        state.reset_pending = r.read_bool()?;
        state.faults = r.read_u64()?;
        state.last_fault = r.read_u16()?;
        state.debt = r.read_u64()?;
        let int = r.read_bool()?;
        let nmi_level = r.read_bool()?;
        let nmi_latch = r.read_bool()?;
        let vector = r.read_u8()?;
        self.session.lock().state = state;
        self.lines.restore((int, nmi_level, nmi_latch, vector));
        Ok(())
    }
}

impl Initiator for Z80 {
    fn requester(&self) -> RequesterId {
        RequesterId(self.requester.load(Ordering::Relaxed))
    }
}

/// The machine layer's half: a Z80 needs its memory space, and — unlike every
/// memory-mapped core in this crate — may need a second one.
///
/// `IN` and `OUT` reach a **separate 64 KiB address space**, not a window into
/// memory: the chip drives `/IORQ` instead of `/MREQ` and a board decodes it
/// differently. `space =` is structural and there is one of it, so the I/O
/// space is named by the `iospace` string property. A machine that names none
/// gets a core whose `IN` reads [`Config::floating_bus`] and whose `OUT` goes
/// nowhere, which is what an unpopulated port bus does.
impl crate::machine::Instance for Z80 {
    fn bind(&self, ctx: &crate::machine::BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: ctx.path().to_string(),
            message: String::from("a Z80 needs an address space to fetch from (`space = mem`)"),
        })?;
        self.attach_space(Arc::clone(space));
        if !self.iospace.is_empty() {
            let io = ctx
                .space_named(&self.iospace)
                .ok_or_else(|| Error::Config {
                    at: ctx.path().to_string(),
                    message: alloc::format!(
                        "`iospace = \"{}\"` names no address space in this machine",
                        self.iospace
                    ),
                })?;
            self.attach_io_space(Arc::clone(io));
        }
        self.set_requester(ctx.requester());
        Ok(())
    }
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS.name, |props| Ok(Arc::new(Z80::from_props(props)?)))
}

/// What the validator should know about `cpu.z80`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS.name)
        .prop(PropSchema::new("cmos", ValueKind::Bool))
        .prop(PropSchema::new("out-c-zero", ValueKind::Uint).range(0, 0xff))
        .prop(PropSchema::new("floating-bus", ValueKind::Uint).range(0, 0xff))
        .prop(PropSchema::new("engine", ValueKind::Str).values(&["interp"]))
        .prop(PropSchema::new("iospace", ValueKind::Str))
        // Inputs only. `/BUSRQ` and `/WAIT` are real pins with no model behind
        // them yet, and the outputs (`/M1`, `/IORQ`, `/RFSH`) are the address
        // space's business rather than a wire's.
        .port("int", PortDir::In)
        .port("nmi", PortDir::In)
        .port("reset", PortDir::In)
}

/// One of the CPU's two interrupt inputs, as something a [`Wire`] can drive.
///
/// A wire hands each sink the level of the *driver that changed*, not the
/// resolved level of the net, because a net with several drivers is resolved
/// by whoever cares. A Z80 machine's `/INT` line typically has several, so
/// this keeps a [`FanIn`] and wire-ORs them — which is what the
/// open-collector line does in hardware.
///
/// [`Wire`]: crate::core::wire::Wire
#[derive(Debug)]
pub struct InterruptPin {
    lines: Arc<Lines>,
    which: Interrupt,
    inputs: FanIn,
    resolve: Resolve,
}

impl InterruptPin {
    /// Connect `which` pin of `cpu` to a net driven by `sources`.
    ///
    /// Wire-OR by default: any source asserting asserts the pin, which is how
    /// an open-collector interrupt line behaves.
    ///
    /// The pin keeps a handle on the core's *input latches*, not on the core:
    /// the core owns the pin — something must, since a net holds only a weak
    /// reference to its sinks — and a pin that owned the core back would be a
    /// cycle the machine could never drop.
    #[must_use]
    pub fn new(cpu: Arc<Z80>, which: Interrupt, sources: &[WireId]) -> InterruptPin {
        InterruptPin::from_lines(Arc::clone(&cpu.lines), which, sources)
    }

    /// The same, given the latches directly.
    fn from_lines(lines: Arc<Lines>, which: Interrupt, sources: &[WireId]) -> InterruptPin {
        InterruptPin {
            lines,
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
            Interrupt::Int => self.lines.set_int(asserted),
            Interrupt::Nmi => self.lines.set_nmi(asserted),
        }
    }
}

/// The core's `/RESET` input, as something a [`Wire`] can drive.
///
/// Separate from [`InterruptPin`] because a reset is not an interrupt: it has
/// no flip-flop to gate it, no mode to vector it, and it clears `I` and `R`
/// rather than pushing anything. Asserting the line latches a request; the
/// sequence runs on the next [`Z80::step`].
///
/// [`Wire`]: crate::core::wire::Wire
#[derive(Debug)]
pub struct ResetPin {
    lines: Arc<Lines>,
    inputs: FanIn,
    resolve: Resolve,
}

impl ResetPin {
    /// Connect `cpu`'s `/RESET` pin to a net driven by `sources`.
    #[must_use]
    pub fn new_for(cpu: Arc<Z80>, sources: &[WireId]) -> ResetPin {
        ResetPin::new(Arc::clone(&cpu.lines), sources)
    }

    /// The same, given the latches directly.
    fn new(lines: Arc<Lines>, sources: &[WireId]) -> ResetPin {
        ResetPin {
            lines,
            inputs: FanIn::new(sources),
            resolve: Resolve::Or,
        }
    }

    /// The per-source levels currently seen.
    #[must_use]
    pub fn inputs(&self) -> &FanIn {
        &self.inputs
    }
}

impl WireSink for ResetPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        // Latch on assertion rather than on release: a machine holding its
        // reset button down should still come up, instead of waiting for a
        // release nobody modelled.
        if self.inputs.resolve(self.resolve).is_high() {
            self.lines.request_reset();
        }
    }
}

/// A description of this core's base page for `rsemu describe cpu.z80`.
///
/// Built from [`isa::BASE`], so it cannot drift from what the interpreter
/// implements.
#[must_use]
pub fn describe_isa() -> String {
    use core::fmt::Write as _;
    let mut out = String::new();
    for opcode in 0..=255u8 {
        let insn = isa::decode(opcode);
        let mark = if insn.class.is_documented() { ' ' } else { '*' };
        let _ = writeln!(
            out,
            "{opcode:02x} {mark}{:<10} {}",
            disasm::mnemonic_and_operands(insn),
            insn.op.summary()
        );
    }
    out
}
