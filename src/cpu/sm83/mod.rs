//! The Sharp SM83 — the Game Boy's processor — as an M-cycle-accurate
//! interpreter.
//!
//! The SM83 is **not a Z80 and not an 8080**, and treating it as either is the
//! standard way to get a Game Boy subtly wrong (`docs/cpu/z80-sm83.md`). It has
//! no `IX`/`IY`, no alternate register set, no block instructions and no
//! separate I/O space; it adds `LDH`, the four `LD (HL±)` forms, `SWAP`, `STOP`,
//! and a `DAA` whose rule is its own. All 256 first-page and 256 `$CB`-page
//! encodings are implemented, and the eleven holes in the matrix hang the
//! processor the way the hardware does rather than acting as `NOP`.
//!
//! # What "M-cycle accurate" means here
//!
//! Not "the instruction took four cycles". Every machine cycle is a read, a
//! write, or a documented internal cycle on the guest's [`AddressSpace`], and
//! the interpreter has no cycle counter it bumps independently — a cycle is
//! charged *because* something happened. See [`exec`](self) and
//! [`isa`], which deliberately has no cycle column.
//!
//! This core's clock domain therefore counts **M-cycles**, one per four crystal
//! periods. On a Game Boy that is 4194304/4 = 1048576 ticks a second, and the
//! machine file expresses it as `clock = master / 4` so the relationship to the
//! PPU's dot clock stays exact by construction (`ROADMAP.md` §4.2).
//!
//! # The interrupt registers live here
//!
//! `IF` (`$FF0F`) and `IE` (`$FFFF`) are part of the processor, not of a
//! separate controller, so this device publishes them as two one-byte regions a
//! `map` statement names ([`IF_REGION`], [`IE_REGION`]) and offers five input
//! pins — `vblank`, `stat`, `timer`, `serial`, `joypad` — whose **rising edge**
//! sets the matching `IF` bit. That is exactly how the hardware works, and it
//! buys one behaviour for free that is otherwise a special case: the LCD's STAT
//! line is a *level*, ORed from up to four enabled conditions, and a second
//! condition becoming true while the first still holds raises no second
//! interrupt. Emulators call that "STAT blocking" and implement it deliberately;
//! here it is what an edge detector on a wire does.
//!
//! # Assembling one
//!
//! ```
//! use std::sync::Arc;
//! use rsemu::core::space::{AddressSpace, RamStore, Region};
//! use rsemu::core::device::{Device, ResetKind};
//! use rsemu::cpu::sm83::{Config, Sm83};
//!
//! // 64 KiB of RAM with one `LD A,$42` at the post-boot entry point.
//! let ram = Arc::new(RamStore::new(0x1_0000));
//! ram.write_u8(0x0100, 0x3e).unwrap();
//! ram.write_u8(0x0101, 0x42).unwrap();
//!
//! let space = AddressSpace::new("cpu", 16);
//! space.topology().map(Region::ram("ram", ram), 0).unwrap();
//!
//! let cpu = Sm83::new(Config::default());
//! cpu.attach_space(Arc::new(space));
//! Device::reset(&cpu, ResetKind::Cold);
//! assert_eq!(cpu.regs().pc, 0x0100);   // the boot ROM's parting gift
//! cpu.step();
//! assert_eq!(cpu.regs().a, 0x42);
//! assert_eq!(cpu.cycles(), 2);          // fetch, then the immediate
//! ```
//!
//! # Modules
//!
//! | Module | Holds |
//! | --- | --- |
//! | [`isa`] | the one declarative instruction description; decode and disassembly both read it |
//! | [`disasm`] | the disassembler generated from that description |
//! | `exec` (private) | the interpreter: one bus access or one documented idle per cycle |
//!
//! # Sources
//!
//! [Pan Docs](https://gbdev.io/pandocs/), which is CC0 and can be quoted
//! verbatim, plus Gekkio's *Game Boy: Complete Technical Reference* for
//! sub-instruction ordering, and the gbdev opcode tables for the matrix. No
//! emulator source of any licence was consulted (`ROADMAP.md` §1).

pub mod disasm;
mod exec;
pub mod isa;

#[cfg(test)]
mod tests;

// The conformance runners read downloaded ROMs off the filesystem, so they exist
// only where there is one (`ROADMAP.md` §12).
#[cfg(all(test, feature = "std"))]
mod conformance;

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{
    Device, DeviceClass, Initiator, PropertySpec, RealizeCtx, ResetKind, SinkPin,
};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::registry::Registry;
use crate::core::sched::{Budget, Consumed};
use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region as MmioRegion, RegionRef,
    RequesterId,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{self, AtomicBool, AtomicU8, AtomicU32, LockRank, Ordering};
use crate::core::value::Width;
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink};

use exec::{Exec, Mode, State};

pub use exec::VECTORS;

/// The four flag bits, which live in the top nibble of `F`.
///
/// The bottom nibble has no storage at all: it reads back as zero whatever is
/// written to it, which is why `PUSH AF` / `POP AF` round-trips are a standard
/// conformance test (Pan Docs, *CPU Registers and Flags*).
pub mod flags {
    /// Zero: the last result was zero.
    pub const Z: u8 = 0x80;
    /// Subtract: the last arithmetic operation was a subtraction. Read only by
    /// `DAA`.
    pub const N: u8 = 0x40;
    /// Half-carry: a carry out of, or borrow into, bit 3.
    pub const H: u8 = 0x20;
    /// Carry.
    pub const C: u8 = 0x10;
    /// The bits that exist. The rest read as zero.
    pub const STORED: u8 = 0xf0;
}

/// The five interrupt sources, as bit numbers in `IF` and `IE`.
///
/// Priority is lowest bit first, so a VBlank and a joypad request arriving
/// together dispatch VBlank (Pan Docs, *Interrupts*).
pub mod interrupt {
    /// The LCD entered VBlank. Vector `$40`.
    pub const VBLANK: u8 = 0;
    /// One of the LCD's four STAT conditions became true. Vector `$48`.
    pub const STAT: u8 = 1;
    /// `TIMA` overflowed. Vector `$50`.
    pub const TIMER: u8 = 2;
    /// A serial transfer completed. Vector `$58`.
    pub const SERIAL: u8 = 3;
    /// A selected joypad line went active. Vector `$60`.
    pub const JOYPAD: u8 = 4;
    /// The five bits that exist. `IF`'s top three read as ones.
    pub const MASK: u8 = 0x1f;
    /// The pin names, indexed by bit number.
    pub const PINS: [&str; 5] = ["vblank", "stat", "timer", "serial", "joypad"];
}

/// The most machine cycles any one step can take.
///
/// Six, for a taken `CALL nn`: the opcode fetch, two immediate fetches, the
/// stack predecrement and two pushes. The interrupt dispatch is five and every
/// instruction is fewer, so this bounds a step from above — which is what lets
/// [`Sm83::run_budget`] decline to *start* something that would overrun rather
/// than overrunning and carrying the difference.
pub const MAX_INSTRUCTION_CYCLES: u64 = 6;

/// Where `IF` sits in the CPU's address space.
pub const IF_ADDRESS: u64 = 0xff0f;

/// Where `IE` sits in the CPU's address space.
pub const IE_ADDRESS: u64 = 0xffff;

/// The name a `map` statement reaches `IF` by.
pub const IF_REGION: &str = "if";

/// The name a `map` statement reaches `IE` by.
pub const IE_REGION: &str = "ie";

/// The architectural register file.
///
/// Public and `Copy` because a debugger, a tracer and a test all want to read it
/// out and put it back — this is the surface a gdbstub serialises — and [`Reg`]
/// enumerates it by name.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Regs {
    /// Accumulator.
    pub a: u8,
    /// Flags. Only the top nibble has storage; see [`flags`].
    pub f: u8,
    /// `B`.
    pub b: u8,
    /// `C`.
    pub c: u8,
    /// `D`.
    pub d: u8,
    /// `E`.
    pub e: u8,
    /// `H`.
    pub h: u8,
    /// `L`.
    pub l: u8,
    /// Stack pointer.
    pub sp: u16,
    /// Program counter.
    pub pc: u16,
}

impl Regs {
    /// The state a cold power-on leaves behind, before any boot ROM has run.
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
            sp: 0,
            pc: 0,
        }
    }

    /// The register file the DMG boot ROM leaves behind when it hands control to
    /// the cartridge at `$0100`.
    ///
    /// Pan Docs, *Power-Up Sequence*. `F = $B0` is what a header whose checksum
    /// is non-zero produces; on real hardware **H** and **C** come out of that
    /// checksum, and essentially every commercial cartridge has a non-zero one.
    /// A machine that boots a real boot ROM never needs this — it is the
    /// substitute for one.
    #[must_use]
    pub const fn post_boot_dmg() -> Regs {
        Regs {
            a: 0x01,
            f: 0xb0,
            b: 0x00,
            c: 0x13,
            d: 0x00,
            e: 0xd8,
            h: 0x01,
            l: 0x4d,
            sp: 0xfffe,
            pc: 0x0100,
        }
    }

    /// `HL`, the pair with an addressing mode of its own.
    #[inline]
    #[must_use]
    pub const fn hl(&self) -> u16 {
        ((self.h as u16) << 8) | self.l as u16
    }

    /// Set `HL`.
    #[inline]
    pub const fn set_hl(&mut self, value: u16) {
        self.h = (value >> 8) as u8;
        self.l = value as u8;
    }

    /// `AF`, with the flag register's unstored low nibble masked off.
    #[inline]
    #[must_use]
    pub const fn af(&self) -> u16 {
        ((self.a as u16) << 8) | (self.f & flags::STORED) as u16
    }

    /// `BC`.
    #[inline]
    #[must_use]
    pub const fn bc(&self) -> u16 {
        ((self.b as u16) << 8) | self.c as u16
    }

    /// `DE`.
    #[inline]
    #[must_use]
    pub const fn de(&self) -> u16 {
        ((self.d as u16) << 8) | self.e as u16
    }

    /// Whether a status flag is set.
    #[inline]
    #[must_use]
    pub const fn flag(&self, mask: u8) -> bool {
        self.f & mask != 0
    }
}

impl fmt::Display for Regs {
    /// The one-line form a trace log wants, in the order Gekkio's test suite
    /// reports registers.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "A:{:02x} F:{:02x} B:{:02x} C:{:02x} D:{:02x} E:{:02x} H:{:02x} L:{:02x} \
             SP:{:04x} PC:{:04x}",
            self.a, self.f, self.b, self.c, self.d, self.e, self.h, self.l, self.sp, self.pc
        )
    }
}

/// One named register, for a debugger that works by name or index.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Reg {
    /// Accumulator.
    A,
    /// Flags.
    F,
    /// `B`.
    B,
    /// `C`.
    C,
    /// `D`.
    D,
    /// `E`.
    E,
    /// `H`.
    H,
    /// `L`.
    L,
    /// Stack pointer.
    Sp,
    /// Program counter.
    Pc,
}

impl Reg {
    /// Every register, in the order a debugger should list them.
    pub const ALL: &'static [Reg] = &[
        Reg::A,
        Reg::F,
        Reg::B,
        Reg::C,
        Reg::D,
        Reg::E,
        Reg::H,
        Reg::L,
        Reg::Sp,
        Reg::Pc,
    ];

    /// The register's name, lowercase, as gdb and the monitor spell it.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Reg::A => "a",
            Reg::F => "f",
            Reg::B => "b",
            Reg::C => "c",
            Reg::D => "d",
            Reg::E => "e",
            Reg::H => "h",
            Reg::L => "l",
            Reg::Sp => "sp",
            Reg::Pc => "pc",
        }
    }

    /// How wide the register is.
    #[must_use]
    pub const fn width(self) -> Width {
        match self {
            Reg::Sp | Reg::Pc => Width::U16,
            _ => Width::U8,
        }
    }

    /// Read this register out of a register file.
    #[must_use]
    pub const fn get(self, regs: &Regs) -> u16 {
        match self {
            Reg::A => regs.a as u16,
            Reg::F => regs.f as u16,
            Reg::B => regs.b as u16,
            Reg::C => regs.c as u16,
            Reg::D => regs.d as u16,
            Reg::E => regs.e as u16,
            Reg::H => regs.h as u16,
            Reg::L => regs.l as u16,
            Reg::Sp => regs.sp,
            Reg::Pc => regs.pc,
        }
    }

    /// Write this register into a register file, truncating to its width.
    pub const fn set(self, regs: &mut Regs, value: u16) {
        match self {
            Reg::A => regs.a = value as u8,
            // The unstored low nibble is masked here too, so a debugger cannot
            // put the core into a state the hardware cannot reach.
            Reg::F => regs.f = (value as u8) & flags::STORED,
            Reg::B => regs.b = value as u8,
            Reg::C => regs.c = value as u8,
            Reg::D => regs.d = value as u8,
            Reg::E => regs.e = value as u8,
            Reg::H => regs.h = value as u8,
            Reg::L => regs.l = value as u8,
            Reg::Sp => regs.sp = value,
            Reg::Pc => regs.pc = value,
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

/// How this particular part is configured.
///
/// Construction properties, never `#[cfg]`: one build of rsemu has to be able to
/// run a Game Boy and a NES at once.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Whether a cold reset installs [`Regs::post_boot_dmg`] instead of zeroes.
    ///
    /// True by default, because rsemu ships no boot ROM: the DMG's is 256 bytes
    /// of Nintendo's copyrighted code and vendoring it is not ours to do
    /// (`ROADMAP.md` §1). A machine that maps a real boot ROM at `$0000` sets
    /// this false and lets the ROM produce the state itself.
    pub post_boot: bool,
    /// This core's identity in `MemAttrs::requester`, for a per-master filter.
    pub requester: RequesterId,
}

impl Config {
    /// The DMG's processor, entering at `$0100` as if a boot ROM had run.
    pub const DMG: Config = Config {
        post_boot: true,
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
        Config::DMG
    }
}

/// The interrupt registers and the input pins, kept outside the execution lock.
///
/// Deliberately atomics rather than fields under the mutex. A device raising an
/// interrupt from inside a write the CPU itself issued would otherwise re-enter
/// the CPU's own critical section, which is a deadlock under `native-std` and a
/// panic under `single`. The re-entrancy contract says mutate your own state in
/// a short critical section and act outward afterwards; a request that is one
/// atomic OR needs no critical section at all (`ROADMAP.md` §4.7).
#[derive(Debug, Default)]
pub(crate) struct Lines {
    /// `IF` (`$FF0F`): which interrupts are requested. Only the low five bits
    /// have storage.
    request: AtomicU8,
    /// `IE` (`$FFFF`): which are enabled. All eight bits are writable and
    /// readable, and the top three do nothing.
    enable: AtomicU8,
    /// Whether any joypad line is active — the condition `STOP` waits on.
    joypad_active: AtomicBool,
    /// A reset assertion nobody has acted on yet.
    reset_req: AtomicBool,
}

impl Lines {
    /// Which interrupts are both requested and enabled.
    fn pending(&self) -> u8 {
        self.request.load(Ordering::Acquire) & self.enable.load(Ordering::Acquire) & interrupt::MASK
    }

    /// Request interrupt `bit`. Idempotent: the flag is a level, not a queue.
    fn request(&self, bit: u8) {
        self.request.fetch_or(1 << bit, Ordering::AcqRel);
    }

    /// Clear one request, which is what dispatching it does.
    fn clear_request(&self, bit: u8) {
        self.request.fetch_and(!(1 << bit), Ordering::AcqRel);
    }

    /// `IF` as the guest reads it: the top three bits are not implemented and
    /// read as ones.
    fn read_if(&self) -> u8 {
        self.request.load(Ordering::Acquire) | !interrupt::MASK
    }

    fn write_if(&self, value: u8) {
        self.request
            .store(value & interrupt::MASK, Ordering::Release);
    }

    fn read_ie(&self) -> u8 {
        self.enable.load(Ordering::Acquire)
    }

    fn write_ie(&self, value: u8) {
        self.enable.store(value, Ordering::Release);
    }

    /// Whether `STOP` should end.
    fn stop_wake(&self) -> bool {
        self.joypad_active.load(Ordering::Acquire)
    }

    fn set_joypad_active(&self, active: bool) {
        self.joypad_active.store(active, Ordering::Release);
    }

    /// Latch a reset request. Idempotent: two pulses are one reset.
    fn request_reset(&self) {
        self.reset_req.store(true, Ordering::Release);
    }

    fn take_reset_request(&self) -> bool {
        self.reset_req.swap(false, Ordering::AcqRel)
    }

    fn snapshot(&self) -> (u8, u8, bool, bool) {
        (
            self.request.load(Ordering::Acquire),
            self.enable.load(Ordering::Acquire),
            self.joypad_active.load(Ordering::Acquire),
            self.reset_req.load(Ordering::Acquire),
        )
    }

    fn restore(&self, (request, enable, joypad, reset): (u8, u8, bool, bool)) {
        self.request.store(request, Ordering::Release);
        self.enable.store(enable, Ordering::Release);
        self.joypad_active.store(joypad, Ordering::Release);
        self.reset_req.store(reset, Ordering::Release);
    }
}

/// Everything the interpreter needs to mutate, behind one lock.
#[derive(Debug)]
struct Session {
    state: State,
    space: Option<Arc<AddressSpace>>,
}

/// A Sharp SM83 core.
///
/// # Locking
///
/// Execution state sits behind one [`sync::Mutex`] at [`LockRank::BUS`]. That
/// rank rather than `DEVICE`, because a CPU is a bus master: it holds this lock
/// while calling into device models, which take their own `DEVICE`-ranked locks,
/// which drive `WIRE`-ranked lines. The ladder runs in the direction calls
/// travel.
///
/// The interrupt registers are *not* under that lock — they are atomics in
/// `Lines` — so a device setting `IF` from inside a write the CPU itself
/// issued cannot re-enter the CPU's own critical section. This is also what lets
/// `IF` and `IE` be mapped regions of this very device: their `MemOps` take no
/// lock at all.
#[derive(Debug)]
pub struct Sm83 {
    cfg: Config,
    lines: Arc<Lines>,
    session: sync::Mutex<Session>,
    /// This core's identity in `MemAttrs::requester`.
    ///
    /// Separate from [`Config`] because the machine layer assigns it at bind
    /// time, long after construction, and every `Device` method takes `&self`.
    requester: AtomicU32,
    /// The `$FF0F` aperture, built once so that every `map` naming it gets the
    /// same region.
    if_region: RegionRef,
    /// The `$FFFF` aperture.
    ie_region: RegionRef,
    /// The wire sinks handed out by [`Device::sink`], kept alive here.
    ///
    /// A net holds only a *weak* reference to a sink (`core::device`), so
    /// something has to own the strong one and it has to be the device. These
    /// hold an `Arc<Lines>` rather than an `Arc<Sm83>` for exactly that reason.
    pins: sync::Mutex<Vec<Arc<InterruptPin>>>,
    reset_pin: sync::Mutex<Option<Arc<ResetPin>>>,
}

impl Sm83 {
    /// A core in its power-on state, with no address space yet.
    ///
    /// Two-phase construction (`ROADMAP.md` §4.4): nothing observable happens
    /// until [`attach_space`](Sm83::attach_space) and [`Device::realize`].
    #[must_use]
    pub fn new(cfg: Config) -> Sm83 {
        let lines = Arc::new(Lines::default());
        let if_region = Arc::new(MmioRegion::io(
            "sm83.if",
            1,
            Arc::new(RegPort {
                lines: Arc::clone(&lines),
                which: WhichReg::If,
            }) as Arc<dyn MemOps>,
        ));
        let ie_region = Arc::new(MmioRegion::io(
            "sm83.ie",
            1,
            Arc::new(RegPort {
                lines: Arc::clone(&lines),
                which: WhichReg::Ie,
            }) as Arc<dyn MemOps>,
        ));
        Sm83 {
            cfg,
            lines,
            session: sync::Mutex::with_rank(
                LockRank::BUS,
                Session {
                    state: State::new(),
                    space: None,
                },
            ),
            requester: AtomicU32::new(cfg.requester.0),
            if_region,
            ie_region,
            pins: sync::Mutex::new(Vec::new()),
            reset_pin: sync::Mutex::new(None),
        }
    }

    /// Build one from machine-description properties.
    ///
    /// # Errors
    ///
    /// If a property has the wrong type or a property nothing here accepts was
    /// given — a typo'd property that was silently ignored is an afternoon lost.
    pub fn from_props(props: &Props) -> Result<Sm83> {
        let mut r = props.reader();
        let post_boot = r.or("post-boot", true)?;
        // Accepted, and for now only one value is: `ROADMAP.md` §5's example
        // writes `engine = "interp"`, and the IR frontend is phase 5.
        let _ = r.or_enum("engine", "interp", &["interp"])?;
        r.finish()?;
        Ok(Sm83::new(Config {
            post_boot,
            requester: RequesterId::ANONYMOUS,
        }))
    }

    /// This core's configuration.
    #[must_use]
    pub fn config(&self) -> Config {
        self.cfg
    }

    /// The configuration as it stands, with the bind-time requester id folded
    /// in.
    fn effective_config(&self) -> Config {
        Config {
            requester: RequesterId(self.requester.load(Ordering::Relaxed)),
            ..self.cfg
        }
    }

    /// Set the id accesses this core initiates carry.
    pub fn set_requester(&self, id: RequesterId) {
        self.requester.store(id.0, Ordering::Relaxed);
    }

    /// Give the core the address space it executes from.
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
        self.session.lock().state.regs
    }

    /// Overwrite the register file — a debugger, a test vector, a snapshot.
    pub fn set_regs(&self, regs: Regs) {
        let mut session = self.session.lock();
        session.state.regs = regs;
        session.state.regs.f &= flags::STORED;
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

    /// Machine cycles executed since power-on. One M-cycle is four clocks.
    #[must_use]
    pub fn cycles(&self) -> u64 {
        self.session.lock().state.cycles
    }

    /// Whether the interrupt master enable is set.
    #[must_use]
    pub fn ime(&self) -> bool {
        self.session.lock().state.ime
    }

    /// Set the interrupt master enable directly, bypassing `EI`'s delay.
    pub fn set_ime(&self, on: bool) {
        let mut session = self.session.lock();
        session.state.ime = on;
        session.state.ei_pending = false;
    }

    /// Whether `HALT` has stopped the core.
    #[must_use]
    pub fn is_halted(&self) -> bool {
        self.session.lock().state.mode == Mode::Halted
    }

    /// Whether `STOP` has stopped the core.
    #[must_use]
    pub fn is_stopped(&self) -> bool {
        self.session.lock().state.mode == Mode::Stopped
    }

    /// Whether one of the eleven unimplemented opcodes has hung the core.
    ///
    /// A locked SM83 keeps its clock — the timer and the LCD carry on — but
    /// fetches nothing until reset.
    #[must_use]
    pub fn is_locked(&self) -> bool {
        self.session.lock().state.mode == Mode::Locked
    }

    /// `IF` as the guest reads it, top three bits set.
    #[must_use]
    pub fn interrupt_flags(&self) -> u8 {
        self.lines.read_if()
    }

    /// `IE`.
    #[must_use]
    pub fn interrupt_enable(&self) -> u8 {
        self.lines.read_ie()
    }

    /// Set `IE` directly, as a write to `$FFFF` would.
    pub fn set_interrupt_enable(&self, value: u8) {
        self.lines.write_ie(value);
    }

    /// Request an interrupt by bit number ([`interrupt`]), as the rising edge of
    /// the matching pin would.
    ///
    /// For a caller wiring a machine by hand; a described machine drives the
    /// pins instead.
    pub fn request_interrupt(&self, bit: u8) {
        if bit < 5 {
            self.lines.request(bit);
        }
    }

    /// Clear one `IF` bit.
    pub fn clear_interrupt(&self, bit: u8) {
        if bit < 5 {
            self.lines.clear_request(bit);
        }
    }

    /// Which interrupts are both requested and enabled.
    #[must_use]
    pub fn pending_interrupts(&self) -> u8 {
        self.lines.pending()
    }

    /// How many accesses the address space refused, and where the last one was.
    ///
    /// The SM83 has no bus-error input: a refused access reads as `$FF`, which
    /// is what the Game Boy's pulled-up data bus does. This counter is how that
    /// becomes visible rather than silent.
    #[must_use]
    pub fn bus_faults(&self) -> (u64, u16) {
        let s = self.session.lock();
        (s.state.faults, s.state.last_fault)
    }

    /// Execute one instruction, one interrupt dispatch, or one idle cycle.
    ///
    /// Returns the machine cycles charged: zero only when there is no address
    /// space to fetch from, which the caller must treat as "stop", not "retry".
    /// A halted core returns one, because a halted SM83's clock is still
    /// running and something has to advance the timer that will wake it.
    pub fn step(&self) -> u64 {
        let cfg = self.effective_config();
        let mut session = self.session.lock();
        if self.lines.take_reset_request() {
            session.state = State::new();
            if cfg.post_boot {
                session.state.regs = Regs::post_boot_dmg();
            }
        }
        let Session { state, space } = &mut *session;
        let Some(space) = space.clone() else {
            return 0;
        };
        Exec::new(state, &space, &cfg, &self.lines).step()
    }

    /// Execute until at least `budget` machine cycles have been charged.
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

    /// Run a scheduler budget of `ticks` machine cycles, reporting exactly how
    /// many were consumed — never more.
    ///
    /// An instruction cannot be stopped halfway, so a core handed a budget has
    /// to choose which side of the boundary to land on, and the choice is not
    /// cosmetic: the scheduler advances the clock forest by exactly what this
    /// reports, and every lazily advanced device is caught up to *that*. Where
    /// the processor stands relative to the number it reported is therefore the
    /// error a guest sees when it reads a timer.
    ///
    /// Two options, and this core takes the second:
    ///
    /// * **Decline to start** an instruction that would not fit — possible here,
    ///   unlike on a 6502, because [`MAX_INSTRUCTION_CYCLES`] bounds a step. It
    ///   never overruns, but it lets the core fall behind virtual time and then
    ///   catch up in bursts, and during a burst the devices are as stale as the
    ///   burst is long. Measured against Gekkio's acceptance suite this is
    ///   **worse**: it cost six tests.
    /// * **Consume the whole budget and carry the overshoot** as `State::debt`,
    ///   deducted from the next budget. The core and the forest then agree at
    ///   every quantum boundary, and the only disagreement inside a quantum is
    ///   the few cycles of the instruction in flight.
    ///
    /// So the overshoot is carried. The residual error — a device is stale by
    /// however far into the current quantum the access falls — is the
    /// intra-quantum staleness `ROADMAP.md` §4.2 records as outstanding, and it
    /// is what the ledger in `dev::gb::conformance` points at.
    pub fn run_budget(&self, ticks: u64) -> u64 {
        let owed = self.session.lock().state.debt;
        if owed >= ticks {
            // The last instruction was longer than this whole budget: charge the
            // budget against the debt and execute nothing.
            self.session.lock().state.debt = owed - ticks;
            return ticks;
        }
        let allowance = ticks - owed;
        let mut used = 0u64;
        while used < allowance {
            let n = self.step();
            if n == 0 {
                // No address space. Retrying would spin.
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

    /// Cycles owed to the next budget — see [`run_budget`](Sm83::run_budget).
    #[must_use]
    pub fn cycle_debt(&self) -> u64 {
        self.session.lock().state.debt
    }

    /// Disassemble `count` instructions starting at `pc`, reading guest memory
    /// with debug attributes.
    ///
    /// Debug attributes are the point: a monitor listing the code around `PC`
    /// must not advance the LCD's dot counter or pop a FIFO on the way
    /// (`ROADMAP.md` §15, invariant 5).
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

/// Which of the two one-byte registers a [`RegPort`] answers for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum WhichReg {
    If,
    Ie,
}

/// `IF` or `IE` as something an address space can dispatch to.
///
/// Takes no lock at all: both registers are atomics in [`Lines`], which is what
/// makes it safe for a device to set `IF` from inside a write the CPU itself is
/// in the middle of issuing.
#[derive(Debug)]
struct RegPort {
    lines: Arc<Lines>,
    which: WhichReg,
}

impl MemOps for RegPort {
    fn read(&self, _offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        *byte = match self.which {
            WhichReg::If => self.lines.read_if(),
            WhichReg::Ie => self.lines.read_ie(),
        };
        Ok(())
    }

    fn write(&self, _offset: u64, src: &[u8], _attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        // A debug write is allowed here, unlike a port with side effects: these
        // two registers are storage, and a debugger setting `IE` is exactly what
        // a debugger is for.
        match self.which {
            WhichReg::If => self.lines.write_if(*value),
            WhichReg::Ie => self.lines.write_ie(*value),
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO.with_widths(Width::U8, Width::U8)
    }
}

/// The `cpu.sm83` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: "cpu.sm83",
    version: 1,
    summary: "Sharp SM83 8-bit CPU core (Game Boy), M-cycle-accurate interpreter",
    properties: &[
        PropertySpec {
            name: "post-boot",
            kind: ValueKind::Bool,
            required: false,
            summary: "start at $0100 with the register file a DMG boot ROM would leave behind",
        },
        PropertySpec {
            name: "engine",
            kind: ValueKind::Str,
            required: false,
            summary: "which execution engine; only `interp` exists until phase 5",
        },
    ],
    construct: |props| Ok(Box::new(Sm83::from_props(props)?)),
};

/// Add this core's class to a registry.
///
/// Registration is explicit per feature rather than link-time magic
/// (`ROADMAP.md` §4.4).
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut Registry) -> Result<()> {
    reg.add(&CLASS)
}

impl Device for Sm83 {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward. A CPU with no address space cannot fetch, but realize
        // runs *before* the machine binds one — that check belongs to
        // `Instance::bind`, which is where the space arrives.
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            IF_REGION => Some(Arc::clone(&self.if_region)),
            IE_REGION => Some(Arc::clone(&self.ie_region)),
            // Deliberately not the empty name: this device publishes two
            // apertures and a `map … = cpu` that silently picked one of them
            // would be a coin toss.
            _ => None,
        }
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        if port == "reset" {
            let pin = Arc::new(ResetPin::new(Arc::clone(&self.lines), sources));
            *self.reset_pin.lock() = Some(Arc::clone(&pin));
            return Some(SinkPin { sink: pin, line: 0 });
        }
        let bit = interrupt::PINS.iter().position(|p| *p == port)? as u8;
        let pin = Arc::new(InterruptPin::new(Arc::clone(&self.lines), bit, sources));
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line: 0 })
    }

    fn is_runnable(&self) -> bool {
        true
    }

    fn run(&self, budget: Budget) -> Consumed {
        Consumed::new(self.run_budget(budget.ticks))
    }

    fn reset(&self, kind: ResetKind) {
        let mut session = self.session.lock();
        session.state = State::new();
        if self.cfg.post_boot {
            session.state.regs = Regs::post_boot_dmg();
        }
        drop(session);
        if kind == ResetKind::Cold {
            self.lines.restore((0, 0, false, false));
        } else {
            // The input *levels* belong to whatever drives them; only the
            // latched requests are ours to clear.
            self.lines.write_if(0);
            self.lines.write_ie(0);
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.session.lock().state;
        w.write_u8(state.regs.a)?;
        w.write_u8(state.regs.f)?;
        w.write_u8(state.regs.b)?;
        w.write_u8(state.regs.c)?;
        w.write_u8(state.regs.d)?;
        w.write_u8(state.regs.e)?;
        w.write_u8(state.regs.h)?;
        w.write_u8(state.regs.l)?;
        w.write_u16(state.regs.sp)?;
        w.write_u16(state.regs.pc)?;
        w.write_u64(state.cycles)?;
        w.write_bool(state.ime)?;
        w.write_bool(state.ei_pending)?;
        w.write_u8(state.mode.tag())?;
        w.write_bool(state.halt_bug)?;
        w.write_u64(state.debt)?;
        w.write_u64(state.faults)?;
        w.write_u16(state.last_fault)?;
        let (request, enable, joypad, reset) = self.lines.snapshot();
        w.write_u8(request)?;
        w.write_u8(enable)?;
        w.write_bool(joypad)?;
        w.write_bool(reset)?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::new();
        state.regs.a = r.read_u8()?;
        state.regs.f = r.read_u8()? & flags::STORED;
        state.regs.b = r.read_u8()?;
        state.regs.c = r.read_u8()?;
        state.regs.d = r.read_u8()?;
        state.regs.e = r.read_u8()?;
        state.regs.h = r.read_u8()?;
        state.regs.l = r.read_u8()?;
        state.regs.sp = r.read_u16()?;
        state.regs.pc = r.read_u16()?;
        state.cycles = r.read_u64()?;
        state.ime = r.read_bool()?;
        state.ei_pending = r.read_bool()?;
        let tag = r.read_u8()?;
        state.mode = Mode::from_tag(tag)
            .ok_or_else(|| Error::State(alloc::format!("unknown SM83 run mode tag {tag}")))?;
        state.halt_bug = r.read_bool()?;
        state.debt = r.read_u64()?;
        state.faults = r.read_u64()?;
        state.last_fault = r.read_u16()?;
        let request = r.read_u8()?;
        let enable = r.read_u8()?;
        let joypad = r.read_bool()?;
        let reset = r.read_bool()?;
        self.session.lock().state = state;
        self.lines.restore((request, enable, joypad, reset));
        Ok(())
    }
}

impl Initiator for Sm83 {
    fn requester(&self) -> RequesterId {
        RequesterId(self.requester.load(Ordering::Relaxed))
    }
}

/// The machine layer's half: an SM83 needs an address space, and this is where
/// the machine gives it one.
impl crate::machine::Instance for Sm83 {
    fn bind(&self, ctx: &crate::machine::BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: String::from(ctx.path()),
            message: String::from(
                "an SM83 needs an address space to fetch from (`space = cpubus`)",
            ),
        })?;
        self.attach_space(Arc::clone(space));
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
    bindings.bind(CLASS.name, |props| Ok(Arc::new(Sm83::from_props(props)?)))
}

/// What the validator should know about `cpu.sm83`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    let mut schema = ClassSchema::new(CLASS.name)
        .prop(PropSchema::new("post-boot", ValueKind::Bool))
        .prop(PropSchema::new("engine", ValueKind::Str).values(&["interp"]))
        .port("reset", PortDir::In)
        .region(IF_REGION)
        .region(IE_REGION);
    for pin in interrupt::PINS {
        schema = schema.port(pin, PortDir::In);
    }
    schema
}

/// One of the CPU's five interrupt inputs, as something a [`Wire`] can drive.
///
/// **Edge-triggered on the way up**, which is not a simplification: the `IF` bit
/// is set by the rising edge of the request line and stays set until the
/// dispatch clears it or the program writes `$FF0F`. That is what makes the
/// LCD's STAT line — a level, ORed from up to four enabled conditions — behave
/// the way hardware does: a second condition becoming true while the first still
/// holds raises no second interrupt.
///
/// A wire hands each sink the level of the *driver that changed*, not the
/// resolved level of the net, so this keeps a [`FanIn`] and wire-ORs them.
///
/// [`Wire`]: crate::core::wire::Wire
#[derive(Debug)]
pub struct InterruptPin {
    lines: Arc<Lines>,
    bit: u8,
    inputs: FanIn,
    resolve: Resolve,
    /// The last resolved level, for edge detection. An atomic rather than a
    /// field under a lock: a sink is driven from inside whatever device changed
    /// the line, which may be several frames below a CPU access.
    last: AtomicBool,
}

impl InterruptPin {
    /// Connect interrupt `bit` to a net driven by `sources`.
    #[must_use]
    pub fn for_cpu(cpu: &Sm83, bit: u8, sources: &[WireId]) -> InterruptPin {
        InterruptPin::new(Arc::clone(&cpu.lines), bit, sources)
    }

    fn new(lines: Arc<Lines>, bit: u8, sources: &[WireId]) -> InterruptPin {
        InterruptPin {
            lines,
            bit,
            inputs: FanIn::new(sources),
            resolve: Resolve::Or,
            last: AtomicBool::new(false),
        }
    }

    /// Which interrupt this pin requests. See [`interrupt`].
    #[must_use]
    pub fn bit(&self) -> u8 {
        self.bit
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
        let now = self.inputs.resolve(self.resolve).is_high();
        let was = self.last.swap(now, Ordering::AcqRel);
        if now && !was {
            self.lines.request(self.bit);
        }
        // The joypad line doubles as the condition `STOP` waits on, and that one
        // is a level rather than an edge.
        if self.bit == interrupt::JOYPAD {
            self.lines.set_joypad_active(now);
        }
    }
}

/// The core's reset input, as something a [`Wire`] can drive.
///
/// [`Wire`]: crate::core::wire::Wire
#[derive(Debug)]
pub struct ResetPin {
    lines: Arc<Lines>,
    inputs: FanIn,
    resolve: Resolve,
}

impl ResetPin {
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
        if self.inputs.resolve(self.resolve).is_high() {
            self.lines.request_reset();
        }
    }
}

/// A description of this core for `rsemu describe cpu.sm83`.
///
/// Built from the instruction tables, so it cannot drift from what the
/// interpreter implements.
#[must_use]
pub fn describe_isa() -> String {
    disasm::describe_isa()
}
