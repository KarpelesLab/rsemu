//! The Sony SPC700 — the SNES audio processor's CPU — as a cycle-accurate
//! interpreter.
//!
//! An 8-bit core with a 16-bit address space, three registers (`A`, `X`,
//! `Y`), an 8-bit stack pointer fixed to page 1, and a direct page that the
//! `P` flag moves between `$00xx` and `$01xx`. It resembles a 6502 in spirit
//! and in nothing that matters to an implementation: the encodings, the flag
//! rules and the bus sequences are all its own. All 256 opcodes are
//! implemented — `MUL`/`DIV`, `DAA`/`DAS`, `TCALL`/`PCALL`, the one-bit
//! instructions, `SLEEP` and `STOP`.
//!
//! # Two ways to use it
//!
//! * **[`Spc700`]** is the `cpu.spc700` device: it executes from an
//!   [`AddressSpace`] like every other core, for a machine file or a test.
//! * **[`Core`]** is the bare architectural state with
//!   [`Core::step`] generic over a [`Bus`]. A chip that *contains* an SPC700
//!   — the S-SMP, `dev::snes::apu` — owns one and steps it over its own
//!   memory decoder, which is what lets that chip clock its timers and its
//!   DSP on every processor cycle without an address space in the hot path.
//!
//! Both run the same interpreter; there is one, and `SingleStepTests/spc700`
//! measures it.
//!
//! # Cycle accounting
//!
//! Every cycle is one [`Bus`] call — a read, a write, or an internal cycle —
//! including the dummy reads the traces record (`exec`). The clock domain this
//! core runs in counts those cycles: 1.024 MHz on a SNES, the APU's 24.576 MHz
//! crystal divided by 24.
//!
//! # Modules
//!
//! | Module | Holds |
//! | --- | --- |
//! | [`isa`] | the one declarative instruction description |
//! | [`disasm`] | the disassembler generated from it |
//! | `exec` (private, re-exported as [`Core`], [`Bus`], [`Mode`]) | the interpreter |
//!
//! # Sources
//!
//! The SNESdev wiki (*SPC-700 instruction set*, *S-SMP*), nocash's
//! *fullsnes* (the SNES APU chapters), anomie's *SPC700* and *APU* documents,
//! and `SingleStepTests/spc700` (MIT) as the oracle for bus traces. No
//! emulator source of any licence was consulted (`CLAUDE.md`, Provenance).
//! `docs/cpu/spc700.md` has the long form.

pub mod disasm;
mod exec;
pub mod isa;

#[cfg(test)]
mod tests;

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, Initiator, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::registry::Registry;
use crate::core::sched::{Budget, Consumed, ExitFlag, TickCursor};
use crate::core::space::{AddressSpace, MemAttrs, RequesterId};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{self, AtomicU32, LockRank, Ordering};
use crate::core::value::Width;

pub use exec::{Bus, Core, Mode};

/// The eight bits of `PSW`.
pub mod flags {
    /// Negative.
    pub const N: u8 = 0x80;
    /// Overflow.
    pub const V: u8 = 0x40;
    /// Direct page select: `$01xx` when set, `$00xx` when clear.
    pub const P: u8 = 0x20;
    /// Break: set by `BRK`.
    pub const B: u8 = 0x10;
    /// Half-carry, out of bit 3 (bit 11 for the word instructions).
    pub const H: u8 = 0x08;
    /// Interrupt enable. The SNES wires no interrupt to the SPC700, so this is
    /// storage and nothing more.
    pub const I: u8 = 0x04;
    /// Zero.
    pub const Z: u8 = 0x02;
    /// Carry.
    pub const C: u8 = 0x01;
}

/// The most cycles one step can take: `DIV YA,X`, twelve.
pub const MAX_INSTRUCTION_CYCLES: u64 = 12;

/// The architectural register file.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Regs {
    /// Accumulator.
    pub a: u8,
    /// `X`.
    pub x: u8,
    /// `Y`.
    pub y: u8,
    /// Stack pointer; the stack is `$0100 | sp`.
    pub sp: u8,
    /// Program status word. See [`flags`].
    pub psw: u8,
    /// Program counter.
    pub pc: u16,
}

impl Regs {
    /// All zero.
    #[must_use]
    pub const fn new() -> Regs {
        Regs {
            a: 0,
            x: 0,
            y: 0,
            sp: 0,
            psw: 0,
            pc: 0,
        }
    }

    /// `YA`, the 16-bit pair the word instructions use.
    #[must_use]
    pub const fn ya(&self) -> u16 {
        ((self.y as u16) << 8) | self.a as u16
    }
}

impl fmt::Display for Regs {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "A:{:02x} X:{:02x} Y:{:02x} SP:{:02x} PSW:{:02x} PC:{:04x}",
            self.a, self.x, self.y, self.sp, self.psw, self.pc
        )
    }
}

/// One named register, for a debugger.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Reg {
    /// Accumulator.
    A,
    /// `X`.
    X,
    /// `Y`.
    Y,
    /// Stack pointer.
    Sp,
    /// Program status word.
    Psw,
    /// Program counter.
    Pc,
}

impl Reg {
    /// Every register, in the order a debugger lists them.
    pub const ALL: &'static [Reg] = &[Reg::A, Reg::X, Reg::Y, Reg::Sp, Reg::Psw, Reg::Pc];

    /// Lowercase name.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Reg::A => "a",
            Reg::X => "x",
            Reg::Y => "y",
            Reg::Sp => "sp",
            Reg::Psw => "psw",
            Reg::Pc => "pc",
        }
    }

    /// Width.
    #[must_use]
    pub const fn width(self) -> Width {
        match self {
            Reg::Pc => Width::U16,
            _ => Width::U8,
        }
    }

    /// Read this register out of a register file.
    #[must_use]
    pub const fn get(self, r: &Regs) -> u16 {
        match self {
            Reg::A => r.a as u16,
            Reg::X => r.x as u16,
            Reg::Y => r.y as u16,
            Reg::Sp => r.sp as u16,
            Reg::Psw => r.psw as u16,
            Reg::Pc => r.pc,
        }
    }

    /// Write this register, truncating to its width.
    pub const fn set(self, r: &mut Regs, v: u16) {
        match self {
            Reg::A => r.a = v as u8,
            Reg::X => r.x = v as u8,
            Reg::Y => r.y = v as u8,
            Reg::Sp => r.sp = v as u8,
            Reg::Psw => r.psw = v as u8,
            Reg::Pc => r.pc = v,
        }
    }

    /// Look a register up by name.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Reg> {
        Reg::ALL.iter().copied().find(|r| r.name() == name)
    }
}

/// Write the core's state to a snapshot chunk. Shared with `dev::snes::apu`,
/// which embeds a [`Core`].
///
/// # Errors
///
/// If the sink refuses a write.
pub fn save_core(core: &Core, w: &mut ChunkWriter<'_>) -> Result<()> {
    w.write_u8(core.regs.a)?;
    w.write_u8(core.regs.x)?;
    w.write_u8(core.regs.y)?;
    w.write_u8(core.regs.sp)?;
    w.write_u8(core.regs.psw)?;
    w.write_u16(core.regs.pc)?;
    w.write_u8(core.mode.tag())?;
    w.write_u64(core.cycles)?;
    Ok(())
}

/// Read what [`save_core`] wrote.
///
/// # Errors
///
/// A truncated chunk or an unknown run-mode tag.
pub fn load_core(r: &mut ChunkReader<'_>) -> Result<Core> {
    let mut core = Core::new();
    core.regs.a = r.read_u8()?;
    core.regs.x = r.read_u8()?;
    core.regs.y = r.read_u8()?;
    core.regs.sp = r.read_u8()?;
    core.regs.psw = r.read_u8()?;
    core.regs.pc = r.read_u16()?;
    let tag = r.read_u8()?;
    core.mode = Mode::from_tag(tag)
        .ok_or_else(|| Error::State(alloc::format!("unknown SPC700 run mode tag {tag}")))?;
    core.cycles = r.read_u64()?;
    Ok(core)
}

/// An [`AddressSpace`] as a [`Bus`]: what the standalone device executes on.
struct SpaceBus<'a> {
    space: &'a AddressSpace,
    attrs: MemAttrs,
    faults: &'a mut u64,
    cursor: Option<&'a TickCursor>,
    cycles: u64,
}

impl SpaceBus<'_> {
    fn tick(&mut self) {
        self.cycles = self.cycles.wrapping_add(1);
        if let Some(c) = self.cursor {
            c.set(self.cycles);
        }
    }
}

impl Bus for SpaceBus<'_> {
    fn read(&mut self, addr: u16) -> u8 {
        self.tick();
        match self.space.read(u64::from(addr), Width::U8, self.attrs) {
            Ok(v) => v as u8,
            Err(_) => {
                // The SPC700 has no bus-error input; the counter is how a
                // refused access becomes visible rather than silent.
                *self.faults = self.faults.wrapping_add(1);
                0xff
            }
        }
    }

    fn write(&mut self, addr: u16, value: u8) {
        self.tick();
        if self
            .space
            .write(u64::from(addr), Width::U8, u64::from(value), self.attrs)
            .is_err()
        {
            *self.faults = self.faults.wrapping_add(1);
        }
    }

    fn idle(&mut self) {
        self.tick();
    }
}

#[derive(Debug)]
struct Session {
    core: Core,
    space: Option<Arc<AddressSpace>>,
    cursor: Option<TickCursor>,
    /// Cycles already executed past the last budget (see the SM83's
    /// `run_budget` for why the overshoot is carried rather than refused).
    debt: u64,
    faults: u64,
}

/// The `cpu.spc700` device: an SPC700 executing from an [`AddressSpace`].
#[derive(Debug)]
pub struct Spc700 {
    session: sync::Mutex<Session>,
    requester: AtomicU32,
}

impl Default for Spc700 {
    fn default() -> Self {
        Spc700::new()
    }
}

impl Spc700 {
    /// A core at power-on, with no address space. The first step runs the
    /// reset sequence.
    #[must_use]
    pub fn new() -> Spc700 {
        Spc700 {
            session: sync::Mutex::with_rank(
                LockRank::BUS,
                Session {
                    core: Core::new(),
                    space: None,
                    cursor: None,
                    debt: 0,
                    faults: 0,
                },
            ),
            requester: AtomicU32::new(RequesterId::ANONYMOUS.0),
        }
    }

    /// Build one from machine-description properties.
    ///
    /// # Errors
    ///
    /// An unknown property.
    pub fn from_props(props: &Props) -> Result<Spc700> {
        let mut r = props.reader();
        let _ = r.or_enum("engine", "interp", &["interp"])?;
        r.finish()?;
        Ok(Spc700::new())
    }

    /// Give the core the address space it executes from.
    pub fn attach_space(&self, space: Arc<AddressSpace>) {
        self.session.lock().space = Some(space);
    }

    /// The address space, if one is attached.
    #[must_use]
    pub fn space(&self) -> Option<Arc<AddressSpace>> {
        self.session.lock().space.clone()
    }

    /// Publish the cycle counter here as the core runs.
    pub fn attach_cursor(&self, cursor: TickCursor) {
        self.session.lock().cursor = Some(cursor);
    }

    /// Set the id accesses this core initiates carry.
    pub fn set_requester(&self, id: RequesterId) {
        self.requester.store(id.0, Ordering::Relaxed);
    }

    /// The register file.
    #[must_use]
    pub fn regs(&self) -> Regs {
        self.session.lock().core.regs
    }

    /// Overwrite the register file. Also marks the reset sequence as done,
    /// because a caller setting `PC` means to execute from it.
    pub fn set_regs(&self, regs: Regs) {
        let mut s = self.session.lock();
        s.core.regs = regs;
        if s.core.mode == Mode::Reset {
            s.core.mode = Mode::Running;
        }
    }

    /// The run state.
    #[must_use]
    pub fn mode(&self) -> Mode {
        self.session.lock().core.mode
    }

    /// Set the run state — a test vector, a debugger.
    pub fn set_mode(&self, mode: Mode) {
        self.session.lock().core.mode = mode;
    }

    /// Cycles executed since power-on.
    #[must_use]
    pub fn cycles(&self) -> u64 {
        self.session.lock().core.cycles
    }

    /// How many accesses the address space refused.
    #[must_use]
    pub fn bus_faults(&self) -> u64 {
        self.session.lock().faults
    }

    /// Execute one instruction and return its cycles; zero with no space.
    pub fn step(&self) -> u64 {
        let attrs =
            MemAttrs::DEFAULT.with_requester(RequesterId(self.requester.load(Ordering::Relaxed)));
        let mut s = self.session.lock();
        let Session {
            core,
            space,
            cursor,
            faults,
            ..
        } = &mut *s;
        let Some(space) = space.clone() else {
            return 0;
        };
        let mut bus = SpaceBus {
            space: &space,
            attrs,
            faults,
            cursor: cursor.as_ref(),
            cycles: core.cycles,
        };
        core.step(&mut bus)
    }

    /// Run a scheduler budget, carrying any overshoot to the next one.
    pub fn run_budget(&self, ticks: u64) -> u64 {
        let owed = self.session.lock().debt;
        if owed >= ticks {
            self.session.lock().debt = owed - ticks;
            return ticks;
        }
        let exit = self
            .session
            .lock()
            .cursor
            .as_ref()
            .map(TickCursor::exit_flag);
        let allowance = ticks - owed;
        let mut used = 0u64;
        while used < allowance {
            let n = self.step();
            if n == 0 {
                break;
            }
            used += n;
            if exit.as_ref().is_some_and(ExitFlag::raised) {
                break;
            }
        }
        if used >= allowance {
            self.session.lock().debt = used - allowance;
            ticks
        } else {
            self.session.lock().debt = 0;
            owed + used
        }
    }

    /// Disassemble `count` instructions at `pc` with side-effect-free reads.
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

/// The `cpu.spc700` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: "cpu.spc700",
    version: 1,
    summary: "Sony SPC700 8-bit CPU core (SNES audio), cycle-accurate interpreter",
    properties: &[PropertySpec {
        name: "engine",
        kind: ValueKind::Str,
        required: false,
        summary: "which execution engine; only `interp` exists",
    }],
    construct: |props| Ok(Box::new(Spc700::from_props(props)?)),
};

/// Add this core's class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut Registry) -> Result<()> {
    reg.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS.name, |props| Ok(Arc::new(Spc700::from_props(props)?)))
}

/// What the validator should know about `cpu.spc700`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    ClassSchema::new(CLASS.name).prop(PropSchema::new("engine", ValueKind::Str).values(&["interp"]))
}

impl Device for Spc700 {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn is_runnable(&self) -> bool {
        true
    }

    fn attach_cursor(&self, cursor: TickCursor) {
        Spc700::attach_cursor(self, cursor);
    }

    fn run(&self, budget: Budget) -> Consumed {
        Consumed::new(self.run_budget(budget.ticks))
    }

    fn reset(&self, _kind: ResetKind) {
        let mut s = self.session.lock();
        s.core = Core::new();
        s.debt = 0;
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let s = self.session.lock();
        save_core(&s.core, w)?;
        w.write_u64(s.debt)?;
        w.write_u64(s.faults)?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let core = load_core(r)?;
        let debt = r.read_u64()?;
        let faults = r.read_u64()?;
        let mut s = self.session.lock();
        s.core = core;
        s.debt = debt;
        s.faults = faults;
        Ok(())
    }
}

impl Initiator for Spc700 {
    fn requester(&self) -> RequesterId {
        RequesterId(self.requester.load(Ordering::Relaxed))
    }
}

impl crate::machine::Instance for Spc700 {
    fn bind(&self, ctx: &crate::machine::BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: String::from(ctx.path()),
            message: String::from("an SPC700 needs an address space to fetch from (`space = ...`)"),
        })?;
        self.attach_space(Arc::clone(space));
        self.set_requester(ctx.requester());
        Ok(())
    }
}

/// A description of this core for `rsemu describe cpu.spc700`.
#[must_use]
pub fn describe_isa() -> String {
    disasm::describe_isa()
}
