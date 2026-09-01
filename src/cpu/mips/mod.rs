//! MIPS — an LSI/MIPS R3000A-compatible 32-bit interpreter.
//!
//! MIPS I integer, R3000-style coprocessor 0 with the full exception model,
//! the 64-entry TLB, the fixed `kuseg`/`kseg0`/`kseg1`/`kseg2` map, `HI`/`LO`
//! with multiply and divide, and the disassembler generated from the same
//! instruction description the interpreter decodes with. This is the
//! interpreter half of the core, which means it is the **oracle** everything
//! later is measured against (CLAUDE.md, "CPU cores").
//!
//! # What is here
//!
//! | Piece | Module |
//! | --- | --- |
//! | the one declarative instruction table, decode, and the unaligned-transfer tables | [`isa`] |
//! | the disassembler generated from that table | [`disasm`] |
//! | the interpreter | `exec` |
//! | coprocessor 0, the exception model, the segments and the TLB | [`cp0`] |
//! | a 32-bit ELF reader, for loading test and level-3 programs | [`elf`] |
//!
//! # What is deliberately **not** here
//!
//! Said plainly rather than half-built, because a stub is worse than an
//! absence — a guest that probes for a feature and finds a broken one has no
//! way to recover, while one that finds nothing takes the other branch:
//!
//! * **The GTE (coprocessor 2)**, the geometry engine on the LR33300. A `COP2`
//!   instruction raises coprocessor-unusable with `Cause.CE = 2`, which is
//!   exactly how a guest discovers there is none.
//! * **Coprocessor 1 / the R3010 floating-point accelerator.** Same treatment,
//!   with `CE = 1`. When it lands it rides on `ROADMAP.md` §9.1's soft-float
//!   rather than on a host FPU.
//! * **The LR33300 scratchpad** (the "fast RAM" a PlayStation maps at
//!   `0x1F80_0000`). That is a board's memory map, not a core's, so it belongs
//!   to a machine file and to whatever board wants it.
//! * **Any PlayStation board.** This core plus a GTE plus that board is a
//!   second pass.
//!
//! # The family is a lattice, not a version number
//!
//! "R3000A compatible" names a family whose members differ in ways that change
//! what an instruction *is*, so — following `ROADMAP.md` §6.1.1, and the way
//! `cpu::riscv` already does it — the differences are a [`Config`] chosen at
//! construction from a `.machine` file, never a `#[cfg]` and never an ordered
//! version comparison. The axes that are load-bearing today:
//!
//! * **A TLB, or no TLB at all.** The LSI **LR33300** has none: it has the
//!   fixed segment map and nothing else, so `kuseg` and `kseg2` are the
//!   identity and `TLBWI` is a reserved instruction rather than a slow no-op.
//!   That is not "R3000 minus a feature" — it changes what a memory access is.
//! * **Byte order**, which is a pin on the package and changes what `LWL`,
//!   `LWR`, `SWL` and `SWR` mean.
//! * **The load interlock**, which MIPS I does not have and MIPS II does.
//!
//! Decode is gated **per table entry** against the configuration
//! ([`isa::Req`]), and an instruction the configured part lacks traps rather
//! than executing — because "we decoded it anyway" is a conformance failure
//! and is also how real guests probe for features.
//!
//! ```
//! use rsemu::cpu::mips::{Arch, Config};
//!
//! // The PlayStation's part has no TLB and no coprocessor 1.
//! assert!(!Config::new(Arch::LR33300).arch.tlb);
//! assert!(Config::new(Arch::R3000A).arch.tlb);
//! ```
//!
//! # Assembling one
//!
//! ```
//! use std::sync::Arc;
//! use rsemu::core::space::{AddressSpace, RamStore, Region};
//! use rsemu::cpu::mips::{Arch, Config, Cpu};
//!
//! // 64 KiB of RAM at physical zero, holding `addiu $v0, $zero, 42`.
//! let ram = Arc::new(RamStore::new(0x1_0000));
//! for (i, b) in 0x2402_002au32.to_le_bytes().iter().enumerate() {
//!     ram.write_u8(i as u64, *b).unwrap();
//! }
//!
//! let space = AddressSpace::new("mem", 32);
//! space.topology().map(Region::ram("ram", ram), 0).unwrap();
//!
//! // Start in kseg1, which is uncached and maps straight down to physical 0.
//! let cpu = Cpu::new(Config::new(Arch::R3000A).with_reset_vector(0xa000_0000));
//! cpu.attach_space(Arc::new(space));
//! cpu.step();
//! assert_eq!(cpu.reg(2), 42);
//! ```
//!
//! # Timing
//!
//! MIPS does not architecturally define instruction timing — that is a
//! property of a particular pipeline and cache — so there is no cycle table
//! anywhere in this core. A cycle is charged *because a bus access happened*:
//! an instruction fetch, a load, a store, an isolated-cache access. That is
//! the accounting `ROADMAP.md` §6 asks for and the only kind that is a fact
//! rather than an invention.
//!
//! # Sources
//!
//! * *IDT R3051/R3052/R3081 Family Hardware User's Manual* — the CP0 register
//!   set, the TLB, the exception model and the vectors.
//! * Gerry Kane and Joe Heinrich, *MIPS RISC Architecture* — the MIPS I
//!   instruction semantics, the encoding tables, and the `LWL`/`LWR` byte
//!   tables.
//! * The LSI Logic LR33300/LR33310 datasheet, for the part with no TLB.
//! * The published PlayStation memory map, for the identity mapping of
//!   `kuseg` on that part — RAM answers at `0x0000_0000`, `0x8000_0000` and
//!   `0xA000_0000` alike, which is only true if `kuseg` is unmapped and
//!   direct.
//!
//! *MIPS32 Architecture for Programmers* was **not** used for coprocessor 0:
//! it documents the R4000-style CP0, which has `Status.EXL`, a `Wired`
//! register and paired `EntryLo`s, none of which an R3000 has. Where an
//! R3000-era source and a MIPS32 one disagree, this core follows the former
//! and says so at the point it matters. No emulator source of any licence was
//! opened for any part of this core.

pub mod cp0;
pub mod disasm;
pub mod elf;
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
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::{
    Device, DeviceClass, Initiator, PropertySpec, RealizeCtx, ResetKind, SinkPin,
};
use crate::core::error::{Error, Result};
use crate::core::exec::{Exit, ExitMask, ExitingCore, Run};
use crate::core::props::{Props, ValueKind};
use crate::core::registry::Registry;
use crate::core::sched::{Budget, Consumed};
use crate::core::space::{AddressSpace, MemAttrs, RequesterId};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{self, AtomicU32, LockRank, Ordering};
use crate::core::value::Width;
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink};

use cp0::{Cp0, Lines, TLB_ENTRIES, Tlb, TlbEntry};
use exec::{Exec, State};
use isa::Endian;

pub use isa::REG_NAMES;

/// The page size the TLB maps. Fixed on an R3000: there is no page-mask
/// register, which is another R4000 addition.
pub const PAGE_SIZE: u32 = 4096;

/// Which part of the family a core is.
///
/// The `ROADMAP.md` §6.1.1 lattice, in the smallest form this architecture
/// needs: independently selectable capabilities with **named presets carrying
/// the part numbers**, so nobody has to assemble one by hand to get a real
/// chip and a machine file can simply name one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Arch {
    /// The part number, for `rsemu describe` and for an error message.
    pub part: &'static str,
    /// Whether the part has a translation lookaside buffer.
    ///
    /// With no TLB, `kuseg` and `kseg2` are the identity and the four TLB
    /// instructions are reserved. That is the LR33300, and it is why this is a
    /// runtime property rather than a build flag.
    pub tlb: bool,
    /// Whether a load's result is visible to the immediately following
    /// instruction.
    ///
    /// **False on MIPS I**, which is the whole family this core covers; MIPS
    /// II and later interlock. A field rather than a constant because it is
    /// the kind of thing a later part changes and nothing else about the core
    /// does.
    pub load_interlock: bool,
    /// Whether coprocessor 1 is present *and implemented here*. Always false:
    /// see the module docs.
    pub cop1: bool,
    /// Whether coprocessor 2 is present *and implemented here*. Always false;
    /// the GTE is out of scope for this pass.
    pub cop2: bool,
    /// Whether coprocessor 3 is present. Always false.
    pub cop3: bool,
    /// How many bytes of data-cache data array `Status.IsC` exposes.
    pub dcache_bytes: u32,
    /// How many bytes of instruction-cache data array `Status.IsC` with
    /// `Status.SwC` exposes.
    pub icache_bytes: u32,
}

impl Arch {
    /// The MIPS R3000A: a TLB, no load interlock, 4 KiB of each cache.
    ///
    /// The generic part, and the right default for a synthetic board.
    pub const R3000A: Arch = Arch {
        part: "r3000a",
        tlb: true,
        load_interlock: false,
        cop1: false,
        cop2: false,
        cop3: false,
        dcache_bytes: 4096,
        icache_bytes: 4096,
    };

    /// The IDT R3051: an R3000A core with 4 KiB of instruction cache and
    /// 2 KiB of data cache on the die.
    pub const IDT_R3051: Arch = Arch {
        part: "r3051",
        dcache_bytes: 2048,
        icache_bytes: 4096,
        ..Arch::R3000A
    };

    /// The LSI Logic LR33300 — the PlayStation's processor.
    ///
    /// **No TLB at all.** 1 KiB of data cache and 4 KiB of instruction cache.
    /// The real part also carries a GTE on coprocessor 2 and a scratchpad;
    /// neither is modelled here (see the module docs), so a `COP2` instruction
    /// raises coprocessor-unusable and a guest that probes finds nothing —
    /// which is the honest answer rather than a broken one.
    pub const LR33300: Arch = Arch {
        part: "lr33300",
        tlb: false,
        dcache_bytes: 1024,
        icache_bytes: 4096,
        ..Arch::R3000A
    };

    /// Every preset this build ships, in catalog order.
    pub const ALL: &'static [Arch] = &[Arch::R3000A, Arch::IDT_R3051, Arch::LR33300];

    /// Look a preset up by the name a `.machine` file writes.
    #[must_use]
    pub fn by_name(name: &str) -> Option<Arch> {
        Arch::ALL.iter().copied().find(|a| a.part == name)
    }

    /// Whether coprocessor `n` is present.
    #[must_use]
    pub const fn coprocessor(self, n: u32) -> bool {
        match n {
            0 => true,
            1 => self.cop1,
            2 => self.cop2,
            3 => self.cop3,
            _ => false,
        }
    }

    /// A mask of the `Status` bits this part will accept a write to.
    ///
    /// Everything outside the four `CU` bits passes through; a `CU` bit for a
    /// coprocessor the part does not have reads back as zero, so a guest that
    /// sets it and reads it back learns the truth instead of being told it has
    /// a GTE and then trapping on the first instruction.
    #[must_use]
    pub const fn coprocessor_mask(self) -> u32 {
        let mut mask = !(0xf << cp0::status::CU_SHIFT);
        mask |= 1 << cp0::status::CU_SHIFT;
        if self.cop1 {
            mask |= 1 << (cp0::status::CU_SHIFT + 1);
        }
        if self.cop2 {
            mask |= 1 << (cp0::status::CU_SHIFT + 2);
        }
        if self.cop3 {
            mask |= 1 << (cp0::status::CU_SHIFT + 3);
        }
        mask
    }
}

/// How a particular processor is configured.
///
/// Construction properties, never `#[cfg]`: one build of rsemu has to be able
/// to run a big-endian R3000A workstation and a little-endian LR33300 board,
/// in the same process if a machine file asks for it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Which part of the family this is.
    pub arch: Arch,
    /// Which way round the byte-order pin is strapped.
    pub endian: Endian,
    /// Where the program counter starts after a reset. `0xBFC0_0000` on every
    /// real MIPS I part, and a property only so a test can start somewhere
    /// else without a boot ROM.
    pub reset_vector: u32,
    /// What `PRId` reports.
    ///
    /// A configuration field rather than a constant because it identifies the
    /// silicon and guests read it to decide what to do; nothing in this core
    /// depends on its value, so a machine that needs a specific one can say
    /// so rather than being told what its processor is.
    pub prid: u32,
    /// This processor's identity in `MemAttrs::requester`.
    pub requester: RequesterId,
}

impl Config {
    /// A configuration for a named part, little-endian, starting at the reset
    /// vector.
    #[must_use]
    pub const fn new(arch: Arch) -> Config {
        Config {
            arch,
            endian: Endian::Little,
            reset_vector: cp0::RESET_VECTOR,
            // Implementation 2, revision 3.0 — the identifier an R3000A
            // reports. Overridable, because the LR33300 and the IDT parts
            // report their own and a guest may care.
            prid: 0x0000_0230,
            requester: RequesterId::ANONYMOUS,
        }
    }

    /// The same configuration with a different byte order.
    #[must_use]
    pub const fn with_endian(mut self, endian: Endian) -> Self {
        self.endian = endian;
        self
    }

    /// The same configuration with a different reset vector.
    #[must_use]
    pub const fn with_reset_vector(mut self, pc: u32) -> Self {
        self.reset_vector = pc;
        self
    }

    /// The same configuration with a different `PRId`.
    #[must_use]
    pub const fn with_prid(mut self, prid: u32) -> Self {
        self.prid = prid;
        self
    }

    /// The same configuration with a different requester id.
    #[must_use]
    pub const fn with_requester(mut self, id: RequesterId) -> Self {
        self.requester = id;
        self
    }

    /// Check the configuration is one this core can build.
    ///
    /// The cache data arrays are indexed by masking an address, so a size that
    /// is not a power of two would alias unpredictably; catching it at `new`
    /// is the two-phase-construction rule (`ROADMAP.md` §4.4).
    fn validate(&self) -> Result<()> {
        for (what, size) in [
            ("dcache", self.arch.dcache_bytes),
            ("icache", self.arch.icache_bytes),
        ] {
            if size != 0 && !size.is_power_of_two() {
                return Err(Error::Property(alloc::format!(
                    "`{what}` is {size} bytes, which is not a power of two; the \
                     cache data array is indexed by masking an address"
                )));
            }
        }
        Ok(())
    }
}

impl Default for Config {
    fn default() -> Self {
        Config::new(Arch::R3000A)
    }
}

/// Everything the interpreter mutates, behind one lock.
#[derive(Debug)]
struct Session {
    state: State,
    space: Option<Arc<AddressSpace>>,
}

/// One MIPS processor.
///
/// # Locking
///
/// Execution state sits behind one [`sync::Mutex`] at [`LockRank::BUS`], for
/// the same reason every other core's does: a CPU is a bus master and holds
/// this lock while calling into device models, which take their own
/// `DEVICE`-ranked locks. The interrupt pins are *not* under it — they are
/// atomics in [`Lines`] — so a controller raising an interrupt from inside a
/// store the processor itself issued cannot re-enter its critical section.
#[derive(Debug)]
pub struct Cpu {
    cfg: Config,
    lines: Arc<Lines>,
    session: sync::Mutex<Session>,
    /// Which architectural traps leave the core instead of vectoring into the
    /// guest ([`ExitMask`]). An atomic rather than a field of [`Config`]
    /// because a consumer changes it *while the core runs*, and because it
    /// must survive a reset.
    exits: AtomicU32,
    /// This processor's identity in `MemAttrs::requester`, assigned at bind
    /// time.
    requester: AtomicU32,
    /// The wire sinks handed out by [`Device::sink`], kept alive here — a net
    /// holds only a weak reference to a sink, so the device owns the strong
    /// one.
    pins: sync::Mutex<Pins>,
}

/// The sinks this processor has published, one per input pin.
#[derive(Debug, Default)]
struct Pins {
    interrupts: Vec<(u32, Arc<InterruptPin>)>,
    reset: Option<Arc<ResetPin>>,
}

impl Cpu {
    /// A processor in its power-on state, with no address space yet.
    ///
    /// Two-phase construction (`ROADMAP.md` §4.4): nothing observable happens
    /// until [`attach_space`](Cpu::attach_space) and [`Device::realize`].
    ///
    /// # Panics
    ///
    /// Never for a preset. A hand-built [`Arch`] with a cache size that is not
    /// a power of two is rejected by [`Cpu::try_new`]; this wrapper is for the
    /// presets and for tests, which cannot get it wrong.
    #[must_use]
    pub fn new(cfg: Config) -> Cpu {
        Cpu::try_new(cfg).expect("a preset configuration is always valid")
    }

    /// A processor, refusing a configuration this core cannot build.
    ///
    /// # Errors
    ///
    /// If a cache data-array size is not a power of two.
    pub fn try_new(cfg: Config) -> Result<Cpu> {
        cfg.validate()?;
        Ok(Cpu {
            lines: Arc::new(Lines::default()),
            session: sync::Mutex::with_rank(
                LockRank::BUS,
                Session {
                    state: State::new(&cfg),
                    space: None,
                },
            ),
            exits: AtomicU32::new(ExitMask::NONE.bits()),
            requester: AtomicU32::new(cfg.requester.0),
            pins: sync::Mutex::new(Pins::default()),
            cfg,
        })
    }

    /// Build one from machine-description properties.
    ///
    /// # Errors
    ///
    /// If a property has the wrong type or an out-of-range value, or a
    /// property nothing here accepts was given — a typo'd property that was
    /// silently ignored is an afternoon lost.
    pub fn from_props(props: &Props) -> Result<Cpu> {
        let mut r = props.reader();
        let names: Vec<&'static str> = Arch::ALL.iter().map(|a| a.part).collect();
        let part = r.or_enum("arch", Arch::R3000A.part, &names)?;
        let arch = Arch::by_name(part).unwrap_or(Arch::R3000A);
        let endian = match r.or_enum("endian", "little", &["little", "big"])? {
            "big" => Endian::Big,
            _ => Endian::Little,
        };
        let reset = r.or("reset", u64::from(cp0::RESET_VECTOR))?;
        let prid = r.or_range("prid", u64::from(arch_prid(arch)), 0..=0xffff_ffff)?;
        // Accepted, and for now only one value is: `ROADMAP.md` §5's example
        // writes `engine = "interp"`, and the IR frontend is phase 5.
        let _ = r.or_enum("engine", "interp", &["interp"])?;
        r.finish()?;

        if reset > 0xffff_ffff {
            return Err(Error::Property(alloc::format!(
                "`reset` is 0x{reset:x}, which does not fit a 32-bit program counter"
            )));
        }
        Cpu::try_new(Config {
            arch,
            endian,
            reset_vector: reset as u32,
            prid: prid as u32,
            requester: RequesterId::ANONYMOUS,
        })
    }

    /// This processor's configuration.
    #[must_use]
    pub fn config(&self) -> Config {
        self.cfg
    }

    /// Give the processor the address space it executes from.
    pub fn attach_space(&self, space: Arc<AddressSpace>) {
        self.session.lock().space = Some(space);
    }

    /// The address space this processor executes from, if one is attached.
    #[must_use]
    pub fn space(&self) -> Option<Arc<AddressSpace>> {
        self.session.lock().space.clone()
    }

    /// Set the id accesses this processor initiates carry.
    pub fn set_requester(&self, id: RequesterId) {
        self.requester.store(id.0, Ordering::Relaxed);
    }

    /// The configuration as it stands, with the bind-time requester folded in.
    fn effective_config(&self) -> Config {
        Config {
            requester: RequesterId(self.requester.load(Ordering::Relaxed)),
            ..self.cfg
        }
    }

    /// Read a general register. `r0` reads as zero.
    ///
    /// This is the **architectural** value: a load issued by the previous
    /// instruction has not landed yet and is not included, which is what makes
    /// [`Cpu::pending_load`] worth exposing separately.
    #[must_use]
    pub fn reg(&self, index: u32) -> u32 {
        self.session.lock().state.regs[(index & 31) as usize]
    }

    /// Write a general register. A write to `r0` is discarded.
    pub fn set_reg(&self, index: u32, value: u32) {
        if index & 31 != 0 {
            self.session.lock().state.regs[(index & 31) as usize] = value;
        }
    }

    /// The load the previous instruction issued, which the next one will not
    /// see.
    ///
    /// Exposed because it is genuinely part of the machine state and a
    /// debugger showing the register file without it would be lying about what
    /// the next instruction reads.
    #[must_use]
    pub fn pending_load(&self) -> Option<(u32, u32)> {
        self.session
            .lock()
            .state
            .pending_load
            .map(|l| (l.reg, l.value))
    }

    /// Install, or clear, the load the next instruction will not see.
    ///
    /// The counterpart of [`Cpu::pending_load`], and public for the same
    /// reason: the delayed write is architectural state, so anything that can
    /// read the machine's state has to be able to write it back — a debugger
    /// editing registers, a conformance vector setting one up, a consumer
    /// starting a thread from a captured state. A write to `r0` is discarded
    /// when it settles, exactly as an immediate one would be.
    pub fn set_pending_load(&self, load: Option<(u32, u32)>) {
        self.session.lock().state.pending_load = load.map(|(reg, value)| exec::PendingLoad {
            reg: reg & 31,
            value,
        });
    }

    /// The `HI` register.
    #[must_use]
    pub fn hi(&self) -> u32 {
        self.session.lock().state.hi
    }

    /// The `LO` register.
    #[must_use]
    pub fn lo(&self) -> u32 {
        self.session.lock().state.lo
    }

    /// Set `HI` and `LO`.
    pub fn set_hi_lo(&self, hi: u32, lo: u32) {
        let mut s = self.session.lock();
        s.state.hi = hi;
        s.state.lo = lo;
    }

    /// The address of the instruction about to execute.
    #[must_use]
    pub fn pc(&self) -> u32 {
        self.session.lock().state.pc
    }

    /// Where control goes after the instruction at [`Cpu::pc`].
    #[must_use]
    pub fn next_pc(&self) -> u32 {
        self.session.lock().state.next_pc
    }

    /// Whether the instruction at [`Cpu::pc`] is in a branch delay slot.
    #[must_use]
    pub fn in_delay_slot(&self) -> bool {
        self.session.lock().state.in_delay
    }

    /// Jump somewhere, discarding any in-flight control or load state.
    ///
    /// This is the consumer-facing contract and it is deliberately blunt: it
    /// sets the whole pair, clears the delay-slot flag and drops any pending
    /// load, because a caller that says "resume here" means a clean start and
    /// not "resume here but also finish the branch you were in the middle of".
    /// [`Cpu::set_control`] is the surgical version.
    pub fn set_pc(&self, pc: u32) {
        self.set_control(pc, pc.wrapping_add(4), false);
        self.session.lock().state.pending_load = None;
    }

    /// Set the whole control pair, delay-slot flag included.
    ///
    /// What a snapshot restore and a test that wants to sit *between* a branch
    /// and its delay slot need.
    pub fn set_control(&self, pc: u32, next_pc: u32, in_delay: bool) {
        let mut s = self.session.lock();
        s.state.pc = pc;
        s.state.next_pc = next_pc;
        s.state.in_delay = in_delay;
    }

    /// A copy of the coprocessor-0 register file, for a debugger or a test.
    #[must_use]
    pub fn cp0(&self) -> Cp0 {
        self.session.lock().state.cp0.clone()
    }

    /// Overwrite the coprocessor-0 register file.
    pub fn set_cp0(&self, cp0: Cp0) {
        self.session.lock().state.cp0 = cp0;
    }

    /// A copy of the TLB.
    ///
    /// Architectural state, not a cache: an operating system writes entries
    /// with `TLBWI` and reads them back with `TLBR`, so this is a register
    /// file rather than something a snapshot may drop.
    #[must_use]
    pub fn tlb(&self) -> Tlb {
        self.session.lock().state.tlb.clone()
    }

    /// Overwrite the TLB.
    pub fn set_tlb(&self, tlb: Tlb) {
        self.session.lock().state.tlb = tlb;
    }

    /// Bus accesses charged since reset.
    #[must_use]
    pub fn cycles(&self) -> u64 {
        self.session.lock().state.cycles
    }

    /// How many accesses the address space refused.
    ///
    /// A MIPS processor *can* report a bus fault to the guest — it becomes an
    /// `IBE` or `DBE` exception — so this is a diagnostic rather than the only
    /// evidence. A machine whose memory map has a hole will show it climbing.
    #[must_use]
    pub fn bus_faults(&self) -> u64 {
        self.session.lock().state.faults
    }

    /// Drive one of the six hardware interrupt pins directly.
    ///
    /// `pin` is 0 to 5, which are `Cause.IP[2]` to `Cause.IP[7]`. This is the
    /// method a test or a hand-wired machine uses; a realized machine drives
    /// the same bits through [`InterruptPin`].
    pub fn set_interrupt(&self, pin: u32, asserted: bool) {
        self.lines.set_hw(pin, asserted);
    }

    /// The current level of the six hardware interrupt pins.
    #[must_use]
    pub fn interrupts(&self) -> u32 {
        self.lines.hw()
    }

    /// Request a reset. It happens on the next [`step`](Cpu::step), because a
    /// reset is a signal rather than a method call.
    pub fn request_reset(&self) {
        self.lines.request_reset();
    }

    /// Execute one instruction, or take one exception.
    ///
    /// Returns the bus accesses charged, which is at least one — a caller can
    /// always make progress.
    pub fn step(&self) -> u64 {
        self.step_to_exit().0
    }

    /// Execute one instruction, reporting an [`Exit`] if it produced one.
    pub fn step_to_exit(&self) -> (u64, Option<Exit>) {
        let cfg = self.effective_config();
        let exits = self.exit_mask();
        let mut session = self.session.lock();
        if self.lines.take_reset_request() {
            session.state = State::new(&cfg);
        }
        let Session { state, space } = &mut *session;
        let Some(space) = space.clone() else {
            return (0, None);
        };
        let mut exec = Exec::new(state, &space, &cfg, &self.lines, exits);
        let used = exec.step();
        (used, exec.take_exit())
    }

    /// Execute until at least `budget` accesses have been charged.
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

    /// Run a scheduler budget, reporting exactly what was consumed — never
    /// more.
    ///
    /// An instruction cannot be stopped half way, so the last one of a budget
    /// usually runs past its end. The scheduler treats an overrun as fatal, so
    /// the overshoot is *carried* — deducted from the next budget — which
    /// keeps the processor's access count and the domain's tick count in step
    /// over any number of quanta while never letting a single one overrun.
    pub fn run_budget(&self, ticks: u64) -> u64 {
        let owed = self.session.lock().state.debt;
        if owed >= ticks {
            self.session.lock().state.debt = owed - ticks;
            return ticks;
        }
        let allowance = ticks - owed;
        let mut used = 0u64;
        while used < allowance {
            let n = self.step();
            if n == 0 {
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

    /// Accesses owed to the next budget — see [`run_budget`](Cpu::run_budget).
    #[must_use]
    pub fn cycle_debt(&self) -> u64 {
        self.session.lock().state.debt
    }

    /// Run until the budget is exhausted or an armed trap leaves the core.
    ///
    /// The level-3 run loop (`ROADMAP.md` §2.1). Scheduler debt is paid down
    /// exactly as [`run_budget`](Cpu::run_budget) pays it, so a core can be
    /// driven this way and by the machine scheduler without two tick
    /// accountings.
    pub fn run_to_exit_ticks(&self, ticks: u64) -> Run {
        let owed = self.session.lock().state.debt;
        if owed >= ticks {
            self.session.lock().state.debt = owed - ticks;
            return Run::completed(Consumed::new(ticks));
        }
        let allowance = ticks - owed;
        let mut used = 0u64;
        while used < allowance {
            let (n, exit) = self.step_to_exit();
            if n == 0 {
                break;
            }
            used += n;
            if let Some(exit) = exit {
                let total = owed + used;
                self.session.lock().state.debt = total.saturating_sub(ticks);
                return Run::exited(Consumed::new(total.min(ticks)), exit);
            }
        }
        if used >= allowance {
            self.session.lock().state.debt = used - allowance;
            Run::completed(Consumed::new(ticks))
        } else {
            self.session.lock().state.debt = 0;
            Run::completed(Consumed::new(owed + used))
        }
    }

    /// Disassemble `count` instructions starting at `pc`, reading guest memory
    /// with debug attributes.
    ///
    /// Debug attributes are the point: a monitor listing the code around the
    /// program counter must not pop a FIFO or clear a status bit on the way.
    /// The read goes through the *segment* map, so a listing at `0x8000_1000`
    /// finds the same bytes the processor would fetch — but never through the
    /// TLB, because walking it would be a guess about which address space a
    /// debugger meant.
    #[must_use]
    pub fn disassemble(&self, pc: u32, count: usize) -> Vec<disasm::Disassembled> {
        let Some(space) = self.space() else {
            return Vec::new();
        };
        disasm::disassemble_run(pc, count, |addr| {
            let vaddr = addr as u32;
            let segment = cp0::Segment::of(vaddr);
            let phys = if segment.mapped() {
                vaddr
            } else {
                cp0::Segment::unmapped_phys(vaddr)
            };
            space
                .read(u64::from(phys), Width::U32, MemAttrs::DEBUG)
                .ok()
                .map(|v| v as u32)
        })
    }
}

/// `PRId` for a preset.
///
/// The generic R3000A identifier for the MIPS-branded parts. The LSI part is
/// widely reported to answer `0x0000_0002`; nothing in this core depends on
/// the value, and a machine file that needs a different one writes `prid = …`.
const fn arch_prid(arch: Arch) -> u32 {
    if arch.tlb { 0x0000_0230 } else { 0x0000_0002 }
}

/// The `cpu.mips` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: "cpu.mips",
    version: 1,
    summary: "MIPS I / R3000A interpreter with CP0, the 64-entry TLB and delay slots",
    properties: &[
        PropertySpec {
            name: "arch",
            kind: ValueKind::Str,
            required: false,
            summary: "which part: `r3000a`, `r3051` or `lr33300` (which has no TLB)",
        },
        PropertySpec {
            name: "endian",
            kind: ValueKind::Str,
            required: false,
            summary: "the byte-order pin: `little` (default) or `big`",
        },
        PropertySpec {
            name: "reset",
            kind: ValueKind::Uint,
            required: false,
            summary: "where the program counter starts (default 0xbfc00000)",
        },
        PropertySpec {
            name: "prid",
            kind: ValueKind::Uint,
            required: false,
            summary: "the value the `PRId` register reports",
        },
        PropertySpec {
            name: "engine",
            kind: ValueKind::Str,
            required: false,
            summary: "which execution engine; only `interp` exists until phase 5",
        },
    ],
    construct: |props| Ok(Box::new(Cpu::from_props(props)?)),
};

/// Add this core's class to a registry.
///
/// Registration is explicit per feature rather than link-time magic
/// (`ROADMAP.md` §4.4), so the machine assembly layer calls this from its own
/// `#[cfg(feature = "cpu-mips")]` arm.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut Registry) -> Result<()> {
    reg.add(&CLASS)
}

/// Which hardware interrupt pin a named input port drives.
///
/// The names are the manual's: `int0` to `int5` are the six external requests,
/// which appear in `Cause.IP[2]` to `Cause.IP[7]`. The two software interrupt
/// bits have no pins — they are written through `MTC0` to `Cause` — so there
/// are six ports and not eight.
fn pin_number(port: &str) -> Option<u32> {
    let n = port.strip_prefix("int")?.parse::<u32>().ok()?;
    (n < 6).then_some(n)
}

impl Device for Cpu {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward. A processor with no address space cannot fetch, but
        // realize runs *before* the machine binds one — that check belongs to
        // `Instance::bind`, which is where the space arrives.
        Ok(())
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        // The fan-in can only be built now: it is told its sources at
        // construction and no `WireId` existed when this core was made.
        let mut pins = self.pins.lock();
        if port == "reset" {
            let pin = Arc::new(ResetPin::new(Arc::clone(&self.lines), sources));
            pins.reset = Some(Arc::clone(&pin));
            return Some(SinkPin { sink: pin, line: 0 });
        }
        let n = pin_number(port)?;
        let pin = Arc::new(InterruptPin::new(Arc::clone(&self.lines), n, sources));
        pins.interrupts.push((n, Arc::clone(&pin)));
        Some(SinkPin { sink: pin, line: n })
    }

    fn is_runnable(&self) -> bool {
        true
    }

    fn run(&self, budget: Budget) -> Consumed {
        Consumed::new(self.run_budget(budget.ticks))
    }

    fn reset(&self, kind: ResetKind) {
        let cfg = self.effective_config();
        let mut session = self.session.lock();
        session.state = State::new(&cfg);
        drop(session);
        if kind == ResetKind::Cold {
            // A cold start has nothing driving the interrupt pins yet. A warm
            // one does, and clearing them would make the reset lie about the
            // machine.
            self.lines.set_all_hw(0);
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let session = self.session.lock();
        let s = &session.state;
        for r in s.regs {
            w.write_u32(r)?;
        }
        for v in [s.hi, s.lo, s.pc, s.next_pc] {
            w.write_u32(v)?;
        }
        w.write_bool(s.in_delay)?;
        // The pending load is architectural: a snapshot taken between a load
        // and the instruction after it must come back with the destination
        // register still holding its *old* value and the new one still on its
        // way, or the restored guest computes a different answer.
        match s.pending_load {
            None => w.write_bool(false)?,
            Some(load) => {
                w.write_bool(true)?;
                w.write_u32(load.reg)?;
                w.write_u32(load.value)?;
            }
        }
        let c = &s.cp0;
        for v in [
            c.index,
            c.random,
            c.entry_lo,
            c.context,
            c.bad_vaddr,
            c.entry_hi,
            c.status,
            c.cause,
            c.epc,
            c.prid,
        ] {
            w.write_u32(v)?;
        }
        for v in c.debug {
            w.write_u32(v)?;
        }
        for entry in s.tlb.entries() {
            w.write_u32(entry.hi)?;
            w.write_u32(entry.lo)?;
        }
        w.write_bytes(&s.dcache)?;
        w.write_bytes(&s.icache)?;
        w.write_u64(s.cycles)?;
        w.write_u64(s.debt)?;
        w.write_u64(s.faults)?;
        // The interrupt pins are architectural: a restored machine whose timer
        // was already firing must still see it.
        w.write_u32(self.lines.hw())?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let cfg = self.effective_config();
        let mut s = State::new(&cfg);
        for slot in &mut s.regs {
            *slot = r.read_u32()?;
        }
        // r0 is architecturally zero whatever a snapshot claims.
        s.regs[0] = 0;
        s.hi = r.read_u32()?;
        s.lo = r.read_u32()?;
        s.pc = r.read_u32()?;
        s.next_pc = r.read_u32()?;
        s.in_delay = r.read_bool()?;
        s.pending_load = if r.read_bool()? {
            let reg = r.read_u32()?;
            let value = r.read_u32()?;
            Some(exec::PendingLoad { reg, value })
        } else {
            None
        };
        let c = &mut s.cp0;
        for slot in [
            &mut c.index,
            &mut c.random,
            &mut c.entry_lo,
            &mut c.context,
            &mut c.bad_vaddr,
            &mut c.entry_hi,
            &mut c.status,
            &mut c.cause,
            &mut c.epc,
            &mut c.prid,
        ] {
            *slot = r.read_u32()?;
        }
        for slot in &mut c.debug {
            *slot = r.read_u32()?;
        }
        for i in 0..TLB_ENTRIES {
            let hi = r.read_u32()?;
            let lo = r.read_u32()?;
            s.tlb.set_entry(i as u32, TlbEntry { hi, lo });
        }
        s.dcache = r.read_bytes()?.to_vec();
        s.icache = r.read_bytes()?.to_vec();
        s.cycles = r.read_u64()?;
        s.debt = r.read_u64()?;
        s.faults = r.read_u64()?;
        let pins = r.read_u32()?;
        self.session.lock().state = s;
        self.lines.set_all_hw(pins);
        Ok(())
    }
}

/// The level-3 seam (`ROADMAP.md` §2.1): a processor that can stop *at* a
/// `syscall` and hand control out rather than vectoring to a guest handler.
///
/// Arming [`ExitMask::USER`] is what turns this core from a machine's CPU into
/// a `qemu-user`-shaped one. Nothing else about it changes: the same
/// interpreter, the same address space, the same snapshot.
///
/// The delay slot is the one place this seam needs care. A fault exit rewinds
/// the **whole control pair**, delay-slot flag included, so a consumer that
/// fixes the fault and resumes gets a processor that is still in the middle of
/// the branch it was in the middle of. A `syscall` exit does not rewind, and
/// [`Exit::resume_pc`] therefore reports `pc + 4` — which is right except for
/// a `syscall` *in* a delay slot, where the core itself has already put the
/// branch target in place and a consumer that overwrites the program counter
/// from `resume_pc` would break the branch. No ABI puts a syscall in a delay
/// slot, and a consumer that does not rewrite the program counter is correct
/// either way.
impl ExitingCore for Cpu {
    fn exit_mask(&self) -> ExitMask {
        // Relaxed: the mask is configuration, and a change to it is ordered
        // against nothing but the next instruction fetch.
        ExitMask::from_bits(self.exits.load(Ordering::Relaxed))
    }

    fn set_exit_mask(&self, mask: ExitMask) {
        self.exits.store(mask.bits(), Ordering::Relaxed);
    }

    fn run_to_exit(&self, budget: Budget) -> Run {
        self.run_to_exit_ticks(budget.ticks)
    }

    fn pc(&self) -> u64 {
        u64::from(Cpu::pc(self))
    }

    fn set_pc(&self, pc: u64) {
        Cpu::set_pc(self, pc as u32);
    }

    fn sp(&self) -> u64 {
        u64::from(self.reg(29))
    }

    fn set_sp(&self, sp: u64) {
        self.set_reg(29, sp as u32);
    }
}

impl Initiator for Cpu {
    fn requester(&self) -> RequesterId {
        RequesterId(self.requester.load(Ordering::Relaxed))
    }
}

/// The machine layer's half: a processor needs an address space, and this is
/// where the machine gives it one.
impl crate::machine::Instance for Cpu {
    fn bind(&self, ctx: &crate::machine::BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: ctx.path().to_string(),
            message: "a MIPS core needs an address space to fetch from (`space = mem`)".to_string(),
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
    bindings.bind(CLASS.name, |props| Ok(Arc::new(Cpu::from_props(props)?)))
}

/// What the validator should know about `cpu.mips`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    let parts: &'static [&'static str] = &["r3000a", "r3051", "lr33300"];
    ClassSchema::new(CLASS.name)
        .prop(PropSchema::new("arch", ValueKind::Str).values(parts))
        .prop(PropSchema::new("endian", ValueKind::Str).values(&["little", "big"]))
        .prop(PropSchema::new("reset", ValueKind::Uint))
        .prop(PropSchema::new("prid", ValueKind::Uint))
        .prop(PropSchema::new("engine", ValueKind::Str).values(&["interp"]))
        // Inputs only: this core drives no line.
        .port("int0", PortDir::In)
        .port("int1", PortDir::In)
        .port("int2", PortDir::In)
        .port("int3", PortDir::In)
        .port("int4", PortDir::In)
        .port("int5", PortDir::In)
        .port("reset", PortDir::In)
}

/// One of the processor's six hardware interrupt inputs, as something a wire
/// can drive.
///
/// A wire hands each sink the level of the *driver that changed*, not the
/// resolved level of the net, so this keeps a [`FanIn`] and wire-ORs the
/// sources — which is what a shared interrupt line does in hardware.
///
/// The pin keeps a handle on the processor's *input latches*, not on the
/// processor: the core owns the pin, and a pin that owned the core back would
/// be a cycle the machine could never drop.
#[derive(Debug)]
pub struct InterruptPin {
    lines: Arc<Lines>,
    pin: u32,
    inputs: FanIn,
    resolve: Resolve,
}

impl InterruptPin {
    /// Connect hardware interrupt `pin` — 0 to 5 — to a net driven by
    /// `sources`.
    #[must_use]
    pub fn new(lines: Arc<Lines>, pin: u32, sources: &[WireId]) -> InterruptPin {
        InterruptPin {
            lines,
            pin,
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

    /// Which hardware interrupt this pin drives.
    #[must_use]
    pub fn pin(&self) -> u32 {
        self.pin
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
        self.lines.set_hw(self.pin, asserted);
    }
}

/// The processor's reset input, as something a wire can drive.
///
/// Separate from [`InterruptPin`] because a reset is not an interrupt: it has
/// no `Cause.IP` bit, no mask and no handler. Asserting the line latches a
/// request; the reset itself happens on the next [`Cpu::step`].
#[derive(Debug)]
pub struct ResetPin {
    lines: Arc<Lines>,
    inputs: FanIn,
    resolve: Resolve,
}

impl ResetPin {
    /// Connect the reset pin to a net driven by `sources`.
    #[must_use]
    pub fn new(lines: Arc<Lines>, sources: &[WireId]) -> ResetPin {
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
        // Latch on assertion rather than on release: a machine whose reset
        // button is still held should still come up, instead of waiting for a
        // release nobody modelled.
        if self.inputs.resolve(self.resolve).is_high() {
            self.lines.request_reset();
        }
    }
}

/// A description of this core for `rsemu describe cpu.mips`.
///
/// Built from [`isa::TABLE`], so it cannot drift from what the interpreter
/// implements.
#[must_use]
pub fn describe_isa() -> String {
    use core::fmt::Write as _;
    let mut out = String::new();
    for insn in isa::TABLE {
        let _ = writeln!(
            out,
            "{:08x}/{:08x} {:<8} {:<6} {}",
            insn.bits,
            insn.mask,
            insn.op.mnemonic(),
            insn.req.name(),
            insn.op.summary()
        );
    }
    out
}
