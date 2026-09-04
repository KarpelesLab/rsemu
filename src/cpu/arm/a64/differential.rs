//! The differential harness: [`lift`] against the interpreter,
//! forever.
//!
//! CLAUDE.md's "CPU cores" rule makes the interpreter the oracle and the IR
//! frontend differentially tested against it. This is that harness. One guest
//! program is executed twice — once as a lifted [`Block`](crate::ir::Block) on
//! [`Interp`], once instruction by instruction on
//! [`Cpu`] — and everything a guest, a snapshot or a state hash
//! can see is compared:
//!
//! * every general register, `X0`–`X30`;
//! * the **stack pointer**, which is a slot rather than a register and which
//!   nothing else in the tree exercises;
//! * `PSTATE.NZCV`, whole, because A64 spells `CMP` as `SUBS` and a lifter
//!   that got one flag wrong would be wrong on every conditional branch;
//! * the program counter;
//! * the cycle counter, which `ROADMAP.md` §0 makes part of the state hash;
//! * the **static tick column** — the [`InsnStart::ticks`] a block publishes —
//!   cross-checked against the ticks actually charged;
//! * guest memory, byte for byte;
//! * whether the two agree that the program faulted, and if so, the
//!   architectural state *at* the fault.
//!
//! # Why the harness reimplements the memory rule
//!
//! `Host::access` is a second implementation of `Exec::load`/`Exec::store`,
//! and that is deliberate rather than an oversight: what is under test is the
//! [`MemOp`] the frontend attached — its width, its sign and its [`Align`] —
//! and a harness that routed through the interpreter's own path would agree
//! with the interpreter by construction on exactly the field it is checking.
//! The rule is small enough to state twice: one bus cycle per access when the
//! address is naturally aligned, one per **byte** when it is not, and a fault
//! before any of them when `SCTLR_EL1.A` is set. That is `Exec::check_align`
//! and `Exec::load` in three lines.
//!
//! The cost is that this covers **bare mode only** — `SCTLR_EL1.M` clear, so
//! [`Origin::Bare`] — and paging is left to `tests/a64_engines.rs`, where a
//! real board runs a real kernel through both engines and compares state
//! hashes. That is the same division `cpu::riscv::differential` draws.
//!
//! # What is not covered, stated rather than discovered
//!
//! * **What the trap handler does next.** A fault stops the comparison; where
//!   `VBAR_EL1` sends the guest is the interpreter's, and the harness arms
//!   [`ExitReason::FAULT`] so the oracle stops *at* the faulting instruction
//!   instead of vectoring into whatever RAM holds.
//! * **Code a running trace overwrites.** A64 requires cache maintenance
//!   between writing instruction memory and executing it, so the architecture
//!   permits a disagreement there — which is exactly why
//!   [`lift`] ends a block at a store, and why the cached path
//!   below asserts that a store into a block's own page invalidates it.
//! * **Anything outside the lifted subset**, which ends the block and is
//!   reported as [`Verdict::Nothing`] when it is the first instruction.

use alloc::format;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::error::BusError;
use crate::core::exec::{ExitMask, ExitReason, ExitingCore};
use crate::core::space::{AddressSpace, MemAttrs, MemResult, RamStore, Region, UnassignedPolicy};
use crate::core::value::Width;
use crate::ir::{Align, InsnStart, Interp, IrHost, MemOp, Outcome, RegSlot, verify};

use super::isa::{self, Nzcv};
use super::lift::{self, Origin, PC, SP, Shape, World, x_slot};
use super::sysreg::sctlr;
use super::{Config, Cpu};

#[cfg(feature = "jit")]
use crate::jit::{
    BlockCache, DirtyPages, Dispatcher, Epoch, FastMem, Frontend, PAGE_MASK, Stop, StoreLog,
    Translation,
};

// ---------------------------------------------------------------------------
// The memory map every case shares
// ---------------------------------------------------------------------------

/// Where a case's program is loaded.
///
/// Away from zero, and not a power of two on its own, so that an `ADR` or an
/// `ADRP` that dropped its base shows up as a wild address rather than as a
/// plausible one.
pub const BASE: u64 = 0x2000_0000;

/// How much RAM a case gets: four pages, so a block is bounded by its page and
/// there is somewhere off that page to store to.
pub const RAM_SIZE: u64 = 4 * 4096;

/// The offset of the data window from [`BASE`]. Page zero is code.
pub const DATA: u64 = 4096;

/// The stack pointer a case starts with, sixteen-byte aligned as the
/// architecture's stack-alignment checks want.
pub const STACK: u64 = DATA + 0xc00;

/// The register the synthesizer draws memory base pointers from, and the ones
/// it draws arithmetic operands from.
///
/// Split on purpose. A corpus whose arithmetic clobbered its own pointers
/// would fault on nearly every access and measure the fault path rather than
/// the lifter; a corpus that never let a pointer be an ALU operand would never
/// reach a load whose address a previous instruction computed. So `X1`–`X4`
/// hold pointers and are *readable* everywhere, while `X5`–`X12` are what gets
/// written.
pub const POINTERS: u32 = 4;

/// One past the highest register the synthesizer writes.
pub const SYNTH_REGS: u32 = 13;

// ---------------------------------------------------------------------------
// Cases
// ---------------------------------------------------------------------------

/// One program, one starting state, one policy.
#[derive(Debug, Clone)]
pub struct Case {
    /// Which part the case runs on. Must be in bare mode with `SCTLR_EL1.M`
    /// clear, which is the reset state.
    pub cfg: Config,
    /// The instruction words, loaded at [`BASE`].
    pub program: Vec<u32>,
    /// `X0`–`X30`.
    pub regs: [u64; 31],
    /// The stack pointer.
    pub sp: u64,
    /// `PSTATE.NZCV`, in the top nibble as the register holds it.
    pub nzcv: u32,
    /// Whether `SCTLR_EL1.A` is set, so an unaligned access faults.
    pub strict_align: bool,
    /// How much the lifter may swallow.
    pub shape: Shape,
}

impl Case {
    /// A case over `program` on a Cortex-A53, with a zeroed register file.
    #[must_use]
    pub fn new(program: Vec<u32>) -> Case {
        Case {
            cfg: Config::cortex_a53(),
            program,
            regs: [0; 31],
            sp: BASE + STACK,
            nzcv: 0,
            strict_align: false,
            shape: Shape::default(),
        }
    }

    /// The same, with `X1`–`X4` pointing into the data window.
    ///
    /// The first is deliberately **not** eight-byte aligned, so the split
    /// access path is reachable from a generated corpus rather than only from
    /// a hand-written case.
    #[must_use]
    pub fn seeded(program: Vec<u32>) -> Case {
        Case::new(program)
            .with_reg(1, BASE + DATA + 0x101)
            .with_reg(2, BASE + DATA + 0x400)
            .with_reg(3, BASE + DATA + 0x600)
            .with_reg(4, BASE + DATA + 0x800)
    }

    /// The same case under a different shape.
    #[must_use]
    pub fn with_shape(mut self, shape: Shape) -> Case {
        self.shape = shape;
        self
    }

    /// The same case with one register set. Register 31 is the stack pointer.
    #[must_use]
    pub fn with_reg(mut self, n: usize, value: u64) -> Case {
        if n == 31 {
            self.sp = value;
        } else {
            self.regs[n] = value;
        }
        self
    }

    /// The same case with the flags set.
    #[must_use]
    pub fn with_nzcv(mut self, nzcv: Nzcv) -> Case {
        self.nzcv = nzcv.0;
        self
    }

    /// The same case with `SCTLR_EL1.A` set, so unaligned accesses fault.
    #[must_use]
    pub fn strict(mut self) -> Case {
        self.strict_align = true;
        self
    }

    /// The same case on a different part.
    #[must_use]
    pub fn with_config(mut self, cfg: Config) -> Case {
        self.cfg = cfg;
        self
    }

    /// The world this case lifts in.
    #[must_use]
    pub fn world(&self) -> World {
        World {
            features: self.cfg.features,
            origin: Origin::Bare,
            strict_align: self.strict_align,
        }
    }
}

/// What a comparison found.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Verdict {
    /// The first instruction was outside the lifted subset, so there was
    /// nothing to compare. Not a failure.
    Nothing,
    /// Both engines stopped on a fault, at the same instruction, in the same
    /// architectural state.
    Trapped {
        /// How many guest instructions retired before it.
        insns: usize,
    },
    /// Both engines ran to the end of the block and agreed on everything.
    Agreed {
        /// How many guest instructions retired.
        insns: usize,
        /// How many ticks both charged.
        ticks: u64,
    },
}

/// A disagreement, with enough to reproduce it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Divergence {
    /// What differed.
    pub what: String,
    /// The program, disassembled, and the non-zero starting state.
    pub program: String,
}

impl core::fmt::Display for Divergence {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}\n{}", self.what, self.program)
    }
}

/// Build a [`Divergence`], disassembling the case so a failure names
/// instructions rather than hex.
fn diverged(case: &Case, what: String) -> Divergence {
    let mut program = String::new();
    for (i, word) in case.program.iter().enumerate() {
        let pc = BASE + 4 * i as u64;
        let line = super::disasm::disassemble(*word, pc, case.cfg.features);
        program.push_str(&format!("  {pc:#x}: {word:08x}  {line}\n"));
    }
    for (n, value) in case.regs.iter().enumerate() {
        if *value != 0 {
            program.push_str(&format!("  x{n} = {value:#x}\n"));
        }
    }
    program.push_str(&format!("  sp = {:#x}\n", case.sp));
    if case.nzcv != 0 {
        program.push_str(&format!("  nzcv = {}\n", Nzcv(case.nzcv)));
    }
    program.push_str(&format!("  shape = {:?}\n", case.shape));
    Divergence { what, program }
}

// ---------------------------------------------------------------------------
// The machine
// ---------------------------------------------------------------------------

/// The address space and the store behind it, with `program` at [`BASE`].
///
/// Two of these are built per comparison, so a store in one engine is
/// invisible to the other.
fn machine(case: &Case) -> (Arc<AddressSpace>, Arc<RamStore>) {
    let ram = Arc::new(RamStore::new(RAM_SIZE));
    // The data window holds a pattern rather than zeroes, and that is not
    // decoration: a mutation pass found that `LDRSB Wt` keeping its sign
    // instead of narrowing to thirty-two bits **survived** a two-thousand-case
    // corpus, because every byte a signed load could reach was zero and a
    // zero has no sign to get wrong. Every byte here has its top bit set for
    // half the window and clear for the other half, and no two adjacent
    // doublewords are equal.
    for off in DATA..RAM_SIZE {
        let byte = (off.wrapping_mul(31).wrapping_add(off >> 5) & 0xff) as u8;
        ram.write_u8(off, byte).expect("inside the store");
    }
    for (i, word) in case.program.iter().enumerate() {
        for (j, byte) in word.to_le_bytes().iter().enumerate() {
            ram.write_u8((i * 4 + j) as u64, *byte)
                .expect("the program fits in the code page");
        }
    }
    let space = AddressSpace::new("mem", 64).with_unassigned(UnassignedPolicy::FAULT);
    space
        .topology()
        .map(Region::ram("ram", Arc::clone(&ram)), BASE)
        .expect("nothing else is mapped");
    (Arc::new(space), ram)
}

/// The interpreter, set up to stop *at* a fault rather than vector into it.
fn oracle(case: &Case, space: Arc<AddressSpace>) -> Cpu {
    let cpu = Cpu::new(case.cfg.with_reset_vector(BASE));
    cpu.attach_space(space);
    // Every synchronous exception leaves the core, so a fault stops the oracle
    // on the faulting instruction with its architectural state intact. Without
    // this the oracle would vector to `VBAR_EL1`, which is zero, and execute
    // whatever the unmapped page below RAM answered with.
    cpu.set_exit_mask(
        ExitMask::NONE
            .with(ExitReason::FAULT)
            .with(ExitReason::SYSCALL)
            .with(ExitReason::BREAKPOINT),
    );
    for (n, value) in case.regs.iter().enumerate() {
        cpu.set_x(n as u32, *value);
    }
    cpu.set_sp(case.sp);
    let mut sys = cpu.sysregs();
    sys.nzcv = Nzcv(case.nzcv);
    if case.strict_align {
        sys.sctlr |= sctlr::A;
    }
    cpu.set_sysregs(sys);
    cpu
}

/// Compare guest memory byte for byte.
fn memory(case: &Case, want: &RamStore, got: &RamStore) -> Result<(), Divergence> {
    for off in 0..RAM_SIZE {
        let a = want.read_u8(off).unwrap_or(0);
        let b = got.read_u8(off).unwrap_or(0);
        if a != b {
            return Err(diverged(
                case,
                format!(
                    "guest memory at {:#x}: the interpreter says {a:#04x}, the block says {b:#04x}",
                    BASE + off
                ),
            ));
        }
    }
    Ok(())
}

/// Compare every column of architectural state.
///
/// One function for the exit and the fault paths alike, because the columns
/// are the same and two copies of them is how one of them silently stops being
/// checked. `pc` is the block's answer — the [`PC`] slot at an exit, and
/// [`Fault::pc`] at a fault, where the slot is deliberately not bound.
fn state(
    case: &Case,
    cpu: &Cpu,
    slots: &[u64; lift::SLOT_COUNT as usize],
    pc: u64,
    ticks: u64,
    what: &str,
) -> Result<(), Divergence> {
    for n in 0..31u32 {
        let want = cpu.x(n);
        let got = slots[x_slot(n).0 as usize];
        if want != got {
            return Err(diverged(
                case,
                format!("{what}: x{n} is {want:#018x} interpreted and {got:#018x} lifted"),
            ));
        }
    }
    let want_sp = cpu.sp();
    let got_sp = slots[SP.0 as usize];
    if want_sp != got_sp {
        return Err(diverged(
            case,
            format!("{what}: sp is {want_sp:#018x} interpreted and {got_sp:#018x} lifted"),
        ));
    }
    let want_flags = cpu.sysregs().nzcv;
    let got_flags = flags_of(slots);
    if want_flags != got_flags {
        return Err(diverged(
            case,
            format!("{what}: PSTATE.NZCV is {want_flags} interpreted and {got_flags} lifted"),
        ));
    }
    let want_pc = cpu.pc();
    if want_pc != pc {
        return Err(diverged(
            case,
            format!("{what}: the pc is {want_pc:#x} interpreted and {pc:#x} lifted"),
        ));
    }
    let want_ticks = cpu.cycles();
    if want_ticks != ticks {
        return Err(diverged(
            case,
            format!(
                "{what}: the cycle counter is {want_ticks} interpreted and {ticks} lifted. \
                 A compiled block must charge exactly what an interpreted one charges \
                 (ROADMAP.md §0)"
            ),
        ));
    }
    Ok(())
}

/// Repack the four flag slots into the word the guest reads.
fn flags_of(slots: &[u64; lift::SLOT_COUNT as usize]) -> Nzcv {
    Nzcv::new(
        slots[lift::N.0 as usize] & 1 != 0,
        slots[lift::Z.0 as usize] & 1 != 0,
        slots[lift::C.0 as usize] & 1 != 0,
        slots[lift::V.0 as usize] & 1 != 0,
    )
}

/// Step the oracle up to `want` instructions, stopping at the first fault.
///
/// One at a time, and stopping at the first trap, so an unpredicted fault is
/// reported here rather than run past into a handler.
fn step_oracle(cpu: &Cpu, want: usize) -> bool {
    for _ in 0..want {
        if cpu.step_to_exit().1.is_some() {
            return true;
        }
    }
    false
}

// ---------------------------------------------------------------------------
// The comparison
// ---------------------------------------------------------------------------

/// Lift `case`, run the block, run the interpreter, and compare.
///
/// # Errors
///
/// A [`Divergence`] naming the column that differed, with the case
/// disassembled.
///
/// # Panics
///
/// If the case is not one this harness can set up — a program that does not
/// fit in the code page, or a configuration with the MMU enabled, both of
/// which are harness misuse rather than findings.
pub fn compare(case: &Case) -> Result<Verdict, Divergence> {
    assert!(
        case.program.len() * 4 <= DATA as usize,
        "the program must fit in the code page"
    );

    let world = case.world();
    let (subject_space, subject_ram) = machine(case);
    let program = case.program.clone();
    let mut src = |addr: u64| {
        let off = addr.checked_sub(BASE)? / 4;
        program.get(off as usize).copied()
    };
    let lifted = lift::lift(&world, BASE, &mut src, lift::MAX_INSNS, case.shape)
        .map_err(|e| diverged(case, format!("the frontend refused this world: {e}")))?;
    if lifted.insns == 0 {
        return Ok(Verdict::Nothing);
    }
    verify(&lifted.block)
        .map_err(|e| diverged(case, format!("the frontend emitted a malformed block: {e}")))?;

    let mut host = Host::new(case, subject_space);
    let mut interp = Interp::new();
    let outcome = interp
        .run(&lifted.block, &mut host)
        .map_err(|e| diverged(case, format!("the IR backend refused the block: {e}")))?;
    // The boundaries the run actually passed, never `Lifted::insns`: a trace
    // covers every instruction on the path it inlined and retires only the
    // ones it reached, and the static number would step the oracle the wrong
    // number of times.
    let retired = interp.boundaries().saturating_sub(1) as usize;
    let fault = match outcome {
        Outcome::Exit => None,
        Outcome::Fault(f) => Some(f),
        other => {
            return Err(diverged(
                case,
                format!("a lifted block must end in exit_tb, and this one said {other:?}"),
            ));
        }
    };

    let (oracle_space, oracle_ram) = machine(case);
    let cpu = oracle(case, oracle_space);
    // One extra step when the block faulted: the faulting instruction is the
    // one that did *not* retire, so the oracle has to attempt it.
    let want = retired + usize::from(fault.is_some());
    let oracle_trapped = step_oracle(&cpu, want);

    if oracle_trapped != fault.is_some() {
        return Err(diverged(
            case,
            format!(
                "after {retired} retired instructions the interpreter {} and the block {}",
                if oracle_trapped { "trapped" } else { "did not" },
                if fault.is_some() {
                    "faulted"
                } else {
                    "did not"
                },
            ),
        ));
    }

    match fault {
        Some(f) => {
            state(
                case,
                &cpu,
                &host.slots,
                f.pc,
                host.ticks,
                "at the faulting instruction",
            )?;
            memory(case, &oracle_ram, &subject_ram)?;
            Ok(Verdict::Trapped { insns: retired })
        }
        None => {
            let pc = host.slots[PC.0 as usize];
            state(case, &cpu, &host.slots, pc, host.ticks, "at the block exit")?;
            // The static column plus what the accesses spent is the whole
            // count. That identity is what says the frontend's own accounting
            // is right rather than merely consistent.
            let column = interp
                .mark()
                .and_then(|m| lifted.block.marks().get(m as usize))
                .map(|m: &InsnStart| m.ticks)
                .expect("a block that ran reached a boundary");
            if column + host.access_ticks != host.ticks {
                return Err(diverged(
                    case,
                    format!(
                        "the static tick column says {column} and the accesses spent \
                         {}, which is {} rather than the {} charged",
                        host.access_ticks,
                        column + host.access_ticks,
                        host.ticks
                    ),
                ));
            }
            memory(case, &oracle_ram, &subject_ram)?;
            Ok(Verdict::Agreed {
                insns: retired,
                ticks: host.ticks,
            })
        }
    }
}

// ---------------------------------------------------------------------------
// The host
// ---------------------------------------------------------------------------

/// The guest state a block reads and writes.
struct Host {
    slots: [u64; lift::SLOT_COUNT as usize],
    space: Arc<AddressSpace>,
    attrs: MemAttrs,
    strict_align: bool,
    /// Every tick charged.
    ticks: u64,
    /// The data-dependent half: what the accesses spent, which the static
    /// column does not carry.
    access_ticks: u64,
    /// The guest-physical pages this host wrote, for the block cache to match
    /// cached translations against. In bare mode the guest address *is* the
    /// physical one, which is what makes this a one-liner here.
    #[cfg(feature = "jit")]
    dirty: DirtyPages,
}

impl Host {
    fn new(case: &Case, space: Arc<AddressSpace>) -> Host {
        let mut slots = [0u64; lift::SLOT_COUNT as usize];
        slots[..31].copy_from_slice(&case.regs);
        slots[SP.0 as usize] = case.sp;
        let flags = Nzcv(case.nzcv);
        slots[lift::N.0 as usize] = u64::from(flags.n());
        slots[lift::Z.0 as usize] = u64::from(flags.z());
        slots[lift::C.0 as usize] = u64::from(flags.c());
        slots[lift::V.0 as usize] = u64::from(flags.v());
        slots[PC.0 as usize] = BASE;
        Host {
            slots,
            space,
            attrs: MemAttrs::DEFAULT.with_requester(case.cfg.requester),
            strict_align: case.strict_align,
            ticks: 0,
            access_ticks: 0,
            #[cfg(feature = "jit")]
            dirty: DirtyPages::new(),
        }
    }

    /// One access that does not cross a page boundary. One bus cycle, charged
    /// whether or not it succeeded — which is what `Exec::read_once` does.
    fn once(&mut self, addr: u64, width: Width, value: Option<u64>) -> MemResult<u64> {
        self.ticks += 1;
        self.access_ticks += 1;
        match value {
            Some(v) => {
                // Whatever landed, landed: a split store that faults on its
                // second page still wrote the first, and a translation of
                // those bytes is stale either way.
                #[cfg(feature = "jit")]
                self.dirty.note(addr, width.bytes());
                self.space.write(addr, width, v, self.attrs).map(|()| 0)
            }
            None => self.space.read(addr, width, self.attrs),
        }
    }

    /// `Exec::load` and `Exec::store`, restated: aligned is one access,
    /// unaligned is one per byte, and `SCTLR_EL1.A` turns the second into a
    /// fault before any of them happen.
    fn access(&mut self, mem: &MemOp, addr: u64, value: Option<u64>) -> MemResult<u64> {
        let bytes = mem.size.bytes();
        if addr.is_multiple_of(bytes) {
            return self.once(addr, mem.size, value);
        }
        if self.strict_align || mem.align == Align::Fault {
            return Err(BusError::BadAccess);
        }
        match value {
            Some(v) => {
                for i in 0..bytes {
                    self.once(addr.wrapping_add(i), Width::U8, Some(v >> (8 * i)))?;
                }
                Ok(0)
            }
            None => {
                let mut out = 0u64;
                for i in 0..bytes {
                    let byte = self.once(addr.wrapping_add(i), Width::U8, None)?;
                    out |= (byte & 0xff) << (8 * i);
                }
                Ok(out)
            }
        }
    }
}

impl IrHost for Host {
    fn read_slot(&mut self, slot: RegSlot) -> u128 {
        u128::from(self.slots[slot.0 as usize])
    }

    fn write_slot(&mut self, slot: RegSlot, value: u128) {
        self.slots[slot.0 as usize] = value as u64;
    }

    fn load(&mut self, mem: &MemOp, addr: u64) -> MemResult<u64> {
        self.access(mem, addr, None)
    }

    fn store(&mut self, mem: &MemOp, addr: u64, value: u64) -> MemResult {
        self.access(mem, addr, Some(value)).map(|_| ())
    }

    fn charge(&mut self, ticks: u64) {
        self.ticks += ticks;
    }

    fn insn_start(&mut self, _mark: &InsnStart) {}
}

// No software TLB here, so no fast path to publish: this host's accesses all
// take the call, which is the default and always correct.
#[cfg(feature = "jit")]
impl FastMem for Host {}

#[cfg(feature = "jit")]
impl StoreLog for Host {
    fn drain_dirty(&mut self, sink: &mut dyn FnMut(u64)) {
        self.dirty.drain_dirty(sink);
    }
}

// ---------------------------------------------------------------------------
// Through the block cache
// ---------------------------------------------------------------------------

/// What a cached or compiled run did, beside agreeing.
///
/// Every "did the mechanism actually run" number is separate from every "did
/// they agree" one, because a harness that stopped exercising a mechanism
/// would otherwise keep passing.
#[cfg(feature = "jit")]
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CachedRun {
    /// What the comparison found.
    pub verdict: Verdict,
    /// How many blocks executed.
    pub blocks: usize,
    /// How many guest instructions those blocks retired.
    pub insns_retired: usize,
    /// How many distinct blocks were translated.
    pub translated: u64,
    /// How many blocks were reached by following a patched exit.
    pub chained: u64,
    /// How many blocks a guest store invalidated.
    pub smc: u64,
    /// How many blocks executed as host code. Zero on the portable backend.
    pub compiled: u64,
}

/// The same comparison, driven through [`Dispatcher`] so the block cache,
/// block chaining and the self-modifying-code check are on the path.
///
/// # Errors
///
/// A [`Divergence`], as [`compare`].
#[cfg(feature = "jit")]
pub fn compare_cached(case: &Case, blocks: usize) -> Result<Verdict, Divergence> {
    Ok(cached(case, blocks, false)?.verdict)
}

/// The same, reporting what the runtime did.
///
/// # Errors
///
/// A [`Divergence`], as [`compare`].
#[cfg(feature = "jit")]
pub fn measure_cached(case: &Case, blocks: usize) -> Result<CachedRun, Divergence> {
    cached(case, blocks, false)
}

/// The same comparison with the host code generator attached.
///
/// # Errors
///
/// A [`Divergence`], as [`compare`].
#[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
#[cfg_attr(docsrs, doc(cfg(feature = "jit-x86")))]
pub fn compare_compiled(case: &Case, blocks: usize) -> Result<Verdict, Divergence> {
    Ok(cached(case, blocks, true)?.verdict)
}

/// The same, reporting what the runtime and the code generator did.
///
/// # Errors
///
/// A [`Divergence`], as [`compare`].
#[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
#[cfg_attr(docsrs, doc(cfg(feature = "jit-x86")))]
pub fn measure_compiled(case: &Case, blocks: usize) -> Result<CachedRun, Divergence> {
    cached(case, blocks, true)
}

/// The frontend half of the dispatcher's contract, over a case's memory.
#[cfg(feature = "jit")]
struct Lifter<'a> {
    case: &'a Case,
    world: World,
    space: Arc<AddressSpace>,
    attrs: MemAttrs,
    rejected: Option<String>,
}

#[cfg(feature = "jit")]
impl<H: ?Sized> Frontend<H> for Lifter<'_> {
    fn epoch(&mut self) -> Epoch {
        Epoch {
            topology: self.space.generation(),
            // Zero, and deliberately: these blocks are not keyed on the guest
            // MMU's generation — bare mode has none — so nothing here is stale
            // against it.
            translation: 0,
        }
    }

    fn key(&mut self) -> u64 {
        lift::key(&self.world, self.case.shape)
    }

    fn pc_slot(&self) -> RegSlot {
        PC
    }

    fn translate(&mut self, pc: u64) -> crate::core::error::Result<Translation> {
        let space = Arc::clone(&self.space);
        let attrs = self.attrs;
        // Read the *bytes in guest memory*, not the case's program, so a store
        // that rewrites the code page is visible to the next translation.
        let mut src = |addr: u64| space.read(addr, Width::U32, attrs).ok().map(|v| v as u32);
        let lifted = lift::lift(&self.world, pc, &mut src, lift::MAX_INSNS, self.case.shape)?;
        if self.rejected.is_none()
            && let Err(e) = verify(&lifted.block)
        {
            self.rejected = Some(format!("{e}"));
        }
        Ok(Translation {
            page: pc & !PAGE_MASK,
            insns: lifted.insns,
            block: lifted.block,
        })
    }
}

#[cfg(feature = "jit")]
fn dispatcher(compiled: bool) -> Dispatcher {
    let disp = Dispatcher::with_cache(BlockCache::with_capacity(256));
    #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
    if compiled && let Some(engine) = crate::jit::x86::Engine::new() {
        return disp.with_backend(engine);
    }
    let _ = compiled;
    disp
}

#[cfg(feature = "jit")]
#[allow(clippy::too_many_lines)]
fn cached(case: &Case, blocks: usize, compiled: bool) -> Result<CachedRun, Divergence> {
    assert!(
        case.program.len() * 4 <= DATA as usize,
        "the program must fit in the code page"
    );
    let world = case.world();
    let (subject_space, subject_ram) = machine(case);
    let mut front = Lifter {
        case,
        world,
        space: Arc::clone(&subject_space),
        attrs: MemAttrs::DEBUG.with_requester(case.cfg.requester),
        rejected: None,
    };
    let mut host = Host::new(case, subject_space);
    let mut disp = dispatcher(compiled);
    let run = disp
        .run(&mut front, &mut host, BASE, blocks)
        .map_err(|e| diverged(case, format!("the dispatcher refused the run: {e}")))?;
    if let Some(e) = front.rejected {
        return Err(diverged(
            case,
            format!("the frontend emitted a block the verifier rejects: {e}"),
        ));
    }
    disp.cache()
        .check()
        .map_err(|e| diverged(case, format!("the block cache is inconsistent: {e}")))?;

    let stats = disp.stats();
    let mut out = CachedRun {
        verdict: Verdict::Nothing,
        blocks: run.blocks,
        insns_retired: run.insns,
        translated: stats.translated,
        chained: stats.chained,
        smc: stats.smc,
        compiled: stats.compiled,
    };
    if run.blocks == 0 || run.insns == 0 {
        return Ok(out);
    }
    let faulted = matches!(run.stop, Stop::Fault(_));
    let fault_pc = match run.stop {
        Stop::Fault(f) => Some(f.pc),
        _ => None,
    };

    let (oracle_space, oracle_ram) = machine(case);
    let cpu = oracle(case, oracle_space);
    let want = run.insns + usize::from(faulted);
    let oracle_trapped = step_oracle(&cpu, want);
    if oracle_trapped != faulted {
        return Err(diverged(
            case,
            format!(
                "after {} retired instructions the interpreter {} and the run {}",
                run.insns,
                if oracle_trapped { "trapped" } else { "did not" },
                if faulted { "faulted" } else { "did not" },
            ),
        ));
    }

    let pc = fault_pc.unwrap_or(run.pc);
    let what = if faulted {
        "at the faulting instruction"
    } else {
        "at the end of the run"
    };
    state(case, &cpu, &host.slots, pc, host.ticks, what)?;
    memory(case, &oracle_ram, &subject_ram)?;
    out.verdict = if faulted {
        Verdict::Trapped { insns: run.insns }
    } else {
        Verdict::Agreed {
            insns: run.insns,
            ticks: host.ticks,
        }
    };
    Ok(out)
}

// ---------------------------------------------------------------------------
// The corpus
// ---------------------------------------------------------------------------

/// Sign-extend the low `bits` of `value`.
const fn sext(value: u32, bits: u32) -> i64 {
    isa::sext(value as u64, bits)
}

/// A register the synthesizer writes: `X5`–`X12`, so the seeded pointers in
/// `X1`–`X4` survive to be used as addresses.
const fn dest(fields: u32) -> u32 {
    5 + (fields % (SYNTH_REGS - 5))
}

/// A register the synthesizer reads: `X1`–`X12`, so a pointer can be an
/// arithmetic operand and a computed value can be an address.
const fn src(fields: u32) -> u32 {
    1 + (fields % (SYNTH_REGS - 1))
}

/// One of the seeded pointers, so a memory access lands somewhere mapped often
/// enough to measure the lifter rather than the fault path.
const fn ptr(fields: u32) -> u32 {
    1 + (fields % POINTERS)
}

/// How many encoding forms [`synthesize`] knows.
pub const FORMS: u32 = 48;

/// Build one A64 instruction from two arbitrary numbers.
///
/// Both arguments are reduced, so *every* pair of numbers encodes something
/// inside the lifted subset — which is what lets a fuzz target and a seeded
/// sweep share one generator. The split a caller should use is the one both
/// sibling harnesses use: the **high** bits of a random word pick the form and
/// the low bits pick the fields, so two forms drawn in sequence do not share
/// their operands.
///
/// A handful of forms deliberately encode something the architecture leaves
/// `UNDEFINED` — a bitmask immediate `DecodeBitMasks` refuses, a 32-bit
/// bitfield naming bit 40 — because "the frontend rejects exactly what the
/// interpreter rejects" is half of what is under test.
#[must_use]
#[allow(clippy::too_many_lines)]
pub fn synthesize(form: u32, fields: u32) -> u32 {
    let form = form % FORMS;
    let sf = (fields >> 20) & 1;
    let rd = dest(fields);
    let rn = src(fields >> 4);
    let rm = src(fields >> 8);
    let ra = src(fields >> 12);
    let cond = (fields >> 16) & 0xf;
    let base = ptr(fields >> 4);
    let rt = dest(fields);
    let rt2 = dest(fields >> 8);
    let width = if sf == 1 { 64 } else { 32 };

    // A shift amount that fits the operand, and an extend amount that fits the
    // three-bit field the architecture allows.
    let amount = (fields >> 10) % width;
    let shift = (fields >> 22) & 3;
    let imm12 = (fields >> 12) & 0xfff;
    let imm16 = (fields >> 4) & 0xffff;
    let immr = (fields >> 6) % width;
    let imms = (fields >> 12) % width;
    let nbit = sf;

    // A small signed displacement, in units of instructions, that stays inside
    // a generated program rather than branching into the data window.
    let disp = ((((fields >> 24) & 7) as i64) - 4) * 4;

    match form {
        // -- add/subtract, immediate -----------------------------------
        0 => addsub_imm(0x1100_0000, sf, rd, rn, imm12, (fields >> 23) & 1),
        1 => addsub_imm(0x3100_0000, sf, 31, rn, imm12, 0),
        2 => addsub_imm(0x5100_0000, sf, rd, rn, imm12, 0),
        3 => addsub_imm(0x7100_0000, sf, rd, rn, imm12, 0),

        // -- logical, immediate ----------------------------------------
        4 => log_imm(0x1200_0000, sf, rd, rn, nbit, immr, imms),
        5 => log_imm(0x3200_0000, sf, rd, rn, nbit, immr, imms),
        6 => log_imm(0x5200_0000, sf, rd, rn, nbit, immr, imms),
        7 => log_imm(0x7200_0000, sf, rd, rn, nbit, immr, imms),

        // -- move wide --------------------------------------------------
        8 => movewide(0x5280_0000, sf, rd, imm16, fields >> 26),
        9 => movewide(0x1280_0000, sf, rd, imm16, fields >> 26),
        10 => movewide(0x7280_0000, sf, rd, imm16, fields >> 26),

        // -- bitfield and extract ---------------------------------------
        11 => bitfield(0x1300_0000, sf, rd, rn, nbit, immr, imms),
        12 => bitfield(0x3300_0000, sf, rd, rn, nbit, immr, imms),
        13 => bitfield(0x5300_0000, sf, rd, rn, nbit, immr, imms),
        14 => (sf << 31) | 0x1380_0000 | (nbit << 22) | (rm << 16) | (imms << 10) | (rn << 5) | rd,

        // -- logical, shifted register ----------------------------------
        15 => shifted(0x0a00_0000, sf, rd, rn, rm, shift, amount),
        16 => shifted(0x0a20_0000, sf, rd, rn, rm, shift, amount),
        17 => shifted(0x2a00_0000, sf, rd, rn, rm, shift, amount),
        18 => shifted(0x2a20_0000, sf, rd, rn, rm, shift, amount),
        19 => shifted(0x4a00_0000, sf, rd, rn, rm, shift, amount),
        20 => shifted(0x6a00_0000, sf, rd, rn, rm, shift, amount),

        // -- add/subtract, shifted register ------------------------------
        21 => shifted(0x0b00_0000, sf, rd, rn, rm, shift, amount),
        22 => shifted(0x2b00_0000, sf, rd, rn, rm, shift, amount),
        23 => shifted(0x4b00_0000, sf, rd, rn, rm, shift, amount),
        24 => shifted(0x6b00_0000, sf, 31, rn, rm, shift, amount),

        // -- add/subtract, extended register -----------------------------
        25 => extended(
            0x0b20_0000,
            sf,
            rd,
            rn,
            rm,
            (fields >> 13) & 7,
            (fields >> 10) % 5,
        ),
        26 => extended(
            0x6b20_0000,
            sf,
            rd,
            rn,
            rm,
            (fields >> 13) & 7,
            (fields >> 10) % 5,
        ),

        // -- add/subtract with carry -------------------------------------
        27 => three(0x1a00_0000, sf, rd, rn, rm),
        28 => three(0x3a00_0000, sf, rd, rn, rm),
        29 => three(0x5a00_0000, sf, rd, rn, rm),
        30 => three(0x7a00_0000, sf, rd, rn, rm),

        // -- conditional --------------------------------------------------
        31 => (sf << 31) | 0x7a40_0000 | (rm << 16) | (cond << 12) | (rn << 5) | (fields & 0xf),
        32 => {
            (sf << 31)
                | 0x3a40_0800
                | (((fields >> 8) & 0x1f) << 16)
                | (cond << 12)
                | (rn << 5)
                | (fields & 0xf)
        }
        // All four conditional selects, chosen by two bits rather than two
        // forms: a mutation pass found `CSINC` decrementing survived a corpus
        // that generated only `CSEL` and `CSNEG`.
        33 => {
            (sf << 31)
                | 0x1a80_0000
                | (((fields >> 6) & 1) << 30)
                | (((fields >> 7) & 1) << 10)
                | (rm << 16)
                | (cond << 12)
                | (rn << 5)
                | rd
        }
        34 => (sf << 31) | 0x5a80_0400 | (rm << 16) | (cond << 12) | (rn << 5) | rd,

        // -- two-source and one-source -------------------------------------
        //
        // `UDIV` and `SDIV` both, because a mutation pass found that lowering
        // `SDIV` to an unsigned divide survived a corpus that only ever
        // generated the unsigned one.
        35 => three(0x1ac0_0800 | (((fields >> 6) & 1) << 10), sf, rd, rn, rm),
        36 => three(0x1ac0_2000 | (((fields >> 6) & 3) << 10), sf, rd, rn, rm),
        37 => (sf << 31) | 0x5ac0_1000 | (((fields >> 6) & 1) << 10) | (rn << 5) | rd,
        38 => (sf << 31) | 0x5ac0_0400 | (rn << 5) | rd,

        // -- three-source ---------------------------------------------------
        39 => (sf << 31) | 0x1b00_0000 | (rm << 16) | (ra << 10) | (rn << 5) | rd,
        40 => 0x9b20_0000 | (((fields >> 6) & 1) << 23) | (rm << 16) | (ra << 10) | (rn << 5) | rd,

        // -- loads and stores -----------------------------------------------
        41 => memory_form(fields, base, rt, rt2),

        // -- branches ---------------------------------------------------------
        42 => match (fields >> 6) & 3 {
            0 => 0x5400_0000 | (((disp >> 2) as u32 & 0x7ffff) << 5) | (cond & 0xf),
            1 => (sf << 31) | 0x3400_0000 | (((disp >> 2) as u32 & 0x7ffff) << 5) | rn,
            2 => {
                (sf << 31)
                    | 0x3600_0000
                    | (((fields >> 8) & 0x1f) << 19)
                    | (((disp >> 2) as u32 & 0x3fff) << 5)
                    | rn
            }
            _ => 0x1400_0000 | ((disp >> 2) as u32 & 0x03ff_ffff),
        },
        // -- PC-relative, the high multiplies, and the hint ---------------------
        43 => match (fields >> 6) & 3 {
            0 => 0x1000_0000 | (((fields >> 8) & 3) << 29) | (((fields >> 10) & 0x7ffff) << 5) | rd,
            1 => 0xd503_201f,
            2 => 0x9b40_7c00 | (((fields >> 8) & 1) << 23) | (rm << 16) | (rn << 5) | rd,
            _ => (sf << 31) | 0x1b00_8000 | (rm << 16) | (ra << 10) | (rn << 5) | rd,
        },

        // -- `ADRP`, which is the only instruction whose result depends on the
        //    *page* the block was lifted at rather than on the instruction's
        //    own address.
        44 => 0x9000_0000 | (((fields >> 8) & 3) << 29) | (((fields >> 10) & 0x7ffff) << 5) | rd,

        // -- the byte-reversal family, whose lane width is in the mnemonic
        //    rather than in a field.
        45 => match (fields >> 6) & 3 {
            0 => 0x5ac0_0800 | (rn << 5) | rd,
            1 => 0xdac0_0800 | (rn << 5) | rd,
            2 => 0xdac0_0c00 | (rn << 5) | rd,
            _ => 0xdac0_0400 | (rn << 5) | rd,
        },

        // -- the stack pointer, which register 31 spells only in the formats
        //    DDI 0487 C1.2.5 lists and which is `XZR` everywhere else. Nothing
        //    above reaches it, so a corpus without this arm would never test
        //    the one slot this frontend numbers that is not a register.
        //
        //    The immediates are small so a run of these walks the pointer
        //    rather than throwing it out of the mapping on the first one.
        46 => match (fields >> 6) & 3 {
            0 => 0x9100_0000 | ((imm12 & 0xff) << 10) | (31 << 5) | 31,
            1 => 0x9100_0000 | ((imm12 & 0xff) << 10) | (31 << 5) | rd,
            2 => 0xd100_0000 | ((imm12 & 0xff) << 10) | (31 << 5) | 31,
            _ => 0xf940_0000 | (((fields >> 18) & 0x3f) << 10) | (31 << 5) | rt,
        },

        // -- the transfers of control a trace cannot merge, and the literal
        //    load, whose address is a constant the *lifter* computes.
        _ => match (fields >> 6) & 3 {
            0 => 0x9400_0000 | ((disp >> 2) as u32 & 0x03ff_ffff),
            // `BR` and `BLR`, and `BLR X30` among them: the link is bound
            // *after* the target is read, and the only encoding that can tell
            // the two orders apart is the one whose target register is the
            // link register.
            1 => {
                let reg = if (fields >> 9) & 1 == 0 {
                    ptr(fields >> 10)
                } else {
                    30
                };
                0xd61f_0000 | (((fields >> 8) & 1) << 21) | (reg << 5)
            }
            2 => 0xd65f_03c0,
            _ => 0x5800_0000 | (((disp >> 2) as u32 & 0x7ffff) << 5) | rt,
        },
    }
}

const fn addsub_imm(op: u32, sf: u32, rd: u32, rn: u32, imm: u32, shift: u32) -> u32 {
    (sf << 31) | op | (shift << 22) | ((imm & 0xfff) << 10) | (rn << 5) | rd
}

const fn log_imm(op: u32, sf: u32, rd: u32, rn: u32, n: u32, immr: u32, imms: u32) -> u32 {
    (sf << 31) | op | (n << 22) | ((immr & 0x3f) << 16) | ((imms & 0x3f) << 10) | (rn << 5) | rd
}

const fn movewide(op: u32, sf: u32, rd: u32, imm: u32, hw: u32) -> u32 {
    let hw = if sf == 1 { hw & 3 } else { hw & 1 };
    (sf << 31) | op | (hw << 21) | ((imm & 0xffff) << 5) | rd
}

const fn bitfield(op: u32, sf: u32, rd: u32, rn: u32, n: u32, immr: u32, imms: u32) -> u32 {
    (sf << 31) | op | (n << 22) | ((immr & 0x3f) << 16) | ((imms & 0x3f) << 10) | (rn << 5) | rd
}

const fn shifted(op: u32, sf: u32, rd: u32, rn: u32, rm: u32, shift: u32, amount: u32) -> u32 {
    (sf << 31) | op | (shift << 22) | (rm << 16) | ((amount & 0x3f) << 10) | (rn << 5) | rd
}

const fn extended(op: u32, sf: u32, rd: u32, rn: u32, rm: u32, option: u32, amount: u32) -> u32 {
    (sf << 31) | op | (rm << 16) | (option << 13) | (amount << 10) | (rn << 5) | rd
}

const fn three(op: u32, sf: u32, rd: u32, rn: u32, rm: u32) -> u32 {
    (sf << 31) | op | (rm << 16) | (rn << 5) | rd
}

/// One of the load and store forms, over a seeded pointer.
fn memory_form(fields: u32, base: u32, rt: u32, rt2: u32) -> u32 {
    let size = (fields >> 26) & 3;
    let opc = (fields >> 28) & 1;
    // Small, signed, and often not a multiple of the access width, so the
    // split path is reachable.
    let simm9 = ((((fields >> 18) & 0x1f) as i64) - 16) as u32 & 0x1ff;
    let uimm12 = (fields >> 18) & 0x3f;
    match (fields >> 6) & 7 {
        // Unsigned scaled offset.
        0 => 0x3900_0000 | (size << 30) | (opc << 22) | (uimm12 << 10) | (base << 5) | rt,
        // Unscaled.
        1 => 0x3800_0000 | (size << 30) | (opc << 22) | (simm9 << 12) | (base << 5) | rt,
        // Post-indexed.
        2 => 0x3800_0400 | (size << 30) | (opc << 22) | (simm9 << 12) | (base << 5) | rt,
        // Pre-indexed.
        3 => 0x3800_0c00 | (size << 30) | (opc << 22) | (simm9 << 12) | (base << 5) | rt,
        // Register offset, with `option<1>` forced set as the encoding requires.
        4 => {
            let option = 2 | ((fields >> 12) & 5);
            0x3820_0800
                | (size << 30)
                | (opc << 22)
                | (ptr(fields >> 14) << 16)
                | (option << 13)
                | (((fields >> 16) & 1) << 12)
                | (base << 5)
                | rt
        }
        // Signed loads, which have their own `opc` values — `0b10` extends
        // to sixty-four bits and `0b11` to thirty-two, and `0b11` at a `size`
        // of two or three is unallocated, which both engines must refuse.
        5 => {
            let opc = 2 + ((fields >> 24) & 1);
            0x3900_0000 | (size << 30) | (opc << 22) | (uimm12 << 10) | (base << 5) | rt
        }
        // A pair, offset form.
        6 => {
            let pair_opc = (fields >> 28) & 3;
            let load = (fields >> 12) & 1;
            let imm7 = (((fields >> 18) & 7) as i64 - 4) as u32 & 0x7f;
            0x2900_0000
                | (pair_opc << 30)
                | (load << 22)
                | (imm7 << 15)
                | (rt2 << 10)
                | (base << 5)
                | rt
        }
        // A pair, pre-indexed.
        _ => {
            let pair_opc = (fields >> 28) & 3;
            let load = (fields >> 12) & 1;
            let imm7 = (((fields >> 18) & 7) as i64 - 4) as u32 & 0x7f;
            0x2980_0000
                | (pair_opc << 30)
                | (load << 22)
                | (imm7 << 15)
                | (rt2 << 10)
                | (base << 5)
                | rt
        }
    }
}

// The displacement helper is only used through `synthesize`; naming it keeps
// the `as` casts in one place rather than four.
const _: fn(u32, u32) -> i64 = sext;

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    /// `movz x5, #1` — inside the subset, and it retires.
    const MOVZ: u32 = 0xd280_0025;
    /// `svc #0` — outside it.
    const SVC: u32 = 0xd400_0001;

    /// Every column [`state`] compares, and a proof that it compares it.
    ///
    /// **A harness's own comparison is the one thing nothing else tests.** A
    /// mutation pass over this file found that switching off the flag, stack
    /// pointer, tick and memory checks one at a time left every test in the
    /// tree passing: the lifter is correct, so no case in a two-thousand-case
    /// corpus ever produces a difference in those columns, and a check that is
    /// never reached is a check that can be deleted without anyone noticing.
    ///
    /// So the difference is manufactured here. A correct run's state is taken
    /// and one column is corrupted, and [`state`] must name that column. It is
    /// the cheapest possible test and it is the only thing standing between
    /// this harness and quietly comparing nothing.
    #[test]
    fn every_column_this_harness_compares_is_one_it_would_report() {
        // A program that binds a register, the stack pointer and all four
        // flags, and writes memory, so every column has something in it.
        let case = Case::seeded(vec![
            MOVZ,        // movz x5, #1
            0x9100_43ff, // add  sp, sp, #16
            0xeb06_00bf, // cmp  x5, x6
            0xf900_0045, // str  x5, [x2]
            SVC,
        ]);
        let world = case.world();
        let (space, ram) = machine(&case);
        let program = case.program.clone();
        let mut src = |addr: u64| {
            let off = addr.checked_sub(BASE)? / 4;
            program.get(off as usize).copied()
        };
        let lifted = lift::lift(&world, BASE, &mut src, lift::MAX_INSNS, case.shape)
            .expect("this world lifts");
        let mut host = Host::new(&case, space);
        let mut interp = Interp::new();
        interp
            .run(&lifted.block, &mut host)
            .expect("the block runs");
        let retired = interp.boundaries().saturating_sub(1) as usize;

        let (oracle_space, oracle_ram) = machine(&case);
        let cpu = oracle(&case, oracle_space);
        assert!(!step_oracle(&cpu, retired), "the program does not fault");
        let pc = host.slots[PC.0 as usize];
        // The uncorrupted comparison passes, which is what makes each failure
        // below attributable to the one column it corrupts.
        state(&case, &cpu, &host.slots, pc, host.ticks, "control")
            .expect("the harness agrees with itself");
        memory(&case, &oracle_ram, &ram).expect("memory agrees");

        let cases: [(&str, &str); 8] = [
            ("x5", "x5"),
            ("sp", "sp"),
            ("N", "PSTATE.NZCV"),
            ("Z", "PSTATE.NZCV"),
            ("C", "PSTATE.NZCV"),
            ("V", "PSTATE.NZCV"),
            ("pc", "the pc"),
            ("ticks", "the cycle counter"),
        ];
        for (which, expect) in cases {
            let mut slots = host.slots;
            let mut pc = pc;
            let mut ticks = host.ticks;
            match which {
                "x5" => slots[x_slot(5).0 as usize] ^= 1,
                "sp" => slots[SP.0 as usize] ^= 1,
                "N" => slots[lift::N.0 as usize] ^= 1,
                "Z" => slots[lift::Z.0 as usize] ^= 1,
                "C" => slots[lift::C.0 as usize] ^= 1,
                "V" => slots[lift::V.0 as usize] ^= 1,
                "pc" => pc ^= 4,
                _ => ticks ^= 1,
            }
            let err = state(&case, &cpu, &slots, pc, ticks, "under test")
                .expect_err("a corrupted column must be reported");
            assert!(
                err.what.contains(expect),
                "corrupting {which} was reported as `{}`, which does not name {expect}",
                err.what
            );
        }

        // And the same for guest memory, which `state` does not cover.
        let byte = ram.read_u8(DATA).expect("in range");
        ram.write_u8(DATA, byte ^ 0xff).expect("in range");
        let err =
            memory(&case, &oracle_ram, &ram).expect_err("a byte that differs must be reported");
        assert!(err.what.contains("guest memory"), "{}", err.what);
    }

    #[test]
    fn a_program_of_arithmetic_agrees_with_the_interpreter() {
        let case = Case::new(vec![MOVZ, 0x9100_04a5, 0xd280_0046, SVC]);
        assert_eq!(
            compare(&case).expect("agreement"),
            Verdict::Agreed { insns: 3, ticks: 3 },
            "three instructions, one fetch tick each"
        );
    }

    #[test]
    fn an_instruction_outside_the_subset_is_nothing_to_compare() {
        let case = Case::new(vec![SVC]);
        assert_eq!(compare(&case).expect("no divergence"), Verdict::Nothing);
    }

    #[test]
    fn an_aligned_access_costs_one_tick_and_a_split_one_costs_eight() {
        // `str x5, [x2]` — x2 is aligned, so one bus cycle on top of the fetch.
        let aligned = Case::seeded(vec![0xf900_0045, SVC]);
        assert_eq!(
            compare(&aligned).expect("agreement"),
            Verdict::Agreed { insns: 1, ticks: 2 }
        );
        // `str x5, [x1]` — x1 is deliberately not eight-byte aligned, so the
        // access splits into eight.
        let split = Case::seeded(vec![0xf900_0025, SVC]);
        assert_eq!(
            compare(&split).expect("agreement"),
            Verdict::Agreed { insns: 1, ticks: 9 }
        );
    }

    #[test]
    fn a_core_that_checks_alignment_traps_where_one_that_does_not_splits() {
        let case = Case::seeded(vec![0xf900_0025, SVC]).strict();
        assert_eq!(
            compare(&case).expect("agreement"),
            Verdict::Trapped { insns: 0 }
        );
    }

    #[test]
    fn a_fault_in_the_middle_of_a_block_reports_the_interpreters_exact_state() {
        // Two retiring instructions, then a store through a register holding
        // an address outside the mapping.
        let case = Case::seeded(vec![MOVZ, 0x9100_04a5, 0xf900_00a5, SVC]).with_reg(5, 0);
        assert_eq!(
            compare(&case).expect("agreement"),
            Verdict::Trapped { insns: 2 }
        );
    }

    #[test]
    fn every_flag_a_subtract_writes_is_compared() {
        // `subs x5, x6, x7` over operands that set each flag in turn.
        for (a, b) in [
            (0u64, 0u64),
            (1, 1),
            (0, 1),
            (u64::MAX, 1),
            (1u64 << 63, 1),
            (0x7fff_ffff_ffff_ffff, u64::MAX),
        ] {
            let case = Case::new(vec![0xeb07_00c5, SVC])
                .with_reg(6, a)
                .with_reg(7, b);
            assert!(
                matches!(compare(&case), Ok(Verdict::Agreed { .. })),
                "subs {a:#x}, {b:#x}: {:?}",
                compare(&case)
            );
        }
    }

    #[test]
    fn blr_reads_its_target_before_it_binds_the_link() {
        // `blr x30` is the one encoding that can tell the two orders apart,
        // and a generated corpus reaches it about once in four thousand
        // instructions — which is why it is written out here as well.
        let case = Case::seeded(vec![0xd63f_03c0, SVC]).with_reg(30, BASE + 0x40);
        assert!(
            matches!(compare(&case), Ok(Verdict::Agreed { .. })),
            "{:?}",
            compare(&case)
        );
    }

    #[test]
    fn a_signed_narrow_load_stops_at_thirty_two_bits() {
        // `LDRSB Wt` sign-extends within `Wt` and then zero-extends into `Xt`,
        // where `LDRSB Xt` sign-extends the whole way. The difference is
        // invisible unless the byte has its top bit set, which is why the data
        // window this harness builds is a pattern rather than zeroes.
        for word in [0x39c0_0845u32, 0x3980_0845] {
            let case = Case::seeded(vec![word, SVC]);
            assert!(
                matches!(compare(&case), Ok(Verdict::Agreed { .. })),
                "{word:#010x}: {:?}",
                compare(&case)
            );
        }
    }

    #[test]
    fn a_signed_divide_is_not_an_unsigned_one() {
        // `sdiv x5, x6, x7` over a negative dividend, which is the only shape
        // that separates the two.
        let case = Case::new(vec![0x9ac7_0cc5, SVC])
            .with_reg(6, (-9i64) as u64)
            .with_reg(7, 2);
        assert!(
            matches!(compare(&case), Ok(Verdict::Agreed { .. })),
            "{:?}",
            compare(&case)
        );
    }

    #[test]
    fn a_trace_merges_a_backward_branch_and_still_agrees() {
        // `add x5, x5, #1` then `b .-4`: the trace unrolls to the limit.
        let case = Case::new(vec![0x9100_04a5, 0x17ff_ffff]);
        let verdict = compare(&case).expect("agreement");
        assert_eq!(
            verdict,
            Verdict::Agreed {
                insns: lift::MAX_INSNS,
                ticks: lift::MAX_INSNS as u64,
            }
        );
    }

    #[test]
    fn every_shape_agrees_on_the_same_program() {
        let program = vec![MOVZ, 0x9100_04a5, 0xf940_0046, 0x9100_04c6, SVC];
        for shape in [Shape::BasicBlock, Shape::Extended, Shape::Trace] {
            let case = Case::seeded(program.clone()).with_shape(shape);
            assert!(
                matches!(compare(&case), Ok(Verdict::Agreed { .. })),
                "{shape:?}: {:?}",
                compare(&case)
            );
        }
    }

    #[test]
    fn the_generator_only_ever_produces_words_in_the_subset_or_rejected_ones() {
        // Not a claim about validity — some forms are deliberately
        // `UNDEFINED`. The claim is that nothing it produces makes the harness
        // itself fail, which is what a fuzz target depends on.
        let mut agreed = 0usize;
        let mut trapped = 0usize;
        let mut nothing = 0usize;
        for i in 0..2000u32 {
            let word = synthesize(i, i.wrapping_mul(2_654_435_761));
            let case = Case::seeded(vec![word, SVC]);
            match compare(&case).expect("no divergence") {
                Verdict::Agreed { .. } => agreed += 1,
                Verdict::Trapped { .. } => trapped += 1,
                Verdict::Nothing => nothing += 1,
            }
        }
        assert!(
            agreed > 500,
            "agreed {agreed} trapped {trapped} nothing {nothing}"
        );
    }

    #[cfg(feature = "jit")]
    mod cached {
        use super::*;

        #[test]
        fn the_same_program_agrees_through_the_block_cache() {
            let case = Case::seeded(vec![MOVZ, 0x9100_04a5, 0xf940_0046, SVC]);
            assert!(matches!(
                compare_cached(&case, 8).expect("agreement"),
                Verdict::Agreed { .. }
            ));
        }

        #[test]
        fn a_loop_chains_rather_than_looking_every_block_up() {
            // `add x5, x5, #1`, `str x5, [x4]`, `b .-8`: the store ends each
            // block, so the loop is a chain of short blocks.
            let case = Case::seeded(vec![0x9100_04a5, 0xf900_0085, 0x17ff_fffe]);
            let run = measure_cached(&case, 32).expect("agreement");
            assert!(run.blocks > 8, "{run:?}");
            assert!(run.chained > 0, "the exit was never patched: {run:?}");
            // **Two** translations, not one, and the reason is the store rule
            // rather than a cache that is not working: the store ends its
            // block, so the loop has two entry PCs — the top, and the `b` the
            // first block exits to — and the `b` is merged into the second,
            // which brings it back to the top. Both are then served from the
            // cache for the rest of the run, which is what `chained` says.
            assert_eq!(run.translated, 2, "two entries, served over and over");
        }

        #[test]
        fn a_store_into_a_running_blocks_own_page_invalidates_it() {
            // `str x5, [x2]` where x2 points at the code page: the block the
            // store is in is thrown away, so the next pass re-lifts.
            let case = Case::seeded(vec![0xf900_0045, 0xd503_201f, SVC]).with_reg(2, BASE + 0x40);
            let run = measure_cached(&case, 8).expect("agreement");
            assert!(
                run.smc > 0,
                "the store did not invalidate anything: {run:?}"
            );
        }
    }
}
