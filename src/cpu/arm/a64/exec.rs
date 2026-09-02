//! The A64 interpreter.
//!
//! # One cycle is one bus access
//!
//! Arm does not architecturally define instruction timing — it is a property
//! of a particular implementation's pipeline, not of the ISA — so there is no
//! cycle table here and no counter that ticks independently of the bus. What
//! this interpreter counts is **accesses**: the instruction fetch is one, each
//! translation-table descriptor read during a walk is one, and each load or
//! store is one. That is a fact about the machine being modelled rather than
//! an invention, and it is the accounting `ROADMAP.md` §6 asks for.
//!
//! # Where the interpreter dispatches on a format rather than an operation
//!
//! The load/store families are dispatched by [`isa::Fmt`]: the addressing mode
//! is the format, and what the access *does* comes from
//! [`isa::ls_access`] reading the `size` and `opc` fields. Everything else
//! dispatches on [`isa::Op`]. See `isa.rs` for why the table still names every
//! instruction.
//!
//! # Sources
//!
//! *Arm Architecture Reference Manual for A-profile architecture* (DDI 0487).
//! Citations sit next to the rules they justify — the `SP`/`XZR` distinction,
//! the flag rules, the exception-vector offsets, the `SPSR_EL1` layout, the
//! interrupt-masking rule at a lower exception level, and the divide-by-zero
//! result. No emulator source of any licence was consulted (`ROADMAP.md` §1).

use crate::core::exec::{Access as ExitAccess, Exit, ExitMask, ExitReason};
use crate::core::space::{AddressSpace, MemAttrs};
use crate::core::value::Width;

use super::isa::{self, Fmt, LsAccess, Nzcv, Op, ShiftKind};
use super::mmu::{self, Access, Tlb};
use super::sysreg::{self, El, SysReg, SysRegs, VectorKind, daif, ec, sctlr};
use super::{Config, Lines};

/// An exception the current instruction raised.
///
/// Carries the syndrome the handler will read, because that value is decided
/// where the fault happens — a data abort reports the faulting address and the
/// direction, an `SVC` reports its immediate — and reconstructing it later is
/// how the two drift apart.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct Trap {
    /// `ESR_ELx.EC`.
    ec: u64,
    /// `ESR_ELx.ISS`.
    iss: u64,
    /// The value for `FAR_EL1`, when the exception has a faulting address.
    far: Option<u64>,
    /// Whether the preferred return address is the *next* instruction.
    ///
    /// DDI 0487 D1.10.1: an `SVC` returns past itself, while a fault
    /// re-executes the instruction that took it. `BRK` is the third case and
    /// returns *to* itself, which is what makes a debugger able to step past
    /// it deliberately.
    advance: bool,
}

impl Trap {
    /// An `UNDEFINED` instruction: exception class "unknown reason", no
    /// syndrome, and a return that re-executes.
    const fn undefined() -> Trap {
        Trap {
            ec: ec::UNKNOWN,
            iss: 0,
            far: None,
            advance: false,
        }
    }
}

/// The architectural state of one core.
///
/// Split from the device wrapper because the interrupt *lines* live outside
/// the execution lock, and because the TLB beside it is derived state a
/// snapshot must not carry.
#[derive(Debug, Clone)]
pub(super) struct State {
    /// `X0`-`X30`. `X31` is not a register: it is `XZR` or `SP` depending on
    /// the encoding, which is why the file has 31 entries rather than 32 —
    /// a 32nd slot is exactly the place a stray `SP` write would land
    /// unnoticed.
    pub x: [u64; 31],
    /// The program counter.
    pub pc: u64,
    /// `PSTATE` and the system registers.
    pub sys: SysRegs,
    /// The address the exclusive monitor is watching, in 16-byte granules.
    pub exclusive: Option<u64>,
    /// Bus accesses since reset.
    pub cycles: u64,
    /// Cycles already executed past the last budget, owed to the next one.
    pub debt: u64,
    /// Whether a `WFI` is stalling the core.
    pub wfi: bool,
    /// How many accesses the address space refused.
    pub faults: u64,
}

impl State {
    /// The reset state for a given configuration.
    pub(super) fn new(cfg: &Config) -> State {
        State {
            x: [0; 31],
            pc: cfg.reset_vector,
            sys: SysRegs::new(),
            exclusive: None,
            cycles: 0,
            debt: 0,
            wfi: false,
            faults: 0,
        }
    }
}

/// Physical memory as the translation-table walker sees it.
///
/// A separate borrow of the address space, because the walker runs while
/// [`Exec`] holds its own `&mut` on the state and the two must not alias.
struct Walker<'a> {
    space: &'a AddressSpace,
    attrs: MemAttrs,
    accesses: u64,
    refused: bool,
}

impl mmu::ReadDescriptor for Walker<'_> {
    fn read_descriptor(&mut self, addr: u64) -> Option<u64> {
        self.accesses += 1;
        match self.space.read(addr, Width::U64, self.attrs) {
            Ok(v) => Some(v),
            Err(_) => {
                self.refused = true;
                None
            }
        }
    }
}

/// Resolve an address the way a debugger asks it: no TLB, no permission
/// check, no cycles, and every descriptor read carrying `MemAttrs::DEBUG`.
///
/// A free function rather than a method on [`Exec`] because there is no step
/// in progress: a monitor listing or a gdb `m` packet arrives between
/// instructions.
pub(super) fn debug_translate(
    st: &State,
    space: &AddressSpace,
    cfg: &Config,
    va: u64,
) -> Option<u64> {
    let mut walker = Walker {
        space,
        attrs: MemAttrs::DEBUG.with_requester(cfg.requester),
        accesses: 0,
        refused: false,
    };
    mmu::translate_debug(&st.sys, &mut walker, va)
}

/// One step's worth of execution, borrowing everything it needs.
pub(super) struct Exec<'a> {
    st: &'a mut State,
    tlb: &'a mut Tlb,
    space: &'a AddressSpace,
    cfg: &'a Config,
    lines: &'a Lines,
    /// Which architectural traps leave the core instead of vectoring into the
    /// guest (`core::exec`). Empty for a level-1 machine.
    exits: ExitMask,
    /// Set instead of vectoring, when a trap is named in `exits`.
    exit: Option<Exit>,
    attrs: MemAttrs,
    /// Cycles charged by this step.
    used: u64,
    /// Where execution continues, unless a branch overrides it.
    next_pc: u64,
    /// The address of the instruction being executed.
    this_pc: u64,
}

impl<'a> Exec<'a> {
    /// Borrow a core for one step.
    pub(super) fn new(
        st: &'a mut State,
        tlb: &'a mut Tlb,
        space: &'a AddressSpace,
        cfg: &'a Config,
        lines: &'a Lines,
        exits: ExitMask,
    ) -> Exec<'a> {
        let attrs = MemAttrs::DEFAULT.with_requester(cfg.requester);
        let this_pc = st.pc;
        Exec {
            st,
            tlb,
            space,
            cfg,
            lines,
            exits,
            exit: None,
            attrs,
            used: 0,
            next_pc: this_pc,
            this_pc,
        }
    }

    /// Execute one instruction, one exception entry, or one stalled `WFI`.
    ///
    /// Returns the bus accesses charged, which is never zero: a caller can
    /// always make progress, and a stalled core is visible through
    /// `State::wfi` rather than through a zero return.
    pub(super) fn step(&mut self) -> u64 {
        if let Some(kind) = self.pending_interrupt() {
            let pc = self.st.pc;
            self.enter_exception(kind, None, None, pc);
            self.st.pc = self.next_pc;
            return self.used.max(1);
        }
        if self.st.wfi {
            // The stall ends when an interrupt becomes pending, whether or not
            // it is unmasked: DDI 0487 D1 makes `WFI` wake on a *WFI wake-up
            // event*, and a pending interrupt is one even when `PSTATE.I`
            // would stop it being taken.
            if self.lines.pending() == 0 {
                self.charge();
                return self.used;
            }
            self.st.wfi = false;
        }

        self.this_pc = self.st.pc;
        self.next_pc = self.st.pc.wrapping_add(4);
        match self.execute() {
            Ok(()) => self.st.pc = self.next_pc,
            Err(trap) => match self.exit_for(&trap) {
                Some(exit) => {
                    self.st.pc = exit.resume_pc();
                    self.exit = Some(exit);
                }
                None => {
                    let ret = if trap.advance {
                        self.this_pc.wrapping_add(4)
                    } else {
                        self.this_pc
                    };
                    self.enter_exception(
                        VectorKind::Synchronous,
                        Some((trap.ec, trap.iss)),
                        trap.far,
                        ret,
                    );
                    self.st.pc = self.next_pc;
                }
            },
        }
        self.used.max(1)
    }

    /// Take the exit this step produced, if it produced one.
    pub(super) fn take_exit(&mut self) -> Option<Exit> {
        self.exit.take()
    }

    /// Whether `trap` should leave the core rather than vector into the guest,
    /// and as what.
    fn exit_for(&self, trap: &Trap) -> Option<Exit> {
        let (reason, access) = match trap.ec {
            ec::SVC64 => (ExitReason::SYSCALL, ExitAccess::None),
            ec::BRK64 => (ExitReason::BREAKPOINT, ExitAccess::None),
            ec::IABT_LOWER | ec::IABT_SAME | ec::PC_ALIGN => {
                (ExitReason::FAULT, ExitAccess::Execute)
            }
            ec::DABT_LOWER | ec::DABT_SAME => {
                // ISS bit 6 is WnR: set for a write.
                if trap.iss & (1 << 6) != 0 {
                    (ExitReason::FAULT, ExitAccess::Write)
                } else {
                    (ExitReason::FAULT, ExitAccess::Read)
                }
            }
            ec::UNKNOWN => (ExitReason::FAULT, ExitAccess::None),
            _ => return None,
        };
        if !self.exits.contains(reason) {
            return None;
        }
        // Every A64 instruction is four bytes, which is what makes
        // `resume_pc` unambiguous here where it is not on a variable-length
        // architecture.
        let exit = Exit::new(reason, self.this_pc, 4).with_detail((trap.ec << 26) | trap.iss);
        match (access, trap.far) {
            (ExitAccess::None, _) | (_, None) => Some(exit),
            (_, Some(far)) => Some(exit.with_access(far, access)),
        }
    }

    // -----------------------------------------------------------------
    // The clock: one access, one cycle
    // -----------------------------------------------------------------

    /// Charge one bus access.
    #[inline]
    fn charge(&mut self) {
        self.used += 1;
        self.st.cycles = self.st.cycles.wrapping_add(1);
    }

    // -----------------------------------------------------------------
    // Exceptions
    // -----------------------------------------------------------------

    /// The interrupt to take now, if any.
    ///
    /// DDI 0487 D1.3: `PSTATE.{I,F}` mask an interrupt only when its target
    /// exception level is the level already executing. An IRQ routed to EL1
    /// and taken from EL0 is **not** maskable — getting that wrong gives a
    /// core where a userspace process can never be preempted, and the bug
    /// looks like a scheduler problem rather than a CPU one.
    fn pending_interrupt(&self) -> Option<VectorKind> {
        let pending = self.lines.pending();
        let masked_here = self.st.sys.el == El::El1;
        // FIQ before IRQ: the architecture leaves the order to the
        // implementation when both are pending, and every Arm core there has
        // ever been takes the fast interrupt first.
        if pending & Lines::FIQ != 0 && (!masked_here || self.st.sys.daif & daif::F == 0) {
            return Some(VectorKind::Fiq);
        }
        if pending & Lines::IRQ != 0 && (!masked_here || self.st.sys.daif & daif::I == 0) {
            return Some(VectorKind::Irq);
        }
        None
    }

    /// Take an exception to EL1.
    ///
    /// DDI 0487 D1.10.2 fixes the vector address: `VBAR_EL1`, plus `0x000` for
    /// a same-level exception taken with `SP_EL0` selected, `0x200` for the
    /// same level with `SP_ELx`, and `0x400` from a lower level in AArch64
    /// state — then `0x00`/`0x80`/`0x100`/`0x180` for synchronous, IRQ, FIQ
    /// and SError within the group.
    fn enter_exception(
        &mut self,
        kind: VectorKind,
        syndrome: Option<(u64, u64)>,
        far: Option<u64>,
        ret: u64,
    ) {
        let group = if self.st.sys.el == El::El0 {
            0x400
        } else if self.st.sys.spsel {
            0x200
        } else {
            0x000
        };
        self.st.sys.spsr_el1 = self.st.sys.spsr();
        self.st.sys.elr_el1 = ret;
        if let Some((class, iss)) = syndrome {
            // IL is set for a 32-bit instruction, and every A64 instruction
            // is one.
            self.st.sys.esr_el1 = (class << 26) | (1 << 25) | (iss & 0x01ff_ffff);
        }
        if let Some(addr) = far {
            self.st.sys.far_el1 = addr;
        }
        // The exception is taken to EL1 with SP_EL1 selected and every
        // asynchronous exception masked.
        self.st.sys.el = El::El1;
        self.st.sys.spsel = true;
        self.st.sys.daif |= daif::ALL;
        // Taking an exception clears the local exclusive monitor, so a
        // load-exclusive interrupted before its store-exclusive fails rather
        // than succeeding against a context that has changed underneath it.
        self.st.exclusive = None;
        self.next_pc = self
            .st
            .sys
            .vbar_el1
            .wrapping_add(group)
            .wrapping_add(kind.offset());
    }

    /// The exception class a data abort takes, which depends on whether the
    /// level changed.
    fn dabt_class(&self) -> u64 {
        if self.st.sys.el == El::El0 {
            ec::DABT_LOWER
        } else {
            ec::DABT_SAME
        }
    }

    /// The exception class an instruction abort takes.
    fn iabt_class(&self) -> u64 {
        if self.st.sys.el == El::El0 {
            ec::IABT_LOWER
        } else {
            ec::IABT_SAME
        }
    }

    /// Build a data-abort trap.
    fn data_abort(&self, va: u64, fault: mmu::Fault, kind: Access) -> Trap {
        // ISS for an abort without instruction syndrome: ISV clear, WnR at
        // bit 6, DFSC in bits 5:0.
        let mut iss = fault.dfsc();
        if kind.is_write() {
            iss |= 1 << 6;
        }
        Trap {
            ec: self.dabt_class(),
            iss,
            far: Some(va),
            advance: false,
        }
    }

    /// Build an instruction-abort trap.
    fn insn_abort(&self, va: u64, fault: mmu::Fault) -> Trap {
        Trap {
            ec: self.iabt_class(),
            iss: fault.dfsc(),
            far: Some(va),
            advance: false,
        }
    }

    // -----------------------------------------------------------------
    // Registers
    // -----------------------------------------------------------------

    /// Read register `idx` as a `width`-bit value.
    ///
    /// DDI 0487 C1.2.5: register 31 is the zero register in most encodings and
    /// the stack pointer in a handful, and `is_sp` is the format's answer to
    /// which — see [`Fmt::rd_is_sp`] and [`Fmt::rn_is_sp`].
    #[inline]
    fn read_reg(&self, idx: u32, width: u32, is_sp: bool) -> u64 {
        let value = if idx == 31 {
            if is_sp { self.st.sys.sp() } else { 0 }
        } else {
            self.st.x[idx as usize]
        };
        value & isa::ones(width)
    }

    /// Write register `idx` with a `width`-bit value.
    ///
    /// A 32-bit result zero-extends into the 64-bit register, which is the
    /// rule that makes `W`-form arithmetic well defined; a write to `XZR` is
    /// discarded.
    #[inline]
    fn write_reg(&mut self, idx: u32, width: u32, is_sp: bool, value: u64) {
        let value = value & isa::ones(width);
        if idx == 31 {
            if is_sp {
                self.st.sys.set_sp(value);
            }
        } else {
            self.st.x[idx as usize] = value;
        }
    }

    // -----------------------------------------------------------------
    // Memory
    // -----------------------------------------------------------------

    /// Translate one virtual address, consulting and filling the TLB.
    fn translate(&mut self, va: u64, kind: Access) -> Result<u64, mmu::Fault> {
        if !self.st.sys.mmu_enabled() {
            return Ok(va);
        }
        let vpn = va >> mmu::PAGE_BITS;
        let el = self.st.sys.el;
        let generation = self.st.sys.translation_gen;
        // The ASID a lookup is tagged with. A global mapping is cached under
        // ASID 0 as well, which is why a `TLBI` bumps the generation rather
        // than trying to evict selectively.
        let asid = self.current_asid();
        if let Some(base) = self.tlb.lookup(kind, vpn, asid, el, generation) {
            return Ok(base | (va & mmu::PAGE_MASK));
        }
        let mut walker = Walker {
            space: self.space,
            attrs: self.attrs,
            accesses: 0,
            refused: false,
        };
        let result = mmu::translate(&self.st.sys, &mut walker, va, kind, el);
        for _ in 0..walker.accesses {
            self.charge();
        }
        if walker.refused {
            self.st.faults = self.st.faults.wrapping_add(1);
        }
        let t = result?;
        let cached_asid = t.asid.unwrap_or(asid);
        self.tlb.insert(
            kind,
            vpn,
            cached_asid,
            el,
            generation,
            t.pa & !mmu::PAGE_MASK,
        );
        Ok(t.pa)
    }

    /// The ASID currently in force, narrowed as `TCR_EL1.AS` says.
    fn current_asid(&self) -> u64 {
        let source = if mmu::tcr::a1(self.st.sys.tcr) {
            self.st.sys.ttbr1
        } else {
            self.st.sys.ttbr0
        };
        let mask = if mmu::tcr::asid16(self.st.sys.tcr) {
            0xffff
        } else {
            0x00ff
        };
        (source >> 48) & mask
    }

    /// One read that does not cross a page boundary.
    fn read_once(&mut self, va: u64, width: Width, kind: Access) -> Result<u64, Trap> {
        let pa = self.translate(va, kind).map_err(|f| match kind {
            Access::Fetch => self.insn_abort(va, f),
            _ => self.data_abort(va, f, kind),
        })?;
        self.charge();
        match self.space.read(pa, width, self.attrs) {
            Ok(v) => Ok(v),
            Err(_) => {
                self.st.faults = self.st.faults.wrapping_add(1);
                Err(match kind {
                    Access::Fetch => self.insn_abort(va, mmu::Fault::External),
                    _ => self.data_abort(va, mmu::Fault::External, kind),
                })
            }
        }
    }

    /// One write that does not cross a page boundary.
    fn write_once(&mut self, va: u64, width: Width, value: u64) -> Result<(), Trap> {
        let pa = self
            .translate(va, Access::Store)
            .map_err(|f| self.data_abort(va, f, Access::Store))?;
        self.charge();
        match self.space.write(pa, width, value, self.attrs) {
            Ok(()) => Ok(()),
            Err(_) => {
                self.st.faults = self.st.faults.wrapping_add(1);
                Err(self.data_abort(va, mmu::Fault::External, Access::Store))
            }
        }
    }

    /// Whether an unaligned access of `bytes` bytes at `va` is allowed.
    ///
    /// `SCTLR_EL1.A` turns on alignment checking for normal loads and stores;
    /// exclusives and atomics require alignment regardless, which is why
    /// `always` exists as a separate argument rather than being folded into
    /// the `SCTLR` read.
    fn check_align(&self, va: u64, bytes: u64, kind: Access, always: bool) -> Result<(), Trap> {
        if va.is_multiple_of(bytes) {
            return Ok(());
        }
        if always || self.st.sys.sctlr & sctlr::A != 0 {
            // `kind` decides `ESR_ELx.ISS.WnR`, so an alignment fault on a
            // store reports a write and one on a load reports a read.
            return Err(self.data_abort(va, mmu::Fault::Alignment, kind));
        }
        Ok(())
    }

    /// Load `bytes` bytes, splitting an unaligned access into bytes.
    ///
    /// Each byte is translated separately, so an access straddling a page
    /// boundary faults on the half that is actually unmapped.
    fn load(&mut self, va: u64, bytes: u64) -> Result<u64, Trap> {
        self.check_align(va, bytes, Access::Load, false)?;
        if va.is_multiple_of(bytes) {
            let width = Width::from_bytes(bytes).ok_or_else(Trap::undefined)?;
            return self.read_once(va, width, Access::Load);
        }
        let mut value = 0u64;
        for i in 0..bytes {
            let byte = self.read_once(va.wrapping_add(i), Width::U8, Access::Load)?;
            value |= (byte & 0xff) << (8 * i);
        }
        Ok(value)
    }

    /// Store `bytes` bytes, splitting an unaligned access into bytes.
    fn store(&mut self, va: u64, bytes: u64, value: u64) -> Result<(), Trap> {
        self.check_align(va, bytes, Access::Store, false)?;
        self.break_reservation(va);
        if va.is_multiple_of(bytes) {
            let width = Width::from_bytes(bytes).ok_or_else(Trap::undefined)?;
            return self.write_once(va, width, value);
        }
        for i in 0..bytes {
            self.write_once(va.wrapping_add(i), Width::U8, value >> (8 * i))?;
        }
        Ok(())
    }

    /// A store into the reserved granule breaks the reservation, which is what
    /// makes a load-exclusive/store-exclusive pair fail when something else
    /// wrote the location in between.
    fn break_reservation(&mut self, va: u64) {
        if let Some(reserved) = self.st.exclusive
            && reserved == va >> 4
        {
            self.st.exclusive = None;
        }
    }

    /// Fetch the instruction at `pc`.
    fn fetch(&mut self) -> Result<u32, Trap> {
        let pc = self.st.pc;
        if pc & 3 != 0 {
            return Err(Trap {
                ec: ec::PC_ALIGN,
                iss: 0,
                far: Some(pc),
                advance: false,
            });
        }
        Ok(self.read_once(pc, Width::U32, Access::Fetch)? as u32)
    }

    // -----------------------------------------------------------------
    // Dispatch
    // -----------------------------------------------------------------

    /// Fetch, decode and execute one instruction.
    fn execute(&mut self) -> Result<(), Trap> {
        let word = self.fetch()?;
        let insn = isa::decode(word, self.cfg.features).ok_or_else(Trap::undefined)?;
        if insn.fmt.is_load_store() {
            return self.load_store(word, insn.fmt);
        }
        match insn.fmt {
            Fmt::LdStExclusive | Fmt::StoreExclusive => return self.exclusive(word, insn.fmt),
            Fmt::Atomic => return self.atomic(word, insn.op),
            _ => {}
        }
        self.data_processing(word, insn.op, insn.fmt)
    }

    /// Everything that is not a load or a store.
    #[allow(clippy::too_many_lines)]
    fn data_processing(&mut self, word: u32, op: Op, fmt: Fmt) -> Result<(), Trap> {
        let width = isa::datasize(word);
        let d = isa::rd(word);
        let n = isa::rn(word);
        let m = isa::rm(word);
        let rd_sp = fmt.rd_is_sp();
        let rn_sp = fmt.rn_is_sp();

        match op {
            // -- PC-relative addressing ------------------------------------
            Op::Adr | Op::Adrp => {
                let imm = isa::sext(
                    ((isa::field(word, 23, 5) as u64) << 2) | u64::from(isa::field(word, 30, 29)),
                    21,
                );
                let value = if op == Op::Adrp {
                    (self.this_pc & !0xfff).wrapping_add((imm as u64) << 12)
                } else {
                    self.this_pc.wrapping_add(imm as u64)
                };
                self.write_reg(d, 64, false, value);
            }

            // -- add/subtract (immediate) ----------------------------------
            Op::AddImm | Op::AddsImm | Op::SubImm | Op::SubsImm => {
                let shift = isa::field(word, 23, 22);
                // `sh` is one bit; `0b1x` is unallocated.
                if shift > 1 {
                    return Err(Trap::undefined());
                }
                let imm = u64::from(isa::imm12(word)) << (12 * shift);
                let subtract = matches!(op, Op::SubImm | Op::SubsImm);
                let sets_flags = matches!(op, Op::AddsImm | Op::SubsImm);
                let a = self.read_reg(n, width, rn_sp);
                let (result, flags) = if subtract {
                    isa::add_with_carry(a, !imm, true, width)
                } else {
                    isa::add_with_carry(a, imm, false, width)
                };
                if sets_flags {
                    self.st.sys.nzcv = flags;
                }
                self.write_reg(d, width, rd_sp, result);
            }

            // -- logical (immediate) ---------------------------------------
            Op::AndImm | Op::OrrImm | Op::EorImm | Op::AndsImm => {
                let (imm, _) = isa::decode_bit_masks(
                    isa::n_bit(word),
                    isa::imms(word),
                    isa::immr(word),
                    true,
                    width,
                )
                .ok_or_else(Trap::undefined)?;
                let a = self.read_reg(n, width, false);
                let result = match op {
                    Op::AndImm | Op::AndsImm => a & imm,
                    Op::OrrImm => a | imm,
                    _ => a ^ imm,
                };
                if op == Op::AndsImm {
                    self.set_logical_flags(result, width);
                }
                self.write_reg(d, width, rd_sp, result);
            }

            // -- move wide -------------------------------------------------
            Op::Movn | Op::Movz | Op::Movk => {
                let hw = isa::field(word, 22, 21);
                // A 32-bit move may only shift by 0 or 16.
                if width == 32 && hw > 1 {
                    return Err(Trap::undefined());
                }
                let pos = hw * 16;
                let imm = u64::from(isa::imm16(word)) << pos;
                let result = match op {
                    Op::Movz => imm,
                    Op::Movn => !imm,
                    _ => {
                        let old = self.read_reg(d, width, false);
                        (old & !(0xffffu64 << pos)) | imm
                    }
                };
                self.write_reg(d, width, false, result);
            }

            // -- bitfield --------------------------------------------------
            Op::Sbfm | Op::Bfm | Op::Ubfm => {
                // `N` must match `sf`, and the fields must fit the operand.
                if isa::n_bit(word) != u32::from(isa::sf(word)) {
                    return Err(Trap::undefined());
                }
                let r = isa::immr(word);
                let s = isa::imms(word);
                let (wmask, tmask) = isa::decode_bit_masks(isa::n_bit(word), s, r, false, width)
                    .ok_or_else(Trap::undefined)?;
                let src = self.read_reg(n, width, false);
                let rotated = ShiftKind::Ror.apply(src, r, width);
                let result = match op {
                    Op::Ubfm => rotated & wmask & tmask,
                    Op::Sbfm => {
                        // The sign is the source's bit `S`, replicated above
                        // the field.
                        let top = if src & (1u64 << (s % width)) != 0 {
                            isa::ones(width)
                        } else {
                            0
                        };
                        (top & !tmask) | (rotated & wmask & tmask)
                    }
                    _ => {
                        let dst = self.read_reg(d, width, false);
                        let bot = (dst & !wmask) | (rotated & wmask);
                        (dst & !tmask) | (bot & tmask)
                    }
                };
                self.write_reg(d, width, false, result);
            }

            Op::Extr => {
                if isa::n_bit(word) != u32::from(isa::sf(word)) {
                    return Err(Trap::undefined());
                }
                let lsb = isa::imms(word);
                if lsb >= width {
                    return Err(Trap::undefined());
                }
                let hi = self.read_reg(n, width, false);
                let lo = self.read_reg(m, width, false);
                let result = if lsb == 0 {
                    lo
                } else {
                    (lo >> lsb) | (hi << (width - lsb))
                };
                self.write_reg(d, width, false, result);
            }

            // -- branches --------------------------------------------------
            Op::B | Op::Bl => {
                if op == Op::Bl {
                    self.st.x[30] = self.this_pc.wrapping_add(4);
                }
                self.next_pc = self.this_pc.wrapping_add(isa::imm26(word) as u64);
            }
            Op::Bcond => {
                if isa::cond_lo(word).holds(self.st.sys.nzcv) {
                    self.next_pc = self.this_pc.wrapping_add(isa::imm19(word) as u64);
                }
            }
            Op::Cbz | Op::Cbnz => {
                let value = self.read_reg(isa::rd(word), width, false);
                let taken = if op == Op::Cbz {
                    value == 0
                } else {
                    value != 0
                };
                if taken {
                    self.next_pc = self.this_pc.wrapping_add(isa::imm19(word) as u64);
                }
            }
            Op::Tbz | Op::Tbnz => {
                // The bit position is split: bit 31 is its top bit, and it
                // also decides the operand width.
                let pos = (u32::from(isa::sf(word)) << 5) | isa::field(word, 23, 19);
                let value = self.read_reg(isa::rd(word), 64, false);
                let set = value & (1u64 << pos) != 0;
                let taken = if op == Op::Tbz { !set } else { set };
                if taken {
                    self.next_pc = self.this_pc.wrapping_add(isa::imm14(word) as u64);
                }
            }
            Op::Br | Op::Blr | Op::Ret => {
                let target = self.read_reg(n, 64, false);
                if op == Op::Blr {
                    self.st.x[30] = self.this_pc.wrapping_add(4);
                }
                self.next_pc = target;
            }
            Op::Eret => {
                if self.st.sys.el != El::El1 {
                    return Err(Trap::undefined());
                }
                let spsr = self.st.sys.spsr_el1;
                let elr = self.st.sys.elr_el1;
                if !self.st.sys.restore_pstate(spsr) {
                    return Err(Trap::undefined());
                }
                self.st.exclusive = None;
                self.next_pc = elr;
            }

            // -- exception generation --------------------------------------
            Op::Svc => {
                return Err(Trap {
                    ec: ec::SVC64,
                    iss: u64::from(isa::imm16(word)),
                    far: None,
                    advance: true,
                });
            }
            Op::Brk => {
                return Err(Trap {
                    ec: ec::BRK64,
                    iss: u64::from(isa::imm16(word)),
                    far: None,
                    advance: false,
                });
            }
            // No EL2 and no EL3, so both calls are UNDEFINED rather than a
            // vector into a level this core does not have.
            Op::Hvc | Op::Smc | Op::Hlt => return Err(Trap::undefined()),

            // -- hints and barriers ----------------------------------------
            Op::Nop | Op::Yield | Op::Sev | Op::Sevl | Op::Hint => {}
            // `WFE` waits for an event, and this core models no event
            // register — so it retires rather than stalling, which is always
            // an architecturally legal wake-up and cannot deadlock a guest
            // whose `SEV` would come from a core that does not exist here.
            Op::Wfe => {}
            Op::Wfi => self.st.wfi = true,
            // Barriers on a core that executes one instruction at a time and
            // completes every access before the next: nothing to order.
            Op::Dsb | Op::Dmb | Op::Isb => {}
            Op::Clrex => self.st.exclusive = None,

            // -- PSTATE and system registers -------------------------------
            Op::MsrSpsel => {
                self.require_el1()?;
                self.st.sys.spsel = isa::field(word, 11, 8) & 1 != 0;
            }
            Op::MsrDaifset | Op::MsrDaifclr => {
                self.require_el1()?;
                // The immediate's four bits are D, A, I, F in that order, and
                // `DAIF` holds them at bits 9:6.
                let bits = u64::from(isa::field(word, 11, 8)) << 6;
                if op == Op::MsrDaifset {
                    self.st.sys.daif |= bits;
                } else {
                    self.st.sys.daif &= !bits;
                }
            }
            Op::Mrs => {
                let value = self.read_sysreg(word)?;
                self.write_reg(isa::rd(word), 64, false, value);
            }
            Op::Msr => {
                let value = self.read_reg(isa::rd(word), 64, false);
                self.write_sysreg(word, value)?;
            }
            Op::Sys | Op::Sysl => self.system_op(word)?,

            // -- literal loads ---------------------------------------------
            Op::LdrLitW | Op::LdrLitX | Op::LdrswLit => {
                let addr = self.this_pc.wrapping_add(isa::imm19(word) as u64);
                let (bytes, dest_width, signed) = match op {
                    Op::LdrLitW => (4, 32, false),
                    Op::LdrLitX => (8, 64, false),
                    _ => (4, 64, true),
                };
                let value = self.load(addr, bytes)?;
                let value = if signed {
                    isa::sext(value, 32) as u64
                } else {
                    value
                };
                self.write_reg(isa::rd(word), dest_width, false, value);
            }
            Op::PrfmLit => {}

            // -- logical and arithmetic, shifted register ------------------
            Op::AndShift
            | Op::BicShift
            | Op::OrrShift
            | Op::OrnShift
            | Op::EorShift
            | Op::EonShift
            | Op::AndsShift
            | Op::BicsShift => {
                let amount = isa::shift_amount(word);
                if width == 32 && amount >= 32 {
                    return Err(Trap::undefined());
                }
                let shift = ShiftKind::from_bits(isa::shift_type(word));
                let operand = shift.apply(self.read_reg(m, width, false), amount, width);
                let invert = matches!(
                    op,
                    Op::BicShift | Op::OrnShift | Op::EonShift | Op::BicsShift
                );
                let operand = if invert {
                    !operand & isa::ones(width)
                } else {
                    operand
                };
                let a = self.read_reg(n, width, false);
                let result = match op {
                    Op::AndShift | Op::BicShift | Op::AndsShift | Op::BicsShift => a & operand,
                    Op::OrrShift | Op::OrnShift => a | operand,
                    _ => a ^ operand,
                };
                if matches!(op, Op::AndsShift | Op::BicsShift) {
                    self.set_logical_flags(result, width);
                }
                self.write_reg(d, width, false, result);
            }
            Op::AddShift | Op::AddsShift | Op::SubShift | Op::SubsShift => {
                let amount = isa::shift_amount(word);
                let shift_bits = isa::shift_type(word);
                // `ROR` is not an addressing mode for add and subtract.
                if shift_bits == 3 || (width == 32 && amount >= 32) {
                    return Err(Trap::undefined());
                }
                let shift = ShiftKind::from_bits(shift_bits);
                let operand = shift.apply(self.read_reg(m, width, false), amount, width);
                let subtract = matches!(op, Op::SubShift | Op::SubsShift);
                let a = self.read_reg(n, width, false);
                let (result, flags) = if subtract {
                    isa::add_with_carry(a, !operand, true, width)
                } else {
                    isa::add_with_carry(a, operand, false, width)
                };
                if matches!(op, Op::AddsShift | Op::SubsShift) {
                    self.st.sys.nzcv = flags;
                }
                self.write_reg(d, width, false, result);
            }
            Op::AddExt | Op::AddsExt | Op::SubExt | Op::SubsExt => {
                let option = isa::extend_option(word);
                let amount = isa::field(word, 12, 10);
                if amount > 4 {
                    return Err(Trap::undefined());
                }
                let operand =
                    isa::extend_reg(self.read_reg(m, 64, false), option, amount) & isa::ones(width);
                let subtract = matches!(op, Op::SubExt | Op::SubsExt);
                let a = self.read_reg(n, width, rn_sp);
                let (result, flags) = if subtract {
                    isa::add_with_carry(a, !operand, true, width)
                } else {
                    isa::add_with_carry(a, operand, false, width)
                };
                if matches!(op, Op::AddsExt | Op::SubsExt) {
                    self.st.sys.nzcv = flags;
                }
                self.write_reg(d, width, rd_sp, result);
            }
            Op::Adc | Op::Adcs | Op::Sbc | Op::Sbcs => {
                let a = self.read_reg(n, width, false);
                let b = self.read_reg(m, width, false);
                let subtract = matches!(op, Op::Sbc | Op::Sbcs);
                let carry = self.st.sys.nzcv.c();
                let (result, flags) = if subtract {
                    isa::add_with_carry(a, !b & isa::ones(width), carry, width)
                } else {
                    isa::add_with_carry(a, b, carry, width)
                };
                if matches!(op, Op::Adcs | Op::Sbcs) {
                    self.st.sys.nzcv = flags;
                }
                self.write_reg(d, width, false, result);
            }

            // -- conditional -----------------------------------------------
            Op::CcmnReg | Op::CcmpReg | Op::CcmnImm | Op::CcmpImm => {
                let cond = isa::cond_hi(word);
                if cond.holds(self.st.sys.nzcv) {
                    let a = self.read_reg(n, width, false);
                    let b = if matches!(op, Op::CcmnImm | Op::CcmpImm) {
                        u64::from(isa::rm(word))
                    } else {
                        self.read_reg(m, width, false)
                    };
                    let subtract = matches!(op, Op::CcmpReg | Op::CcmpImm);
                    let (_, flags) = if subtract {
                        isa::add_with_carry(a, !b & isa::ones(width), true, width)
                    } else {
                        isa::add_with_carry(a, b, false, width)
                    };
                    self.st.sys.nzcv = flags;
                } else {
                    self.st.sys.nzcv = Nzcv::from_nibble(word & 0xf);
                }
            }
            Op::Csel | Op::Csinc | Op::Csinv | Op::Csneg => {
                let cond = isa::cond_hi(word);
                let result = if cond.holds(self.st.sys.nzcv) {
                    self.read_reg(n, width, false)
                } else {
                    let b = self.read_reg(m, width, false);
                    match op {
                        Op::Csel => b,
                        Op::Csinc => b.wrapping_add(1),
                        Op::Csinv => !b,
                        _ => (!b).wrapping_add(1),
                    }
                };
                self.write_reg(d, width, false, result);
            }

            // -- two-source ------------------------------------------------
            Op::Udiv | Op::Sdiv => {
                let a = self.read_reg(n, width, false);
                let b = self.read_reg(m, width, false);
                // DDI 0487: a division by zero produces zero, and does not
                // trap. There is no divide-by-zero exception in A64.
                let result = if b == 0 {
                    0
                } else if op == Op::Udiv {
                    a / b
                } else {
                    // Signed, on the operand width, with the most-negative
                    // divided by -1 wrapping rather than trapping.
                    let sa = isa::sext(a, width);
                    let sb = isa::sext(b, width);
                    sa.wrapping_div(sb) as u64
                };
                self.write_reg(d, width, false, result);
            }
            Op::Lslv | Op::Lsrv | Op::Asrv | Op::Rorv => {
                let a = self.read_reg(n, width, false);
                // The shift amount is taken modulo the operand width, which is
                // why an A64 shift by 64 is a no-op rather than a zero.
                let amount = (self.read_reg(m, width, false) % u64::from(width)) as u32;
                let kind = match op {
                    Op::Lslv => ShiftKind::Lsl,
                    Op::Lsrv => ShiftKind::Lsr,
                    Op::Asrv => ShiftKind::Asr,
                    _ => ShiftKind::Ror,
                };
                self.write_reg(d, width, false, kind.apply(a, amount, width));
            }
            Op::Crc32b
            | Op::Crc32h
            | Op::Crc32w
            | Op::Crc32x
            | Op::Crc32cb
            | Op::Crc32ch
            | Op::Crc32cw
            | Op::Crc32cx => {
                let acc = self.read_reg(n, 32, false) as u32;
                let (bytes, castagnoli) = match op {
                    Op::Crc32b => (1, false),
                    Op::Crc32h => (2, false),
                    Op::Crc32w => (4, false),
                    Op::Crc32x => (8, false),
                    Op::Crc32cb => (1, true),
                    Op::Crc32ch => (2, true),
                    Op::Crc32cw => (4, true),
                    _ => (8, true),
                };
                let value = self.read_reg(m, if bytes == 8 { 64 } else { 32 }, false);
                let result = crc32(acc, value, bytes, castagnoli);
                self.write_reg(d, 32, false, u64::from(result));
            }

            // -- one-source ------------------------------------------------
            Op::Rbit => {
                let a = self.read_reg(n, width, false);
                let result = if width == 32 {
                    u64::from((a as u32).reverse_bits())
                } else {
                    a.reverse_bits()
                };
                self.write_reg(d, width, false, result);
            }
            Op::Rev16 => {
                let a = self.read_reg(n, width, false);
                let mut result = 0u64;
                for i in 0..width / 16 {
                    let half = (a >> (16 * i)) & 0xffff;
                    result |= u64::from((half as u16).swap_bytes()) << (16 * i);
                }
                self.write_reg(d, width, false, result);
            }
            Op::Rev32 => {
                let a = self.read_reg(n, 64, false);
                let lo = u64::from((a as u32).swap_bytes());
                let hi = u64::from(((a >> 32) as u32).swap_bytes());
                self.write_reg(d, 64, false, lo | (hi << 32));
            }
            Op::RevW => {
                let a = self.read_reg(n, 32, false) as u32;
                self.write_reg(d, 32, false, u64::from(a.swap_bytes()));
            }
            Op::RevX => {
                let a = self.read_reg(n, 64, false);
                self.write_reg(d, 64, false, a.swap_bytes());
            }
            Op::Clz => {
                let a = self.read_reg(n, width, false);
                let count = if width == 32 {
                    (a as u32).leading_zeros()
                } else {
                    a.leading_zeros()
                };
                self.write_reg(d, width, false, u64::from(count));
            }
            Op::Cls => {
                let a = self.read_reg(n, width, false);
                // Count the sign bits above the top one: the number of
                // leading zeroes of `x XOR (x >> 1)` within `width - 1` bits.
                let folded = (a ^ (a >> 1)) & isa::ones(width - 1);
                let count = if folded == 0 {
                    width - 1
                } else {
                    (width - 1) - (64 - folded.leading_zeros())
                };
                self.write_reg(d, width, false, u64::from(count));
            }

            // -- three-source ----------------------------------------------
            Op::Madd | Op::Msub => {
                let a = self.read_reg(n, width, false);
                let b = self.read_reg(m, width, false);
                let acc = self.read_reg(isa::ra(word), width, false);
                let product = a.wrapping_mul(b);
                let result = if op == Op::Madd {
                    acc.wrapping_add(product)
                } else {
                    acc.wrapping_sub(product)
                };
                self.write_reg(d, width, false, result);
            }
            Op::Smaddl | Op::Smsubl | Op::Umaddl | Op::Umsubl => {
                let signed = matches!(op, Op::Smaddl | Op::Smsubl);
                // The sources are the *word* halves and the accumulator is a
                // doubleword: widening after narrowing, never the reverse.
                let a = self.read_reg(n, 32, false);
                let b = self.read_reg(m, 32, false);
                let product = if signed {
                    (isa::sext(a, 32).wrapping_mul(isa::sext(b, 32))) as u64
                } else {
                    a.wrapping_mul(b)
                };
                let acc = self.read_reg(isa::ra(word), 64, false);
                let result = if matches!(op, Op::Smaddl | Op::Umaddl) {
                    acc.wrapping_add(product)
                } else {
                    acc.wrapping_sub(product)
                };
                self.write_reg(d, 64, false, result);
            }
            Op::Smulh => {
                let a = i128::from(self.read_reg(n, 64, false) as i64);
                let b = i128::from(self.read_reg(m, 64, false) as i64);
                self.write_reg(d, 64, false, ((a * b) >> 64) as u64);
            }
            Op::Umulh => {
                let a = u128::from(self.read_reg(n, 64, false));
                let b = u128::from(self.read_reg(m, 64, false));
                self.write_reg(d, 64, false, ((a * b) >> 64) as u64);
            }

            // The load/store formats are handled before this match, and
            // `PRFM` is a hint with no architectural effect.
            Op::PrfmImm | Op::Prfum | Op::PrfmReg => {}
            _ => return Err(Trap::undefined()),
        }
        Ok(())
    }

    /// `N` and `Z` from the result; `C` and `V` cleared. The logical
    /// instructions never set the arithmetic flags, which is what makes
    /// `TST` followed by `B.CS` a bug rather than an idiom.
    fn set_logical_flags(&mut self, result: u64, width: u32) {
        let sign = 1u64 << (width - 1);
        self.st.sys.nzcv = Nzcv::new(result & sign != 0, result == 0, false, false);
    }

    /// Refuse an operation that EL0 may not perform.
    fn require_el1(&self) -> Result<(), Trap> {
        if self.st.sys.el == El::El1 {
            Ok(())
        } else {
            Err(Trap::undefined())
        }
    }

    // -----------------------------------------------------------------
    // System registers
    // -----------------------------------------------------------------

    /// Read the system register an `MRS` names.
    fn read_sysreg(&mut self, word: u32) -> Result<u64, Trap> {
        let key = isa::field(word, 20, 5) as u16;
        let spec = sysreg::lookup(key).ok_or_else(Trap::undefined)?;
        if !spec.access.readable_at(self.st.sys.el) {
            return Err(Trap::undefined());
        }
        let s = &self.st.sys;
        Ok(match spec.reg {
            SysReg::Midr => self.cfg.midr,
            SysReg::Mpidr => self.cfg.mpidr,
            SysReg::Revidr => 0,
            SysReg::IdAa64Pfr0 => self.cfg.id_aa64pfr0(),
            SysReg::IdAa64Pfr1 => 0,
            SysReg::IdAa64Isar0 => self.cfg.id_aa64isar0(),
            SysReg::IdAa64Isar1 => 0,
            SysReg::IdAa64Mmfr0 => Config::ID_AA64MMFR0,
            SysReg::IdAa64Mmfr1 | SysReg::IdAa64Mmfr2 => 0,
            SysReg::Ctr => Config::CTR,
            // DZP set: `DC ZVA` is prohibited, because this core does not
            // implement it.
            SysReg::Dczid => 0x10,
            SysReg::Sctlr => s.sctlr,
            SysReg::Actlr => s.actlr,
            SysReg::Cpacr => s.cpacr,
            SysReg::Ttbr0 => s.ttbr0,
            SysReg::Ttbr1 => s.ttbr1,
            SysReg::Tcr => s.tcr,
            SysReg::Mair => s.mair,
            SysReg::Amair => s.amair,
            SysReg::Contextidr => s.contextidr,
            SysReg::Spsr => s.spsr_el1,
            SysReg::Elr => s.elr_el1,
            SysReg::SpEl0 => s.sp_el0,
            SysReg::Afsr0 => s.afsr0,
            SysReg::Afsr1 => s.afsr1,
            SysReg::Esr => s.esr_el1,
            SysReg::Far => s.far_el1,
            SysReg::Vbar => s.vbar_el1,
            SysReg::Spsel => u64::from(s.spsel),
            SysReg::CurrentEl => s.el.bits() << 2,
            SysReg::NzcvReg => u64::from(s.nzcv.0),
            SysReg::DaifReg => s.daif,
            SysReg::TpidrEl1 => s.tpidr_el1,
            SysReg::TpidrEl0 => s.tpidr_el0,
            SysReg::TpidrroEl0 => s.tpidrro_el0,
            SysReg::Mdscr => s.mdscr,
        })
    }

    /// Write the system register an `MSR` names.
    fn write_sysreg(&mut self, word: u32, value: u64) -> Result<(), Trap> {
        let key = isa::field(word, 20, 5) as u16;
        let spec = sysreg::lookup(key).ok_or_else(Trap::undefined)?;
        if !spec.access.writable_at(self.st.sys.el) {
            // A write to a register that exists but is read-only here, or one
            // EL0 may not reach, is UNDEFINED rather than silently ignored:
            // silently ignoring it is how a guest ends up believing it
            // configured something.
            return Err(Trap::undefined());
        }
        // Anything that changes the translation regime invalidates every
        // cached translation. Bumping the generation is the whole
        // invalidation (`ROADMAP.md` §4.5).
        let s = &mut self.st.sys;
        let mut retranslate = || s.translation_gen = s.translation_gen.wrapping_add(1);
        match spec.reg {
            SysReg::Sctlr => {
                s.sctlr = value;
                retranslate();
            }
            SysReg::Ttbr0 => {
                s.ttbr0 = value;
                retranslate();
            }
            SysReg::Ttbr1 => {
                s.ttbr1 = value;
                retranslate();
            }
            SysReg::Tcr => {
                s.tcr = value;
                retranslate();
            }
            SysReg::Mair => s.mair = value,
            SysReg::Amair => s.amair = value,
            SysReg::Actlr => s.actlr = value,
            SysReg::Cpacr => s.cpacr = value,
            SysReg::Contextidr => s.contextidr = value,
            SysReg::Spsr => s.spsr_el1 = value,
            SysReg::Elr => s.elr_el1 = value,
            SysReg::SpEl0 => s.sp_el0 = value,
            SysReg::Afsr0 => s.afsr0 = value,
            SysReg::Afsr1 => s.afsr1 = value,
            SysReg::Esr => s.esr_el1 = value,
            SysReg::Far => s.far_el1 = value,
            SysReg::Vbar => s.vbar_el1 = value,
            SysReg::Spsel => s.spsel = value & 1 != 0,
            SysReg::NzcvReg => s.nzcv = Nzcv((value as u32) & 0xf000_0000),
            SysReg::DaifReg => s.daif = value & daif::ALL,
            SysReg::TpidrEl1 => s.tpidr_el1 = value,
            SysReg::TpidrEl0 => s.tpidr_el0 = value,
            SysReg::TpidrroEl0 => s.tpidrro_el0 = value,
            SysReg::Mdscr => s.mdscr = value,
            // Everything else in the table is read-only, and `writable_at`
            // already refused it.
            _ => return Err(Trap::undefined()),
        }
        Ok(())
    }

    /// `SYS` and `SYSL`: the `TLBI`, `DC` and `IC` aliases.
    fn system_op(&mut self, word: u32) -> Result<(), Trap> {
        self.require_el1()?;
        let crn = isa::field(word, 15, 12);
        match crn {
            // TLB maintenance. Every variant invalidates at least what this
            // core caches, so bumping the generation is a correct — if
            // generous — answer to all of them, and a generous invalidation
            // can only cost walks, never correctness.
            8 => {
                self.st.sys.translation_gen = self.st.sys.translation_gen.wrapping_add(1);
                Ok(())
            }
            // Cache maintenance on a core with no cache. `DC ZVA` is the one
            // that is not a no-op, and `DCZID_EL0.DZP` already tells the guest
            // it is prohibited.
            7 => {
                let crm = isa::field(word, 11, 8);
                let op1 = isa::field(word, 18, 16);
                let op2 = isa::field(word, 7, 5);
                if op1 == 3 && crm == 4 && op2 == 1 {
                    Err(Trap::undefined())
                } else {
                    Ok(())
                }
            }
            _ => Err(Trap::undefined()),
        }
    }

    // -----------------------------------------------------------------
    // Loads and stores
    // -----------------------------------------------------------------

    /// The load/store families, dispatched by addressing mode.
    fn load_store(&mut self, word: u32, fmt: Fmt) -> Result<(), Trap> {
        match fmt {
            Fmt::LdStPairOff | Fmt::LdStPairPost | Fmt::LdStPairPre => self.pair(word, fmt),
            _ => self.single(word, fmt),
        }
    }

    /// A single-register load or store.
    fn single(&mut self, word: u32, fmt: Fmt) -> Result<(), Trap> {
        let size = isa::ls_size(word);
        let access = isa::ls_access(size, isa::ls_opc(word)).ok_or_else(Trap::undefined)?;
        let t = isa::rd(word);
        let n = isa::rn(word);
        let base = self.read_reg(n, 64, true);

        let (addr, writeback) = match fmt {
            Fmt::LdStUImm => (base.wrapping_add(u64::from(isa::imm12(word)) << size), None),
            Fmt::LdStUnscaled => (base.wrapping_add(isa::imm9(word) as u64), None),
            Fmt::LdStPost => (base, Some(base.wrapping_add(isa::imm9(word) as u64))),
            Fmt::LdStPre => {
                let a = base.wrapping_add(isa::imm9(word) as u64);
                (a, Some(a))
            }
            Fmt::LdStRegOff => {
                let option = isa::extend_option(word);
                // `option<1>` must be set: the encodings with it clear are
                // unallocated rather than an `LSL` by another name.
                if option & 2 == 0 {
                    return Err(Trap::undefined());
                }
                let amount = if isa::bit(word, 12) { size } else { 0 };
                let index =
                    isa::extend_reg(self.read_reg(isa::rm(word), 64, false), option, amount);
                (base.wrapping_add(index), None)
            }
            _ => return Err(Trap::undefined()),
        };

        match access {
            LsAccess::Prefetch => {}
            LsAccess::Store { bytes } => {
                let value = self.read_reg(t, 64, false);
                self.store(addr, bytes, value)?;
            }
            LsAccess::Load { bytes, wide } => {
                let value = self.load(addr, bytes)?;
                self.write_reg(t, if wide { 64 } else { 32 }, false, value);
            }
            LsAccess::LoadSigned { bytes, wide } => {
                let value = self.load(addr, bytes)?;
                let value = isa::sext(value, (bytes * 8) as u32) as u64;
                self.write_reg(t, if wide { 64 } else { 32 }, false, value);
            }
        }
        // The write-back happens after the access, so a fault leaves the base
        // register untouched and the instruction can be restarted.
        if let Some(value) = writeback {
            self.write_reg(n, 64, true, value);
        }
        Ok(())
    }

    /// A load or store of a register pair.
    fn pair(&mut self, word: u32, fmt: Fmt) -> Result<(), Trap> {
        let opc = isa::field(word, 31, 30);
        let load = isa::bit(word, 22);
        // `opc` is not the single-register `size` field: `0b00` is a word,
        // `0b01` is `LDPSW` (word operands, doubleword registers) and `0b10`
        // is a doubleword.
        let (scale, signed, wide) = match opc {
            0b00 => (2u32, false, false),
            0b01 if load => (2u32, true, true),
            0b10 => (3u32, false, true),
            _ => return Err(Trap::undefined()),
        };
        let bytes = 1u64 << scale;
        let t = isa::rd(word);
        let t2 = isa::ra(word);
        let n = isa::rn(word);
        let base = self.read_reg(n, 64, true);
        let offset = (isa::imm7(word) << scale) as u64;

        let (addr, writeback) = match fmt {
            Fmt::LdStPairOff => (base.wrapping_add(offset), None),
            Fmt::LdStPairPost => (base, Some(base.wrapping_add(offset))),
            _ => {
                let a = base.wrapping_add(offset);
                (a, Some(a))
            }
        };

        if load {
            let first = self.load(addr, bytes)?;
            let second = self.load(addr.wrapping_add(bytes), bytes)?;
            let extend = |v: u64| {
                if signed { isa::sext(v, 32) as u64 } else { v }
            };
            let width = if wide { 64 } else { 32 };
            self.write_reg(t, width, false, extend(first));
            self.write_reg(t2, width, false, extend(second));
        } else {
            let first = self.read_reg(t, 64, false);
            let second = self.read_reg(t2, 64, false);
            self.store(addr, bytes, first)?;
            self.store(addr.wrapping_add(bytes), bytes, second)?;
        }
        if let Some(value) = writeback {
            self.write_reg(n, 64, true, value);
        }
        Ok(())
    }

    /// The exclusives and the acquire/release ordinary accesses.
    ///
    /// One function because they share an encoding group: `o2` (bit 23)
    /// chooses between the exclusive pair and the plain `LDAR`/`STLR`, and
    /// `L` (bit 22) chooses the direction.
    fn exclusive(&mut self, word: u32, fmt: Fmt) -> Result<(), Trap> {
        let bytes = 1u64 << isa::ls_size(word);
        let plain = isa::bit(word, 23);
        let load = isa::bit(word, 22);
        let t = isa::rd(word);
        let n = isa::rn(word);
        let addr = self.read_reg(n, 64, true);
        let width = if bytes == 8 { 64 } else { 32 };
        // An exclusive or acquire/release access must be aligned whatever
        // `SCTLR_EL1.A` says.
        let kind = if load { Access::Load } else { Access::Store };
        self.check_align(addr, bytes, kind, true)?;

        if plain {
            if load {
                let value = self.load(addr, bytes)?;
                self.write_reg(t, width, false, value);
            } else {
                let value = self.read_reg(t, 64, false);
                self.store(addr, bytes, value)?;
            }
            return Ok(());
        }

        if fmt == Fmt::LdStExclusive {
            let value = self.load(addr, bytes)?;
            self.st.exclusive = Some(addr >> 4);
            self.write_reg(t, width, false, value);
            return Ok(());
        }

        // `STXR Ws, Rt, [Rn]`: `Ws` reports 0 on success and 1 on failure.
        let status_reg = isa::rm(word);
        let matched = self.st.exclusive == Some(addr >> 4);
        if matched {
            let value = self.read_reg(t, 64, false);
            self.store(addr, bytes, value)?;
        }
        // The monitor is cleared by the attempt, successful or not.
        self.st.exclusive = None;
        self.write_reg(status_reg, 32, false, u64::from(!matched));
        Ok(())
    }

    /// The `FEAT_LSE` atomics.
    fn atomic(&mut self, word: u32, op: Op) -> Result<(), Trap> {
        let bytes = 1u64 << isa::ls_size(word);
        let width = if bytes == 8 { 64 } else { 32 };
        let s = isa::rm(word);
        let t = isa::rd(word);
        let n = isa::rn(word);
        let addr = self.read_reg(n, 64, true);
        // An atomic is a read-modify-*write*, so an alignment fault on one
        // reports a write.
        self.check_align(addr, bytes, Access::Store, true)?;

        if matches!(op, Op::CasW | Op::CasX) {
            let compare = self.read_reg(s, width, false);
            let old = self.load(addr, bytes)?;
            if old == compare {
                let new = self.read_reg(t, width, false);
                self.store(addr, bytes, new)?;
            }
            // `Rs` is both the comparand and the destination, and it is
            // written with the old value whether or not the swap happened.
            self.write_reg(s, width, false, old);
            return Ok(());
        }

        let operand = self.read_reg(s, width, false);
        let old = self.load(addr, bytes)?;
        // `o3` (bit 15) with `opc == 0` is `SWP`; otherwise `opc` (bits 14:12)
        // names the read-modify-write.
        let swap = isa::bit(word, 15) && isa::field(word, 14, 12) == 0;
        let new = if swap {
            operand
        } else {
            match isa::field(word, 14, 12) {
                0 => old.wrapping_add(operand),
                1 => old & !operand,
                2 => old ^ operand,
                3 => old | operand,
                4 => {
                    let (a, b) = (isa::sext(old, width), isa::sext(operand, width));
                    a.max(b) as u64
                }
                5 => {
                    let (a, b) = (isa::sext(old, width), isa::sext(operand, width));
                    a.min(b) as u64
                }
                6 => old.max(operand),
                _ => old.min(operand),
            }
        };
        self.store(addr, bytes, new)?;
        // `Rt == 31` is the `ST<op>` spelling, which discards the old value —
        // and `XZR` discarding it is exactly what `write_reg` already does.
        self.write_reg(t, width, false, old);
        Ok(())
    }
}

/// The reflected CRC-32 the `CRC32`/`CRC32C` instructions compute.
///
/// DDI 0487 defines these as a bit-reversed accumulator and value fed through
/// `Poly32Mod2` and reversed again; that is exactly the table-free reflected
/// algorithm below, with the polynomial in its reversed form and no pre- or
/// post-inversion — which is why an Arm `CRC32W` of a buffer is not the same
/// number as a zlib `crc32` of it unless the caller inverts at both ends.
fn crc32(acc: u32, value: u64, bytes: u32, castagnoli: bool) -> u32 {
    let poly: u32 = if castagnoli { 0x82f6_3b78 } else { 0xedb8_8320 };
    let mut crc = acc;
    for i in 0..bytes {
        crc ^= ((value >> (8 * i)) & 0xff) as u32;
        for _ in 0..8 {
            // Branchless, and the mask is the whole trick: subtracting the low
            // bit gives all-ones when it was set.
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (poly & mask);
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Checked against a constant nobody can fudge: `zlib`'s CRC-32 of a
    /// single zero byte is `0xd202ef8d`, and zlib inverts at both ends while
    /// the Arm instruction does not — so the instruction's answer from an
    /// all-ones accumulator is that value inverted.
    #[test]
    fn crc32_is_the_reflected_polynomial() {
        assert_eq!(crc32(0, 0, 1, false), 0);
        assert_eq!(crc32(0xffff_ffff, 0, 1, false), !0xd202_ef8du32);
        // CRC32C uses the Castagnoli polynomial and gives a different answer
        // for the same input, which is the point of having both.
        assert_ne!(crc32(0xffff_ffff, 0, 1, true), !0xd202_ef8du32);
    }
}
