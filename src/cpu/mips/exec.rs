//! The interpreter.
//!
//! # Two program counters, because MIPS has delay slots
//!
//! Every other core in this tree steps one instruction to one program counter.
//! MIPS cannot: the instruction *after* a branch always executes, whether or
//! not the branch is taken, so the machine's control state is a **pair** —
//! where we are, and where we go next. [`State::pc`] and [`State::next_pc`]
//! are that pair, and [`State::in_delay`] records whether the instruction at
//! `pc` is the one that got pulled along.
//!
//! The pair is advanced *before* the instruction executes, so a branch is
//! simply an overwrite of `next_pc` and nothing has to reason about ordering.
//!
//! The reason `in_delay` is state rather than a local is [`Cause.BD`]: an
//! exception taken on a delay-slot instruction sets that bit and puts the
//! **branch's** address in `EPC`, so the return re-executes the branch and the
//! delay slot runs again with the branch's decision remade. Getting this wrong
//! is invisible until an interrupt lands in a delay slot — which, in a system
//! with a timer, happens constantly.
//!
//! [`Cause.BD`]: super::cp0::cause_bits::BD
//!
//! # MIPS I has no load interlock, and that is architectural
//!
//! The instruction after a load sees the destination register's **old** value.
//! This is not a modelling nicety: MIPS I assemblers schedule around it, real
//! R3000 code depends on it, and a core that interlocks silently computes
//! different answers. [`State::pending_load`] is the delayed write, and the
//! ordering inside one step is exactly the pipeline's:
//!
//! 1. read `rs` and `rt` — the values in the register file, which do *not*
//!    include the pending load;
//! 2. **settle** the pending load into the register file;
//! 3. execute, whose own register writes therefore win over a load that
//!    targeted the same register (the load's write-back is a cycle earlier).
//!
//! An exception settles the pending load too, because R3000 exceptions are
//! precise: the load instruction retired, so its write-back happened, and the
//! handler's return re-executes the *faulting* instruction — which then sees
//! the new value rather than the old one. That is a real architectural wart
//! and the reason assemblers avoid putting faulting instructions in load delay
//! slots.
//!
//! [`Config::load_interlock`](super::Config) turns the delay off, for MIPS II
//! and later parts. It is a construction property, not a constant.
//!
//! # One cycle is one bus access
//!
//! There is no cycle table here. An instruction fetch is one access, a load or
//! a store is one, an unaligned transfer is one (it touches a single aligned
//! word), and an isolated-cache access is one. That is the accounting
//! `ROADMAP.md` §6 asks for and the only kind that is a fact about the machine
//! rather than an invention about a pipeline.
//!
//! # Sources
//!
//! Kane & Heinrich, *MIPS RISC Architecture*, for the instruction semantics,
//! the delay-slot rules and the unaligned-transfer definitions; the *IDT
//! R3051/R3052/R3081 Family Hardware User's Manual* for the exception model,
//! the vectors and the TLB. Specific citations sit next to the rules they
//! justify — the division results, the branch-and-link rule, the `RFE` shift,
//! and the cache-isolation behaviour.

use alloc::vec;
use alloc::vec::Vec;

use crate::core::exec::{Access as ExitAccess, Exit, ExitMask, ExitReason};
use crate::core::space::{AddressSpace, MemAttrs};
use crate::core::value::Width;

use super::cp0::{
    Cp0, GENERAL_VECTOR, GENERAL_VECTOR_BEV, Lines, Lookup, REFILL_VECTOR, REFILL_VECTOR_BEV,
    Segment, TLB_ENTRIES, Tlb, TlbEntry, cause_bits, exc, reg, status,
};
use super::isa::{self, Endian, Op, Req};
use super::{Config, PAGE_SIZE};

/// Which kind of access a translation is for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Access {
    /// An instruction fetch.
    Fetch,
    /// A data read.
    Load,
    /// A data write.
    Store,
}

impl Access {
    /// The address-error code this access raises.
    const fn address_error(self) -> u32 {
        match self {
            Access::Store => exc::ADES,
            _ => exc::ADEL,
        }
    }

    /// The TLB-fault code this access raises.
    const fn tlb_error(self) -> u32 {
        match self {
            Access::Store => exc::TLBS,
            _ => exc::TLBL,
        }
    }

    /// The bus-error code this access raises.
    const fn bus_error(self) -> u32 {
        match self {
            Access::Fetch => exc::IBE,
            _ => exc::DBE,
        }
    }
}

/// An exception the current instruction raised.
///
/// Carries everything the exception entry needs, because each piece is decided
/// where the fault happens — a TLB miss knows the address and whether it
/// belongs on the refill vector, a coprocessor fault knows which coprocessor —
/// and reconstructing it later is how the two drift apart.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct Trap {
    /// The `Cause.ExcCode` value.
    pub code: u32,
    /// The address to publish in `BadVAddr`, and to fold into `Context` and
    /// `EntryHi` when the exception is a TLB one.
    pub bad_vaddr: Option<u32>,
    /// Whether the exception takes the dedicated **TLB refill** vector rather
    /// than the general one. Only a `kuseg` miss does.
    pub refill: bool,
    /// Which coprocessor a `CpU` exception names, for `Cause.CE`.
    pub ce: u32,
    /// Whether `EntryHi` and `Context` should be reloaded from `bad_vaddr`.
    pub tlb: bool,
}

impl Trap {
    /// An exception with no address and no coprocessor.
    const fn bare(code: u32) -> Trap {
        Trap {
            code,
            bad_vaddr: None,
            refill: false,
            ce: 0,
            tlb: false,
        }
    }

    /// An address error, which reports the address but does not touch the TLB
    /// registers.
    const fn address(code: u32, vaddr: u32) -> Trap {
        Trap {
            code,
            bad_vaddr: Some(vaddr),
            refill: false,
            ce: 0,
            tlb: false,
        }
    }

    /// A coprocessor-unusable exception naming `n`.
    const fn coprocessor(n: u32) -> Trap {
        Trap {
            code: exc::CPU,
            bad_vaddr: None,
            refill: false,
            ce: n,
            tlb: false,
        }
    }
}

/// A load whose value is not visible yet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct PendingLoad {
    /// Which register it will land in.
    pub reg: u32,
    /// The value it will land with.
    pub value: u32,
}

/// The architectural state of one processor.
#[derive(Debug, Clone)]
pub(super) struct State {
    /// The general register file. `r0` is architecturally zero and is kept
    /// zero rather than special-cased on read.
    pub regs: [u32; 32],
    /// The high half of a multiply, or the remainder of a divide.
    pub hi: u32,
    /// The low half of a multiply, or the quotient of a divide.
    pub lo: u32,
    /// The address of the instruction about to execute.
    pub pc: u32,
    /// The address of the instruction after it — which a branch overwrites,
    /// and which is why control state is a pair.
    pub next_pc: u32,
    /// Whether the instruction at [`State::pc`] is in a branch delay slot.
    pub in_delay: bool,
    /// A load issued by the previous instruction, not yet in the register
    /// file. **Architectural state**: a snapshot taken between a load and the
    /// instruction after it must restore it, or the guest sees a value it
    /// never would have.
    pub pending_load: Option<PendingLoad>,
    /// The system control coprocessor.
    pub cp0: Cp0,
    /// The translation lookaside buffer, when the part has one.
    ///
    /// Guest-visible through `TLBR` and `TLBP`, so it is saved — unlike the
    /// RISC-V core's software TLB, which is a derived cache.
    pub tlb: Tlb,
    /// The data cache's data array, as seen through `Status.IsC`.
    pub dcache: Vec<u8>,
    /// The instruction cache's data array, as seen through `Status.IsC` with
    /// `Status.SwC` also set.
    pub icache: Vec<u8>,
    /// Bus accesses since reset.
    pub cycles: u64,
    /// Accesses already executed past the last budget, owed to the next one.
    pub debt: u64,
    /// How many accesses the address space refused.
    pub faults: u64,
}

impl State {
    /// The reset state for a configuration.
    pub(super) fn new(cfg: &Config) -> State {
        let pc = cfg.reset_vector;
        State {
            regs: [0; 32],
            hi: 0,
            lo: 0,
            pc,
            next_pc: pc.wrapping_add(4),
            in_delay: false,
            pending_load: None,
            cp0: Cp0::new(cfg.prid),
            tlb: Tlb::new(),
            dcache: vec![0; cfg.arch.dcache_bytes as usize],
            icache: vec![0; cfg.arch.icache_bytes as usize],
            cycles: 0,
            debt: 0,
            faults: 0,
        }
    }
}

/// One step's worth of execution, borrowing everything it needs.
pub(super) struct Exec<'a> {
    st: &'a mut State,
    space: &'a AddressSpace,
    cfg: &'a Config,
    lines: &'a Lines,
    /// Which architectural traps leave the core instead of vectoring into the
    /// guest (`core::exec`). Empty for a level-1 machine.
    exits: ExitMask,
    /// Set instead of vectoring, when a trap is named in `exits`.
    exit: Option<Exit>,
    attrs: MemAttrs,
    /// Accesses charged by this step.
    used: u64,
    /// The address of the instruction being executed.
    this_pc: u32,
    /// The `next_pc` this step started with, so a fault exit can rewind the
    /// pair exactly rather than guessing that it was `this_pc + 4`.
    entry_next_pc: u32,
    /// Whether this instruction is in a branch delay slot.
    in_delay: bool,
    /// The load issued by the *previous* instruction, waiting to settle.
    delayed: Option<PendingLoad>,
    /// Which register [`Exec::settle`] wrote, and what it held before — so
    /// that a load claiming the same slot can put it back.
    settled: Option<(u32, u32)>,
    /// The load issued by *this* instruction.
    issued: Option<PendingLoad>,
}

impl<'a> Exec<'a> {
    /// Borrow a processor for one step.
    pub(super) fn new(
        st: &'a mut State,
        space: &'a AddressSpace,
        cfg: &'a Config,
        lines: &'a Lines,
        exits: ExitMask,
    ) -> Exec<'a> {
        let attrs = MemAttrs::DEFAULT
            .with_requester(cfg.requester)
            .with_privileged(st.cp0.kernel_mode());
        let this_pc = st.pc;
        let entry_next_pc = st.next_pc;
        let in_delay = st.in_delay;
        let delayed = st.pending_load.take();
        Exec {
            st,
            space,
            cfg,
            lines,
            exits,
            exit: None,
            attrs,
            used: 0,
            this_pc,
            entry_next_pc,
            in_delay,
            delayed,
            settled: None,
            issued: None,
        }
    }

    /// Execute one instruction, or take one exception.
    ///
    /// Returns the bus accesses charged, which is never zero: a caller can
    /// always make progress.
    pub(super) fn step(&mut self) -> u64 {
        if let Some(pending) = self.pending_interrupt() {
            // The instruction the interrupt aborts has not run, but the load
            // the *previous* one issued has already written back — R3000
            // exceptions are precise.
            self.settle();
            let _ = pending;
            self.enter_exception(Trap::bare(exc::INT));
            return self.finish();
        }
        match self.execute() {
            Ok(()) => {
                self.settle();
                self.st.pending_load = self.issued.take();
                self.st.cp0.tick_random();
            }
            Err(trap) => {
                self.settle();
                match self.exit_for(&trap) {
                    Some(exit) => {
                        if exit.reason != ExitReason::SYSCALL {
                            // A fault is re-executed, so the control pair is
                            // rewound to exactly what it was — including the
                            // delay-slot flag, which `this_pc + 4` would lose.
                            self.st.pc = self.this_pc;
                            self.st.next_pc = self.entry_next_pc;
                            self.st.in_delay = self.in_delay;
                        }
                        self.exit = Some(exit);
                    }
                    None => self.enter_exception(trap),
                }
            }
        }
        self.finish()
    }

    /// Every step charges at least one access.
    fn finish(&mut self) -> u64 {
        self.used.max(1)
    }

    /// Take the exit this step produced, if it produced one.
    pub(super) fn take_exit(&mut self) -> Option<Exit> {
        self.exit.take()
    }

    /// Whether `trap` should leave the core rather than vector into the guest,
    /// and as what.
    fn exit_for(&self, trap: &Trap) -> Option<Exit> {
        let (reason, access) = match trap.code {
            exc::SYS => (ExitReason::SYSCALL, ExitAccess::None),
            exc::BP => (ExitReason::BREAKPOINT, ExitAccess::None),
            exc::RI | exc::CPU | exc::OV => (ExitReason::FAULT, ExitAccess::None),
            exc::ADEL | exc::TLBL | exc::IBE => (ExitReason::FAULT, ExitAccess::Read),
            exc::ADES | exc::TLBS | exc::MOD | exc::DBE => (ExitReason::FAULT, ExitAccess::Write),
            _ => return None,
        };
        if !self.exits.contains(reason) {
            return None;
        }
        // Every MIPS instruction is four bytes, so there is no length to work
        // out — which is the one thing this architecture makes easy.
        let exit = Exit::new(reason, u64::from(self.this_pc), 4).with_detail(u64::from(trap.code));
        match (access, trap.bad_vaddr) {
            (ExitAccess::None, _) | (_, None) => Some(exit),
            (_, Some(addr)) => Some(exit.with_access(u64::from(addr), access)),
        }
    }

    // -----------------------------------------------------------------
    // The clock and the register file
    // -----------------------------------------------------------------

    /// Charge one bus access.
    #[inline]
    fn charge(&mut self) {
        self.used += 1;
        self.st.cycles = self.st.cycles.wrapping_add(1);
    }

    /// Read a general register. `r0` reads as zero.
    #[inline]
    fn reg(&self, i: u32) -> u32 {
        self.st.regs[(i & 31) as usize]
    }

    /// Write a general register. A write to `r0` is discarded.
    #[inline]
    fn set_reg(&mut self, i: u32, value: u32) {
        if i & 31 != 0 {
            self.st.regs[(i & 31) as usize] = value;
        }
    }

    /// Land the previous instruction's load in the register file.
    ///
    /// Idempotent, so it can be called from the fault path without knowing
    /// whether the success path already did it.
    fn settle(&mut self) {
        if let Some(load) = self.delayed.take() {
            self.settled = Some((load.reg & 31, self.reg(load.reg)));
            self.set_reg(load.reg, load.value);
        }
    }

    /// Schedule a load's result.
    ///
    /// On a part with a load interlock the value is written straight away,
    /// which is what MIPS II and later do; on MIPS I it waits one instruction.
    ///
    /// **A load into the register a previous load was about to write cancels
    /// it.** There is one delayed-write slot and this load claims it, so the
    /// earlier value is never seen at all — the same rule an ordinary
    /// instruction gets by writing its result after the settle, applied to the
    /// one case where the second write is itself delayed. The architecture
    /// calls a load delay slot that writes the load's own destination
    /// UNPREDICTABLE, so this is a choice; it is *this* choice because the
    /// alternative would make `lw $t0,x; lw $t0,y` behave differently from
    /// `lw $t0,x; addiu $t0,…`, and nothing about the hardware distinguishes
    /// them.
    fn deliver(&mut self, reg: u32, value: u32) {
        if self.cfg.arch.load_interlock {
            self.set_reg(reg, value);
            return;
        }
        if let Some((settled, old)) = self.settled
            && settled == reg & 31
        {
            self.set_reg(settled, old);
            self.settled = None;
        }
        self.issued = Some(PendingLoad { reg, value });
    }

    /// The value the unaligned-transfer instructions merge into.
    ///
    /// `LWL` and `LWR` read their destination register a stage later than an
    /// ordinary instruction does, which is why the manual allows the two of
    /// them to sit adjacent with no `NOP` between: the second one sees the
    /// first one's result. Every other instruction reads `rt` before the
    /// pending load settles; these read it after.
    fn merge_source(&self, rt: u32) -> u32 {
        self.reg(rt)
    }

    // -----------------------------------------------------------------
    // Address translation
    // -----------------------------------------------------------------

    /// Translate one virtual address to a physical one.
    ///
    /// The segment map is fixed — there is no register that moves a boundary —
    /// and only `kuseg` and `kseg2` reach the TLB. On a part with no TLB those
    /// two are the identity, which is why a PlayStation's two megabytes of RAM
    /// answer at `0x0000_0000`, `0x8000_0000` and `0xA000_0000` alike.
    fn translate(&mut self, vaddr: u32, kind: Access) -> Result<u32, Trap> {
        let segment = Segment::of(vaddr);
        if !self.st.cp0.kernel_mode() && !segment.user_accessible() {
            return Err(Trap::address(kind.address_error(), vaddr));
        }
        if !segment.mapped() {
            return Ok(Segment::unmapped_phys(vaddr));
        }
        if !self.cfg.arch.tlb {
            return Ok(vaddr);
        }
        let asid = self.st.cp0.asid();
        match self.st.tlb.lookup(vaddr, asid) {
            Lookup::Hit { pfn, writable, .. } => {
                if kind == Access::Store && !writable {
                    // The `D` bit is a write-enable, not a hardware-maintained
                    // dirty bit: the kernel clears it, catches the first store
                    // here, and sets it in the `Mod` handler.
                    return Err(Trap {
                        code: exc::MOD,
                        bad_vaddr: Some(vaddr),
                        refill: false,
                        ce: 0,
                        tlb: true,
                    });
                }
                Ok(pfn | (vaddr & (PAGE_SIZE - 1)))
            }
            Lookup::Invalid => Err(Trap {
                code: kind.tlb_error(),
                bad_vaddr: Some(vaddr),
                // A *matching* entry with `V` clear is not a refill: the page
                // table already has this page and the fast refill handler has
                // nothing to add, so it goes to the general vector.
                refill: false,
                ce: 0,
                tlb: true,
            }),
            Lookup::Conflict => {
                // Two entries matched. Real silicon can damage itself here and
                // latches `TS`, which is never cleared except by a reset.
                self.st.cp0.status |= status::TS;
                Err(Trap {
                    code: kind.tlb_error(),
                    bad_vaddr: Some(vaddr),
                    refill: segment.uses_refill_vector(),
                    ce: 0,
                    tlb: true,
                })
            }
            Lookup::Miss => Err(Trap {
                code: kind.tlb_error(),
                bad_vaddr: Some(vaddr),
                refill: segment.uses_refill_vector(),
                ce: 0,
                tlb: true,
            }),
        }
    }

    /// Whether data accesses are currently going to the isolated cache instead
    /// of the bus, and which array they land in.
    ///
    /// `Status.IsC` is what firmware uses to scribble the data cache — write a
    /// pattern, read it back, size the cache — and with `Status.SwC` also set,
    /// to invalidate the instruction cache. A model that let those stores
    /// through would silently corrupt guest RAM, which is precisely the
    /// "never a silent success" rule (CLAUDE.md, `MemResult`). Instruction
    /// fetches are unaffected: `IsC` isolates the *data* cache, which is why
    /// firmware can set it while running.
    fn isolated(&self) -> Option<bool> {
        if self.st.cp0.status & status::ISC == 0 {
            None
        } else {
            Some(self.st.cp0.status & status::SWC != 0)
        }
    }

    /// Where in the cache's data array an address lands, and which byte of the
    /// transfer goes there.
    ///
    /// The array is byte-addressed and direct-mapped, so the index is simply
    /// the low address bits — which is what makes a firmware "write a pattern
    /// through the whole cache and read it back" loop behave.
    fn cache_offsets(len: usize, phys: u32, width: Width, endian: Endian) -> [(usize, u32); 4] {
        let mask = (len - 1) as u32;
        let bytes = width.bytes() as u32;
        let mut out = [(0usize, 0u32); 4];
        for i in 0..bytes {
            // Byte `i` counted from the low address holds shift `s` of the
            // value, which depends on the byte order exactly as it does on the
            // bus.
            let shift = if endian.is_big() {
                8 * (bytes - 1 - i)
            } else {
                8 * i
            };
            out[i as usize] = ((phys.wrapping_add(i) & mask) as usize, shift);
        }
        out
    }

    /// Read from an isolated cache's data array.
    fn cache_read(&mut self, phys: u32, width: Width, swapped: bool, endian: Endian) -> u32 {
        self.charge();
        let array = if swapped {
            &self.st.icache
        } else {
            &self.st.dcache
        };
        if array.is_empty() {
            return 0;
        }
        let plan = Self::cache_offsets(array.len(), phys, width, endian);
        let mut value = 0u32;
        for (at, shift) in plan.iter().take(width.bytes() as usize) {
            value |= u32::from(array[*at]) << shift;
        }
        value
    }

    /// Write into an isolated cache's data array.
    fn cache_write(&mut self, phys: u32, width: Width, value: u32, swapped: bool, endian: Endian) {
        self.charge();
        let array = if swapped {
            &mut self.st.icache
        } else {
            &mut self.st.dcache
        };
        if array.is_empty() {
            return;
        }
        let plan = Self::cache_offsets(array.len(), phys, width, endian);
        for (at, shift) in plan.iter().take(width.bytes() as usize) {
            array[*at] = (value >> shift) as u8;
        }
    }

    // -----------------------------------------------------------------
    // Memory
    // -----------------------------------------------------------------

    /// Read `width` bytes from a virtual address, honouring alignment and the
    /// isolated cache.
    fn read(&mut self, vaddr: u32, width: Width, kind: Access) -> Result<u32, Trap> {
        if !width.is_aligned(u64::from(vaddr)) {
            return Err(Trap::address(kind.address_error(), vaddr));
        }
        let phys = self.translate(vaddr, kind)?;
        if kind != Access::Fetch
            && let Some(swapped) = self.isolated()
        {
            let endian = self.st.cp0.data_endian(self.cfg.endian);
            return Ok(self.cache_read(phys, width, swapped, endian));
        }
        self.charge();
        match self.space.read(u64::from(phys), width, self.attrs) {
            Ok(v) => Ok(self.reversed(v as u32, width, kind)),
            Err(_) => {
                self.st.faults = self.st.faults.wrapping_add(1);
                Err(Trap::address(kind.bus_error(), vaddr))
            }
        }
    }

    /// Write `width` bytes to a virtual address.
    fn write(&mut self, vaddr: u32, width: Width, value: u32) -> Result<(), Trap> {
        if !width.is_aligned(u64::from(vaddr)) {
            return Err(Trap::address(exc::ADES, vaddr));
        }
        let phys = self.translate(vaddr, Access::Store)?;
        if let Some(swapped) = self.isolated() {
            let endian = self.st.cp0.data_endian(self.cfg.endian);
            self.cache_write(phys, width, value, swapped, endian);
            return Ok(());
        }
        let value = self.reversed(value, width, Access::Store);
        self.charge();
        match self
            .space
            .write(u64::from(phys), width, u64::from(value), self.attrs)
        {
            Ok(()) => Ok(()),
            Err(_) => {
                self.st.faults = self.st.faults.wrapping_add(1);
                Err(Trap::address(exc::DBE, vaddr))
            }
        }
    }

    /// Apply `Status.RE` to a data value.
    ///
    /// The address space already assembles bytes in the region's byte order,
    /// which a machine file matches to the core's endianness pin. `RE` flips
    /// that for **user-mode data accesses only**, which is how a big-endian
    /// kernel runs a little-endian user program. Modelled as a byte swap of
    /// the assembled value, which is equivalent for the aligned accesses that
    /// are the only ones an R3000 performs directly.
    fn reversed(&self, value: u32, width: Width, kind: Access) -> u32 {
        if kind == Access::Fetch || self.st.cp0.data_endian(self.cfg.endian) == self.cfg.endian {
            return value;
        }
        match width {
            Width::U16 => u32::from((value as u16).swap_bytes()),
            Width::U32 => value.swap_bytes(),
            _ => value,
        }
    }

    /// The unaligned half of a store: write only the bytes the transfer
    /// covers, and none of the others.
    ///
    /// Real hardware does this in **one bus cycle with byte enables**: `SWL`
    /// and `SWR` drive between one and four byte strobes and never read. The
    /// obvious alternative — read the aligned word, merge, write it back —
    /// gets the same answer in RAM and is wrong twice over: it performs a read
    /// the silicon does not, which a read-sensitive MMIO register would
    /// notice, and it writes bytes the silicon leaves alone, which a
    /// write-only register would.
    ///
    /// `core::space` has no byte-enable concept, so the transfer is expressed
    /// as up to four byte writes and **charged as the one bus cycle it is**.
    /// That is an accounting decision about an API artefact, not a claim about
    /// the hardware.
    fn store_partial(
        &mut self,
        addr: u32,
        value: u32,
        left: bool,
        endian: Endian,
    ) -> Result<(), Trap> {
        let b = addr & 3;
        let big = endian.is_big();
        // Where in the aligned word the transfer starts and how many bytes it
        // covers, derived from the same shifts [`isa::swl`] and [`isa::swr`]
        // use — the tests assert the two agree, so this is a second expression
        // of one rule rather than a second rule.
        let (first, count) = match (left, big) {
            (true, true) | (false, false) => (b, 4 - b),
            _ => (0, b + 1),
        };
        let aligned = addr & !3;
        // One bus cycle, whatever the byte-enable pattern.
        self.charge();
        for i in 0..count {
            // Which byte of the register goes to this address.
            let take = match (left, big) {
                (true, true) => 3 - i,
                (true, false) => i + 3 - b,
                (false, true) => b - i,
                (false, false) => i,
            };
            let datum = u64::from((value >> (8 * take)) & 0xff);
            let at = aligned.wrapping_add(first + i);
            let phys = self.translate(at, Access::Store)?;
            if let Some(swapped) = self.isolated() {
                self.cache_write(phys, Width::U8, datum as u32, swapped, endian);
                continue;
            }
            if self
                .space
                .write(u64::from(phys), Width::U8, datum, self.attrs)
                .is_err()
            {
                self.st.faults = self.st.faults.wrapping_add(1);
                return Err(Trap::address(exc::DBE, at));
            }
        }
        Ok(())
    }

    /// Fetch the instruction at `pc`.
    fn fetch(&mut self, pc: u32) -> Result<u32, Trap> {
        self.read(pc, Width::U32, Access::Fetch)
    }

    // -----------------------------------------------------------------
    // Exceptions
    // -----------------------------------------------------------------

    /// Which interrupt requests are pending, enabled and unmasked.
    ///
    /// The R3000 rule is simpler than a privileged architecture's: one global
    /// enable bit, one eight-bit mask, and no delegation. `Cause.IP[7:2]` is
    /// the *live level* of the six pins rather than a latch, which is why the
    /// pin state is read here rather than stored.
    fn pending_interrupt(&self) -> Option<u32> {
        if !self.st.cp0.interrupts_enabled() {
            return None;
        }
        let ready = self.st.cp0.ready_interrupts(self.lines.hw());
        if ready == 0 { None } else { Some(ready) }
    }

    /// Take an exception: publish the cause, push the mode stack, and vector.
    fn enter_exception(&mut self, trap: Trap) {
        let bd = self.in_delay;
        // `EPC` names the **branch**, not the delay slot, so the return
        // re-executes the branch and its decision is remade. The delay slot is
        // always the word after the branch, so the branch is four back.
        let epc = if bd {
            self.this_pc.wrapping_sub(4)
        } else {
            self.this_pc
        };
        let cp0 = &mut self.st.cp0;
        cp0.epc = epc;
        cp0.cause = (cp0.cause & !(cause_bits::EXC_CODE | cause_bits::CE | cause_bits::BD))
            | ((trap.code << cause_bits::EXC_SHIFT) & cause_bits::EXC_CODE)
            | ((trap.ce << cause_bits::CE_SHIFT) & cause_bits::CE)
            | if bd { cause_bits::BD } else { 0 };
        if let Some(vaddr) = trap.bad_vaddr {
            cp0.bad_vaddr = vaddr;
            if trap.tlb {
                // The refill handler reads the faulting page out of `EntryHi`
                // and indexes its page table with `Context`, so both are set
                // before the handler runs. The ASID is left alone: the fault
                // belongs to the address space that is already current.
                cp0.entry_hi = (cp0.entry_hi & 0x0000_0fc0) | (vaddr & 0xffff_f000);
                cp0.set_context_vpn(vaddr);
            }
        }
        cp0.push_mode();
        let bev = cp0.status & status::BEV != 0;
        let vector = match (trap.refill, bev) {
            (true, false) => REFILL_VECTOR,
            (true, true) => REFILL_VECTOR_BEV,
            (false, false) => GENERAL_VECTOR,
            (false, true) => GENERAL_VECTOR_BEV,
        };
        self.st.pc = vector;
        self.st.next_pc = vector.wrapping_add(4);
        self.st.in_delay = false;
        self.issued = None;
    }

    // -----------------------------------------------------------------
    // The instruction body
    // -----------------------------------------------------------------

    /// Fetch, decode and execute one instruction.
    #[allow(clippy::too_many_lines)]
    fn execute(&mut self) -> Result<(), Trap> {
        let word = self.fetch(self.this_pc)?;

        // Advance the pair *before* executing, so a branch is nothing but an
        // overwrite of `next_pc` and no arm has to reason about ordering.
        self.st.pc = self.st.next_pc;
        self.st.next_pc = self.st.next_pc.wrapping_add(4);
        self.st.in_delay = false;

        let insn = isa::decode(word).ok_or(Trap::bare(exc::RI))?;
        self.check_requirement(insn.req)?;

        let rs = isa::rs(word);
        let rt = isa::rt(word);
        let rd = isa::rd(word);
        // Read both operands before the pending load settles: that ordering
        // *is* the load delay slot.
        let a = self.reg(rs);
        let b = self.reg(rt);
        self.settle();

        // Where the branch displacement and the jump region are measured from,
        // and where a link register points.
        //
        // Both are relative to the **delay slot**, and the delay slot is the
        // instruction that really runs next — which is `st.pc`, not
        // `this_pc + 4`. The two are the same except when this instruction is
        // *itself* in a delay slot, where `st.pc` already holds the earlier
        // branch's target and the earlier branch's successor is therefore this
        // one's delay slot. Computing from `this_pc + 4` gets the common case
        // right and the nested case wrong, in the top four bits of a `J`'s
        // target and in the whole of a conditional branch's.
        let delay_pc = self.st.pc;
        let link = delay_pc.wrapping_add(4);

        match insn.op {
            // -- shifts ------------------------------------------------------
            Op::Sll => self.set_reg(rd, b << isa::sa(word)),
            Op::Srl => self.set_reg(rd, b >> isa::sa(word)),
            Op::Sra => self.set_reg(rd, ((b as i32) >> isa::sa(word)) as u32),
            Op::Sllv => self.set_reg(rd, b << (a & 31)),
            Op::Srlv => self.set_reg(rd, b >> (a & 31)),
            Op::Srav => self.set_reg(rd, ((b as i32) >> (a & 31)) as u32),

            // -- register jumps ---------------------------------------------
            Op::Jr => self.branch_to(a),
            Op::Jalr => {
                // `rd` is written literally: `jalr $ra, $rs` is an assembler
                // convention for the one-operand form, not a hardware default,
                // so `jalr $zero, $rs` discards the link rather than quietly
                // writing `$31`. The link goes in *before* the jump takes
                // effect, which is what makes `jalr $ra, $ra` legal.
                self.set_reg(rd, link);
                self.branch_to(a);
            }

            // -- traps -------------------------------------------------------
            Op::Syscall => return Err(Trap::bare(exc::SYS)),
            Op::Break => return Err(Trap::bare(exc::BP)),

            // -- HI and LO ---------------------------------------------------
            Op::Mfhi => self.set_reg(rd, self.st.hi),
            Op::Mflo => self.set_reg(rd, self.st.lo),
            Op::Mthi => self.st.hi = a,
            Op::Mtlo => self.st.lo = a,
            Op::Mult => {
                let p = i64::from(a as i32).wrapping_mul(i64::from(b as i32));
                self.st.lo = p as u32;
                self.st.hi = (p >> 32) as u32;
            }
            Op::Multu => {
                let p = u64::from(a).wrapping_mul(u64::from(b));
                self.st.lo = p as u32;
                self.st.hi = (p >> 32) as u32;
            }
            Op::Div => {
                let (hi, lo) = divide_signed(a as i32, b as i32);
                self.st.hi = hi;
                self.st.lo = lo;
            }
            Op::Divu => {
                let (hi, lo) = divide_unsigned(a, b);
                self.st.hi = hi;
                self.st.lo = lo;
            }

            // -- three-register arithmetic -----------------------------------
            Op::Add => self.set_reg(rd, checked_add(a, b)?),
            Op::Addu => self.set_reg(rd, a.wrapping_add(b)),
            Op::Sub => self.set_reg(rd, checked_sub(a, b)?),
            Op::Subu => self.set_reg(rd, a.wrapping_sub(b)),
            Op::And => self.set_reg(rd, a & b),
            Op::Or => self.set_reg(rd, a | b),
            Op::Xor => self.set_reg(rd, a ^ b),
            Op::Nor => self.set_reg(rd, !(a | b)),
            Op::Slt => self.set_reg(rd, u32::from((a as i32) < (b as i32))),
            Op::Sltu => self.set_reg(rd, u32::from(a < b)),

            // -- branches ----------------------------------------------------
            Op::Beq => {
                if a == b {
                    self.branch_to(isa::branch_target(delay_pc, word));
                }
            }
            Op::Bne => {
                if a != b {
                    self.branch_to(isa::branch_target(delay_pc, word));
                }
            }
            Op::Blez => {
                if (a as i32) <= 0 {
                    self.branch_to(isa::branch_target(delay_pc, word));
                }
            }
            Op::Bgtz => {
                if (a as i32) > 0 {
                    self.branch_to(isa::branch_target(delay_pc, word));
                }
            }
            Op::Bltz => {
                if (a as i32) < 0 {
                    self.branch_to(isa::branch_target(delay_pc, word));
                }
            }
            Op::Bgez => {
                if (a as i32) >= 0 {
                    self.branch_to(isa::branch_target(delay_pc, word));
                }
            }
            Op::Bltzal | Op::Bgezal => {
                // The link happens **whether or not the branch is taken**, and
                // it happens before the comparison's effect, so
                // `bltzal $ra, …` compares the old `$ra` and then overwrites
                // it. Kane & Heinrich state both halves explicitly.
                let taken = if insn.op == Op::Bltzal {
                    (a as i32) < 0
                } else {
                    (a as i32) >= 0
                };
                self.set_reg(31, link);
                if taken {
                    self.branch_to(isa::branch_target(delay_pc, word));
                }
            }
            Op::J => self.branch_to(isa::jump_target(delay_pc, word)),
            Op::Jal => {
                self.set_reg(31, link);
                self.branch_to(isa::jump_target(delay_pc, word));
            }

            // -- immediate arithmetic ----------------------------------------
            Op::Addi => self.set_reg(rt, checked_add(a, isa::simm(word))?),
            Op::Addiu => self.set_reg(rt, a.wrapping_add(isa::simm(word))),
            Op::Slti => self.set_reg(rt, u32::from((a as i32) < (isa::simm(word) as i32))),
            Op::Sltiu => self.set_reg(rt, u32::from(a < isa::simm(word))),
            Op::Andi => self.set_reg(rt, a & isa::imm(word)),
            Op::Ori => self.set_reg(rt, a | isa::imm(word)),
            Op::Xori => self.set_reg(rt, a ^ isa::imm(word)),
            Op::Lui => self.set_reg(rt, isa::imm(word) << 16),

            // -- loads -------------------------------------------------------
            Op::Lb => {
                let v = self.read(a.wrapping_add(isa::simm(word)), Width::U8, Access::Load)?;
                self.deliver(rt, v as u8 as i8 as i32 as u32);
            }
            Op::Lbu => {
                let v = self.read(a.wrapping_add(isa::simm(word)), Width::U8, Access::Load)?;
                self.deliver(rt, v & 0xff);
            }
            Op::Lh => {
                let v = self.read(a.wrapping_add(isa::simm(word)), Width::U16, Access::Load)?;
                self.deliver(rt, v as u16 as i16 as i32 as u32);
            }
            Op::Lhu => {
                let v = self.read(a.wrapping_add(isa::simm(word)), Width::U16, Access::Load)?;
                self.deliver(rt, v & 0xffff);
            }
            Op::Lw => {
                let v = self.read(a.wrapping_add(isa::simm(word)), Width::U32, Access::Load)?;
                self.deliver(rt, v);
            }
            Op::Lwl | Op::Lwr => {
                let addr = a.wrapping_add(isa::simm(word));
                // Deliberately no alignment check: an unaligned address is the
                // entire point, and the access is to the aligned word that
                // contains it, so it never crosses a page.
                let word_value = self.read(addr & !3, Width::U32, Access::Load)?;
                let endian = self.st.cp0.data_endian(self.cfg.endian);
                let old = self.merge_source(rt);
                let merged = if insn.op == Op::Lwl {
                    isa::lwl(old, word_value, addr, endian)
                } else {
                    isa::lwr(old, word_value, addr, endian)
                };
                self.deliver(rt, merged);
            }

            // -- stores ------------------------------------------------------
            Op::Sb => self.write(a.wrapping_add(isa::simm(word)), Width::U8, b & 0xff)?,
            Op::Sh => self.write(a.wrapping_add(isa::simm(word)), Width::U16, b & 0xffff)?,
            Op::Sw => self.write(a.wrapping_add(isa::simm(word)), Width::U32, b)?,
            Op::Swl | Op::Swr => {
                let addr = a.wrapping_add(isa::simm(word));
                let endian = self.st.cp0.data_endian(self.cfg.endian);
                self.store_partial(addr, b, insn.op == Op::Swl, endian)?;
            }

            // -- coprocessor 0 -----------------------------------------------
            Op::Mfc0 => {
                // A coprocessor-to-register move has the same one-instruction
                // delay a load does, and for the same reason: the value
                // arrives a stage late.
                let v = self.read_cp0(rd);
                self.deliver(rt, v);
            }
            Op::Mtc0 => self.write_cp0(rd, b),
            Op::Tlbr => {
                let entry = self.st.tlb.entry(self.tlb_index());
                self.st.cp0.entry_hi = entry.hi;
                self.st.cp0.entry_lo = entry.lo;
            }
            Op::Tlbwi => {
                let index = self.tlb_index();
                self.write_tlb(index);
            }
            Op::Tlbwr => {
                let index = self.st.cp0.random;
                self.write_tlb(index);
            }
            Op::Tlbp => {
                match self.st.tlb.probe(self.st.cp0.entry_hi) {
                    Some(i) => self.st.cp0.index = (i & 0x3f) << 8,
                    // Bit 31 is `P`, the probe-failed flag. The index field is
                    // left undefined by the manual; leaving it as it was is
                    // the least surprising choice and is documented as such.
                    None => self.st.cp0.index |= 0x8000_0000,
                }
            }
            Op::Rfe => {
                // `RFE` does **not** jump. It pops the mode stack and nothing
                // else, and it is executed in the delay slot of the `jr $k0`
                // that does the jumping.
                self.st.cp0.pop_mode();
                self.attrs = self.attrs.with_privileged(self.st.cp0.kernel_mode());
            }

            // -- the coprocessors this core does not have --------------------
            //
            // Unreachable: `check_requirement` has already refused every one
            // of these, because no configuration this core accepts claims a
            // coprocessor it cannot execute. The arm exists so that adding a
            // coprocessor is a compile error here rather than a silent no-op.
            Op::Cop1 | Op::Cop2 | Op::Cop3 => {
                return Err(Trap::coprocessor(insn.req.coprocessor().unwrap_or(0)));
            }
            Op::Lwc1 | Op::Lwc2 | Op::Lwc3 | Op::Swc1 | Op::Swc2 | Op::Swc3 => {
                return Err(Trap::coprocessor(insn.req.coprocessor().unwrap_or(0)));
            }
        }
        // **Every** branch has a delay slot, taken or not. The delay slot is
        // fetched before the condition is resolved, so an exception in it sets
        // `Cause.BD` and points `EPC` at the branch whichever way the branch
        // went — and a model that only set the flag on the taken path would
        // have identical control flow and a wrong `EPC` exactly when an
        // interrupt lands after a branch that fell through.
        //
        // Set from the table's own [`Insn::is_branch`] rather than from each
        // arm, so an instruction added to `isa::TABLE` with a control-transfer
        // format cannot forget it.
        if insn.is_branch() {
            self.st.in_delay = true;
        }
        Ok(())
    }

    /// Take a branch: the target lands in `next_pc`.
    ///
    /// The delay-slot flag is *not* set here — see the end of `execute`, which
    /// sets it for taken and untaken branches alike.
    ///
    /// A branch *in* a delay slot is UNPREDICTABLE on MIPS I. What happens
    /// here is that the second branch wins and its own delay slot is the
    /// instruction at the first branch's target — a defensible reading, and
    /// documented rather than accidental.
    fn branch_to(&mut self, target: u32) {
        self.st.next_pc = target;
    }

    /// Refuse an instruction the configured part does not have.
    ///
    /// `ROADMAP.md` §6.1.1: an absent instruction must trap, and it must trap
    /// as the *right* exception — a missing coprocessor is `CpU` with
    /// `Cause.CE` naming it, which is how a guest probes for a GTE, while a
    /// missing TLB is a plain reserved instruction.
    fn check_requirement(&self, req: Req) -> Result<(), Trap> {
        match req {
            Req::Base => Ok(()),
            Req::Cop0 | Req::Tlb => {
                if !self.st.cp0.coprocessor_usable(0) {
                    return Err(Trap::coprocessor(0));
                }
                if req == Req::Tlb && !self.cfg.arch.tlb {
                    return Err(Trap::bare(exc::RI));
                }
                Ok(())
            }
            Req::Cop1 | Req::Cop2 | Req::Cop3 => {
                let n = req.coprocessor().unwrap_or(0);
                if self.cfg.arch.coprocessor(n) && self.st.cp0.coprocessor_usable(n) {
                    // No configuration this build accepts reaches here: the
                    // coprocessors are not implemented, so `Arch` never claims
                    // one. Left as a fall-through rather than an
                    // `unreachable!` so that implementing one is an ordinary
                    // change.
                    Ok(())
                } else {
                    Err(Trap::coprocessor(n))
                }
            }
        }
    }

    /// Which TLB entry `Index` names.
    fn tlb_index(&self) -> u32 {
        (self.st.cp0.index >> 8) & ((TLB_ENTRIES - 1) as u32)
    }

    /// Copy `EntryHi`/`EntryLo` into one TLB entry.
    fn write_tlb(&mut self, index: u32) {
        let entry = TlbEntry {
            hi: self.st.cp0.entry_hi,
            lo: self.st.cp0.entry_lo,
        };
        self.st.tlb.set_entry(index, entry);
    }

    // -----------------------------------------------------------------
    // Coprocessor 0's register file
    // -----------------------------------------------------------------

    /// Read one CP0 register.
    ///
    /// The sixteen numbers an R3000 does not implement read as zero rather
    /// than raising anything: the manual leaves them undefined and reading
    /// zero is what the silicon does with an unconnected bus.
    fn read_cp0(&self, n: u32) -> u32 {
        let cp0 = &self.st.cp0;
        match n {
            reg::INDEX => cp0.index,
            reg::RANDOM => (cp0.random & 0x3f) << 8,
            reg::ENTRY_LO => cp0.entry_lo,
            reg::CONTEXT => cp0.context,
            reg::BAD_VADDR => cp0.bad_vaddr,
            reg::ENTRY_HI => cp0.entry_hi,
            reg::STATUS => cp0.status,
            reg::CAUSE => cp0.cause_with(self.lines.hw()),
            reg::EPC => cp0.epc,
            reg::PRID => cp0.prid,
            reg::BPC => cp0.debug[0],
            reg::BDA => cp0.debug[1],
            reg::JUMP_DEST => cp0.debug[2],
            reg::DCIC => cp0.debug[3],
            reg::BDAM => cp0.debug[4],
            reg::BPCM => cp0.debug[5],
            _ => 0,
        }
    }

    /// Write one CP0 register, honouring which bits are software-writable.
    fn write_cp0(&mut self, n: u32, value: u32) {
        let usable = self.cfg.arch.coprocessor_mask();
        let cp0 = &mut self.st.cp0;
        match n {
            // `P` and the six-bit index. `Random` is read-only.
            reg::INDEX => cp0.index = value & 0x8000_3f00,
            reg::RANDOM | reg::BAD_VADDR | reg::PRID => {}
            reg::ENTRY_LO => cp0.entry_lo = value & 0xffff_ff00,
            // Only `PTEBase` is writable; `BadVPN` is hardware's.
            reg::CONTEXT => cp0.context = (cp0.context & 0x001f_fffc) | (value & 0xffe0_0000),
            reg::ENTRY_HI => cp0.entry_hi = value & 0xffff_ffc0,
            reg::STATUS => {
                let mask = status::WRITABLE & usable;
                cp0.status = (cp0.status & !mask) | (value & mask);
            }
            // The only bits of `Cause` software may write are the two software
            // interrupt requests. Everything else is hardware's, and letting a
            // guest write `ExcCode` would make the register a lie.
            reg::CAUSE => {
                cp0.cause = (cp0.cause & !cause_bits::WRITABLE) | (value & cause_bits::WRITABLE);
            }
            reg::EPC => cp0.epc = value,
            reg::BPC => cp0.debug[0] = value,
            reg::BDA => cp0.debug[1] = value,
            reg::JUMP_DEST => cp0.debug[2] = value,
            reg::DCIC => cp0.debug[3] = value,
            reg::BDAM => cp0.debug[4] = value,
            reg::BPCM => cp0.debug[5] = value,
            _ => {}
        }
        if n == reg::STATUS {
            self.attrs = self.attrs.with_privileged(self.st.cp0.kernel_mode());
        }
    }
}

/// `ADD` and `ADDI`: trap on signed overflow, and write nothing when they do.
fn checked_add(a: u32, b: u32) -> Result<u32, Trap> {
    match (a as i32).checked_add(b as i32) {
        Some(v) => Ok(v as u32),
        None => Err(Trap::bare(exc::OV)),
    }
}

/// `SUB`: trap on signed overflow.
fn checked_sub(a: u32, b: u32) -> Result<u32, Trap> {
    match (a as i32).checked_sub(b as i32) {
        Some(v) => Ok(v as u32),
        None => Err(Trap::bare(exc::OV)),
    }
}

/// Signed divide, returning `(HI, LO)`.
///
/// **MIPS does not trap on division by zero** — there is no divide exception
/// in the architecture at all, and a compiler that wants one emits an explicit
/// compare and `BREAK`. The instruction manual calls the result UNPREDICTABLE;
/// what R3000A silicon actually produces, and what these values are, is
/// documented hardware behaviour rather than an architectural guarantee:
///
/// * divisor zero — `LO` is `-1` when the dividend is non-negative and `1`
///   when it is negative (the quotient saturates towards the dividend's sign),
///   and `HI` is the dividend.
/// * the one overflowing case, `-2^31 / -1`, whose true quotient does not fit
///   — `LO` is `-2^31` and `HI` is zero.
fn divide_signed(a: i32, b: i32) -> (u32, u32) {
    if b == 0 {
        let lo: i32 = if a >= 0 { -1 } else { 1 };
        (a as u32, lo as u32)
    } else if a == i32::MIN && b == -1 {
        (0, i32::MIN as u32)
    } else {
        ((a % b) as u32, (a / b) as u32)
    }
}

/// Unsigned divide, returning `(HI, LO)`.
///
/// Divisor zero gives an all-ones quotient and leaves the dividend in `HI`,
/// and does not trap.
fn divide_unsigned(a: u32, b: u32) -> (u32, u32) {
    if b == 0 {
        (a, u32::MAX)
    } else {
        (a % b, a / b)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn division_by_zero_does_not_trap_and_has_defined_results() {
        assert_eq!(divide_signed(7, 0), (7, 0xffff_ffff));
        assert_eq!(divide_signed(-7, 0), ((-7i32) as u32, 1));
        assert_eq!(divide_signed(0, 0), (0, 0xffff_ffff));
        assert_eq!(divide_unsigned(7, 0), (7, 0xffff_ffff));
        assert_eq!(divide_unsigned(0, 0), (0, 0xffff_ffff));
    }

    #[test]
    fn the_one_overflowing_division_has_a_defined_result() {
        // -2^31 / -1 does not fit in 32 bits. The hardware leaves the dividend
        // in LO and zero in HI rather than faulting, so a model that used
        // Rust's `/` here would panic in debug and produce a different answer
        // in release — exactly the profile dependence CLAUDE.md forbids.
        assert_eq!(divide_signed(i32::MIN, -1), (0, 0x8000_0000));
        assert_eq!(divide_signed(i32::MIN, 1), (0, 0x8000_0000));
    }

    #[test]
    fn ordinary_division_truncates_towards_zero() {
        assert_eq!(divide_signed(7, 2), (1, 3));
        assert_eq!(divide_signed(-7, 2), ((-1i32) as u32, (-3i32) as u32));
        assert_eq!(divide_signed(7, -2), (1, (-3i32) as u32));
        assert_eq!(divide_unsigned(7, 2), (1, 3));
    }

    #[test]
    fn overflow_is_signed_and_the_unsigned_forms_do_not_check_it() {
        assert!(checked_add(0x7fff_ffff, 1).is_err());
        assert_eq!(checked_add(0xffff_ffff, 1), Ok(0));
        assert!(checked_sub(0x8000_0000, 1).is_err());
        assert_eq!(checked_sub(0, 1), Ok(0xffff_ffff));
    }
}
