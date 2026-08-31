//! The interpreter.
//!
//! # One cycle is one bus access
//!
//! RISC-V does not architecturally define instruction timing — that is a
//! property of a particular implementation's pipeline, not of the ISA — so
//! there is no cycle table here and no cycle counter that ticks independently
//! of the bus. What this interpreter counts is **accesses**: an instruction
//! fetch is one (two for an uncompressed instruction, because it is fetched as
//! two halfwords and either half may fault on its own page), each page-table
//! read during a walk is one, and each load or store is one. That is a fact
//! about the machine being modelled rather than an invention, and it is what
//! `ROADMAP.md` §6 asks for: accounting driven through the bus.
//!
//! # The compressed extension has no code here
//!
//! Volume I defines every 16-bit encoding as an alias for exactly one 32-bit
//! instruction, so [`isa::expand`] turns a compressed instruction into its
//! base form and the rest of this file never learns that `C` exists. One
//! implementation, not two that can disagree.
//!
//! # Sources
//!
//! *The RISC-V Instruction Set Manual, Volume I: Unprivileged ISA* and
//! *Volume II: Privileged Architecture* (both CC-BY-4.0). Specific citations
//! sit next to the rules they justify: the division-by-zero results, the trap
//! delegation sequence, the `MRET`/`SRET` field shuffle, the interrupt
//! priority order, and the NaN-boxing rule for single-precision values held in
//! double-precision registers.

use crate::core::space::{AddressSpace, MemAttrs};
use crate::core::value::Width;

use super::csr::{self, Csrs, Lines, Priv, cause, irq, status};
use super::float::{self, B32, B64, Format, Round};
use super::isa::{self, Op, Xlen};
use super::mmu::{self, Access, Tlb};
use super::{Config, PAGE_MASK};

/// A trap the current instruction raised.
///
/// Carries the value `mtval`/`stval` will hold, because that value is decided
/// where the fault happens — a page fault reports the address, an illegal
/// instruction reports the encoding — and reconstructing it later is how the
/// two drift apart.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct Trap {
    /// The exception code, without the interrupt bit.
    pub cause: u64,
    /// The value to write to `mtval` or `stval`.
    pub tval: u64,
}

impl Trap {
    /// A trap with no meaningful trap value.
    const fn bare(cause: u64) -> Trap {
        Trap { cause, tval: 0 }
    }

    /// An illegal-instruction trap reporting the encoding that caused it.
    const fn illegal(encoding: u64) -> Trap {
        Trap {
            cause: cause::ILLEGAL_INSN,
            tval: encoding,
        }
    }
}

/// The architectural state of one hart.
///
/// Split from the device wrapper because the interrupt *lines* live outside
/// the execution lock (see [`Lines`]), and because the TLB beside it is
/// derived state that a snapshot must not carry.
#[derive(Debug, Clone)]
pub(super) struct State {
    /// The integer register file. `x[0]` is architecturally zero and is kept
    /// zero rather than special-cased on read.
    pub x: [u64; 32],
    /// The floating-point register file, always 64 bits wide, with
    /// single-precision values NaN-boxed.
    pub f: [u64; 32],
    /// The program counter.
    pub pc: u64,
    /// Every CSR, and the current privilege mode.
    pub csrs: Csrs,
    /// The address `LR` reserved, if any.
    pub reservation: Option<u64>,
    /// Bus accesses since reset.
    pub cycles: u64,
    /// Cycles already executed past the last budget, owed to the next one.
    pub debt: u64,
    /// Whether a `WFI` is stalling the hart.
    pub wfi: bool,
    /// How many accesses the address space refused.
    pub faults: u64,
}

impl State {
    /// The reset state for a given configuration.
    pub(super) fn new(cfg: &Config) -> State {
        State {
            x: [0; 32],
            f: [0; 32],
            pc: cfg.xlen.trunc(cfg.reset_vector),
            csrs: Csrs::new(cfg.xlen, cfg.ext, cfg.hartid, cfg.pmp_count),
            reservation: None,
            cycles: 0,
            debt: 0,
            wfi: false,
            faults: 0,
        }
    }
}

/// One step's worth of execution, borrowing everything it needs.
pub(super) struct Exec<'a> {
    st: &'a mut State,
    tlb: &'a mut Tlb,
    space: &'a AddressSpace,
    cfg: &'a Config,
    lines: &'a Lines,
    attrs: MemAttrs,
    /// Cycles charged by this step.
    used: u64,
    /// Where execution continues, unless a jump overrides it.
    next_pc: u64,
    /// The address of the instruction being executed, for `mepc`.
    this_pc: u64,
    /// Set when this instruction wrote `minstret` itself.
    ///
    /// Volume II: a write to `minstret` takes precedence over the increment
    /// the writing instruction would otherwise cause, so the value written is
    /// the value the *next* instruction reads. Without this the counter is
    /// always one ahead of what software asked for.
    wrote_instret: bool,
}

/// Physical memory as the page-table walker sees it.
///
/// A separate borrow of the address space, because the walker runs while
/// `Exec` is holding its own `&mut` on the state and the two must not alias.
struct Walker<'a> {
    space: &'a AddressSpace,
    attrs: MemAttrs,
    accesses: u64,
}

impl mmu::PhysMem for Walker<'_> {
    fn read_pte(&mut self, addr: u64, bytes: u32) -> Option<u64> {
        self.accesses += 1;
        let width = Width::from_bytes(u64::from(bytes))?;
        self.space.read(addr, width, self.attrs).ok()
    }

    fn write_pte(&mut self, addr: u64, bytes: u32, value: u64) -> Option<()> {
        self.accesses += 1;
        let width = Width::from_bytes(u64::from(bytes))?;
        self.space.write(addr, width, value, self.attrs).ok()
    }
}

impl<'a> Exec<'a> {
    /// Borrow a hart for one step.
    pub(super) fn new(
        st: &'a mut State,
        tlb: &'a mut Tlb,
        space: &'a AddressSpace,
        cfg: &'a Config,
        lines: &'a Lines,
    ) -> Exec<'a> {
        let attrs = MemAttrs::DEFAULT.with_requester(cfg.requester);
        let this_pc = st.pc;
        Exec {
            st,
            tlb,
            space,
            cfg,
            lines,
            attrs,
            used: 0,
            next_pc: this_pc,
            this_pc,
            wrote_instret: false,
        }
    }

    /// Execute one instruction, or take one trap.
    ///
    /// Returns the bus accesses charged, which is never zero: even a stalled
    /// `WFI` charges one, so a scheduler always makes progress.
    pub(super) fn step(&mut self) -> u64 {
        if let Some(code) = self.pending_interrupt() {
            self.enter_trap(Trap::bare(code), true);
            self.st.pc = self.next_pc;
            return self.used.max(1);
        }
        if self.st.wfi {
            // Volume II: the stall ends when an enabled interrupt becomes
            // pending. It is also architecturally legal to wake for any other
            // reason, but waking only on an interrupt is the behaviour a guest
            // can actually reason about.
            if self.lines.pending() & self.st.csrs.mie == 0 {
                self.charge();
                return self.used;
            }
            self.st.wfi = false;
        }

        self.this_pc = self.st.pc;
        self.next_pc = self.st.pc;
        match self.execute() {
            Ok(()) => {
                if self.st.csrs.mcountinhibit & 0b100 == 0 && !self.wrote_instret {
                    self.st.csrs.minstret = self.st.csrs.minstret.wrapping_add(1);
                }
                self.st.pc = self.cfg.xlen.trunc(self.next_pc);
            }
            Err(trap) => {
                self.enter_trap(trap, false);
                self.st.pc = self.next_pc;
            }
        }
        self.used.max(1)
    }

    // -----------------------------------------------------------------
    // The clock: one access, one cycle
    // -----------------------------------------------------------------

    /// Charge one bus access.
    #[inline]
    fn charge(&mut self) {
        self.used += 1;
        self.st.cycles = self.st.cycles.wrapping_add(1);
        if self.st.csrs.mcountinhibit & 1 == 0 {
            self.st.csrs.mcycle = self.st.csrs.mcycle.wrapping_add(1);
        }
    }

    // -----------------------------------------------------------------
    // Registers
    // -----------------------------------------------------------------

    /// Read an integer register. `x0` reads as zero.
    #[inline]
    fn x(&self, i: u32) -> u64 {
        self.st.x[i as usize]
    }

    /// Write an integer register, sign-extended to the configured width.
    ///
    /// Writes to `x0` are discarded rather than written and masked back:
    /// keeping the slot zero means every read is unconditional.
    #[inline]
    fn set_x(&mut self, i: u32, value: u64) {
        if i != 0 {
            self.st.x[i as usize] = self.cfg.xlen.sext(value);
        }
    }

    /// Read a double-precision register.
    #[inline]
    fn f(&self, i: u32) -> u64 {
        self.st.f[i as usize]
    }

    /// Write a double-precision register, marking the FP state dirty.
    #[inline]
    fn set_f(&mut self, i: u32, value: u64) {
        self.st.f[i as usize] = value;
        self.st.csrs.dirty_fp();
    }

    /// Read a single-precision register, honouring the NaN box.
    ///
    /// Volume I, "NaN Boxing of Narrower Values": a single-precision value in
    /// a double-precision register is valid only when the upper 32 bits are
    /// all ones. Anything else is *not* the number it looks like — it is
    /// interpreted as the canonical NaN, so a program that stores a double and
    /// reads it back as a float gets a NaN rather than nonsense.
    #[inline]
    fn fs(&self, i: u32) -> u64 {
        let v = self.st.f[i as usize];
        if v >> 32 == 0xffff_ffff {
            v & 0xffff_ffff
        } else {
            B32::CANONICAL_NAN
        }
    }

    /// Write a single-precision register, NaN-boxing it.
    #[inline]
    fn set_fs(&mut self, i: u32, value: u64) {
        self.set_f(i, 0xffff_ffff_0000_0000 | (value & 0xffff_ffff));
    }

    /// Accumulate floating-point exception flags into `fcsr`.
    #[inline]
    fn raise(&mut self, flags: u32) {
        if flags != 0 {
            self.st.csrs.fcsr |= u64::from(flags);
            self.st.csrs.dirty_fp();
        }
    }

    // -----------------------------------------------------------------
    // Memory
    // -----------------------------------------------------------------

    /// The privilege an ordinary load or store is checked against.
    ///
    /// Volume II: `mstatus.MPRV` makes M-mode loads and stores — but never
    /// instruction fetches — use `MPP`'s privilege instead of the current one,
    /// which is how machine-mode firmware reaches a supervisor's address
    /// space.
    fn effective_priv(&self, kind: Access) -> Priv {
        if kind != Access::Fetch
            && self.st.csrs.priv_mode == Priv::Machine
            && self.st.csrs.mstatus & status::MPRV != 0
        {
            Priv::from_bits((self.st.csrs.mstatus & status::MPP) >> status::MPP_SHIFT)
                .unwrap_or(Priv::Machine)
        } else {
            self.st.csrs.priv_mode
        }
    }

    /// Which cause a translation failure raises for this access type.
    fn fault_cause(kind: Access, fault: mmu::Fault) -> u64 {
        match (kind, fault) {
            (Access::Fetch, mmu::Fault::Page) => cause::INSN_PAGE_FAULT,
            (Access::Fetch, mmu::Fault::Access) => cause::INSN_ACCESS,
            (Access::Load, mmu::Fault::Page) => cause::LOAD_PAGE_FAULT,
            (Access::Load, mmu::Fault::Access) => cause::LOAD_ACCESS,
            (Access::Store, mmu::Fault::Page) => cause::STORE_PAGE_FAULT,
            (Access::Store, mmu::Fault::Access) => cause::STORE_ACCESS,
        }
    }

    /// Translate one virtual address, consulting and filling the TLB.
    fn translate(&mut self, vaddr: u64, kind: Access, len: u64) -> Result<u64, Trap> {
        let mode = self.effective_priv(kind);
        let phys = if mmu::translation_active(&self.st.csrs, mode) {
            let vpn = vaddr >> mmu::PAGE_BITS;
            let asid = mmu::asid(&self.st.csrs);
            let generation = self.st.csrs.translation_gen;
            if let Some(base) = self.tlb.lookup(kind, vpn, asid, mode, generation) {
                base | (vaddr & PAGE_MASK)
            } else {
                let mut walker = Walker {
                    space: self.space,
                    attrs: self.attrs,
                    accesses: 0,
                };
                let result = mmu::translate(&self.st.csrs, &mut walker, vaddr, kind, mode);
                for _ in 0..walker.accesses {
                    self.charge();
                }
                let phys = result.map_err(|f| Trap {
                    cause: Self::fault_cause(kind, f),
                    tval: vaddr,
                })?;
                self.tlb
                    .insert(kind, vpn, asid, mode, generation, phys & !PAGE_MASK);
                phys
            }
        } else {
            vaddr
        };
        // PMP applies to physical addresses in every mode, and a region may be
        // smaller than a page, so it is checked per access rather than cached
        // alongside the translation.
        if !mmu::pmp_allows(&self.st.csrs, phys, len, kind, mode) {
            return Err(Trap {
                cause: Self::fault_cause(kind, mmu::Fault::Access),
                tval: vaddr,
            });
        }
        Ok(phys)
    }

    /// One read that does not cross a page boundary.
    fn read_once(&mut self, vaddr: u64, width: Width, kind: Access) -> Result<u64, Trap> {
        let phys = self.translate(vaddr, kind, width.bytes())?;
        self.charge();
        match self.space.read(phys, width, self.attrs) {
            Ok(v) => Ok(v),
            Err(_) => {
                // A refused access is a bus fault, which RISC-V *does* have a
                // way to report — unlike the 6502, where it can only be open
                // bus. So it becomes an access-fault exception and the guest
                // gets to decide.
                self.st.faults = self.st.faults.wrapping_add(1);
                Err(Trap {
                    cause: Self::fault_cause(kind, mmu::Fault::Access),
                    tval: vaddr,
                })
            }
        }
    }

    /// One write that does not cross a page boundary.
    fn write_once(&mut self, vaddr: u64, width: Width, value: u64) -> Result<(), Trap> {
        let phys = self.translate(vaddr, Access::Store, width.bytes())?;
        self.charge();
        match self.space.write(phys, width, value, self.attrs) {
            Ok(()) => Ok(()),
            Err(_) => {
                self.st.faults = self.st.faults.wrapping_add(1);
                Err(Trap {
                    cause: cause::STORE_ACCESS,
                    tval: vaddr,
                })
            }
        }
    }

    /// Load `bytes` bytes, splitting a misaligned access into bytes.
    ///
    /// Volume I leaves misaligned loads and stores to the implementation:
    /// either the hardware performs them or it raises a misaligned exception.
    /// This core performs them, byte by byte, because that is what lets code
    /// written for a hart that supports them run — and each byte is translated
    /// separately, so an access straddling a page boundary faults on the half
    /// that is actually unmapped.
    fn load(&mut self, vaddr: u64, bytes: u64) -> Result<u64, Trap> {
        let width = Width::from_bytes(bytes).ok_or(Trap::bare(cause::LOAD_ACCESS))?;
        if vaddr.is_multiple_of(bytes) {
            return self.read_once(vaddr, width, Access::Load);
        }
        if !self.cfg.misaligned {
            return Err(Trap {
                cause: cause::LOAD_MISALIGNED,
                tval: vaddr,
            });
        }
        let mut value = 0u64;
        for i in 0..bytes {
            let byte = self.read_once(vaddr.wrapping_add(i), Width::U8, Access::Load)?;
            value |= (byte & 0xff) << (8 * i);
        }
        Ok(value)
    }

    /// Store `bytes` bytes, splitting a misaligned access into bytes.
    fn store(&mut self, vaddr: u64, bytes: u64, value: u64) -> Result<(), Trap> {
        let width = Width::from_bytes(bytes).ok_or(Trap::bare(cause::STORE_ACCESS))?;
        // A store into the reservation set breaks it, which is what makes a
        // load-reserved/store-conditional pair fail when something else wrote
        // the location in between.
        if let Some(reserved) = self.st.reservation
            && reserved >> 3 == vaddr >> 3
        {
            self.st.reservation = None;
        }
        if vaddr.is_multiple_of(bytes) {
            return self.write_once(vaddr, width, value);
        }
        if !self.cfg.misaligned {
            return Err(Trap {
                cause: cause::STORE_MISALIGNED,
                tval: vaddr,
            });
        }
        for i in 0..bytes {
            self.write_once(vaddr.wrapping_add(i), Width::U8, value >> (8 * i))?;
        }
        Ok(())
    }

    /// Fetch the instruction at `pc`.
    ///
    /// Returns the 32-bit encoding, the raw encoding for `mtval`, and the
    /// instruction's length. The two halves are fetched separately because
    /// they may lie on different pages and the second one may be the half that
    /// faults.
    fn fetch(&mut self) -> Result<(u32, u64, u64), Trap> {
        let pc = self.st.pc;
        // Without C, an instruction address with bit 1 set is misaligned; the
        // check belongs on the *jump*, so by the time a fetch happens the
        // address is already known good.
        let low = self.read_once(pc, Width::U16, Access::Fetch)? as u16;
        if !isa::is_32bit(low) {
            if !self.cfg.ext.c {
                return Err(Trap::illegal(u64::from(low)));
            }
            let word = isa::expand(low, self.cfg.xlen).ok_or(Trap::illegal(u64::from(low)))?;
            return Ok((word, u64::from(low), 2));
        }
        let high = self.read_once(pc.wrapping_add(2), Width::U16, Access::Fetch)? as u16;
        let word = u32::from(low) | (u32::from(high) << 16);
        Ok((word, u64::from(word), 4))
    }

    /// Whether an instruction's extension is present in this configuration.
    fn has(&self, ext: isa::Ext) -> bool {
        match ext {
            isa::Ext::I | isa::Ext::Priv | isa::Ext::Zicsr | isa::Ext::Zifencei => true,
            isa::Ext::M => self.cfg.ext.m,
            isa::Ext::A => self.cfg.ext.a,
            isa::Ext::F => self.cfg.ext.f,
            isa::Ext::D => self.cfg.ext.d,
            isa::Ext::C => self.cfg.ext.c,
        }
    }

    // -----------------------------------------------------------------
    // Traps
    // -----------------------------------------------------------------

    /// The highest-priority interrupt that may be taken right now.
    ///
    /// Volume II fixes both halves of this: the priority order (MEI, MSI, MTI,
    /// SEI, SSI, STI — not the numeric order) and the enable rule, which is
    /// that an interrupt destined for a *higher* privilege than the current
    /// one is always taken, one destined for the current privilege is taken
    /// only if that privilege's global enable is set, and one destined for a
    /// lower privilege is never taken.
    fn pending_interrupt(&self) -> Option<u64> {
        let ready = self.lines.pending() & self.st.csrs.mie;
        if ready == 0 {
            return None;
        }
        let current = self.st.csrs.priv_mode;
        for code in irq::PRIORITY {
            let bit = 1u64 << code;
            if ready & bit == 0 {
                continue;
            }
            let target = if self.st.csrs.mideleg & bit != 0 {
                Priv::Supervisor
            } else {
                Priv::Machine
            };
            let enabled = if current < target {
                true
            } else if current == target {
                let mask = if target == Priv::Machine {
                    status::MIE
                } else {
                    status::SIE
                };
                self.st.csrs.mstatus & mask != 0
            } else {
                false
            };
            if enabled {
                return Some(code);
            }
        }
        None
    }

    /// Enter a trap handler.
    ///
    /// Volume II, "Trap Entry": the previous interrupt-enable bit is saved
    /// into `xPIE`, interrupts are disabled, the previous privilege is saved
    /// into `xPP`, and the hart moves to the handling privilege. Delegation
    /// decides which of the two register sets is used.
    fn enter_trap(&mut self, trap: Trap, interrupt: bool) {
        let csrs = &mut self.st.csrs;
        let bit = 1u64 << trap.cause;
        let delegated = if interrupt {
            csrs.mideleg & bit != 0
        } else {
            csrs.medeleg & bit != 0
        };
        let to_supervisor = delegated && csrs.ext.s && csrs.priv_mode <= Priv::Supervisor;
        let coded = trap.cause
            | if interrupt {
                1 << (csrs.xlen.bits() - 1)
            } else {
                0
            };
        let from = csrs.priv_mode;

        let tvec = if to_supervisor {
            csrs.sepc = self.this_pc;
            csrs.scause = coded;
            csrs.stval = trap.tval;
            let sie = csrs.mstatus & status::SIE != 0;
            csrs.mstatus &= !(status::SPIE | status::SIE | status::SPP);
            if sie {
                csrs.mstatus |= status::SPIE;
            }
            if from == Priv::Supervisor {
                csrs.mstatus |= status::SPP;
            }
            csrs.priv_mode = Priv::Supervisor;
            csrs.stvec
        } else {
            csrs.mepc = self.this_pc;
            csrs.mcause = coded;
            csrs.mtval = trap.tval;
            let mie = csrs.mstatus & status::MIE != 0;
            csrs.mstatus &= !(status::MPIE | status::MIE | status::MPP);
            if mie {
                csrs.mstatus |= status::MPIE;
            }
            csrs.mstatus |= from.bits() << status::MPP_SHIFT;
            csrs.priv_mode = Priv::Machine;
            csrs.mtvec
        };

        // A trap always breaks any outstanding reservation: the hart is about
        // to run code that has no idea one was held.
        self.st.reservation = None;
        // Vectored mode sends *interrupts* to base + 4 * cause; exceptions
        // always go to the base, even in vectored mode.
        let base = tvec & !3;
        self.next_pc = if tvec & 3 == 1 && interrupt {
            base.wrapping_add(4 * trap.cause)
        } else {
            base
        };
        self.next_pc = self.cfg.xlen.trunc(self.next_pc);
    }

    /// Return from a trap: the `MRET`/`SRET` field shuffle.
    ///
    /// Volume II, "Trap Return": restore the saved interrupt enable, set the
    /// saved-enable bit, drop the saved privilege to the least privileged
    /// supported mode, and — because the new mode may be less privileged —
    /// clear `MPRV` unless returning to machine mode.
    fn trap_return(&mut self, machine: bool) {
        let csrs = &mut self.st.csrs;
        let lowest = if csrs.ext.u {
            Priv::User
        } else {
            Priv::Machine
        };
        let to = if machine {
            let mpp = Priv::from_bits((csrs.mstatus & status::MPP) >> status::MPP_SHIFT)
                .unwrap_or(Priv::Machine);
            let mpie = csrs.mstatus & status::MPIE != 0;
            csrs.mstatus &= !(status::MIE | status::MPP);
            if mpie {
                csrs.mstatus |= status::MIE;
            }
            csrs.mstatus |= status::MPIE;
            csrs.mstatus |= lowest.bits() << status::MPP_SHIFT;
            self.next_pc = csrs.mepc;
            mpp
        } else {
            let spp = if csrs.mstatus & status::SPP != 0 {
                Priv::Supervisor
            } else {
                Priv::User
            };
            let spie = csrs.mstatus & status::SPIE != 0;
            csrs.mstatus &= !(status::SIE | status::SPP);
            if spie {
                csrs.mstatus |= status::SIE;
            }
            csrs.mstatus |= status::SPIE;
            self.next_pc = csrs.sepc;
            spp
        };
        if to != Priv::Machine {
            csrs.mstatus &= !status::MPRV;
        }
        csrs.priv_mode = to;
        // The privilege is part of every TLB tag, so nothing has to be
        // flushed; but SUM, MXR and MPRV may have changed with it.
        csrs.bump_translation();
        self.next_pc = self.cfg.xlen.trunc(self.next_pc);
    }

    /// Check that a jump target is a legal instruction address.
    fn check_target(&self, target: u64) -> Result<(), Trap> {
        let mask = if self.cfg.ext.c { 1 } else { 3 };
        if target & mask != 0 {
            Err(Trap {
                cause: cause::INSN_MISALIGNED,
                tval: target,
            })
        } else {
            Ok(())
        }
    }

    // -----------------------------------------------------------------
    // Width-dependent arithmetic
    // -----------------------------------------------------------------

    /// A shift amount, masked to the register width.
    #[inline]
    fn shamt(&self, raw: u64) -> u32 {
        (raw & u64::from(self.cfg.xlen.bits() - 1)) as u32
    }

    /// Shift left, in the guest's width.
    fn sll(&self, a: u64, sh: u32) -> u64 {
        match self.cfg.xlen {
            Xlen::Rv32 => u64::from((a as u32) << sh),
            Xlen::Rv64 => a << sh,
        }
    }

    /// Shift right logical, in the guest's width.
    ///
    /// The width matters here in a way it does not for shifting left: on RV32
    /// the vacated bits must come from the 32-bit value, not from the
    /// sign-extended one held in the register.
    fn srl(&self, a: u64, sh: u32) -> u64 {
        match self.cfg.xlen {
            Xlen::Rv32 => u64::from((a as u32) >> sh),
            Xlen::Rv64 => a >> sh,
        }
    }

    /// Shift right arithmetic, in the guest's width.
    fn sra(&self, a: u64, sh: u32) -> u64 {
        match self.cfg.xlen {
            Xlen::Rv32 => ((a as u32 as i32) >> sh) as u64,
            Xlen::Rv64 => ((a as i64) >> sh) as u64,
        }
    }

    // -----------------------------------------------------------------
    // The instruction body
    // -----------------------------------------------------------------

    /// Fetch, decode and execute one instruction.
    #[allow(clippy::too_many_lines)]
    fn execute(&mut self) -> Result<(), Trap> {
        let (word, encoding, len) = self.fetch()?;
        self.next_pc = self.this_pc.wrapping_add(len);

        let insn = isa::decode(word, self.cfg.xlen).ok_or(Trap::illegal(encoding))?;
        if !self.has(insn.ext) {
            return Err(Trap::illegal(encoding));
        }
        // Every F and D instruction — including a plain load or store of a
        // float — is illegal while mstatus.FS says the unit is off.
        if matches!(insn.ext, isa::Ext::F | isa::Ext::D) && !self.st.csrs.fp_enabled() {
            return Err(Trap::illegal(encoding));
        }

        let rd = isa::rd(word);
        let rs1 = isa::rs1(word);
        let rs2 = isa::rs2(word);
        let a = self.x(rs1);
        let b = self.x(rs2);

        match insn.op {
            // -- integer computation ------------------------------------
            Op::Lui => self.set_x(rd, isa::imm_u(word) as u64),
            Op::Auipc => self.set_x(rd, self.this_pc.wrapping_add(isa::imm_u(word) as u64)),
            Op::Addi => self.set_x(rd, a.wrapping_add(isa::imm_i(word) as u64)),
            Op::Slti => self.set_x(rd, u64::from((a as i64) < isa::imm_i(word))),
            Op::Sltiu => {
                // The immediate is sign-extended first and *then* compared as
                // unsigned, which is what makes `sltiu rd, rs, 1` the standard
                // "is zero" idiom.
                self.set_x(
                    rd,
                    u64::from(a < self.cfg.xlen.sext(isa::imm_i(word) as u64)),
                );
            }
            Op::Xori => self.set_x(rd, a ^ (isa::imm_i(word) as u64)),
            Op::Ori => self.set_x(rd, a | (isa::imm_i(word) as u64)),
            Op::Andi => self.set_x(rd, a & (isa::imm_i(word) as u64)),
            Op::Slli | Op::Srli | Op::Srai => {
                let shamt = isa::shamt(word);
                if shamt >= self.cfg.xlen.bits() {
                    return Err(Trap::illegal(encoding));
                }
                let v = match insn.op {
                    Op::Slli => self.sll(a, shamt),
                    Op::Srli => self.srl(a, shamt),
                    _ => self.sra(a, shamt),
                };
                self.set_x(rd, v);
            }
            Op::Add => self.set_x(rd, a.wrapping_add(b)),
            Op::Sub => self.set_x(rd, a.wrapping_sub(b)),
            Op::Sll => {
                let sh = self.shamt(b);
                self.set_x(rd, self.sll(a, sh));
            }
            Op::Slt => self.set_x(rd, u64::from((a as i64) < (b as i64))),
            Op::Sltu => self.set_x(rd, u64::from(a < b)),
            Op::Xor => self.set_x(rd, a ^ b),
            Op::Srl => {
                let sh = self.shamt(b);
                self.set_x(rd, self.srl(a, sh));
            }
            Op::Sra => {
                let sh = self.shamt(b);
                self.set_x(rd, self.sra(a, sh));
            }
            Op::Or => self.set_x(rd, a | b),
            Op::And => self.set_x(rd, a & b),

            // -- RV64 word forms ----------------------------------------
            Op::Addiw => {
                self.set_x(
                    rd,
                    (a as u32).wrapping_add(isa::imm_i(word) as u32) as i32 as u64,
                );
            }
            Op::Slliw | Op::Srliw | Op::Sraiw => {
                let shamt = isa::shamt(word) & 31;
                let v = match insn.op {
                    Op::Slliw => (a as u32) << shamt,
                    Op::Srliw => (a as u32) >> shamt,
                    _ => ((a as i32) >> shamt) as u32,
                };
                self.set_x(rd, v as i32 as u64);
            }
            Op::Addw => self.set_x(rd, (a as u32).wrapping_add(b as u32) as i32 as u64),
            Op::Subw => self.set_x(rd, (a as u32).wrapping_sub(b as u32) as i32 as u64),
            Op::Sllw => self.set_x(rd, ((a as u32) << (b & 31)) as i32 as u64),
            Op::Srlw => self.set_x(rd, ((a as u32) >> (b & 31)) as i32 as u64),
            Op::Sraw => self.set_x(rd, ((a as i32) >> (b & 31)) as u64),

            // -- M -------------------------------------------------------
            Op::Mul => self.set_x(rd, a.wrapping_mul(b)),
            Op::Mulh => {
                let v = match self.cfg.xlen {
                    Xlen::Rv32 => ((i64::from(a as i32) * i64::from(b as i32)) >> 32) as u64,
                    Xlen::Rv64 => ((i128::from(a as i64) * i128::from(b as i64)) >> 64) as u64,
                };
                self.set_x(rd, v);
            }
            Op::Mulhsu => {
                let v = match self.cfg.xlen {
                    Xlen::Rv32 => ((i64::from(a as i32) * i64::from(b as u32)) >> 32) as u64,
                    Xlen::Rv64 => ((i128::from(a as i64) * i128::from(b)) >> 64) as u64,
                };
                self.set_x(rd, v);
            }
            Op::Mulhu => {
                let v = match self.cfg.xlen {
                    Xlen::Rv32 => (u64::from(a as u32) * u64::from(b as u32)) >> 32,
                    Xlen::Rv64 => ((u128::from(a) * u128::from(b)) >> 64) as u64,
                };
                self.set_x(rd, v);
            }
            Op::Div | Op::Divu | Op::Rem | Op::Remu => {
                let v = self.divide(insn.op, a, b, self.cfg.xlen.bits());
                self.set_x(rd, v);
            }
            Op::Mulw => self.set_x(rd, (a as u32).wrapping_mul(b as u32) as i32 as u64),
            Op::Divw | Op::Divuw | Op::Remw | Op::Remuw => {
                let op = match insn.op {
                    Op::Divw => Op::Div,
                    Op::Divuw => Op::Divu,
                    Op::Remw => Op::Rem,
                    _ => Op::Remu,
                };
                // Sign-extend the operands into the 32-bit domain first: the
                // word forms divide the low halves, and the *result* is then
                // sign-extended, which is not the same as truncating a 64-bit
                // division.
                let (a32, b32) = match op {
                    Op::Div | Op::Rem => (a as i32 as u64, b as i32 as u64),
                    _ => (u64::from(a as u32), u64::from(b as u32)),
                };
                let v = self.divide(op, a32, b32, 32);
                self.set_x(rd, v as i32 as u64);
            }

            // -- control flow --------------------------------------------
            Op::Jal => {
                let target = self
                    .cfg
                    .xlen
                    .trunc(self.this_pc.wrapping_add(isa::imm_j(word) as u64));
                self.check_target(target)?;
                self.set_x(rd, self.next_pc);
                self.next_pc = target;
            }
            Op::Jalr => {
                // The low bit of the computed target is cleared, not checked:
                // Volume I says so explicitly, which is what lets a linker set
                // it as a tag.
                let target = self.cfg.xlen.trunc(a.wrapping_add(isa::imm_i(word) as u64)) & !1;
                self.check_target(target)?;
                let link = self.next_pc;
                self.next_pc = target;
                self.set_x(rd, link);
            }
            Op::Beq | Op::Bne | Op::Blt | Op::Bge | Op::Bltu | Op::Bgeu => {
                let taken = match insn.op {
                    Op::Beq => a == b,
                    Op::Bne => a != b,
                    Op::Blt => (a as i64) < (b as i64),
                    Op::Bge => (a as i64) >= (b as i64),
                    Op::Bltu => a < b,
                    _ => a >= b,
                };
                if taken {
                    let target = self
                        .cfg
                        .xlen
                        .trunc(self.this_pc.wrapping_add(isa::imm_b(word) as u64));
                    self.check_target(target)?;
                    self.next_pc = target;
                }
            }

            // -- loads and stores ----------------------------------------
            Op::Lb | Op::Lh | Op::Lw | Op::Ld | Op::Lbu | Op::Lhu | Op::Lwu => {
                let addr = self.cfg.xlen.trunc(a.wrapping_add(isa::imm_i(word) as u64));
                let (bytes, signed) = match insn.op {
                    Op::Lb => (1, true),
                    Op::Lbu => (1, false),
                    Op::Lh => (2, true),
                    Op::Lhu => (2, false),
                    Op::Lw => (4, true),
                    Op::Lwu => (4, false),
                    _ => (8, true),
                };
                let raw = self.load(addr, bytes)?;
                let v = if signed {
                    sign_extend(raw, bytes * 8)
                } else {
                    raw
                };
                self.set_x(rd, v);
            }
            Op::Sb | Op::Sh | Op::Sw | Op::Sd => {
                let addr = self.cfg.xlen.trunc(a.wrapping_add(isa::imm_s(word) as u64));
                let bytes = match insn.op {
                    Op::Sb => 1,
                    Op::Sh => 2,
                    Op::Sw => 4,
                    _ => 8,
                };
                self.store(addr, bytes, b)?;
            }

            // -- fences ---------------------------------------------------
            // This core executes one instruction at a time in program order
            // and has no store buffer, so both fences are architecturally
            // complete as no-ops. FENCE.I additionally invalidates any decoded
            // instruction cache; there is none yet, and when the JIT lands it
            // will hook here.
            Op::Fence | Op::FenceI => {}

            // -- A ---------------------------------------------------------
            Op::LrW | Op::LrD => {
                let bytes = if insn.op == Op::LrW { 4 } else { 8 };
                let a = self.cfg.xlen.trunc(a);
                if !a.is_multiple_of(bytes) {
                    return Err(Trap {
                        cause: cause::LOAD_MISALIGNED,
                        tval: a,
                    });
                }
                let width = Width::from_bytes(bytes).expect("4 or 8");
                let v = self.read_once(a, width, Access::Load)?;
                self.st.reservation = Some(a);
                self.set_x(rd, sign_extend(v, bytes * 8));
            }
            Op::ScW | Op::ScD => {
                let bytes = if insn.op == Op::ScW { 4 } else { 8 };
                let a = self.cfg.xlen.trunc(a);
                if !a.is_multiple_of(bytes) {
                    return Err(Trap {
                        cause: cause::STORE_MISALIGNED,
                        tval: a,
                    });
                }
                let held = self.st.reservation == Some(a);
                // The reservation is given up whether the store succeeds or
                // not: Volume I requires an SC to end the reservation, which
                // is what stops a livelock of retries.
                self.st.reservation = None;
                if held {
                    let width = Width::from_bytes(bytes).expect("4 or 8");
                    self.write_once(a, width, b)?;
                    self.set_x(rd, 0);
                } else {
                    self.set_x(rd, 1);
                }
            }
            Op::AmoswapW
            | Op::AmoaddW
            | Op::AmoxorW
            | Op::AmoandW
            | Op::AmoorW
            | Op::AmominW
            | Op::AmomaxW
            | Op::AmominuW
            | Op::AmomaxuW
            | Op::AmoswapD
            | Op::AmoaddD
            | Op::AmoxorD
            | Op::AmoandD
            | Op::AmoorD
            | Op::AmominD
            | Op::AmomaxD
            | Op::AmominuD
            | Op::AmomaxuD => {
                self.amo(insn.op, rd, self.cfg.xlen.trunc(a), b)?;
            }

            // -- system ----------------------------------------------------
            Op::Ecall => {
                let code = match self.st.csrs.priv_mode {
                    Priv::User => cause::ECALL_U,
                    Priv::Supervisor => cause::ECALL_S,
                    Priv::Machine => cause::ECALL_M,
                };
                return Err(Trap::bare(code));
            }
            Op::Ebreak => {
                return Err(Trap {
                    cause: cause::BREAKPOINT,
                    tval: self.this_pc,
                });
            }
            Op::Mret => {
                if self.st.csrs.priv_mode != Priv::Machine {
                    return Err(Trap::illegal(encoding));
                }
                self.trap_return(true);
            }
            Op::Sret => {
                if self.st.csrs.priv_mode < Priv::Supervisor || !self.cfg.ext.s {
                    return Err(Trap::illegal(encoding));
                }
                // TSR lets machine-mode firmware intercept a supervisor's
                // return, which is how a hypervisor virtualises one.
                if self.st.csrs.priv_mode == Priv::Supervisor
                    && self.st.csrs.mstatus & status::TSR != 0
                {
                    return Err(Trap::illegal(encoding));
                }
                self.trap_return(false);
            }
            Op::Wfi => {
                if self.st.csrs.priv_mode != Priv::Machine && self.st.csrs.mstatus & status::TW != 0
                {
                    return Err(Trap::illegal(encoding));
                }
                self.st.wfi = true;
            }
            Op::SfenceVma => {
                if self.st.csrs.priv_mode == Priv::User
                    || (self.st.csrs.priv_mode == Priv::Supervisor
                        && self.st.csrs.mstatus & status::TVM != 0)
                {
                    return Err(Trap::illegal(encoding));
                }
                // A generation bump invalidates the whole TLB at once. The
                // instruction can name one address and one ASID; honouring
                // that would be a refinement, and over-invalidating is always
                // architecturally safe.
                self.st.csrs.bump_translation();
            }
            Op::Csrrw | Op::Csrrs | Op::Csrrc | Op::Csrrwi | Op::Csrrsi | Op::Csrrci => {
                self.csr_access(insn.op, word, encoding)?;
            }

            // -- F and D ---------------------------------------------------
            _ => self.float(insn.op, word, encoding)?,
        }
        Ok(())
    }

    /// The `M` extension's division rules.
    ///
    /// Volume I, "Division Operations", specifies exact results for the two
    /// cases that would otherwise be undefined, and specifies that **neither
    /// traps** — which is why there is no error return here.
    fn divide(&self, op: Op, a: u64, b: u64, bits: u32) -> u64 {
        let mask = if bits >= 64 {
            u64::MAX
        } else {
            (1u64 << bits) - 1
        };
        let min = 1u64 << (bits - 1);
        match op {
            Op::Div => {
                if b & mask == 0 {
                    u64::MAX
                } else if a & mask == min && b & mask == mask {
                    // The signed overflow case: the quotient is not
                    // representable, and the specification says the result is
                    // the dividend.
                    a
                } else {
                    ((a as i64).wrapping_div(b as i64)) as u64
                }
            }
            Op::Divu => {
                if b & mask == 0 {
                    u64::MAX
                } else if bits >= 64 {
                    a / b
                } else {
                    (a & mask) / (b & mask)
                }
            }
            Op::Rem => {
                if b & mask == 0 {
                    a
                } else if a & mask == min && b & mask == mask {
                    0
                } else {
                    ((a as i64).wrapping_rem(b as i64)) as u64
                }
            }
            _ => {
                if b & mask == 0 {
                    a
                } else if bits >= 64 {
                    a % b
                } else {
                    (a & mask) % (b & mask)
                }
            }
        }
    }

    /// One read-modify-write atomic.
    ///
    /// The `aq` and `rl` bits are decoded but have no effect: this core
    /// executes one instruction at a time in program order, so every access is
    /// already sequentially consistent and there is nothing for an ordering
    /// bit to constrain. They will matter to the JIT, not here.
    fn amo(&mut self, op: Op, rd: u32, addr: u64, operand: u64) -> Result<(), Trap> {
        let bytes: u64 = if matches!(
            op,
            Op::AmoswapW
                | Op::AmoaddW
                | Op::AmoxorW
                | Op::AmoandW
                | Op::AmoorW
                | Op::AmominW
                | Op::AmomaxW
                | Op::AmominuW
                | Op::AmomaxuW
        ) {
            4
        } else {
            8
        };
        // Volume I requires an AMO's address to be naturally aligned.
        if !addr.is_multiple_of(bytes) {
            return Err(Trap {
                cause: cause::STORE_MISALIGNED,
                tval: addr,
            });
        }
        let width = Width::from_bytes(bytes).expect("4 or 8");
        let old = self.read_once(addr, width, Access::Load)?;
        let bits = (bytes * 8) as u32;
        let s_old = sign_extend(old, bytes * 8) as i64;
        let s_arg = sign_extend(operand, bytes * 8) as i64;
        let u_old = old & mask_bits(bits);
        let u_arg = operand & mask_bits(bits);
        let new = match op {
            Op::AmoswapW | Op::AmoswapD => operand,
            Op::AmoaddW | Op::AmoaddD => old.wrapping_add(operand),
            Op::AmoxorW | Op::AmoxorD => old ^ operand,
            Op::AmoandW | Op::AmoandD => old & operand,
            Op::AmoorW | Op::AmoorD => old | operand,
            Op::AmominW | Op::AmominD => {
                if s_old < s_arg {
                    old
                } else {
                    operand
                }
            }
            Op::AmomaxW | Op::AmomaxD => {
                if s_old > s_arg {
                    old
                } else {
                    operand
                }
            }
            Op::AmominuW | Op::AmominuD => {
                if u_old < u_arg {
                    old
                } else {
                    operand
                }
            }
            _ => {
                if u_old > u_arg {
                    old
                } else {
                    operand
                }
            }
        };
        // An AMO writes the reservation set, so it breaks any reservation
        // covering the same location.
        if let Some(reserved) = self.st.reservation
            && reserved >> 3 == addr >> 3
        {
            self.st.reservation = None;
        }
        self.write_once(addr, width, new)?;
        self.set_x(rd, sign_extend(old, bytes * 8));
        Ok(())
    }

    /// A CSR read-modify-write.
    ///
    /// Volume I, "CSR Instructions": the *read* side effect is skipped when
    /// `rd` is `x0` for the write forms, and the *write* side effect is
    /// skipped when the source is `x0` or a zero immediate — which is what
    /// makes `csrr` and `csrw` safe on registers with read or write side
    /// effects.
    fn csr_access(&mut self, op: Op, word: u32, encoding: u64) -> Result<(), Trap> {
        let num = isa::csr(word);
        let rd = isa::rd(word);
        let rs1 = isa::rs1(word);
        let immediate = matches!(op, Op::Csrrwi | Op::Csrrsi | Op::Csrrci);
        let source = if immediate {
            u64::from(rs1)
        } else {
            self.x(rs1)
        };
        let write_form = matches!(op, Op::Csrrw | Op::Csrrwi);
        let will_write = write_form || rs1 != 0;
        let will_read = !write_form || rd != 0;

        let pending = self.lines.pending();
        let old = if will_read {
            Some(
                self.st
                    .csrs
                    .read(num, pending)
                    .ok_or(Trap::illegal(encoding))?,
            )
        } else {
            // Still check the register exists and is reachable, so a bad
            // number is illegal even when nothing is read.
            self.st
                .csrs
                .read(num, pending)
                .ok_or(Trap::illegal(encoding))?;
            None
        };
        if will_write {
            let current = old.unwrap_or_else(|| self.st.csrs.read(num, pending).unwrap_or(0));
            let value = match op {
                Op::Csrrw | Op::Csrrwi => source,
                Op::Csrrs | Op::Csrrsi => current | source,
                _ => current & !source,
            };
            let updated = self
                .st
                .csrs
                .write(num, value, pending)
                .ok_or(Trap::illegal(encoding))?;
            if let Some(bits) = updated {
                self.lines.set_all_pending(bits);
            }
            if matches!(num, csr::num::MINSTRET | csr::num::MINSTRETH) {
                self.wrote_instret = true;
            }
        }
        if let Some(v) = old {
            self.set_x(rd, v);
        }
        Ok(())
    }

    /// Resolve an instruction's rounding mode against `fcsr.frm`.
    ///
    /// An `rm` field of 7 means "dynamic"; a reserved mode in either place
    /// makes the instruction illegal, which is the only way a program finds
    /// out it wrote nonsense to `frm`.
    fn rounding(&self, word: u32) -> Option<Round> {
        let field = isa::funct3(word);
        let mode = if field == 7 {
            ((self.st.csrs.fcsr >> 5) & 7) as u32
        } else {
            field
        };
        Round::from_bits(mode)
    }

    /// Every `F` and `D` instruction.
    #[allow(clippy::too_many_lines)]
    fn float(&mut self, op: Op, word: u32, encoding: u64) -> Result<(), Trap> {
        let rd = isa::rd(word);
        let rs1 = isa::rs1(word);
        let rs2 = isa::rs2(word);
        let rs3 = isa::rs3(word);

        // The instructions whose funct3 is a rounding mode rather than an
        // opcode field resolve it up front, so a reserved mode is illegal
        // before anything is computed.
        let rm = || self.rounding(word).ok_or(Trap::illegal(encoding));

        match op {
            Op::Flw | Op::Fld => {
                let addr = self
                    .cfg
                    .xlen
                    .trunc(self.x(rs1).wrapping_add(isa::imm_i(word) as u64));
                if op == Op::Flw {
                    let v = self.load(addr, 4)?;
                    self.set_fs(rd, v);
                } else {
                    let v = self.load(addr, 8)?;
                    self.set_f(rd, v);
                }
            }
            Op::Fsw | Op::Fsd => {
                let addr = self
                    .cfg
                    .xlen
                    .trunc(self.x(rs1).wrapping_add(isa::imm_s(word) as u64));
                if op == Op::Fsw {
                    let v = self.f(rs2);
                    self.store(addr, 4, v)?;
                } else {
                    let v = self.f(rs2);
                    self.store(addr, 8, v)?;
                }
            }

            Op::FaddS | Op::FsubS | Op::FmulS | Op::FdivS => {
                let rm = rm()?;
                let (x, y) = (self.fs(rs1), self.fs(rs2));
                let (v, f) = match op {
                    Op::FaddS => float::add::<B32>(x, y, rm),
                    Op::FsubS => float::sub::<B32>(x, y, rm),
                    Op::FmulS => float::mul::<B32>(x, y, rm),
                    _ => float::div::<B32>(x, y, rm),
                };
                self.set_fs(rd, v);
                self.raise(f);
            }
            Op::FaddD | Op::FsubD | Op::FmulD | Op::FdivD => {
                let rm = rm()?;
                let (x, y) = (self.f(rs1), self.f(rs2));
                let (v, f) = match op {
                    Op::FaddD => float::add::<B64>(x, y, rm),
                    Op::FsubD => float::sub::<B64>(x, y, rm),
                    Op::FmulD => float::mul::<B64>(x, y, rm),
                    _ => float::div::<B64>(x, y, rm),
                };
                self.set_f(rd, v);
                self.raise(f);
            }
            Op::FsqrtS => {
                let rm = rm()?;
                let (v, f) = float::sqrt::<B32>(self.fs(rs1), rm);
                self.set_fs(rd, v);
                self.raise(f);
            }
            Op::FsqrtD => {
                let rm = rm()?;
                let (v, f) = float::sqrt::<B64>(self.f(rs1), rm);
                self.set_f(rd, v);
                self.raise(f);
            }

            Op::FmaddS | Op::FmsubS | Op::FnmsubS | Op::FnmaddS => {
                let rm = rm()?;
                // Volume I defines FNMSUB as -(a*b) + c and FNMADD as
                // -(a*b) - c, which is exactly a fused multiply-add with the
                // signs of the first multiplicand and of the addend flipped.
                let sign = B32::SIGN;
                let (x, y, z) = (self.fs(rs1), self.fs(rs2), self.fs(rs3));
                let (x, z) = match op {
                    Op::FmaddS => (x, z),
                    Op::FmsubS => (x, z ^ sign),
                    Op::FnmsubS => (x ^ sign, z),
                    _ => (x ^ sign, z ^ sign),
                };
                let (v, f) = float::fma::<B32>(x, y, z, rm);
                self.set_fs(rd, v);
                self.raise(f);
            }
            Op::FmaddD | Op::FmsubD | Op::FnmsubD | Op::FnmaddD => {
                let rm = rm()?;
                let sign = B64::SIGN;
                let (x, y, z) = (self.f(rs1), self.f(rs2), self.f(rs3));
                let (x, z) = match op {
                    Op::FmaddD => (x, z),
                    Op::FmsubD => (x, z ^ sign),
                    Op::FnmsubD => (x ^ sign, z),
                    _ => (x ^ sign, z ^ sign),
                };
                let (v, f) = float::fma::<B64>(x, y, z, rm);
                self.set_f(rd, v);
                self.raise(f);
            }

            Op::FsgnjS | Op::FsgnjnS | Op::FsgnjxS => {
                let (x, y) = (self.fs(rs1), self.fs(rs2));
                let sign = match op {
                    Op::FsgnjS => y & B32::SIGN,
                    Op::FsgnjnS => !y & B32::SIGN,
                    _ => (x ^ y) & B32::SIGN,
                };
                self.set_fs(rd, (x & !B32::SIGN) | sign);
            }
            Op::FsgnjD | Op::FsgnjnD | Op::FsgnjxD => {
                let (x, y) = (self.f(rs1), self.f(rs2));
                let sign = match op {
                    Op::FsgnjD => y & B64::SIGN,
                    Op::FsgnjnD => !y & B64::SIGN,
                    _ => (x ^ y) & B64::SIGN,
                };
                self.set_f(rd, (x & !B64::SIGN) | sign);
            }

            Op::FminS | Op::FmaxS => {
                let (x, y) = (self.fs(rs1), self.fs(rs2));
                let (v, f) = if op == Op::FminS {
                    float::min::<B32>(x, y)
                } else {
                    float::max::<B32>(x, y)
                };
                self.set_fs(rd, v);
                self.raise(f);
            }
            Op::FminD | Op::FmaxD => {
                let (x, y) = (self.f(rs1), self.f(rs2));
                let (v, f) = if op == Op::FminD {
                    float::min::<B64>(x, y)
                } else {
                    float::max::<B64>(x, y)
                };
                self.set_f(rd, v);
                self.raise(f);
            }

            Op::FeqS | Op::FltS | Op::FleS => {
                let (x, y) = (self.fs(rs1), self.fs(rs2));
                let (v, f) = match op {
                    Op::FeqS => float::eq::<B32>(x, y),
                    Op::FltS => float::lt::<B32>(x, y),
                    _ => float::le::<B32>(x, y),
                };
                self.set_x(rd, u64::from(v));
                self.raise(f);
            }
            Op::FeqD | Op::FltD | Op::FleD => {
                let (x, y) = (self.f(rs1), self.f(rs2));
                let (v, f) = match op {
                    Op::FeqD => float::eq::<B64>(x, y),
                    Op::FltD => float::lt::<B64>(x, y),
                    _ => float::le::<B64>(x, y),
                };
                self.set_x(rd, u64::from(v));
                self.raise(f);
            }

            Op::FclassS => self.set_x(rd, float::classify::<B32>(self.fs(rs1))),
            Op::FclassD => self.set_x(rd, float::classify::<B64>(self.f(rs1))),

            Op::FcvtSD => {
                let rm = rm()?;
                let (v, f) = float::convert::<B64, B32>(self.f(rs1), rm);
                self.set_fs(rd, v);
                self.raise(f);
            }
            Op::FcvtDS => {
                let rm = rm()?;
                let (v, f) = float::convert::<B32, B64>(self.fs(rs1), rm);
                self.set_f(rd, v);
                self.raise(f);
            }

            Op::FcvtWS | Op::FcvtWuS | Op::FcvtLS | Op::FcvtLuS => {
                let rm = rm()?;
                let x = self.fs(rs1);
                let (v, f) = match op {
                    Op::FcvtWS => {
                        let (v, f) = float::to_signed::<B32>(x, 32, rm);
                        (v as u64, f)
                    }
                    // Even the unsigned word conversions sign-extend their
                    // result to XLEN, which Volume I calls out because it is
                    // surprising.
                    Op::FcvtWuS => {
                        let (v, f) = float::to_unsigned::<B32>(x, 32, rm);
                        (v as u32 as i32 as u64, f)
                    }
                    Op::FcvtLS => {
                        let (v, f) = float::to_signed::<B32>(x, 64, rm);
                        (v as u64, f)
                    }
                    _ => float::to_unsigned::<B32>(x, 64, rm),
                };
                self.set_x(rd, v);
                self.raise(f);
            }
            Op::FcvtWD | Op::FcvtWuD | Op::FcvtLD | Op::FcvtLuD => {
                let rm = rm()?;
                let x = self.f(rs1);
                let (v, f) = match op {
                    Op::FcvtWD => {
                        let (v, f) = float::to_signed::<B64>(x, 32, rm);
                        (v as u64, f)
                    }
                    Op::FcvtWuD => {
                        let (v, f) = float::to_unsigned::<B64>(x, 32, rm);
                        (v as u32 as i32 as u64, f)
                    }
                    Op::FcvtLD => {
                        let (v, f) = float::to_signed::<B64>(x, 64, rm);
                        (v as u64, f)
                    }
                    _ => float::to_unsigned::<B64>(x, 64, rm),
                };
                self.set_x(rd, v);
                self.raise(f);
            }
            Op::FcvtSW | Op::FcvtSWu | Op::FcvtSL | Op::FcvtSLu => {
                let rm = rm()?;
                let x = self.x(rs1);
                let (v, f) = match op {
                    Op::FcvtSW => float::from_signed::<B32>(x as i64, 32, rm),
                    Op::FcvtSWu => float::from_unsigned::<B32>(x, 32, rm),
                    Op::FcvtSL => float::from_signed::<B32>(x as i64, 64, rm),
                    _ => float::from_unsigned::<B32>(x, 64, rm),
                };
                self.set_fs(rd, v);
                self.raise(f);
            }
            Op::FcvtDW | Op::FcvtDWu | Op::FcvtDL | Op::FcvtDLu => {
                let rm = rm()?;
                let x = self.x(rs1);
                let (v, f) = match op {
                    Op::FcvtDW => float::from_signed::<B64>(x as i64, 32, rm),
                    Op::FcvtDWu => float::from_unsigned::<B64>(x, 32, rm),
                    Op::FcvtDL => float::from_signed::<B64>(x as i64, 64, rm),
                    _ => float::from_unsigned::<B64>(x, 64, rm),
                };
                self.set_f(rd, v);
                self.raise(f);
            }

            // The moves are bit copies and raise nothing: they do not
            // interpret the value at all, which is why FMV.X.W of a signaling
            // NaN is silent.
            Op::FmvXW => self.set_x(rd, self.f(rs1) as u32 as i32 as u64),
            Op::FmvWX => self.set_fs(rd, self.x(rs1)),
            Op::FmvXD => self.set_x(rd, self.f(rs1)),
            Op::FmvDX => self.set_f(rd, self.x(rs1)),

            _ => return Err(Trap::illegal(encoding)),
        }
        Ok(())
    }
}

/// Sign-extend the low `bits` of a value to 64 bits.
#[inline]
fn sign_extend(value: u64, bits: u64) -> u64 {
    if bits >= 64 {
        value
    } else {
        let shift = 64 - bits as u32;
        (((value << shift) as i64) >> shift) as u64
    }
}

/// A mask of the low `bits` bits.
#[inline]
fn mask_bits(bits: u32) -> u64 {
    if bits >= 64 {
        u64::MAX
    } else {
        (1u64 << bits) - 1
    }
}
