//! `PSTATE`, the exception levels, and the system registers `MRS`/`MSR` reach.
//!
//! Two things live here that a 32-bit Arm core keeps in CP15 and in `CPSR`,
//! and both change shape enough in AArch64 to be worth stating:
//!
//! * **`PSTATE` is not a register.** It has no encoding of its own; the guest
//!   reads and writes *fields* of it through named system registers (`NZCV`,
//!   `DAIF`, `CurrentEL`, `SPSel`) and through `MSR DAIFSet, #imm`. There is
//!   no `MRS Xt, PSTATE`. So it is a struct of fields here rather than one
//!   word, and [`SysRegs::spsr`] assembles the one word the architecture *does*
//!   define — the value an exception saves into `SPSR_EL1`.
//! * **System registers are a flat encoding space**, not a coprocessor:
//!   `op0:op1:CRn:CRm:op2`, sixteen bits, read out of the instruction word
//!   unchanged. So the table below is keyed on that number directly.
//!
//! # The table is declarative for the same reason the instruction table is
//!
//! Every register is one row: its encoding, the name the disassembler prints,
//! whether a guest may write it, and a summary. The disassembler prints
//! `mrs x0, sctlr_el1` from this table, the interpreter's `MRS` and `MSR`
//! dispatch on the same [`SysReg`] identifier, and an encoding with no row is
//! `UNDEFINED` — which is what real silicon does with an unallocated
//! system register, and what a guest probing for a feature is relying on.
//!
//! # Sources
//!
//! *Arm Architecture Reference Manual for A-profile architecture* (DDI 0487):
//! chapter D for the AArch64 system registers and their encodings, D1 for the
//! exception model, `PSTATE` and the `SPSR_EL1` layout, and C5.2 for the
//! `MRS`/`MSR` instruction descriptions.

use core::fmt;

use super::isa::Nzcv;

/// An exception level.
///
/// Only EL0 and EL1 are modelled: this core has no EL2 and no EL3, so `HVC`
/// and `SMC` raise `UNDEFINED` rather than pretending to a hypervisor that is
/// not there.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum El {
    /// Unprivileged: applications.
    El0,
    /// Privileged: the operating system kernel.
    El1,
}

impl El {
    /// The level as `CurrentEL` bits 3:2 and `SPSR_EL1.M[3:2]` spell it.
    #[must_use]
    pub const fn bits(self) -> u64 {
        match self {
            El::El0 => 0,
            El::El1 => 1,
        }
    }

    /// The name a monitor prints.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            El::El0 => "EL0",
            El::El1 => "EL1",
        }
    }
}

impl fmt::Display for El {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// The `DAIF` interrupt masks, at the bit positions `DAIF` and `SPSR_EL1`
/// hold them.
pub mod daif {
    /// Debug exceptions.
    pub const D: u64 = 1 << 9;
    /// SError (asynchronous abort).
    pub const A: u64 = 1 << 8;
    /// IRQ.
    pub const I: u64 = 1 << 7;
    /// FIQ.
    pub const F: u64 = 1 << 6;
    /// Every mask bit.
    pub const ALL: u64 = D | A | I | F;
}

/// Exception classes, as `ESR_ELx.EC` (bits 31:26) spells them.
///
/// Only the classes this core can raise are named. DDI 0487 D17.2.37 has the
/// full list.
pub mod ec {
    /// Unknown reason — an `UNDEFINED` instruction, among others.
    pub const UNKNOWN: u64 = 0x00;
    /// A trapped `WFI` or `WFE`. Not raised here; named because the bit
    /// pattern is easy to confuse with `UNKNOWN`.
    pub const WF: u64 = 0x01;
    /// An access to SIMD or floating-point functionality that `CPACR_EL1.FPEN`
    /// traps.
    ///
    /// **Not** `UNKNOWN`: a guest whose `CPACR_EL1` still has its reset value
    /// takes this on its first `FADD`, and Linux's lazy FPU state switching is
    /// built on telling this exception class apart from an undefined
    /// instruction. Reporting `UNKNOWN` instead would make an FP-using process
    /// die with SIGILL rather than get its registers restored.
    pub const FP_ACCESS: u64 = 0x07;
    /// `SVC` executed in AArch64 state.
    pub const SVC64: u64 = 0x15;
    /// A trapped `MRS`, `MSR` or system instruction.
    pub const SYSREG: u64 = 0x18;
    /// Instruction abort from a lower exception level.
    pub const IABT_LOWER: u64 = 0x20;
    /// Instruction abort taken without a change of exception level.
    pub const IABT_SAME: u64 = 0x21;
    /// Misaligned PC.
    pub const PC_ALIGN: u64 = 0x22;
    /// Data abort from a lower exception level.
    pub const DABT_LOWER: u64 = 0x24;
    /// Data abort taken without a change of exception level.
    pub const DABT_SAME: u64 = 0x25;
    /// Misaligned SP.
    pub const SP_ALIGN: u64 = 0x26;
    /// `BRK` executed in AArch64 state.
    pub const BRK64: u64 = 0x3c;
}

/// Which of the four vectors in a `VBAR_EL1` group an exception takes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VectorKind {
    /// A synchronous exception: the offset-0 vector of its group.
    Synchronous,
    /// An IRQ.
    Irq,
    /// An FIQ.
    Fiq,
    /// An SError. Never raised by this core, and present so the table is the
    /// architecture's rather than a subset of it.
    SError,
}

impl VectorKind {
    /// The offset within a vector group, in bytes.
    #[must_use]
    pub const fn offset(self) -> u64 {
        match self {
            VectorKind::Synchronous => 0x000,
            VectorKind::Irq => 0x080,
            VectorKind::Fiq => 0x100,
            VectorKind::SError => 0x180,
        }
    }
}

// ---------------------------------------------------------------------------
// The system register table
// ---------------------------------------------------------------------------

/// The 16-bit key a system register is identified by: `op0:op1:CRn:CRm:op2`,
/// which is bits 20:5 of an `MRS` or `MSR` encoding, unchanged.
#[must_use]
pub const fn enc(op0: u32, op1: u32, crn: u32, crm: u32, op2: u32) -> u16 {
    (((op0 & 3) << 14) | ((op1 & 7) << 11) | ((crn & 15) << 7) | ((crm & 15) << 3) | (op2 & 7))
        as u16
}

/// Whether a guest may write a register, and at which level it exists.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Access {
    /// Readable and writable at EL1.
    El1Rw,
    /// Readable at EL1, writes are ignored — the identification registers.
    El1Ro,
    /// Readable and writable at EL0 as well: the `PSTATE` views and the
    /// thread pointer the C library keeps there.
    El0Rw,
    /// Readable at EL0, writable only at EL1 — `TPIDRRO_EL0`.
    El0Ro,
}

impl Access {
    /// Whether a read is permitted from `el`.
    #[must_use]
    pub const fn readable_at(self, el: El) -> bool {
        match self {
            Access::El1Rw | Access::El1Ro => matches!(el, El::El1),
            Access::El0Rw | Access::El0Ro => true,
        }
    }

    /// Whether a write is permitted from `el`.
    ///
    /// A read-only register is *not* a trap: DDI 0487 gives the identification
    /// registers no write behaviour at EL1, so a write there is UNDEFINED.
    /// That is what `false` here means, and the caller raises the exception.
    #[must_use]
    pub const fn writable_at(self, el: El) -> bool {
        match self {
            Access::El1Rw => matches!(el, El::El1),
            Access::El0Rw => true,
            Access::El1Ro | Access::El0Ro => false,
        }
    }
}

/// One row of the system-register description.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SysRegSpec {
    /// Which register.
    pub reg: SysReg,
    /// `op0:op1:CRn:CRm:op2`.
    pub enc: u16,
    /// Who may read and write it.
    pub access: Access,
}

/// Declare the register identifier, its name, its summary and the lookup table
/// from one list of rows.
macro_rules! sysregs {
    ($($op0:literal $op1:literal $crn:literal $crm:literal $op2:literal
       $reg:ident $name:literal $access:ident $summary:literal;)*) => {
        /// One system register this core implements.
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        #[non_exhaustive]
        pub enum SysReg {
            $(
                #[doc = $summary]
                $reg,
            )*
        }

        impl SysReg {
            /// The name a disassembler prints, lower case as `objdump` spells
            /// it.
            #[must_use]
            pub const fn name(self) -> &'static str {
                match self { $(SysReg::$reg => $name,)* }
            }

            /// A one-line description, for `rsemu describe` and the monitor.
            #[must_use]
            pub const fn summary(self) -> &'static str {
                match self { $(SysReg::$reg => $summary,)* }
            }

            /// Every register this core implements, in encoding order.
            pub const ALL: &'static [SysReg] = &[$(SysReg::$reg,)*];
        }

        /// The system-register table: the only description of them in the
        /// crate.
        pub static SYSREGS: &[SysRegSpec] = &[
            $(SysRegSpec {
                reg: SysReg::$reg,
                enc: enc($op0, $op1, $crn, $crm, $op2),
                access: Access::$access,
            },)*
        ];
    };
}

sysregs! {
    // -- identification ------------------------------------------------------
    3 0  0 0 0 Midr         "midr_el1"         El1Ro "the part, variant and revision this core reports";
    3 0  0 0 5 Mpidr        "mpidr_el1"        El1Ro "which processor in which cluster this is";
    3 0  0 0 6 Revidr       "revidr_el1"       El1Ro "implementation-specific revision detail";
    3 0  0 4 0 IdAa64Pfr0   "id_aa64pfr0_el1"  El1Ro "which exception levels and features are implemented";
    3 0  0 4 1 IdAa64Pfr1   "id_aa64pfr1_el1"  El1Ro "further processor feature identification";
    3 0  0 6 0 IdAa64Isar0  "id_aa64isar0_el1" El1Ro "which instruction-set extensions are implemented";
    3 0  0 6 1 IdAa64Isar1  "id_aa64isar1_el1" El1Ro "further instruction-set identification";
    3 0  0 7 0 IdAa64Mmfr0  "id_aa64mmfr0_el1" El1Ro "which translation granules and address sizes are supported";
    3 0  0 7 1 IdAa64Mmfr1  "id_aa64mmfr1_el1" El1Ro "further memory-model identification";
    3 0  0 7 2 IdAa64Mmfr2  "id_aa64mmfr2_el1" El1Ro "further memory-model identification";
    3 3  0 0 1 Ctr          "ctr_el0"          El0Ro "cache type: the line sizes software must respect";
    3 3  0 0 7 Dczid        "dczid_el0"        El0Ro "the block size DC ZVA operates on, and whether it is allowed";

    // -- system control ------------------------------------------------------
    3 0  1 0 0 Sctlr        "sctlr_el1"   El1Rw "the EL1 system control register: MMU, alignment and cache enables";
    3 0  1 0 1 Actlr        "actlr_el1"   El1Rw "implementation-defined auxiliary control";
    3 0  1 0 2 Cpacr        "cpacr_el1"   El1Rw "which coprocessor and SIMD accesses EL0 and EL1 may make";

    // -- floating point ------------------------------------------------------
    //
    // Both are `El0Rw`: unprivileged code sets its own rounding mode and reads
    // its own sticky flags, which is what makes `fenv.h` work without a system
    // call.
    3 3  4 4 0 Fpcr         "fpcr"        El0Rw "the floating-point control register: rounding, flushing and the default NaN";
    3 3  4 4 1 Fpsr         "fpsr"        El0Rw "the floating-point status register: the cumulative exception flags";

    // -- translation ---------------------------------------------------------
    3 0  2 0 0 Ttbr0        "ttbr0_el1"   El1Rw "the base of the low half's translation table";
    3 0  2 0 1 Ttbr1        "ttbr1_el1"   El1Rw "the base of the high half's translation table";
    3 0  2 0 2 Tcr          "tcr_el1"     El1Rw "translation control: address sizes, granules and walk enables";
    3 0 10 2 0 Mair         "mair_el1"    El1Rw "the eight memory attribute encodings a descriptor indexes";
    3 0 10 3 0 Amair        "amair_el1"   El1Rw "implementation-defined auxiliary memory attributes";
    3 0 13 0 1 Contextidr   "contextidr_el1" El1Rw "the context identifier a trace unit records";

    // -- exception handling --------------------------------------------------
    3 0  4 0 0 Spsr         "spsr_el1"    El1Rw "the PSTATE an exception saved";
    3 0  4 0 1 Elr          "elr_el1"     El1Rw "the address an exception return goes to";
    3 0  4 1 0 SpEl0        "sp_el0"      El1Rw "the EL0 stack pointer, as EL1 sees it";
    3 0  5 1 0 Afsr0        "afsr0_el1"   El1Rw "implementation-defined auxiliary fault status";
    3 0  5 1 1 Afsr1        "afsr1_el1"   El1Rw "implementation-defined auxiliary fault status";
    3 0  5 2 0 Esr          "esr_el1"     El1Rw "why the last exception was taken";
    3 0  6 0 0 Far          "far_el1"     El1Rw "the address that faulted";
    3 0 12 0 0 Vbar         "vbar_el1"    El1Rw "the base of the EL1 exception vector table";
    3 0  4 2 0 Spsel        "spsel"       El1Rw "which stack pointer PSTATE selects";
    3 0  4 2 2 CurrentEl    "currentel"   El1Ro "which exception level is executing";
    3 3  4 2 0 NzcvReg      "nzcv"        El0Rw "the four condition flags";
    3 3  4 2 1 DaifReg      "daif"        El0Rw "the four interrupt mask bits";

    // -- software thread pointers -------------------------------------------
    3 0 13 0 4 TpidrEl1     "tpidr_el1"   El1Rw "a doubleword the kernel keeps per processor";
    3 3 13 0 2 TpidrEl0     "tpidr_el0"   El0Rw "a doubleword software keeps per thread";
    3 3 13 0 3 TpidrroEl0   "tpidrro_el0" El0Ro "a doubleword the kernel publishes to a thread";

    // -- debug ---------------------------------------------------------------
    2 0  0 2 2 Mdscr        "mdscr_el1"   El1Rw "the debug system control register";
}

/// Look a system register up by its `op0:op1:CRn:CRm:op2` encoding.
///
/// `None` is UNDEFINED. A linear scan over a few dozen rows on an instruction
/// a guest executes at boot and then rarely — an `MSR` in a hot loop is not a
/// shape real software has — so no index earns its keep here.
#[must_use]
pub fn lookup(enc: u16) -> Option<&'static SysRegSpec> {
    SYSREGS.iter().find(|spec| spec.enc == enc)
}

// ---------------------------------------------------------------------------
// The register file
// ---------------------------------------------------------------------------

/// `PSTATE` and every system register the core holds state for.
///
/// The identification registers are **not** here: they are computed from the
/// core's configuration when `MRS` reads them, because a snapshot that carried
/// `MIDR_EL1` could restore a core claiming to be a part it is not.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SysRegs {
    /// The condition flags. `PSTATE.NZCV`.
    pub nzcv: Nzcv,
    /// The interrupt masks, at their `DAIF` bit positions.
    pub daif: u64,
    /// Which exception level is executing. `PSTATE.EL`.
    pub el: El,
    /// Whether `SP` selects `SP_ELx` rather than `SP_EL0`. `PSTATE.SP`.
    ///
    /// Meaningful only at EL1: at EL0 the architecture uses `SP_EL0`
    /// regardless, and [`SysRegs::sp_is_el1`] applies that rule in one place.
    pub spsel: bool,
    /// The EL0 stack pointer.
    pub sp_el0: u64,
    /// The EL1 stack pointer.
    pub sp_el1: u64,
    /// `SCTLR_EL1`.
    pub sctlr: u64,
    /// `ACTLR_EL1`, which is implementation defined and here is storage.
    pub actlr: u64,
    /// `CPACR_EL1`.
    pub cpacr: u64,
    /// `TTBR0_EL1`.
    pub ttbr0: u64,
    /// `TTBR1_EL1`.
    pub ttbr1: u64,
    /// `TCR_EL1`.
    pub tcr: u64,
    /// `MAIR_EL1`.
    pub mair: u64,
    /// `AMAIR_EL1`.
    pub amair: u64,
    /// `CONTEXTIDR_EL1`.
    pub contextidr: u64,
    /// `SPSR_EL1`.
    pub spsr_el1: u64,
    /// `ELR_EL1`.
    pub elr_el1: u64,
    /// `ESR_EL1`.
    pub esr_el1: u64,
    /// `FAR_EL1`.
    pub far_el1: u64,
    /// `VBAR_EL1`.
    pub vbar_el1: u64,
    /// `AFSR0_EL1`, implementation defined.
    pub afsr0: u64,
    /// `AFSR1_EL1`, implementation defined.
    pub afsr1: u64,
    /// `TPIDR_EL1`.
    pub tpidr_el1: u64,
    /// `TPIDR_EL0`.
    pub tpidr_el0: u64,
    /// `TPIDRRO_EL0`.
    pub tpidrro_el0: u64,
    /// `MDSCR_EL1`.
    pub mdscr: u64,
    /// `FPCR`.
    pub fpcr: u64,
    /// `FPSR`.
    pub fpsr: u64,
    /// Bumped by anything that invalidates a cached translation, so the TLB
    /// beside this can drop its entries without being reached into.
    ///
    /// Derived-state bookkeeping rather than guest state, and it is
    /// deliberately part of the reset value rather than of a snapshot.
    pub translation_gen: u64,
}

/// The `SCTLR_EL1` bits this core acts on.
pub mod sctlr {
    /// MMU enable.
    pub const M: u64 = 1 << 0;
    /// Alignment check enable: unaligned accesses fault.
    pub const A: u64 = 1 << 1;
    /// Data cache enable. Modelled as storage — there is no cache here.
    pub const C: u64 = 1 << 2;
    /// Stack alignment check at EL1.
    pub const SA: u64 = 1 << 3;
    /// Stack alignment check at EL0.
    pub const SA0: u64 = 1 << 4;
    /// Instruction cache enable. Modelled as storage.
    pub const I: u64 = 1 << 12;
    /// Write permission implies execute-never.
    pub const WXN: u64 = 1 << 19;
    /// Exception endianness. Big-endian is not implemented.
    pub const EE: u64 = 1 << 25;
    /// EL0 endianness. Big-endian is not implemented.
    pub const E0E: u64 = 1 << 24;
}

impl SysRegs {
    /// The reset state of the system registers.
    ///
    /// DDI 0487: most of `SCTLR_EL1` resets to an architecturally UNKNOWN
    /// value, but `M`, `A`, `C`, `SA` and `I` reset to zero — the MMU off and
    /// no checks — and that is what a boot loader relies on. `PSTATE` comes up
    /// at EL1h with every interrupt masked, which is the one part of the reset
    /// state software genuinely depends on.
    #[must_use]
    pub fn new() -> SysRegs {
        SysRegs {
            nzcv: Nzcv::default(),
            daif: daif::ALL,
            el: El::El1,
            spsel: true,
            sp_el0: 0,
            sp_el1: 0,
            sctlr: 0,
            actlr: 0,
            cpacr: 0,
            ttbr0: 0,
            ttbr1: 0,
            tcr: 0,
            mair: 0,
            amair: 0,
            contextidr: 0,
            spsr_el1: 0,
            elr_el1: 0,
            esr_el1: 0,
            far_el1: 0,
            vbar_el1: 0,
            afsr0: 0,
            afsr1: 0,
            tpidr_el1: 0,
            tpidr_el0: 0,
            tpidrro_el0: 0,
            mdscr: 0,
            // DDI 0487: `FPCR` resets to an architecturally UNKNOWN value.
            // Zero is round-to-nearest, no flushing and NaN propagation on —
            // the IEEE default, and the state a guest that never writes
            // `FPCR` is entitled to assume nothing about but always gets here.
            fpcr: 0,
            fpsr: 0,
            translation_gen: 0,
        }
    }

    /// Which of the two stack pointers `SP` currently names.
    ///
    /// DDI 0487 D1: at EL0 the stack pointer is always `SP_EL0`, whatever
    /// `SPSel` says — `SPSel` is only meaningful at EL1 and above. Getting
    /// this wrong gives a core where a return to userspace keeps using the
    /// kernel stack, which is why the rule lives in one function.
    #[inline]
    #[must_use]
    pub const fn sp_is_el1(&self) -> bool {
        matches!(self.el, El::El1) && self.spsel
    }

    /// The stack pointer `SP` currently names.
    #[inline]
    #[must_use]
    pub const fn sp(&self) -> u64 {
        if self.sp_is_el1() {
            self.sp_el1
        } else {
            self.sp_el0
        }
    }

    /// Write the stack pointer `SP` currently names.
    #[inline]
    pub const fn set_sp(&mut self, value: u64) {
        if self.sp_is_el1() {
            self.sp_el1 = value;
        } else {
            self.sp_el0 = value;
        }
    }

    /// Whether an access to SIMD or floating-point functionality traps.
    ///
    /// DDI 0487 D: `CPACR_EL1.FPEN` (bits 21:20) is `0b00` or `0b10` to trap
    /// at both EL0 and EL1, `0b01` to trap at EL0 only, and `0b11` to trap
    /// neither. Two encodings meaning the same thing is not a transcription
    /// error — the architecture allocates `0b10` that way — and writing this
    /// as `fpen != 0b11` would get the EL0-only case wrong in the direction
    /// that lets an unprivileged process use registers the kernel has not
    /// saved for it.
    ///
    /// `CPACR_EL1` resets to zero, so a guest takes this trap on its first
    /// floating-point instruction unless it enabled access first. That is the
    /// architecture and not an inconvenience: it is how a kernel knows a
    /// process has started using the FPU.
    #[must_use]
    pub const fn fp_access_trapped(&self) -> bool {
        match (self.cpacr >> 20) & 3 {
            0b01 => matches!(self.el, El::El0),
            0b11 => false,
            _ => true,
        }
    }

    /// Whether the MMU is enabled for the current translation regime.
    #[inline]
    #[must_use]
    pub const fn mmu_enabled(&self) -> bool {
        self.sctlr & sctlr::M != 0
    }

    /// The `SPSR_EL1` value an exception taken now would save.
    ///
    /// DDI 0487 D1.11: `NZCV` at 31:28, `DAIF` at 9:6, `M[4]` clear for a
    /// return to AArch64, and `M[3:0]` naming the level and stack pointer —
    /// `0b0000` for EL0t, `0b0100` for EL1t, `0b0101` for EL1h.
    #[must_use]
    pub const fn spsr(&self) -> u64 {
        let mut value = self.nzcv.0 as u64;
        value |= self.daif & daif::ALL;
        value |= self.el.bits() << 2;
        if self.sp_is_el1() {
            value |= 1;
        }
        value
    }

    /// Restore `PSTATE` from an `SPSR` value, as `ERET` does.
    ///
    /// An `M[4]` set would be a return to AArch32, which this core does not
    /// implement; an `M[3:0]` naming EL2 or EL3 names a level it does not
    /// have. Both are *illegal exception returns*: the architecture sets
    /// `PSTATE.IL` and takes the return anyway. Modelling `IL` is out of scope
    /// here, so `false` reports the refusal to the caller, which raises
    /// `UNDEFINED` — a visibly wrong return rather than a silently plausible
    /// one.
    #[must_use]
    pub fn restore_pstate(&mut self, spsr: u64) -> bool {
        if spsr & (1 << 4) != 0 {
            return false;
        }
        let (el, spsel) = match spsr & 0xf {
            0b0000 => (El::El0, false),
            0b0100 => (El::El1, false),
            0b0101 => (El::El1, true),
            _ => return false,
        };
        self.nzcv = Nzcv((spsr as u32) & 0xf000_0000);
        self.daif = spsr & daif::ALL;
        self.el = el;
        self.spsel = spsel;
        true
    }
}

impl Default for SysRegs {
    fn default() -> Self {
        SysRegs::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encodings_are_unique() {
        for (i, a) in SYSREGS.iter().enumerate() {
            for b in &SYSREGS[i + 1..] {
                assert_ne!(
                    a.enc, b.enc,
                    "{:?} and {:?} share an encoding",
                    a.reg, b.reg
                );
            }
        }
    }

    /// The encodings, checked against the instruction words a real assembler
    /// emits. `mrs x0, sctlr_el1` is `0xd5381000`, and bits 20:5 of that word
    /// are the key the table is built on.
    #[test]
    fn encodings_match_assembled_instructions() {
        let key = |word: u32| ((word >> 5) & 0xffff) as u16;
        assert_eq!(lookup(key(0xd538_1000)).unwrap().reg, SysReg::Sctlr);
        assert_eq!(lookup(key(0xd538_2000)).unwrap().reg, SysReg::Ttbr0);
        assert_eq!(lookup(key(0xd538_2020)).unwrap().reg, SysReg::Ttbr1);
        assert_eq!(lookup(key(0xd538_2040)).unwrap().reg, SysReg::Tcr);
        assert_eq!(lookup(key(0xd538_c000)).unwrap().reg, SysReg::Vbar);
        assert_eq!(lookup(key(0xd538_4020)).unwrap().reg, SysReg::Elr);
        assert_eq!(lookup(key(0xd538_5200)).unwrap().reg, SysReg::Esr);
        assert_eq!(lookup(key(0xd538_a200)).unwrap().reg, SysReg::Mair);
        assert_eq!(lookup(key(0xd53b_d040)).unwrap().reg, SysReg::TpidrEl0);
    }

    #[test]
    fn spsr_round_trips_through_pstate() {
        let mut regs = SysRegs::new();
        regs.nzcv = Nzcv::new(true, false, true, false);
        regs.daif = daif::I;
        regs.el = El::El1;
        regs.spsel = true;
        let saved = regs.spsr();
        assert_eq!(saved & 0xf, 0b0101, "EL1h");

        let mut other = SysRegs::new();
        assert!(other.restore_pstate(saved));
        assert_eq!(other.nzcv, regs.nzcv);
        assert_eq!(other.daif, regs.daif);
        assert_eq!(other.el, El::El1);
        assert!(other.spsel);

        // A return to EL0 uses SP_EL0 whatever SPSel held.
        assert!(other.restore_pstate(0));
        assert_eq!(other.el, El::El0);
        assert!(!other.sp_is_el1());
    }

    #[test]
    fn an_aarch32_return_is_refused() {
        let mut regs = SysRegs::new();
        // M[4] set: a return to AArch32, which this core does not implement.
        assert!(!regs.restore_pstate(1 << 4));
        // EL2h: a level this core does not have.
        assert!(!regs.restore_pstate(0b1001));
    }

    #[test]
    fn el0_ignores_spsel() {
        let mut regs = SysRegs::new();
        regs.el = El::El0;
        regs.spsel = true;
        regs.sp_el0 = 0x1000;
        regs.sp_el1 = 0x2000;
        assert_eq!(regs.sp(), 0x1000);
        regs.set_sp(0x1008);
        assert_eq!(regs.sp_el0, 0x1008);
        assert_eq!(regs.sp_el1, 0x2000);
    }
}
