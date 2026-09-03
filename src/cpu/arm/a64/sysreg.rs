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
    /// Readable at EL0, writable only at EL1 — `TPIDRRO_EL0`, `CNTFRQ_EL0`.
    El0Ro,
    /// Read-only at **every** level: `CTR_EL0`, `DCZID_EL0`, `CNTPCT_EL0`,
    /// `CNTVCT_EL0`.
    ///
    /// Distinct from [`Access::El1Ro`], which is "EL1 may read it and EL0
    /// cannot see it at all" — the `_EL1` identification registers. These are
    /// the opposite shape: every level may read them and no level may write
    /// them, because they report what the hardware *is* rather than holding
    /// something software put there.
    ///
    /// The three read-only-ish variants are genuinely three, and collapsing
    /// any pair of them silently breaks a register: `llvm-mc` has no writable
    /// name for `CTR_EL0` and does have one for `TPIDRRO_EL0`, which is how
    /// this distinction was checked rather than argued.
    AllRo,
}

impl Access {
    /// Whether a read is permitted from `el`.
    #[must_use]
    pub const fn readable_at(self, el: El) -> bool {
        match self {
            Access::El1Rw | Access::El1Ro => matches!(el, El::El1),
            Access::El0Rw | Access::El0Ro | Access::AllRo => true,
        }
    }

    /// Whether a write is permitted from `el`.
    ///
    /// A read-only register is *not* a trap: DDI 0487 gives the identification
    /// registers no write behaviour at EL1, so a write there is UNDEFINED.
    /// That is what `false` here means, and the caller raises the exception.
    ///
    /// [`Access::El0Ro`] is read-only **at EL0 only**. `TPIDRRO_EL0` is the
    /// register the name is about: the "RO" is the thread's view, and the
    /// kernel writing it at EL1 is the entire purpose of the register. This
    /// used to return `false` for it at both levels, so `msr tpidrro_el0, x0`
    /// raised UNDEFINED on a core whose own table said the write was allowed.
    #[must_use]
    pub const fn writable_at(self, el: El) -> bool {
        match self {
            Access::El1Rw | Access::El0Ro => matches!(el, El::El1),
            Access::El0Rw => true,
            Access::El1Ro | Access::AllRo => false,
        }
    }
}

/// The `CNTKCTL_EL1` bits that decide what EL0 may reach of the generic timer.
///
/// DDI 0487 D11.2: an EL0 access to a counter or timer register that the
/// matching bit does not permit is **trapped to EL1** with `ESR_EL1.EC` 0x18
/// — not UNDEFINED. The difference matters: a kernel that wants to virtualise
/// the counter for one process leaves the bit clear and emulates the read in
/// its own handler, which it can only do if it is told which register was
/// named. `CNTKCTL_EL1` resets to zero, so EL0 reaches none of it until a
/// kernel says otherwise.
pub mod cntkctl {
    /// EL0 may read `CNTPCT_EL0` and `CNTFRQ_EL0`.
    pub const EL0PCTEN: u64 = 1 << 0;
    /// EL0 may read `CNTVCT_EL0`.
    pub const EL0VCTEN: u64 = 1 << 1;
    /// EL0 may reach the `CNTV_*` timer registers.
    pub const EL0VTEN: u64 = 1 << 8;
    /// EL0 may reach the `CNTP_*` timer registers.
    pub const EL0PTEN: u64 = 1 << 9;
    /// Every bit this core acts on. The rest of the register — the event
    /// stream fields `EVNTEN`, `EVNTDIR` and `EVNTI` — is storage, because
    /// this core has no event stream and `WFE` does not stall.
    pub const ACTED_ON: u64 = EL0PCTEN | EL0VCTEN | EL0VTEN | EL0PTEN;
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
    /// Which [`cntkctl`] bit EL0 additionally needs, or zero for a register
    /// whose reach is decided by [`access`](SysRegSpec::access) alone.
    ///
    /// A column rather than more [`Access`] variants because the two are
    /// genuinely independent axes: `CNTFRQ_EL0` and `TPIDRRO_EL0` have the
    /// same permissions and only one of them is gated.
    pub el0_gate: u64,
}

/// Declare the register identifier, its name, its summary and the lookup table
/// from one list of rows.
macro_rules! sysregs {
    ($($op0:literal $op1:literal $crn:literal $crm:literal $op2:literal
       $reg:ident $name:literal $access:ident $($gate:ident)? $summary:literal;)*) => {
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
                el0_gate: 0 $(| cntkctl::$gate)?,
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
    3 3  0 0 1 Ctr          "ctr_el0"          AllRo "cache type: the line sizes software must respect";
    3 3  0 0 7 Dczid        "dczid_el0"        AllRo "the block size DC ZVA operates on, and whether it is allowed";

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

    // -- the generic timer ---------------------------------------------------
    //
    // The counter views are `AllRo` — every level reads them, no level
    // writes them — while `CNTFRQ_EL0` is `El0Ro`, because the frequency is a
    // *number firmware puts there* rather than something the hardware
    // reports. That asymmetry is the architecture's and is the reason a guest
    // that trusts `CNTFRQ_EL0` without firmware having programmed it reads a
    // frequency nothing guarantees.
    //
    // The fourth column is the `CNTKCTL_EL1` bit EL0 additionally needs. All
    // four reset clear, so out of reset EL0 reaches none of this and a
    // `mrs x0, cntvct_el0` from userspace traps to EL1 — which is what a
    // kernel that has not yet set up its `vDSO` wants.
    3 3 14 0 0 Cntfrq       "cntfrq_el0"     El0Ro     EL0PCTEN "how fast the system counter counts, as firmware declared it";
    3 3 14 0 1 Cntpct       "cntpct_el0"     AllRo     EL0PCTEN "the physical count: the system counter, read directly";
    3 3 14 0 2 Cntvct       "cntvct_el0"     AllRo     EL0VCTEN "the virtual count, which without EL2 is the physical one";
    3 0 14 1 0 Cntkctl      "cntkctl_el1"    El1Rw              "what EL0 may reach of the counter and the timers";
    3 3 14 2 0 CntpTval     "cntp_tval_el0"  El0Rw     EL0PTEN  "how long until the EL1 physical timer fires, as a 32-bit countdown";
    3 3 14 2 1 CntpCtl      "cntp_ctl_el0"   El0Rw     EL0PTEN  "the EL1 physical timer's enable, mask and status";
    3 3 14 2 2 CntpCval     "cntp_cval_el0"  El0Rw     EL0PTEN  "the count at which the EL1 physical timer fires";
    3 3 14 3 0 CntvTval     "cntv_tval_el0"  El0Rw     EL0VTEN  "how long until the EL1 virtual timer fires, as a 32-bit countdown";
    3 3 14 3 1 CntvCtl      "cntv_ctl_el0"   El0Rw     EL0VTEN  "the EL1 virtual timer's enable, mask and status";
    3 3 14 3 2 CntvCval     "cntv_cval_el0"  El0Rw     EL0VTEN  "the count at which the EL1 virtual timer fires";

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
    /// `CNTFRQ_EL0`: the counter frequency firmware declared, in Hz.
    ///
    /// Guest state rather than configuration, because the architecture makes
    /// it a register software writes. Its reset value comes from the board
    /// (`Config::cntfrq`) so a guest that never programs it still reads the
    /// truth, which real silicon does not promise.
    pub cntfrq: u64,
    /// `CNTKCTL_EL1`.
    pub cntkctl: u64,
    /// `CNTP_CTL_EL0`, holding only the two writable bits — `ISTATUS` is
    /// computed from the counter on every read rather than stored, because a
    /// stored status bit is one that can be stale.
    pub cntp_ctl: u64,
    /// `CNTP_CVAL_EL0`.
    pub cntp_cval: u64,
    /// `CNTV_CTL_EL0`, as [`cntp_ctl`](SysRegs::cntp_ctl).
    pub cntv_ctl: u64,
    /// `CNTV_CVAL_EL0`.
    pub cntv_cval: u64,
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

/// The `CNTP_CTL_EL0` and `CNTV_CTL_EL0` bits.
///
/// DDI 0487 D11.2.4. `ISTATUS` is **read-only and computed**: it is the answer
/// to a comparison against the counter, not a latch, so a timer whose
/// comparator a guest moves past the count stops asserting on the same
/// instruction rather than on the next write to the register.
pub mod cntctl {
    /// The timer is enabled.
    pub const ENABLE: u64 = 1 << 0;
    /// The timer's output is masked. Note the polarity: set means *masked*,
    /// which is the opposite of `ENABLE` and is a standing source of
    /// off-by-inversion in timer drivers.
    pub const IMASK: u64 = 1 << 1;
    /// The timer condition is met. Read-only.
    pub const ISTATUS: u64 = 1 << 2;
    /// The bits a guest may write.
    pub const WRITABLE: u64 = ENABLE | IMASK;
}

/// Whether a timer with comparator `cval` has reached its deadline at `count`.
///
/// DDI 0487 D11.2.4 states the comparison as `Count - CompareValue >= 0` in
/// **signed 64-bit** arithmetic, and the wording is load-bearing: an unsigned
/// `count >= cval` gets every deadline that wraps the counter wrong, and a
/// guest that sets a comparator just below the wrap point would see its timer
/// fire immediately and then never again. The subtraction wraps and the
/// *result* is read as signed, which makes the comparison a statement about
/// the distance between two points rather than about their order.
#[inline]
#[must_use]
pub const fn timer_condition_met(cval: u64, count: u64) -> bool {
    count.wrapping_sub(cval) as i64 >= 0
}

/// What a `CNT{P,V}_CTL_EL0` read reports, with `ISTATUS` filled in.
///
/// `ISTATUS` reads as zero while `ENABLE` is clear, whatever the comparator
/// says — the architecture is explicit, and it is why a disabled timer cannot
/// be polled for "would it have fired".
#[inline]
#[must_use]
pub const fn timer_ctl(ctl: u64, cval: u64, count: u64) -> u64 {
    let stored = ctl & cntctl::WRITABLE;
    if stored & cntctl::ENABLE != 0 && timer_condition_met(cval, count) {
        stored | cntctl::ISTATUS
    } else {
        stored
    }
}

/// Whether a timer is asserting its interrupt output.
///
/// `ENABLE && ISTATUS && !IMASK` — all three, which is the whole of the
/// timer's outward behaviour.
#[inline]
#[must_use]
pub const fn timer_output(ctl: u64, cval: u64, count: u64) -> bool {
    let live = timer_ctl(ctl, cval, count);
    live & cntctl::ISTATUS != 0 && live & cntctl::IMASK == 0
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
            // DDI 0487 D11.2: `CNTFRQ_EL0` resets to an architecturally
            // UNKNOWN value and `CNTP_CVAL_EL0`/`CNTV_CVAL_EL0` likewise; the
            // two `CTL` registers reset with `ENABLE` clear, which is the part
            // software depends on — a timer that fired on its own before a
            // kernel had a vector table would be unrecoverable. `CNTFRQ_EL0`
            // is overwritten from the board's `cntfrq` by `State::new`.
            cntfrq: 0,
            cntkctl: 0,
            cntp_ctl: 0,
            cntp_cval: 0,
            cntv_ctl: 0,
            cntv_cval: 0,
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

    /// Whether either EL1 timer is asserting its interrupt at `count`.
    ///
    /// One function because the two timers are wired together here. On a real
    /// SoC they are two private peripheral interrupts a GIC forwards
    /// separately (PPI 30 and PPI 27); this core has no GIC, so both land on
    /// the same internal `IRQ` and a handler tells them apart by reading
    /// `ISTATUS`, exactly as it would with a shared line.
    #[inline]
    #[must_use]
    pub const fn timer_irq(&self, count: u64) -> bool {
        timer_output(self.cntp_ctl, self.cntp_cval, count)
            || timer_output(self.cntv_ctl, self.cntv_cval, count)
    }

    /// Whether an EL0 access to `spec` is one `CNTKCTL_EL1` permits.
    ///
    /// Always true at EL1: the gate is about what a *thread* may reach, and
    /// the kernel that owns the gate is never gated by it.
    #[inline]
    #[must_use]
    pub const fn cnt_gate_open(&self, spec: &SysRegSpec) -> bool {
        match self.el {
            El::El1 => true,
            El::El0 => spec.el0_gate == 0 || self.cntkctl & spec.el0_gate != 0,
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

    /// Only the generic timer's counter and timer views are gated, and each by
    /// the bit DDI 0487 D11.2 names. A gate on the wrong row would either open
    /// something EL0 must not reach or shut something a `vDSO` needs.
    #[test]
    fn only_the_timer_rows_are_gated() {
        for spec in SYSREGS {
            let want = match spec.reg {
                SysReg::Cntfrq | SysReg::Cntpct => cntkctl::EL0PCTEN,
                SysReg::Cntvct => cntkctl::EL0VCTEN,
                SysReg::CntpTval | SysReg::CntpCtl | SysReg::CntpCval => cntkctl::EL0PTEN,
                SysReg::CntvTval | SysReg::CntvCtl | SysReg::CntvCval => cntkctl::EL0VTEN,
                _ => 0,
            };
            assert_eq!(spec.el0_gate, want, "{:?} has the wrong EL0 gate", spec.reg);
            assert_eq!(
                spec.el0_gate & !cntkctl::ACTED_ON,
                0,
                "{:?} names a CNTKCTL_EL1 bit this core does not act on",
                spec.reg
            );
        }
    }

    /// The three read-only shapes are three, and each row has the one the
    /// architecture gives it. `El0Ro` is read-only *at EL0* — `TPIDRRO_EL0` is
    /// the register the name is about, and a kernel writing it at EL1 is the
    /// whole point of it — while `AllRo` is read-only everywhere.
    #[test]
    fn the_read_only_shapes_do_not_collapse_into_each_other() {
        assert!(Access::El0Ro.writable_at(El::El1));
        assert!(!Access::El0Ro.writable_at(El::El0));
        assert!(!Access::AllRo.writable_at(El::El1));
        assert!(Access::AllRo.readable_at(El::El0));
        assert!(!Access::El1Ro.readable_at(El::El0));

        let of = |reg| SYSREGS.iter().find(|s| s.reg == reg).unwrap().access;
        assert_eq!(of(SysReg::TpidrroEl0), Access::El0Ro);
        for reg in [SysReg::Ctr, SysReg::Dczid, SysReg::Cntpct, SysReg::Cntvct] {
            assert_eq!(of(reg), Access::AllRo, "{reg:?} must be read-only at EL1");
        }
        assert_eq!(of(SysReg::Cntfrq), Access::El0Ro, "EL1 programs it");
    }

    /// `ISTATUS` is `ENABLE && (count - cval >= 0)` and the output is that with
    /// `IMASK` clear — all three, and the table is small enough to write out.
    #[test]
    fn the_timer_output_needs_every_one_of_its_three_bits() {
        use cntctl::{ENABLE, IMASK};
        // Met, enabled, unmasked.
        assert_eq!(timer_ctl(ENABLE, 10, 10), ENABLE | cntctl::ISTATUS);
        assert!(timer_output(ENABLE, 10, 10));
        // Masked: the status still reads, the output does not assert.
        assert_eq!(
            timer_ctl(ENABLE | IMASK, 10, 10),
            ENABLE | IMASK | cntctl::ISTATUS
        );
        assert!(!timer_output(ENABLE | IMASK, 10, 10));
        // Disabled: no status at all, whatever the comparator says.
        assert_eq!(timer_ctl(0, 0, 1000), 0);
        assert!(!timer_output(0, 0, 1000));
        // Enabled but not yet met.
        assert_eq!(timer_ctl(ENABLE, 1000, 10), ENABLE);
        assert!(!timer_output(ENABLE, 1000, 10));
    }

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
        // The generic timer, every row of it. These are the encodings that
        // decide whether a guest's `mrs x0, cntvct_el0` reaches the counter or
        // is UNDEFINED, and getting one `CRm` wrong would be invisible to any
        // test built out of this table.
        assert_eq!(lookup(key(0xd53b_e000)).unwrap().reg, SysReg::Cntfrq);
        assert_eq!(lookup(key(0xd53b_e020)).unwrap().reg, SysReg::Cntpct);
        assert_eq!(lookup(key(0xd53b_e040)).unwrap().reg, SysReg::Cntvct);
        assert_eq!(lookup(key(0xd538_e100)).unwrap().reg, SysReg::Cntkctl);
        assert_eq!(lookup(key(0xd53b_e200)).unwrap().reg, SysReg::CntpTval);
        assert_eq!(lookup(key(0xd53b_e220)).unwrap().reg, SysReg::CntpCtl);
        assert_eq!(lookup(key(0xd53b_e240)).unwrap().reg, SysReg::CntpCval);
        assert_eq!(lookup(key(0xd53b_e300)).unwrap().reg, SysReg::CntvTval);
        assert_eq!(lookup(key(0xd53b_e320)).unwrap().reg, SysReg::CntvCtl);
        assert_eq!(lookup(key(0xd53b_e340)).unwrap().reg, SysReg::CntvCval);
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
