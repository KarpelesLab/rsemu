//! The control and status registers, and the privileged architecture's state.
//!
//! *The RISC-V Instruction Set Manual, Volume II: Privileged Architecture*
//! (CC-BY-4.0) is the source for everything here: the machine and supervisor
//! CSR maps, the `mstatus` field layout, trap delegation through `medeleg` and
//! `mideleg`, the exception and interrupt cause numbers, and the `satp` and
//! PMP register formats.
//!
//! # Why the registers are named fields
//!
//! A CSR file is *not* a 4096-entry array. Almost every register is WARL — it
//! silently discards the bits the implementation does not support — and
//! several are windows onto another register rather than storage of their own:
//! `sstatus` is a masked view of `mstatus`, `sie` and `sip` are `mie` and
//! `mip` filtered through `mideleg`, and `fflags` and `frm` are two halves of
//! `fcsr`. Modelled as an array, every one of those becomes a special case in
//! the access path anyway, and the masks stop being visible. Named fields put
//! each register's rules next to it.
//!
//! # What `mip` is not
//!
//! `mip` lives in [`Lines`] rather than here, as an atomic outside the
//! execution lock. A device asserting an external interrupt from inside a
//! write the CPU itself issued would otherwise re-enter the CPU's own critical
//! section, which is a deadlock under `native-std` and a panic under `single`
//! (`ROADMAP.md` §4.7). The 6502's interrupt pins are the same shape for the
//! same reason.

use crate::core::sync::{AtomicBool, AtomicU64, Ordering};

use super::isa::Xlen;

/// A privilege mode.
///
/// The encoding is the specification's, because it appears directly in
/// `mstatus.MPP` and in the ECALL cause numbers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Priv {
    /// User mode.
    User = 0,
    /// Supervisor mode.
    Supervisor = 1,
    /// Machine mode: always implemented, and the mode a reset leaves the hart
    /// in.
    Machine = 3,
}

impl Priv {
    /// Decode a two-bit privilege field. Level 2 is reserved.
    #[must_use]
    pub const fn from_bits(bits: u64) -> Option<Priv> {
        match bits {
            0 => Some(Priv::User),
            1 => Some(Priv::Supervisor),
            3 => Some(Priv::Machine),
            _ => None,
        }
    }

    /// The two-bit encoding.
    #[inline]
    #[must_use]
    pub const fn bits(self) -> u64 {
        self as u64
    }

    /// The mode's short name, for a trace or the monitor.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Priv::User => "U",
            Priv::Supervisor => "S",
            Priv::Machine => "M",
        }
    }
}

/// Trap cause numbers.
///
/// Volume II, "Machine Cause Register": the exception codes are the value of
/// `mcause` with its interrupt bit clear, the interrupt codes with it set.
pub mod cause {
    /// Instruction address misaligned.
    pub const INSN_MISALIGNED: u64 = 0;
    /// Instruction access fault.
    pub const INSN_ACCESS: u64 = 1;
    /// Illegal instruction.
    pub const ILLEGAL_INSN: u64 = 2;
    /// Breakpoint.
    pub const BREAKPOINT: u64 = 3;
    /// Load address misaligned.
    pub const LOAD_MISALIGNED: u64 = 4;
    /// Load access fault.
    pub const LOAD_ACCESS: u64 = 5;
    /// Store or AMO address misaligned.
    pub const STORE_MISALIGNED: u64 = 6;
    /// Store or AMO access fault.
    pub const STORE_ACCESS: u64 = 7;
    /// Environment call from U-mode.
    pub const ECALL_U: u64 = 8;
    /// Environment call from S-mode.
    pub const ECALL_S: u64 = 9;
    /// Environment call from M-mode.
    pub const ECALL_M: u64 = 11;
    /// Instruction page fault.
    pub const INSN_PAGE_FAULT: u64 = 12;
    /// Load page fault.
    pub const LOAD_PAGE_FAULT: u64 = 13;
    /// Store or AMO page fault.
    pub const STORE_PAGE_FAULT: u64 = 15;

    /// Supervisor software interrupt.
    pub const IRQ_S_SOFT: u64 = 1;
    /// Machine software interrupt.
    pub const IRQ_M_SOFT: u64 = 3;
    /// Supervisor timer interrupt.
    pub const IRQ_S_TIMER: u64 = 5;
    /// Machine timer interrupt.
    pub const IRQ_M_TIMER: u64 = 7;
    /// Supervisor external interrupt.
    pub const IRQ_S_EXT: u64 = 9;
    /// Machine external interrupt.
    pub const IRQ_M_EXT: u64 = 11;
}

/// Bit masks for the interrupt-pending and interrupt-enable registers.
///
/// Each is `1 << cause`, which is how `mip`, `mie`, `sip` and `sie` are laid
/// out.
pub mod irq {
    /// Supervisor software interrupt pending/enable.
    pub const SSI: u64 = 1 << super::cause::IRQ_S_SOFT;
    /// Machine software interrupt pending/enable.
    pub const MSI: u64 = 1 << super::cause::IRQ_M_SOFT;
    /// Supervisor timer interrupt pending/enable.
    pub const STI: u64 = 1 << super::cause::IRQ_S_TIMER;
    /// Machine timer interrupt pending/enable.
    pub const MTI: u64 = 1 << super::cause::IRQ_M_TIMER;
    /// Supervisor external interrupt pending/enable.
    pub const SEI: u64 = 1 << super::cause::IRQ_S_EXT;
    /// Machine external interrupt pending/enable.
    pub const MEI: u64 = 1 << super::cause::IRQ_M_EXT;

    /// Every interrupt a supervisor may see.
    pub const S_MASK: u64 = SSI | STI | SEI;
    /// Every interrupt this core models.
    pub const ALL: u64 = SSI | MSI | STI | MTI | SEI | MEI;

    /// The order interrupts are taken in, highest priority first.
    ///
    /// Volume II, "Machine Interrupt Registers": MEI, MSI, MTI, SEI, SSI, STI.
    /// Not the numeric order, which is why it is written out.
    pub const PRIORITY: [u64; 6] = [
        super::cause::IRQ_M_EXT,
        super::cause::IRQ_M_SOFT,
        super::cause::IRQ_M_TIMER,
        super::cause::IRQ_S_EXT,
        super::cause::IRQ_S_SOFT,
        super::cause::IRQ_S_TIMER,
    ];
}

/// `mstatus` field positions.
///
/// Volume II, "Machine Status Registers": the RV64 layout, which this core
/// stores even when configured for RV32 — the RV32 register is the low 32 bits
/// of the same state, with `SD` moved to bit 31 on the way out.
pub mod status {
    /// Supervisor interrupt enable.
    pub const SIE: u64 = 1 << 1;
    /// Machine interrupt enable.
    pub const MIE: u64 = 1 << 3;
    /// Supervisor previous interrupt enable.
    pub const SPIE: u64 = 1 << 5;
    /// User byte order.
    pub const UBE: u64 = 1 << 6;
    /// Machine previous interrupt enable.
    pub const MPIE: u64 = 1 << 7;
    /// Supervisor previous privilege (one bit: U or S).
    pub const SPP: u64 = 1 << 8;
    /// Shift of the two-bit machine previous privilege field.
    pub const MPP_SHIFT: u32 = 11;
    /// The machine previous privilege field.
    pub const MPP: u64 = 3 << MPP_SHIFT;
    /// Shift of the two-bit floating-point state field.
    pub const FS_SHIFT: u32 = 13;
    /// The floating-point state field.
    pub const FS: u64 = 3 << FS_SHIFT;
    /// Shift of the two-bit extension state field.
    pub const XS_SHIFT: u32 = 15;
    /// The extension state field.
    pub const XS: u64 = 3 << XS_SHIFT;
    /// Modify privilege: loads and stores use `MPP`'s privilege.
    pub const MPRV: u64 = 1 << 17;
    /// Permit supervisor user memory access.
    pub const SUM: u64 = 1 << 18;
    /// Make executable readable.
    pub const MXR: u64 = 1 << 19;
    /// Trap virtual memory.
    pub const TVM: u64 = 1 << 20;
    /// Timeout wait: `WFI` traps below M-mode.
    pub const TW: u64 = 1 << 21;
    /// Trap `SRET`.
    pub const TSR: u64 = 1 << 22;
    /// Shift of the user XLEN field (RV64 only).
    pub const UXL_SHIFT: u32 = 32;
    /// Shift of the supervisor XLEN field (RV64 only).
    pub const SXL_SHIFT: u32 = 34;

    /// The `FS` value meaning the floating-point unit is off.
    pub const FS_OFF: u64 = 0;
    /// The `FS` value meaning floating-point state has been modified.
    pub const FS_DIRTY: u64 = 3;

    /// Every bit a write to `mstatus` may change.
    ///
    /// `XS` is read-only zero because this core has no other extension state,
    /// and `SD` is derived rather than stored.
    pub const M_WRITABLE: u64 =
        SIE | MIE | SPIE | UBE | MPIE | SPP | MPP | FS | MPRV | SUM | MXR | TVM | TW | TSR;

    /// Every bit `sstatus` exposes — a strict subset, which is what makes
    /// `sstatus` a view rather than a register.
    pub const S_VISIBLE: u64 = SIE | SPIE | UBE | SPP | FS | XS | SUM | MXR;
    /// Every bit a write to `sstatus` may change.
    pub const S_WRITABLE: u64 = SIE | SPIE | UBE | SPP | FS | SUM | MXR;
}

/// CSR numbers, as the specification assigns them.
pub mod num {
    /// Accrued floating-point exception flags.
    pub const FFLAGS: u32 = 0x001;
    /// Floating-point dynamic rounding mode.
    pub const FRM: u32 = 0x002;
    /// Floating-point control and status.
    pub const FCSR: u32 = 0x003;

    /// Supervisor status.
    pub const SSTATUS: u32 = 0x100;
    /// Supervisor interrupt enable.
    pub const SIE: u32 = 0x104;
    /// Supervisor trap vector.
    pub const STVEC: u32 = 0x105;
    /// Supervisor counter enable.
    pub const SCOUNTEREN: u32 = 0x106;
    /// Supervisor environment configuration.
    pub const SENVCFG: u32 = 0x10a;
    /// Supervisor scratch.
    pub const SSCRATCH: u32 = 0x140;
    /// Supervisor exception program counter.
    pub const SEPC: u32 = 0x141;
    /// Supervisor cause.
    pub const SCAUSE: u32 = 0x142;
    /// Supervisor trap value.
    pub const STVAL: u32 = 0x143;
    /// Supervisor interrupt pending.
    pub const SIP: u32 = 0x144;
    /// Supervisor address translation and protection.
    pub const SATP: u32 = 0x180;

    /// Machine status.
    pub const MSTATUS: u32 = 0x300;
    /// Machine ISA.
    pub const MISA: u32 = 0x301;
    /// Machine exception delegation.
    pub const MEDELEG: u32 = 0x302;
    /// Machine interrupt delegation.
    pub const MIDELEG: u32 = 0x303;
    /// Machine interrupt enable.
    pub const MIE: u32 = 0x304;
    /// Machine trap vector.
    pub const MTVEC: u32 = 0x305;
    /// Machine counter enable.
    pub const MCOUNTEREN: u32 = 0x306;
    /// Machine environment configuration.
    pub const MENVCFG: u32 = 0x30a;
    /// Machine status, high half (RV32 only).
    pub const MSTATUSH: u32 = 0x310;
    /// Machine environment configuration, high half (RV32 only).
    pub const MENVCFGH: u32 = 0x31a;
    /// Machine counter inhibit.
    pub const MCOUNTINHIBIT: u32 = 0x320;
    /// Machine scratch.
    pub const MSCRATCH: u32 = 0x340;
    /// Machine exception program counter.
    pub const MEPC: u32 = 0x341;
    /// Machine cause.
    pub const MCAUSE: u32 = 0x342;
    /// Machine trap value.
    pub const MTVAL: u32 = 0x343;
    /// Machine interrupt pending.
    pub const MIP: u32 = 0x344;

    /// First physical memory protection configuration register.
    pub const PMPCFG0: u32 = 0x3a0;
    /// Last physical memory protection configuration register.
    pub const PMPCFG15: u32 = 0x3af;
    /// First physical memory protection address register.
    pub const PMPADDR0: u32 = 0x3b0;
    /// Last physical memory protection address register.
    pub const PMPADDR63: u32 = 0x3ef;

    /// Trigger select (debug trigger module).
    pub const TSELECT: u32 = 0x7a0;
    /// Trigger data 1.
    pub const TDATA1: u32 = 0x7a1;
    /// Trigger data 2.
    pub const TDATA2: u32 = 0x7a2;
    /// Trigger data 3.
    pub const TDATA3: u32 = 0x7a3;
    /// Trigger information.
    pub const TINFO: u32 = 0x7a4;
    /// Trigger control.
    pub const TCONTROL: u32 = 0x7a5;

    /// Machine cycle counter.
    pub const MCYCLE: u32 = 0xb00;
    /// Machine instructions-retired counter.
    pub const MINSTRET: u32 = 0xb02;
    /// Machine cycle counter, high half (RV32 only).
    pub const MCYCLEH: u32 = 0xb80;
    /// Machine instructions-retired counter, high half (RV32 only).
    pub const MINSTRETH: u32 = 0xb82;

    /// Read-only cycle counter.
    pub const CYCLE: u32 = 0xc00;
    /// Read-only wall-clock counter.
    pub const TIME: u32 = 0xc01;
    /// Read-only instructions-retired counter.
    pub const INSTRET: u32 = 0xc02;
    /// Read-only cycle counter, high half (RV32 only).
    pub const CYCLEH: u32 = 0xc80;
    /// Read-only wall-clock counter, high half (RV32 only).
    pub const TIMEH: u32 = 0xc81;
    /// Read-only instructions-retired counter, high half (RV32 only).
    pub const INSTRETH: u32 = 0xc82;

    /// Vendor identity.
    pub const MVENDORID: u32 = 0xf11;
    /// Architecture identity.
    pub const MARCHID: u32 = 0xf12;
    /// Implementation identity.
    pub const MIMPID: u32 = 0xf13;
    /// Hart identity.
    pub const MHARTID: u32 = 0xf14;
}

/// The canonical name of a CSR number, for the disassembler and the monitor.
///
/// A second list beside [`num`], which is exactly the drift CLAUDE.md warns
/// about — so `every_named_csr_has_a_name` in this file's tests walks the
/// constants and fails if one gains a number without gaining a name.
/// Unimplemented numbers, and the PMP registers past the first of each kind,
/// return `None` and print as hex.
#[must_use]
pub fn csr_name(number: u32) -> Option<&'static str> {
    let name = match number {
        num::FFLAGS => "fflags",
        num::FRM => "frm",
        num::FCSR => "fcsr",
        num::SSTATUS => "sstatus",
        num::SIE => "sie",
        num::STVEC => "stvec",
        num::SCOUNTEREN => "scounteren",
        num::SENVCFG => "senvcfg",
        num::SSCRATCH => "sscratch",
        num::SEPC => "sepc",
        num::SCAUSE => "scause",
        num::STVAL => "stval",
        num::SIP => "sip",
        num::SATP => "satp",
        num::MSTATUS => "mstatus",
        num::MISA => "misa",
        num::MEDELEG => "medeleg",
        num::MIDELEG => "mideleg",
        num::MIE => "mie",
        num::MTVEC => "mtvec",
        num::MCOUNTEREN => "mcounteren",
        num::MENVCFG => "menvcfg",
        num::MSTATUSH => "mstatush",
        num::MENVCFGH => "menvcfgh",
        num::MCOUNTINHIBIT => "mcountinhibit",
        num::MSCRATCH => "mscratch",
        num::MEPC => "mepc",
        num::MCAUSE => "mcause",
        num::MTVAL => "mtval",
        num::MIP => "mip",
        num::PMPCFG0 => "pmpcfg0",
        num::PMPADDR0 => "pmpaddr0",
        num::TSELECT => "tselect",
        num::TDATA1 => "tdata1",
        num::TDATA2 => "tdata2",
        num::TDATA3 => "tdata3",
        num::TINFO => "tinfo",
        num::TCONTROL => "tcontrol",
        num::MCYCLE => "mcycle",
        num::MINSTRET => "minstret",
        num::MCYCLEH => "mcycleh",
        num::MINSTRETH => "minstreth",
        num::CYCLE => "cycle",
        num::TIME => "time",
        num::INSTRET => "instret",
        num::CYCLEH => "cycleh",
        num::TIMEH => "timeh",
        num::INSTRETH => "instreth",
        num::MVENDORID => "mvendorid",
        num::MARCHID => "marchid",
        num::MIMPID => "mimpid",
        num::MHARTID => "mhartid",
        _ => return None,
    };
    Some(name)
}

/// How many PMP entries this core can implement.
///
/// Sixteen is the smallest count real hardware ships and the count OpenSBI
/// expects to find. How many are *actually* implemented is
/// [`Csrs::pmp_count`], a construction property: zero means PMP is absent,
/// which the specification defines as every access passing, and is the right
/// configuration for a hart with no firmware to program it.
pub const PMP_ENTRIES: usize = 16;

/// The interrupt and reset inputs, kept outside the execution lock.
///
/// Atomics rather than fields under the mutex, for the reason `core::device`'s
/// re-entrancy contract gives: a PLIC or CLINT raising an interrupt from
/// inside an MMIO write the CPU itself issued must not have to take the lock
/// the CPU is holding. `mip` is the whole of that state, because every
/// interrupt this core can take is one of its bits.
#[derive(Debug, Default)]
pub struct Lines {
    /// Machine interrupt pending, driven by wires and by M-mode CSR writes.
    mip: AtomicU64,
    /// A reset assertion nobody has acted on yet.
    reset_req: AtomicBool,
}

impl Lines {
    /// Set or clear one interrupt-pending bit.
    pub fn set_pending(&self, mask: u64, asserted: bool) {
        if asserted {
            self.mip.fetch_or(mask, Ordering::AcqRel);
        } else {
            self.mip.fetch_and(!mask, Ordering::AcqRel);
        }
    }

    /// The whole interrupt-pending register.
    #[must_use]
    pub fn pending(&self) -> u64 {
        self.mip.load(Ordering::Acquire)
    }

    /// Replace the whole interrupt-pending register.
    pub fn set_all_pending(&self, value: u64) {
        self.mip.store(value & irq::ALL, Ordering::Release);
    }

    /// Latch a reset request. Idempotent: two pulses are one reset.
    pub fn request_reset(&self) {
        self.reset_req.store(true, Ordering::Release);
    }

    /// Consume the reset request, reporting whether there was one.
    pub fn take_reset_request(&self) -> bool {
        self.reset_req.swap(false, Ordering::AcqRel)
    }

    /// Whether a reset is latched, without consuming it.
    #[must_use]
    pub fn reset_requested(&self) -> bool {
        self.reset_req.load(Ordering::Acquire)
    }
}

/// Which extensions a core implements, and therefore what `misa` reports and
/// which encodings decode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Extensions {
    /// Integer multiply and divide.
    pub m: bool,
    /// Atomics.
    pub a: bool,
    /// Single-precision floating point.
    pub f: bool,
    /// Double-precision floating point. Implies `f`.
    pub d: bool,
    /// The compressed encodings.
    pub c: bool,
    /// Supervisor mode.
    pub s: bool,
    /// User mode.
    pub u: bool,
}

impl Extensions {
    /// Everything RV64GC implies, plus supervisor and user mode: the
    /// configuration a Linux-capable hart needs.
    pub const GC: Extensions = Extensions {
        m: true,
        a: true,
        f: true,
        d: true,
        c: true,
        s: true,
        u: true,
    };

    /// The bare integer core: no extensions beyond `I`, machine mode only.
    pub const I: Extensions = Extensions {
        m: false,
        a: false,
        f: false,
        d: false,
        c: false,
        s: false,
        u: false,
    };

    /// The extension bitmap `misa` reports, one bit per letter from `A`.
    #[must_use]
    pub const fn misa_bits(self) -> u64 {
        let mut bits = 1 << 8; // 'I'
        if self.m {
            bits |= 1 << 12;
        }
        if self.a {
            bits |= 1;
        }
        if self.f {
            bits |= 1 << 5;
        }
        if self.d {
            bits |= 1 << 3;
        }
        if self.c {
            bits |= 1 << 2;
        }
        if self.s {
            bits |= 1 << 18;
        }
        if self.u {
            bits |= 1 << 20;
        }
        bits
    }
}

/// The control and status registers of one hart.
///
/// Everything here is architectural state that a snapshot must carry; nothing
/// here is derived (the TLB, which is, lives in [`super::mmu`]).
#[derive(Debug, Clone)]
pub struct Csrs {
    /// The register width, which decides how wide every CSR reads.
    pub xlen: Xlen,
    /// Which extensions are present.
    pub ext: Extensions,
    /// The current privilege mode.
    pub priv_mode: Priv,

    /// `mstatus`, always in its RV64 layout.
    pub mstatus: u64,
    /// `medeleg`: which exceptions are delegated to S-mode.
    pub medeleg: u64,
    /// `mideleg`: which interrupts are delegated to S-mode.
    pub mideleg: u64,
    /// `mie`.
    pub mie: u64,
    /// `mtvec`.
    pub mtvec: u64,
    /// `mcounteren`.
    pub mcounteren: u64,
    /// `mcountinhibit`.
    pub mcountinhibit: u64,
    /// `mscratch`.
    pub mscratch: u64,
    /// `mepc`.
    pub mepc: u64,
    /// `mcause`.
    pub mcause: u64,
    /// `mtval`.
    pub mtval: u64,
    /// `menvcfg`.
    pub menvcfg: u64,

    /// `stvec`.
    pub stvec: u64,
    /// `scounteren`.
    pub scounteren: u64,
    /// `sscratch`.
    pub sscratch: u64,
    /// `sepc`.
    pub sepc: u64,
    /// `scause`.
    pub scause: u64,
    /// `stval`.
    pub stval: u64,
    /// `satp`.
    pub satp: u64,
    /// `senvcfg`.
    pub senvcfg: u64,

    /// `fcsr`: the rounding mode in bits 7:5 and the sticky flags in bits 4:0.
    pub fcsr: u64,

    /// The hart identity `mhartid` reports.
    pub hartid: u64,
    /// Retired-instruction count, which `minstret` and `instret` report.
    pub minstret: u64,
    /// Cycle count, which `mcycle` and `cycle` report.
    pub mcycle: u64,
    /// The platform timer `time` reports.
    pub mtime: u64,

    /// How many PMP entries are implemented, from 0 to [`PMP_ENTRIES`].
    ///
    /// Zero means PMP is not implemented at all, and Volume II is explicit
    /// that every access then passes. Any other count means an S-mode or
    /// U-mode access matching no entry is *refused*, so a machine whose
    /// firmware does not program PMP must be built with zero.
    pub pmp_count: usize,
    /// PMP configuration bytes, one per entry.
    pub pmpcfg: [u8; PMP_ENTRIES],
    /// PMP address registers.
    pub pmpaddr: [u64; PMP_ENTRIES],

    /// Bumped whenever a write invalidates address translation, so the
    /// software TLB can be tagged rather than flushed by hand at each site.
    pub translation_gen: u64,
}

impl Csrs {
    /// The reset state.
    ///
    /// Volume II is deliberately sparse about reset values: only `mstatus`'s
    /// `MIE`/`MPRV`, `misa`, `mcause` and the PMP `A` fields are architecturally
    /// required to be reset, and the hart starts in M-mode with `pc` at the
    /// implementation-defined reset vector. Everything else here is zeroed,
    /// which is the reproducible choice (`ROADMAP.md` §0).
    #[must_use]
    pub fn new(xlen: Xlen, ext: Extensions, hartid: u64, pmp_count: usize) -> Csrs {
        let mut mstatus = 0u64;
        if xlen == Xlen::Rv64 {
            // UXL and SXL report 64-bit and are read-only: this core does not
            // support running S-mode or U-mode in a narrower width than M.
            mstatus |= 2 << status::UXL_SHIFT;
            if ext.s {
                mstatus |= 2 << status::SXL_SHIFT;
            }
        }
        Csrs {
            xlen,
            ext,
            priv_mode: Priv::Machine,
            mstatus,
            medeleg: 0,
            mideleg: 0,
            mie: 0,
            mtvec: 0,
            mcounteren: 0,
            mcountinhibit: 0,
            mscratch: 0,
            mepc: 0,
            mcause: 0,
            mtval: 0,
            menvcfg: 0,
            stvec: 0,
            scounteren: 0,
            sscratch: 0,
            sepc: 0,
            scause: 0,
            stval: 0,
            satp: 0,
            senvcfg: 0,
            fcsr: 0,
            hartid,
            minstret: 0,
            mcycle: 0,
            mtime: 0,
            pmp_count: pmp_count.min(PMP_ENTRIES),
            pmpcfg: [0; PMP_ENTRIES],
            pmpaddr: [0; PMP_ENTRIES],
            translation_gen: 1,
        }
    }

    /// A mask of every bit a CSR of this width holds.
    #[inline]
    #[must_use]
    pub const fn xmask(&self) -> u64 {
        match self.xlen {
            Xlen::Rv32 => 0xffff_ffff,
            Xlen::Rv64 => u64::MAX,
        }
    }

    /// The `misa` value: the width in the top two bits, the extensions below.
    #[must_use]
    pub fn misa(&self) -> u64 {
        let mxl: u64 = match self.xlen {
            Xlen::Rv32 => 1,
            Xlen::Rv64 => 2,
        };
        (mxl << (self.xlen.bits() - 2)) | self.ext.misa_bits()
    }

    /// The current floating-point state field.
    #[inline]
    #[must_use]
    pub fn fs(&self) -> u64 {
        (self.mstatus & status::FS) >> status::FS_SHIFT
    }

    /// Whether floating-point instructions may execute at all.
    ///
    /// Volume II: with `mstatus.FS` off, every FP instruction — including a
    /// read of `fcsr` — raises an illegal-instruction exception. That is what
    /// lets an operating system defer saving the FP registers until a process
    /// actually uses them.
    #[inline]
    #[must_use]
    pub fn fp_enabled(&self) -> bool {
        self.ext.f && self.fs() != status::FS_OFF
    }

    /// Mark the floating-point state dirty, which every FP write must do.
    #[inline]
    pub fn dirty_fp(&mut self) {
        self.mstatus = (self.mstatus & !status::FS) | (status::FS_DIRTY << status::FS_SHIFT);
    }

    /// `mstatus` as read, with the derived `SD` summary bit added.
    #[must_use]
    pub fn read_mstatus(&self) -> u64 {
        let mut v = self.mstatus;
        if (v & status::FS) == status::FS || (v & status::XS) == status::XS {
            v |= 1 << (self.xlen.bits() - 1);
        }
        v & self.xmask()
    }

    /// Note that address translation may have changed.
    #[inline]
    pub fn bump_translation(&mut self) {
        self.translation_gen = self.translation_gen.wrapping_add(1);
    }

    /// Whether `num` names a register that exists and may be accessed from
    /// `priv_mode`.
    ///
    /// Volume II, "CSR Address Mapping Conventions": bits 9:8 of the address
    /// are the lowest privilege that may access it, and bits 11:10 being `11`
    /// marks it read-only.
    fn accessible(&self, num: u32, write: bool) -> bool {
        if write && (num >> 10) & 3 == 3 {
            return false;
        }
        let needed = (num >> 8) & 3;
        u64::from(needed) <= self.priv_mode.bits()
    }

    /// Whether a counter is readable from the current privilege.
    ///
    /// `mcounteren` gates S-mode and, together with `scounteren`, U-mode.
    fn counter_enabled(&self, bit: u32) -> bool {
        match self.priv_mode {
            Priv::Machine => true,
            Priv::Supervisor => self.mcounteren & (1 << bit) != 0,
            Priv::User => self.mcounteren & (1 << bit) != 0 && self.scounteren & (1 << bit) != 0,
        }
    }

    /// Read a CSR.
    ///
    /// `None` means the access raises an illegal-instruction exception:
    /// the register does not exist, the privilege is too low, or a counter is
    /// disabled by `mcounteren`.
    #[must_use]
    pub fn read(&self, num: u32, pending: u64) -> Option<u64> {
        if !self.accessible(num, false) {
            return None;
        }
        let mask = self.xmask();
        let v = match num {
            num::FFLAGS => {
                if !self.fp_enabled() {
                    return None;
                }
                self.fcsr & 0x1f
            }
            num::FRM => {
                if !self.fp_enabled() {
                    return None;
                }
                (self.fcsr >> 5) & 7
            }
            num::FCSR => {
                if !self.fp_enabled() {
                    return None;
                }
                self.fcsr & 0xff
            }

            num::SSTATUS => {
                if !self.ext.s {
                    return None;
                }
                let mut v = self.mstatus & status::S_VISIBLE;
                if self.xlen == Xlen::Rv64 {
                    v |= self.mstatus & (3 << status::UXL_SHIFT);
                }
                if (self.mstatus & status::FS) == status::FS {
                    v |= 1 << (self.xlen.bits() - 1);
                }
                v
            }
            num::SIE => self.s_view(self.mie)?,
            num::STVEC => self.s_reg(self.stvec)?,
            num::SCOUNTEREN => self.s_reg(self.scounteren)?,
            num::SENVCFG => self.s_reg(self.senvcfg)?,
            num::SSCRATCH => self.s_reg(self.sscratch)?,
            num::SEPC => self.s_reg(self.sepc)?,
            num::SCAUSE => self.s_reg(self.scause)?,
            num::STVAL => self.s_reg(self.stval)?,
            num::SIP => self.s_view(pending)?,
            num::SATP => {
                if !self.ext.s {
                    return None;
                }
                // TVM makes a supervisor read of satp trap, so a hypervisor
                // can intercept it.
                if self.priv_mode == Priv::Supervisor && self.mstatus & status::TVM != 0 {
                    return None;
                }
                self.satp
            }

            num::MSTATUS => self.read_mstatus(),
            num::MSTATUSH => {
                if self.xlen != Xlen::Rv32 {
                    return None;
                }
                self.mstatus >> 32
            }
            num::MISA => self.misa(),
            num::MEDELEG => self.medeleg,
            num::MIDELEG => self.mideleg,
            num::MIE => self.mie,
            num::MTVEC => self.mtvec,
            num::MCOUNTEREN => self.mcounteren,
            num::MCOUNTINHIBIT => self.mcountinhibit,
            num::MENVCFG => self.menvcfg,
            num::MENVCFGH => {
                if self.xlen != Xlen::Rv32 {
                    return None;
                }
                self.menvcfg >> 32
            }
            num::MSCRATCH => self.mscratch,
            num::MEPC => self.mepc,
            num::MCAUSE => self.mcause,
            num::MTVAL => self.mtval,
            num::MIP => pending,

            num::MCYCLE | num::CYCLE => {
                if num == num::CYCLE && !self.counter_enabled(0) {
                    return None;
                }
                self.mcycle
            }
            num::TIME => {
                if !self.counter_enabled(1) {
                    return None;
                }
                self.mtime
            }
            num::MINSTRET | num::INSTRET => {
                if num == num::INSTRET && !self.counter_enabled(2) {
                    return None;
                }
                self.minstret
            }
            num::MCYCLEH | num::CYCLEH => {
                if self.xlen != Xlen::Rv32 {
                    return None;
                }
                if num == num::CYCLEH && !self.counter_enabled(0) {
                    return None;
                }
                self.mcycle >> 32
            }
            num::TIMEH => {
                if self.xlen != Xlen::Rv32 || !self.counter_enabled(1) {
                    return None;
                }
                self.mtime >> 32
            }
            num::MINSTRETH | num::INSTRETH => {
                if self.xlen != Xlen::Rv32 {
                    return None;
                }
                if num == num::INSTRETH && !self.counter_enabled(2) {
                    return None;
                }
                self.minstret >> 32
            }

            // The debug trigger module, reporting that this hart implements
            // no triggers. Volume II leaves the module optional, and the debug
            // specification defines a zero `tdata1` type field as exactly
            // that — so these read as zero rather than raising illegal
            // instruction, which lets debug-aware software discover the answer
            // instead of trapping on the question.
            num::TSELECT | num::TDATA1 | num::TDATA2 | num::TDATA3 | num::TCONTROL => 0,
            // `tinfo` bit 0 says trigger type 0 — "none" — is what is
            // supported.
            num::TINFO => 1,

            num::MVENDORID | num::MARCHID | num::MIMPID => 0,
            num::MHARTID => self.hartid,

            num::PMPCFG0..=num::PMPCFG15 => self.read_pmpcfg(num - num::PMPCFG0)?,
            num::PMPADDR0..=num::PMPADDR63 => {
                let i = (num - num::PMPADDR0) as usize;
                if i >= self.pmp_count {
                    0
                } else {
                    self.pmpaddr[i]
                }
            }

            // The hardware performance monitors are architecturally required
            // to exist and are allowed to be hard-wired to zero. Linux reads
            // them; refusing would be a spurious trap.
            0xb03..=0xb1f | 0xb83..=0xb9f | 0x323..=0x33f | 0xc03..=0xc1f | 0xc83..=0xc9f => 0,

            _ => return None,
        };
        Some(v & mask)
    }

    /// A supervisor register, or `None` when S-mode is not implemented.
    fn s_reg(&self, value: u64) -> Option<u64> {
        if self.ext.s { Some(value) } else { None }
    }

    /// `sie`/`sip`: the machine register filtered through `mideleg`.
    fn s_view(&self, value: u64) -> Option<u64> {
        if self.ext.s {
            Some(value & self.mideleg & irq::S_MASK)
        } else {
            None
        }
    }

    /// Assemble a `pmpcfg` register from the per-entry bytes.
    ///
    /// On RV64 only the even-numbered registers exist and each holds eight
    /// entries; on RV32 every register exists and each holds four.
    fn read_pmpcfg(&self, index: u32) -> Option<u64> {
        let (per, first) = match self.xlen {
            Xlen::Rv32 => (4usize, index as usize * 4),
            Xlen::Rv64 => {
                if index & 1 != 0 {
                    return None;
                }
                (8usize, index as usize / 2 * 8)
            }
        };
        let mut v = 0u64;
        for i in 0..per {
            let entry = first + i;
            if entry < self.pmp_count {
                v |= u64::from(self.pmpcfg[entry]) << (8 * i);
            }
        }
        Some(v)
    }

    /// Write the per-entry bytes of a `pmpcfg` register.
    ///
    /// A locked entry (`L` set) ignores writes until the hart is reset, which
    /// is the whole point of the bit.
    fn write_pmpcfg(&mut self, index: u32, value: u64) -> Option<()> {
        let (per, first) = match self.xlen {
            Xlen::Rv32 => (4usize, index as usize * 4),
            Xlen::Rv64 => {
                if index & 1 != 0 {
                    return None;
                }
                (8usize, index as usize / 2 * 8)
            }
        };
        for i in 0..per {
            let entry = first + i;
            if entry >= self.pmp_count {
                break;
            }
            if self.pmpcfg[entry] & 0x80 != 0 {
                continue;
            }
            let mut byte = ((value >> (8 * i)) & 0xff) as u8;
            // Bits 6:5 are reserved and read as zero.
            byte &= 0x9f;
            // W without R is reserved; the specification says such an encoding
            // must not be produced, so it is cleared rather than honoured.
            if byte & 0b11 == 0b10 {
                byte &= !0b10;
            }
            self.pmpcfg[entry] = byte;
        }
        Some(())
    }

    /// Write a CSR.
    ///
    /// `None` means the access raises an illegal-instruction exception.
    /// `pending` is the current `mip`; a `Some(new)` return value replaces it,
    /// which is how a write to `mip` or `sip` reaches [`Lines`].
    #[must_use]
    pub fn write(&mut self, num: u32, value: u64, pending: u64) -> Option<Option<u64>> {
        if !self.accessible(num, true) {
            return None;
        }
        let value = value & self.xmask();
        let mut new_pending = None;
        match num {
            num::FFLAGS => {
                if !self.fp_enabled() {
                    return None;
                }
                self.fcsr = (self.fcsr & !0x1f) | (value & 0x1f);
                self.dirty_fp();
            }
            num::FRM => {
                if !self.fp_enabled() {
                    return None;
                }
                self.fcsr = (self.fcsr & !0xe0) | ((value & 7) << 5);
                self.dirty_fp();
            }
            num::FCSR => {
                if !self.fp_enabled() {
                    return None;
                }
                self.fcsr = value & 0xff;
                self.dirty_fp();
            }

            num::SSTATUS => {
                if !self.ext.s {
                    return None;
                }
                self.mstatus = (self.mstatus & !status::S_WRITABLE) | (value & status::S_WRITABLE);
                self.bump_translation();
            }
            num::SIE => {
                if !self.ext.s {
                    return None;
                }
                let m = self.mideleg & irq::S_MASK;
                self.mie = (self.mie & !m) | (value & m);
            }
            num::STVEC => {
                if !self.ext.s {
                    return None;
                }
                self.stvec = value & !2;
            }
            num::SCOUNTEREN => {
                if !self.ext.s {
                    return None;
                }
                self.scounteren = value & 0xffff_ffff;
            }
            num::SENVCFG => {
                if !self.ext.s {
                    return None;
                }
                self.senvcfg = value & 1;
            }
            num::SSCRATCH => {
                if !self.ext.s {
                    return None;
                }
                self.sscratch = value;
            }
            num::SEPC => {
                if !self.ext.s {
                    return None;
                }
                self.sepc = self.align_epc(value);
            }
            num::SCAUSE => {
                if !self.ext.s {
                    return None;
                }
                self.scause = value;
            }
            num::STVAL => {
                if !self.ext.s {
                    return None;
                }
                self.stval = value;
            }
            num::SIP => {
                if !self.ext.s {
                    return None;
                }
                // A supervisor may only clear its own software interrupt.
                let m = self.mideleg & irq::SSI;
                new_pending = Some((pending & !m) | (value & m));
            }
            num::SATP => {
                if !self.ext.s {
                    return None;
                }
                if self.priv_mode == Priv::Supervisor && self.mstatus & status::TVM != 0 {
                    return None;
                }
                // MODE is WARL: an unsupported mode leaves satp unchanged
                // rather than half-applying, which is what stops a guest
                // enabling Sv48 on a core that only walks Sv39.
                if self.satp_mode_supported(value) {
                    self.satp = value;
                    self.bump_translation();
                }
            }

            num::MSTATUS => {
                self.mstatus = (self.mstatus & !status::M_WRITABLE) | (value & status::M_WRITABLE);
                // A reserved MPP encoding is WARL; the specification lets an
                // implementation pick, and machine mode is the safe pick.
                if Priv::from_bits((self.mstatus & status::MPP) >> status::MPP_SHIFT).is_none() {
                    self.mstatus |= status::MPP;
                }
                if !self.ext.s {
                    self.mstatus &= !(status::SPP | status::SIE | status::SPIE | status::TVM);
                }
                self.bump_translation();
            }
            num::MSTATUSH => {
                if self.xlen != Xlen::Rv32 {
                    return None;
                }
                // Only SBE and MBE live here, and this core is little-endian
                // throughout, so both are read-only zero.
            }
            // misa is WARL and this core cannot turn an extension off at
            // runtime, so a write is legal and does nothing.
            num::MISA => {}
            num::MEDELEG => {
                if !self.ext.s {
                    return None;
                }
                // An M-mode ECALL can never be delegated: there is nothing
                // above M to delegate from.
                self.medeleg = value & !(1 << cause::ECALL_M);
            }
            num::MIDELEG => {
                if !self.ext.s {
                    return None;
                }
                self.mideleg = value & irq::S_MASK;
            }
            num::MIE => self.mie = value & irq::ALL,
            num::MTVEC => self.mtvec = value & !2,
            num::MCOUNTEREN => self.mcounteren = value & 0xffff_ffff,
            num::MCOUNTINHIBIT => self.mcountinhibit = value & 0xffff_fffd,
            num::MENVCFG => self.menvcfg = value,
            num::MENVCFGH => {
                if self.xlen != Xlen::Rv32 {
                    return None;
                }
            }
            num::MSCRATCH => self.mscratch = value,
            num::MEPC => self.mepc = self.align_epc(value),
            num::MCAUSE => self.mcause = value,
            num::MTVAL => self.mtval = value,
            num::MIP => {
                // MSIP, MTIP and MEIP are driven by the platform, not by
                // software; only the supervisor bits are writable here.
                let m = irq::S_MASK;
                new_pending = Some((pending & !m) | (value & m));
            }

            num::MCYCLE => self.mcycle = value,
            num::MINSTRET => self.minstret = value,
            num::MCYCLEH => {
                if self.xlen != Xlen::Rv32 {
                    return None;
                }
                self.mcycle = (self.mcycle & 0xffff_ffff) | (value << 32);
            }
            num::MINSTRETH => {
                if self.xlen != Xlen::Rv32 {
                    return None;
                }
                self.minstret = (self.minstret & 0xffff_ffff) | (value << 32);
            }

            num::PMPCFG0..=num::PMPCFG15 => {
                self.write_pmpcfg(num - num::PMPCFG0, value)?;
                self.bump_translation();
            }
            num::PMPADDR0..=num::PMPADDR63 => {
                let i = (num - num::PMPADDR0) as usize;
                if i < self.pmp_count {
                    // A locked entry is immutable, and so is the entry below a
                    // locked TOR entry, whose base address it supplies.
                    let locked = self.pmpcfg[i] & 0x80 != 0;
                    let next_tor_locked = i + 1 < self.pmp_count
                        && self.pmpcfg[i + 1] & 0x80 != 0
                        && (self.pmpcfg[i + 1] >> 3) & 3 == 1;
                    if !locked && !next_tor_locked {
                        self.pmpaddr[i] = value & 0x003f_ffff_ffff_ffff;
                        self.bump_translation();
                    }
                }
            }

            // No triggers exist, so there is nothing for a write to select or
            // configure.
            num::TSELECT | num::TDATA1 | num::TDATA2 | num::TDATA3 | num::TCONTROL => {}

            0xb03..=0xb1f | 0xb83..=0xb9f | 0x323..=0x33f => {}

            _ => return None,
        }
        Some(new_pending)
    }

    /// Whether the `MODE` field of a proposed `satp` names a scheme this core
    /// walks.
    fn satp_mode_supported(&self, value: u64) -> bool {
        match self.xlen {
            // RV32 has exactly one scheme, Sv32, plus Bare.
            Xlen::Rv32 => true,
            Xlen::Rv64 => matches!(value >> 60, 0 | 8),
        }
    }

    /// Force `mepc`/`sepc` to a legal instruction address.
    ///
    /// Volume II: the low bit is always zero, and the second-lowest is too
    /// unless `C` is implemented — writing an unaligned value must not be able
    /// to make a return jump to one.
    fn align_epc(&self, value: u64) -> u64 {
        if self.ext.c { value & !1 } else { value & !3 }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn csrs() -> Csrs {
        Csrs::new(Xlen::Rv64, Extensions::GC, 0, PMP_ENTRIES)
    }

    #[test]
    fn reset_starts_in_machine_mode_with_a_64_bit_misa() {
        let c = csrs();
        assert_eq!(c.priv_mode, Priv::Machine);
        assert_eq!(c.misa() >> 62, 2, "MXL must say 64-bit");
        // "imafdcsu"
        for letter in ['i', 'm', 'a', 'f', 'd', 'c', 's', 'u'] {
            let bit = 1u64 << (letter as u32 - 'a' as u32);
            assert_ne!(c.misa() & bit, 0, "misa is missing {letter}");
        }
    }

    #[test]
    fn read_only_csrs_reject_writes() {
        let mut c = csrs();
        assert!(c.read(num::MHARTID, 0).is_some());
        assert!(c.write(num::MHARTID, 1, 0).is_none());
        assert!(c.read(num::CYCLE, 0).is_some());
        assert!(c.write(num::CYCLE, 1, 0).is_none());
    }

    #[test]
    fn privilege_gates_the_machine_registers() {
        let mut c = csrs();
        c.priv_mode = Priv::Supervisor;
        assert!(c.read(num::MSTATUS, 0).is_none());
        assert!(c.write(num::MSTATUS, 0, 0).is_none());
        assert!(c.read(num::SSTATUS, 0).is_some());
        c.priv_mode = Priv::User;
        assert!(c.read(num::SSTATUS, 0).is_none());
    }

    #[test]
    fn sstatus_is_a_masked_view_of_mstatus() {
        let mut c = csrs();
        c.write(num::MSTATUS, status::MIE | status::SIE | status::MPP, 0)
            .unwrap();
        let s = c.read(num::SSTATUS, 0).unwrap();
        assert_ne!(s & status::SIE, 0);
        assert_eq!(s & status::MIE, 0, "MIE is not visible in sstatus");
        assert_eq!(s & status::MPP, 0, "MPP is not visible in sstatus");
        // Writing sstatus must not disturb the machine-only fields.
        c.write(num::SSTATUS, 0, 0).unwrap();
        assert_ne!(c.mstatus & status::MIE, 0);
        assert_eq!(c.mstatus & status::SIE, 0);
    }

    #[test]
    fn sie_and_sip_are_filtered_through_mideleg() {
        let mut c = csrs();
        c.write(num::MIDELEG, irq::SSI | irq::STI, 0).unwrap();
        c.write(num::MIE, irq::ALL, 0).unwrap();
        let sie = c.read(num::SIE, 0).unwrap();
        assert_eq!(sie, irq::SSI | irq::STI);
        // A supervisor write can only reach the delegated bits.
        c.priv_mode = Priv::Supervisor;
        c.write(num::SIE, 0, 0).unwrap();
        assert_eq!(c.mie & (irq::SSI | irq::STI), 0);
        assert_ne!(c.mie & irq::MEI, 0, "the machine bits survive");
    }

    #[test]
    fn mip_writes_reach_the_lines_not_the_csr_file() {
        let mut c = csrs();
        let out = c.write(num::MIP, irq::SSI | irq::MEI, 0).unwrap();
        // Only the supervisor bits are writable; MEIP belongs to the platform.
        assert_eq!(out, Some(irq::SSI));
    }

    #[test]
    fn medeleg_cannot_delegate_a_machine_ecall() {
        let mut c = csrs();
        c.write(num::MEDELEG, u64::MAX, 0).unwrap();
        assert_eq!(c.medeleg & (1 << cause::ECALL_M), 0);
    }

    #[test]
    fn satp_rejects_a_mode_this_core_cannot_walk() {
        let mut c = csrs();
        // Sv48 is mode 9, which this core does not implement.
        c.write(num::SATP, 9 << 60, 0).unwrap();
        assert_eq!(c.satp, 0, "an unsupported mode leaves satp unchanged");
        c.write(num::SATP, 8 << 60 | 0x1234, 0).unwrap();
        assert_eq!(c.satp >> 60, 8);
    }

    #[test]
    fn epc_is_forced_to_an_aligned_address() {
        let mut c = csrs();
        c.write(num::MEPC, 0x1003, 0).unwrap();
        assert_eq!(c.mepc, 0x1002, "C is present, so bit 1 survives");
        let mut c = Csrs::new(Xlen::Rv64, Extensions::I, 0, PMP_ENTRIES);
        c.write(num::MEPC, 0x1003, 0).unwrap();
        assert_eq!(c.mepc, 0x1000, "without C the low two bits are zero");
    }

    #[test]
    fn floating_point_csrs_need_fs_on() {
        let mut c = csrs();
        assert_eq!(c.fs(), status::FS_OFF);
        assert!(c.read(num::FCSR, 0).is_none());
        c.write(num::MSTATUS, 1 << status::FS_SHIFT, 0).unwrap();
        assert!(c.read(num::FCSR, 0).is_some());
        c.write(num::FRM, 3, 0).unwrap();
        assert_eq!(c.read(num::FRM, 0).unwrap(), 3);
        assert_eq!(c.read(num::FCSR, 0).unwrap(), 3 << 5);
        assert_eq!(c.fs(), status::FS_DIRTY, "an FP write dirties the state");
        // SD summarises FS being dirty.
        assert_ne!(c.read_mstatus() >> 63, 0);
    }

    #[test]
    fn counters_are_gated_by_mcounteren() {
        let mut c = csrs();
        c.priv_mode = Priv::Supervisor;
        assert!(c.read(num::CYCLE, 0).is_none());
        c.priv_mode = Priv::Machine;
        c.write(num::MCOUNTEREN, 1, 0).unwrap();
        c.priv_mode = Priv::Supervisor;
        assert!(c.read(num::CYCLE, 0).is_some());
        assert!(c.read(num::TIME, 0).is_none());
    }

    #[test]
    fn pmp_configuration_packs_eight_entries_per_register_on_rv64() {
        let mut c = csrs();
        c.write(num::PMPCFG0, 0x0f0f_0f0f_0f0f_0f0f, 0).unwrap();
        assert_eq!(c.pmpcfg[0], 0x0f);
        assert_eq!(c.pmpcfg[7], 0x0f);
        assert_eq!(c.read(num::PMPCFG0, 0).unwrap(), 0x0f0f_0f0f_0f0f_0f0f);
        // The odd-numbered registers do not exist on RV64.
        assert!(c.read(num::PMPCFG0 + 1, 0).is_none());
        // A locked entry ignores further writes.
        c.write(num::PMPCFG0 + 2, 0x8f, 0).unwrap();
        assert_eq!(c.pmpcfg[8], 0x8f);
        c.write(num::PMPCFG0 + 2, 0x00, 0).unwrap();
        assert_eq!(c.pmpcfg[8], 0x8f);
    }

    #[test]
    fn every_named_csr_has_a_name() {
        // The one guard against `num` and `csr_name` drifting apart.
        for (number, expected) in [
            (num::MSTATUS, "mstatus"),
            (num::SATP, "satp"),
            (num::MHARTID, "mhartid"),
            (num::FCSR, "fcsr"),
            (num::PMPADDR0, "pmpaddr0"),
        ] {
            assert_eq!(csr_name(number), Some(expected));
        }
        assert_eq!(csr_name(0x7ff), None);
        // Every register the access path knows must also have a name, or the
        // monitor prints a bare number for something this core implements.
        let c = csrs();
        for number in 0u32..0x1000 {
            // The counters and the performance monitors are deliberately
            // nameless past the first few, and the PMP banks past entry zero.
            let banked = (num::PMPCFG0..=num::PMPADDR63).contains(&number)
                || (0xb03..=0xc9f).contains(&number)
                || (0x323..=0x33f).contains(&number);
            if c.read(number, 0).is_some() && !banked {
                assert!(csr_name(number).is_some(), "csr {number:#x} has no name");
            }
        }
    }

    #[test]
    fn unknown_csrs_are_illegal() {
        let c = csrs();
        assert!(c.read(0x7ff, 0).is_none());
        assert!(c.read(0x200, 0).is_none());
    }

    #[test]
    fn interrupt_lines_are_atomic_and_outside_the_lock() {
        let lines = Lines::default();
        lines.set_pending(irq::MTI, true);
        assert_eq!(lines.pending(), irq::MTI);
        lines.set_pending(irq::MEI, true);
        assert_eq!(lines.pending(), irq::MTI | irq::MEI);
        lines.set_pending(irq::MTI, false);
        assert_eq!(lines.pending(), irq::MEI);
        assert!(!lines.take_reset_request());
        lines.request_reset();
        assert!(lines.take_reset_request());
        assert!(!lines.take_reset_request());
    }
}
