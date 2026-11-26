//! ARMv6-M: what the Cortex-M0, Cortex-M0+ and SecurCore SC000 take away
//! from — and, in two places, change in — the ARMv7-M core this module sits
//! inside.
//!
//! Selected per instance by [`Config::CORTEX_M0`](super::Config::CORTEX_M0),
//! [`Config::CORTEX_M0PLUS`](super::Config::CORTEX_M0PLUS) or
//! [`Config::SC000`](super::Config::SC000) (`part = "cortex-m0"`,
//! `"cortex-m0plus"` / `"cortex-m0+"`, `"sc000"`), and compiled only with the
//! `cpu-arm-v6m` feature, so a v7-M-only build links none of it. The same
//! interpreter runs every M-profile architecture; what differs is keyed off
//! one flag, [`Config::is_v6m`](super::Config::is_v6m), never off part
//! names.
//!
//! # Why the v7-M core and not a core of its own
//!
//! ARMv6-M keeps every row of the table in `super::super`'s module docs that
//! made the M profile a separate core: Thumb only, one register bank and two
//! stack pointers, `xPSR`, a memory-mapped SCB, a relocatable NVIC table. It
//! is a strict subset of ARMv7-M's programmers' model with the instruction
//! set cut down, which is exactly the shape a construction-time `Arch` exists
//! for (`ROADMAP.md` §6.1.1).
//!
//! # The instruction set
//!
//! [`decode`] is the gate, applied per encoding in front of the shared
//! decoder (DDI 0419E A5.2, A5.3): every sixteen-bit encoding ARMv7-M has
//! *except* `CBZ`/`CBNZ` and `IT`, and of the thirty-two-bit space only `BL`,
//! `MSR`, `MRS`, `DMB`, `DSB`, `ISB` and the permanently undefined `UDF.W`.
//! Everything else — `B.W`, `CLREX`, `NOP.W`, `LDREX`, `MOVW`, `SDIV`, every
//! data-processing, load/store, DSP and coprocessor encoding — is UNDEFINED
//! and so a HardFault, which is what the silicon does and how firmware
//! probes. "Thumb instructions present in other architecture variants are
//! UNDEFINED in ARMv6-M" (A5.1.1).
//!
//! The disassembler uses the same gate ([`decode_for`](super::isa::decode_for)),
//! so a Cortex-M0 listing shows `CBZ` as the undefined encoding it is there.
//!
//! # The system model
//!
//! | | ARMv6-M, as modelled |
//! | --- | --- |
//! | Faults | HardFault only (B1.5.14). The interpreter still raises its usual UsageFault / BusFault / MemManage internally, and because their enables in `SHCSR` cannot be set here every one escalates to HardFault — the outcome B1.5.14's table gives for each cause. The `CFSR`/`HFSR` bits that path records are kept for a debugger ([`Sys`] through `with_sys`) but read as zero from the guest: v6-M has no fault status registers (B3.2.11) |
//! | Lockup | an `SVC` or `BKPT` at priority −1 or −2 locks up (B1.5.15, Table B1-7); an NMI ends a lockup that is not inside NMI |
//! | Lost frames | a fault on the stacking writes pends a HardFault, or locks up from priority −1 (B1.5.11); one on the unstacking reads locks up from the main stack and pends a HardFault from the process stack (B1.5.8) |
//! | Priority | two bits (`[7:6]`), no `PRIGROUP`, no `BASEPRI`, no `FAULTMASK` (B1.5.4, B4.1.1) |
//! | `MRS`/`MSR` | `APSR`/`IPSR`/`EPSR` forms, `MSP`, `PSP`, `PRIMASK`, `CONTROL`; anything else reads zero and ignores writes. `MSR APSR` writes `N`, `Z`, `C`, `V` only (B4.2.2, B4.2.3) |
//! | `CONTROL` | `SPSEL`, and `nPRIV` only with the Unprivileged/Privileged Extension (B1.4.4) |
//! | `CPS` | `PRIMASK` only — the `F` form has no `FAULTMASK` to reach (B4.2.1) |
//! | Exception return | the three integer `EXC_RETURN` values; anything else is a HardFault. The stacked `xPSR` restores `N`, `Z`, `C`, `V`, `T` and, on a return to Handler mode, `IPSR[5:0]`, and nothing else; a return to Thread mode sets `IPSR` to zero whatever the frame says (B1.5.8 `PopStack`) |
//! | Alignment | `CCR` is read-only with `UNALIGN_TRP` and `STKALIGN` reading one: every unaligned halfword or word access faults, every frame is 8-byte aligned (B3.2.8) |
//! | NVIC | one word each of `ISER`/`ICER`/`ISPR`/`ICPR` — 32 interrupts — and `IPR0`–`IPR7`; no `IABR`, no `STIR`, no `ICTR` (B3.4.2) |
//! | SCB | `CPUID`, `ICSR` (no `RETTOBASE`; `ISRPENDING` is debug-only and reads zero), `VTOR` if the part has one, `AIRCR` (no `PRIGROUP`), `SCR`, `CCR`, `SHPR2` (SVCall) and `SHPR3` (PendSV, SysTick), `SHCSR.SVCALLPENDED` (B3.2.2) |
//! | MPU | PMSAv6 where the part has one: `TYPE`, `CTRL`, `RNR`, `RBAR` (`ADDR` is bits `[31:8]`), `RASR`; no alias registers (B3.5.2) |
//! | PPB | unprivileged access is a HardFault and the access is not made (B3.1.1) |
//! | Debug | no `DEMCR`, no DWT cycle counter — ARMv6-M's DWT has none; both read as zero |
//!
//! Everything else in the private peripheral bus reads as zero and ignores
//! writes, which is what "reserved, UNK/SBZP" is allowed to be.
//!
//! # Timing
//!
//! The shared model (`exec`'s module docs) already gives the Cortex-M0's
//! figures for data processing, single loads and stores, `B`, `MOV PC`,
//! `LDM`/`STM`/`PUSH`/`POP`. On top of it a v6-M part charges what the
//! Cortex-M0 TRM's Table 3-1 adds: `BL` is 4 rather than 3, `POP {.., PC}` is
//! 4 + *N*, `MRS`, `MSR`, `DMB`, `DSB`, `ISB` are 4, and `WFE`, `WFI` are 2
//! (not counting the sleep). The Cortex-M0+'s two-stage pipeline (a taken
//! branch is 2, Cortex-M0+ TRM Table 3-1) is **not** modelled separately: an
//! M0+ runs the M0's figures. `MULS` is charged as the fast single-cycle
//! multiplier.
//!
//! # Not modelled, stated plainly
//!
//! - SysTick is always present; the architecture and both TRMs make it an
//!   implementation option.
//! - The `irq` port bank still offers 240 lines; a line above 31 can be
//!   wired but never taken, because its enable bit does not exist.
//! - The SC000's anti-tampering features and its IMPLEMENTATION DEFINED
//!   `SFCR` at `0xE000EF90` (named by CMSIS) read as zero: their description
//!   is under a SecurCore NDA.
//! - The PPB's "word accesses only" rule (B3.1.1) is not enforced: byte and
//!   halfword accesses are UNPREDICTABLE there, and this core keeps doing the
//!   read-modify-write the v7-M map does.
//!
//! # Sources
//!
//! *ARMv6-M Architecture Reference Manual*, ARM DDI 0419E: A5 (the encoding
//! tables), B1.4 (registers), B1.5 (the exception model), B3.1–B3.5 (the
//! address map, SCS, SysTick, NVIC and PMSAv6), B4 (the system instructions).
//! The *Cortex-M0 Technical Reference Manual* (DDI 0432C) and the
//! *Cortex-M0+ Technical Reference Manual* (DDI 0484C) for `CPUID`, the
//! configuration options and the instruction timings. The SC000 Technical
//! Reference Manual (DDI 0456) is not public; see
//! [`Config::SC000`](super::Config::SC000) for what that preset rests on. No
//! emulator source of any licence was consulted (`ROADMAP.md` §1).

use super::isa::{self, Insn};
use super::sys::{Exception, Sys, ccr, in_ppb};

/// `CPUID` for a Cortex-M0 r0p0: implementer `0x41`, variant 0,
/// architecture `0xC` (ARMv6-M), part number `0xC20`, revision 0
/// (Cortex-M0 TRM, Table 4-1).
pub const CPUID_CORTEX_M0: u32 = 0x410c_c200;

/// `CPUID` for a Cortex-M0+ r0p1: part number `0xC60`, revision 1
/// (Cortex-M0+ TRM, Table 4-2).
pub const CPUID_CORTEX_M0PLUS: u32 = 0x410c_c601;

/// `CPUID` for an SC000 r0p0: implementer `0x41`, variant 0, architecture
/// `0xC` (ARMv6-M), part number `0xC30`, revision 0.
///
/// The SC000 TRM is under NDA, so this is not from a document: it is the
/// value the project owner read from `0xE000ED00` on SC000 silicon. Another
/// revision sets its own with the `cpuid` property.
pub const CPUID_SC000: u32 = 0x410c_c300;

// The registers the v6-M map treats differently from the v7-M one, by the
// names DDI 0419E gives them.
const SYST_CSR: u32 = 0xe000_e010;
const SYST_CALIB: u32 = 0xe000_e01c;
const NVIC_ISER: u32 = 0xe000_e100;
const NVIC_ICER: u32 = 0xe000_e180;
const NVIC_ISPR: u32 = 0xe000_e200;
const NVIC_ICPR: u32 = 0xe000_e280;
const NVIC_IPR0: u32 = 0xe000_e400;
const NVIC_IPR7: u32 = 0xe000_e41c;
const CPUID: u32 = 0xe000_ed00;
const ICSR: u32 = 0xe000_ed04;
const VTOR: u32 = 0xe000_ed08;
const AIRCR: u32 = 0xe000_ed0c;
const SCR: u32 = 0xe000_ed10;
const CCR: u32 = 0xe000_ed14;
const SHPR2: u32 = 0xe000_ed1c;
const SHPR3: u32 = 0xe000_ed20;
const SHCSR: u32 = 0xe000_ed24;
const MPU_TYPE: u32 = 0xe000_ed90;
const MPU_RBAR: u32 = 0xe000_ed9c;
const MPU_RASR: u32 = 0xe000_eda0;

/// `ICSR.RETTOBASE` and `ICSR.ISRPENDING`: the first is ARMv7-M only, the
/// second exists only with the Debug Extension (DDI 0419E Table B3-6).
const ICSR_V7_ONLY: u32 = (1 << 11) | (1 << 22);

/// `SHCSR.SVCALLPENDED`, the one bit of the register CMSIS names for an
/// ARMv6-M part.
const SHCSR_SVCALLPENDED: u32 = 1 << 15;

/// `SCR`'s bits: `SEVONPEND`, `SLEEPDEEP`, `SLEEPONEXIT` (DDI 0419E B3.2.7).
const SCR_WRITABLE: u32 = (1 << 4) | (1 << 2) | (1 << 1);

/// Decode one instruction as an ARMv6-M part sees it.
///
/// The shared decoder does the decoding; this decides which encodings exist
/// at all (DDI 0419E A5.2 and A5.3). An encoding outside the v6-M set is
/// [`Insn::Undefined`], whatever ARMv7-M would have made of it.
#[must_use]
pub fn decode(first: u16, second: u16) -> Insn {
    if isa::is_32bit(first) {
        return if permits_32(first, second) {
            isa::decode(first, second)
        } else {
            Insn::Undefined
        };
    }
    match isa::decode_16(first) {
        // "The If-Then (IT) instruction is not supported in ARMv6-M. The
        // encoding space is UNDEFINED" (Table A5-7); `CBZ`/`CBNZ` are not in
        // Table A5-6, whose other encodings "are UNDEFINED".
        Insn::Cbz { .. } | Insn::It { .. } => Insn::Undefined,
        // Armv8-M's `BXNS`/`BLXNS` encodings are a `BX`/`BLX` with
        // should-be-zero bits set here, which is how they execute on any
        // non-v8-M part; list them as what they are on this one.
        #[cfg(feature = "cpu-arm-v8m")]
        Insn::Bxns { link, rm } => {
            if link {
                Insn::Blx { rm }
            } else {
                Insn::Bx { rm }
            }
        }
        insn => insn,
    }
}

/// Whether a thirty-two-bit encoding exists on ARMv6-M (DDI 0419E Tables
/// A5-9, A5-10 and A5-11).
///
/// Only the "branch and miscellaneous control" class is allocated
/// (`hw1[15:11] == 11110`, `hw2[15] == 1`), and inside it: `BL`
/// (`op2 == 1x1`), `MSR` (`op1 == 011100x`), `MRS` (`011111x`), the three
/// barriers under `0111011` (`hw2[7:4]` of `0100`, `0101`, `0110`), and the
/// permanently undefined `UDF.W` (`op1 == 1111111`, `op2 == 010`).
#[must_use]
pub const fn permits_32(first: u16, second: u16) -> bool {
    let a = first as u32;
    let b = second as u32;
    if (a >> 11) & 0x1f != 0b11110 || b & (1 << 15) == 0 {
        return false;
    }
    let op1 = (a >> 4) & 0x7f;
    let op2 = (b >> 12) & 0b111;
    if op2 & 0b101 == 0b101 {
        return true;
    }
    // `op2 == 0x0`: bit 13 is not part of the decode for `MSR`, `MRS` and
    // the barriers, and `UDF.W` is the `010` half of the same space.
    if op2 & 0b101 == 0b000 {
        return match op1 {
            0b011_1000 | 0b011_1001 | 0b011_1110 | 0b011_1111 => true,
            0b011_1011 => matches!((b >> 4) & 0xf, 0b0100..=0b0110),
            0b111_1111 => op2 == 0b010,
            _ => false,
        };
    }
    false
}

impl Sys {
    /// The same block on an ARMv6-M part: the v6-M register map, two
    /// priority bits, and a read-only `CCR` reading `0x00000208` —
    /// `STKALIGN` and `UNALIGN_TRP` both one (DDI 0419E B3.2.8).
    ///
    /// [`Sys::vtor_mask`] is the caller's to set: it is the part's, not the
    /// architecture's.
    #[must_use]
    pub fn with_v6m(mut self) -> Sys {
        self.v6m = true;
        self.priority_bits = 2;
        self.ccr = ccr::STKALIGN | ccr::UNALIGN_TRP;
        self
    }

    /// The v6-M answer for a register read, or `None` to let the shared map
    /// answer for a register the two architectures define alike.
    pub(super) fn read_word_v6m(&self, word: u32) -> Option<u32> {
        Some(match word {
            SYST_CSR..=SYST_CALIB
            | NVIC_ISER
            | NVIC_ICER
            | NVIC_ISPR
            | NVIC_ICPR
            | NVIC_IPR0..=NVIC_IPR7
            | CPUID
            | VTOR
            | AIRCR
            | SCR
            | CCR
            | MPU_TYPE..=MPU_RASR => return None,
            ICSR => self.icsr() & !ICSR_V7_ONLY,
            SHPR2 => u32::from(self.priority[usize::from(Exception::SVCALL.0)]) << 24,
            SHPR3 => {
                (u32::from(self.priority[usize::from(Exception::PEND_SV.0)]) << 16)
                    | (u32::from(self.priority[usize::from(Exception::SYSTICK.0)]) << 24)
            }
            SHCSR => {
                if self.is_pending(Exception::SVCALL) {
                    SHCSR_SVCALLPENDED
                } else {
                    0
                }
            }
            // Everything else is reserved on ARMv6-M: the fault status and
            // address registers, `SHPR1`, `CPACR`, the FP block, `STIR`,
            // `IABR`, the NVIC words past the first, `IPR8` up, the MPU
            // aliases, `ACTLR`, `ICTR`, `DEMCR` and the DWT.
            _ if in_ppb(word) => 0,
            _ => return None,
        })
    }

    /// The v6-M writes that differ, returning whether this handled it (a
    /// reserved address is handled by ignoring it).
    pub(super) fn write_word_v6m(&mut self, word: u32, value: u32) -> bool {
        match word {
            SYST_CSR..=SYST_CALIB
            | NVIC_ISER
            | NVIC_ICER
            | NVIC_ISPR
            | NVIC_ICPR
            | NVIC_IPR0..=NVIC_IPR7
            | CPUID
            | ICSR
            | VTOR
            | MPU_TYPE..=MPU_RASR
                if word != MPU_RBAR || self.mpu_regions == 0 =>
            {
                return false;
            }
            MPU_RBAR => {
                // As ARMv7-M — `VALID` selects the region named in `REGION`
                // — except that `ADDR` is bits [31:8]: the smallest PMSAv6
                // region is 256 bytes (DDI 0419E B3.5.6, B3.5.7).
                let n = if value & (1 << 4) != 0 {
                    let n = (value & 0xf) as usize & (super::sys::MPU_REGIONS - 1);
                    self.mpu_rnr = n as u32;
                    n
                } else {
                    (self.mpu_rnr as usize) & (super::sys::MPU_REGIONS - 1)
                };
                self.mpu_rbar[n] = (value & 0xffff_ff00) | (n as u32);
            }
            AIRCR => {
                // The key as on ARMv7-M; no `PRIGROUP` to set
                // (DDI 0419E B3.2.6).
                if value >> 16 == 0x05fa {
                    if value & (1 << 2) != 0 {
                        self.reset_requested = true;
                    }
                    if value & (1 << 1) != 0 {
                        self.active = [0; Exception::COUNT / 32];
                    }
                }
            }
            SCR => self.scr = value & SCR_WRITABLE,
            // Read-only on ARMv6-M.
            CCR => {}
            SHPR2 => {
                let p = self.quantize_priority((value >> 24) as u8);
                self.priority[usize::from(Exception::SVCALL.0)] = p;
            }
            SHPR3 => {
                let pendsv = self.quantize_priority((value >> 16) as u8);
                let systick = self.quantize_priority((value >> 24) as u8);
                self.priority[usize::from(Exception::PEND_SV.0)] = pendsv;
                self.priority[usize::from(Exception::SYSTICK.0)] = systick;
            }
            SHCSR => self.set_pending(Exception::SVCALL, value & SHCSR_SVCALLPENDED != 0),
            _ if in_ppb(word) => {}
            _ => return false,
        }
        true
    }
}
