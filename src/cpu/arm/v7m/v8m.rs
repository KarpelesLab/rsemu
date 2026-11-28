//! Armv8-M Mainline: what a Cortex-M33 adds to, and changes in, the ARMv7E-M
//! core this module sits inside.
//!
//! Selected per instance by [`Config::CORTEX_M33`](super::Config::CORTEX_M33)
//! (or `part = "cortex-m33"` / `"cortex-m33f"`; the Cortex-M35P presets,
//! `"cortex-m35p"` / `"cortex-m35pf"`, are the same architecture with their
//! own `CPUID`), and compiled only with the
//! `cpu-arm-v8m` feature, so a v7-M-only build links none of it. The same
//! interpreter runs both architectures: what differs is keyed off one flag,
//! [`Config::is_v8m`](super::Config::is_v8m), rather than off part names.
//!
//! # Modelled without the Security Extension
//!
//! A Cortex-M33 may be built with or without TrustZone for Armv8-M, and an
//! STM32U5 has it in silicon but turns it off with the `TZEN` option byte —
//! the factory default — so firmware for one sees a core with **one** security
//! state. That is what this models. Concretely:
//!
//! - there is no Secure state, no SAU or IDAU, and no banking: the SAU's
//!   registers at `0xE000EDD0`–`0xE000EDE8` (`SAU_CTRL`, `SAU_TYPE`, …,
//!   `SFSR`, `SFAR`) and `NVIC_ITNS` read as zero and ignore writes, which is
//!   what `SAU_TYPE.SREGION = 0` and an absent extension look like;
//! - `MRS`/`MSR` of the `_NS` aliases (`SYSm` with bit 7 set) read as zero
//!   and ignore writes;
//! - `TTA`/`TTAT`, `BXNS` and `BLXNS` are UNDEFINED; `SG` is a `NOP`;
//! - `EXC_RETURN.S` and `.ES` must be zero and `.DCRS` one — the only values
//!   a core with one security state ever produces — and any other value is an
//!   `INVPC` UsageFault, as is a v7-M-format `0xFFFFFFFD` (which has `S` and
//!   `ES` set). Firmware ported from a v7-M RTOS that ORs `0xD` into `LR`
//!   faults here rather than running on by luck;
//! - `AIRCR.BFHFNMINS`/`PRIS`/`SYSRESETREQS`, `SHCSR.SECUREFAULT*`,
//!   `FPCCR.S`/`SFRDY`/`TS`/`CLRONRET`/`CLRONRETS`/`LSPENS` and
//!   `CONTROL.SFPA` read as zero and ignore writes.
//!
//! # The Secure-only option
//!
//! A part configured with
//! [`Security::SecureOnly`](super::Security::SecureOnly) implements the
//! extension and never leaves Secure state, where the PE resets: firmware that
//! never configures the SAU nor branches to Non-secure code. The one thing
//! that changes for it is `EXC_RETURN`: every exception is taken from and to
//! Secure state, so entry generates `S` and `ES` set (`0xFFFFFFF1`,
//! `0xFFFFFFF9`, `0xFFFFFFFD` — numerically the ARMv7-M values) and return
//! requires them. A return to Non-secure state (`S` clear), a Non-secure
//! exception (`ES` clear) or a frame carrying the callee-saved registers
//! (`DCRS` clear) needs the Non-secure world, which is not modelled, and is
//! `INVPC`. Everything listed above for the SAU, the `_NS` aliases and
//! `TTA`/`BXNS`/`BLXNS` is as without the extension.
//!
//! # What is added
//!
//! | Mechanism | Where |
//! | --- | --- |
//! | `MSPLIM`/`PSPLIM` and the `STKOF` UsageFault | `MRS`/`MSR` `SYSm` 10/11; `exec`'s stack-pointer writes and exception entry |
//! | The v8-M `EXC_RETURN` | [`decode_exc_return`]; `0xFFFFFFBC` and friends |
//! | `LDA*`, `STL*`, `LDAEX*`, `STLEX*` | [`Insn::AcqRel`](super::isa::Insn::AcqRel) |
//! | `TT`, `TTT` | [`Sys::tt_response`] |
//! | `SG`, `CSDB`, `SSBB`, `PSSBB` | `NOP`s, decoded so they disassemble |
//! | PMSAv8 | [`Sys::mpu_permits_v8`] and the register map below |
//! | `CCR` | `STKOFHFNMIGN`; `NONBASETHRDENA` and `STKALIGN` RES1 |
//! | `FPCCR` | `UFRDY`, `SPLIMVIOL` |
//!
//! # Stack limits, as modelled
//!
//! DDI 0553's stack-limit pseudocode (the `SP` write helper and
//! `ViolatesSPLim`, and `PushStack` for exception entry) gives two cases, and
//! this is the reading implemented:
//!
//! - **An instruction** whose new `SP` would be below the limit of the stack
//!   it is writing raises a synchronous UsageFault with `UFSR.STKOF`, stacked
//!   at the faulting instruction, and **`SP` keeps its old value**. A store
//!   that writes back the stack pointer (`PUSH`, `STMDB sp!`, `STR rX,
//!   [sp, #-n]!`, `VPUSH`) is checked *before* any of its memory accesses, so
//!   nothing below the limit is written — the architecture forbids those
//!   writes and leaves the ones above it IMPLEMENTATION DEFINED, and making
//!   none is the simpler of the two legal answers.
//! - **Exception entry** whose frame would cross the limit writes only the
//!   frame words at or above it, sets that stack pointer **to the limit**,
//!   and pends a UsageFault with `STKOF` (a HardFault with `HFSR.FORCED` when
//!   UsageFault is disabled) as a derived exception; the ordinary priority
//!   arbitration then decides whether it preempts the handler just entered.
//!
//! `MSR MSP`/`MSR PSP` are **not** checked, so a scheduler may move a stack
//! and its limit in either order; neither are the unstacking reads of an
//! exception return. `CCR.STKOFHFNMIGN` set suppresses the check while the
//! execution priority is negative. The limits' bits `[2:0]` are RES0.
//!
//! # PMSAv8
//!
//! A region is a base (`RBAR[31:5]`) and an inclusive limit
//! (`RLAR[31:5]:11111`) rather than a naturally aligned power of two, and has
//! no sub-regions. `RBAR.AP` is two bits — `00` privileged read/write, `01`
//! read/write, `10` privileged read-only, `11` read-only — and `RBAR.XN`
//! forbids execution. Unlike PMSAv7 there is **no priority among regions**: an
//! address two enabled regions both cover is a MemManage fault whatever the
//! regions say. `PRIVDEFENA` and `HFNMIENA` are as before. `RBAR_A1`–`A3` and
//! `RLAR_A1`–`A3` reach regions `RNR[7:2]:01`, `:10` and `:11`; `MAIR0`/`MAIR1`
//! are stored and never interpreted. `RLAR.PXN` is Armv8.1-M and RES0 here.
//!
//! # Not modelled
//!
//! The ID registers (`ID_PFR0` … `ID_ISAR5` at `0xE000ED40`) read as zero, as
//! they do on this core's v7-M parts: no source in hand gives the Cortex-M33's
//! values, and a guessed feature word is worse than an honest zero. The
//! Cortex-M33's `ACTLR` bits, its optional coprocessor interface and the
//! custom datapath extension are not modelled, and nor is the Armv8.1-M
//! (Cortex-M55/M85) material — MVE, `PXN`, low-overhead loops.
//!
//! # Sources
//!
//! *Armv8-M Architecture Reference Manual*, Arm DDI 0553 — the instruction
//! descriptions (C2.4), the exception model, `EXC_RETURN` and the stack-limit
//! checks (B3), PMSAv8 (B10 "the Armv8-M Protected Memory System
//! Architecture") and the System Control Space register descriptions (D1);
//! the *Cortex-M33 Technical Reference Manual* for `CPUID` and the reset value
//! of `CCR`. No emulator source of any licence was consulted (`ROADMAP.md`
//! §1).

use super::sys::{Access, MPU_REGIONS, Sys, ccr, exc_return, fpccr};

/// `CPUID` for a Arm Cortex-M33 r0p4: implementer `0x41`, variant 0,
/// architecture `0xF`, part number `0xD21`, revision 4.
///
/// r0p4 is the revision ST ships in the STM32U5 family. A board modelling
/// different silicon sets [`Config::cpuid`](super::Config::cpuid).
pub const CPUID_CORTEX_M33: u32 = 0x410f_d214;

/// `CPUID` for an Arm Cortex-M35P r1p1: implementer `0x41`, variant 1,
/// architecture `0xF`, revision 1 — and a part number of **zero**.
///
/// r1p1 is the revision Arm put through Common Criteria (*Arm Cortex-M35P
/// r1p1 Lite Security Target*, PJDOC-466751330-13922, §1: "The TOE is
/// identified as Cortex-M35P r1p1"), and DDI 0553 puts the major revision in
/// `Variant` and the minor in `Revision`. The part number is in the
/// Cortex-M35P TRM (100883), which Arm distributes only on request, and no
/// permissively licensed source this project may read prints it; the field
/// reads zero rather than carry a number of unknown provenance, as
/// [`CPUID_SC300`](super::CPUID_SC300)'s does. A board that knows its
/// silicon's value sets [`Config::cpuid`](super::Config::cpuid) (or the
/// `cpuid` property).
pub const CPUID_CORTEX_M35P: u32 = 0x411f_0001;

/// The `CCR` bits a v8-M part without caches lets software write.
const CCR_WRITABLE: u32 =
    ccr::USERSETMPEND | ccr::UNALIGN_TRP | ccr::DIV_0_TRP | ccr::BFHFNMIGN | ccr::STKOFHFNMIGN;

/// The `CCR` bits that read as one whatever is written: `STKALIGN`, and
/// `NONBASETHRDENA`, which Armv8-M made RES1 — a return to Thread mode with
/// exceptions still active is always permitted. That gives the Cortex-M33
/// TRM's reset value of `0x00000201`.
const CCR_RES1: u32 = ccr::STKALIGN | ccr::NONBASETHRDENA;

/// The `FPCCR` bits a v8-M part without the Security Extension implements:
/// ARMv7-M's, plus `UFRDY` and `SPLIMVIOL`.
const FPCCR_WRITABLE: u32 = fpccr::WRITABLE | fpccr::UFRDY | fpccr::SPLIMVIOL;

/// `MPU_TYPE` .. `MPU_MAIR1`.
const MPU_TYPE: u32 = 0xe000_ed90;
const MPU_CTRL: u32 = 0xe000_ed94;
const MPU_RNR: u32 = 0xe000_ed98;
const MPU_RBAR: u32 = 0xe000_ed9c;
const MPU_RLAR_A3: u32 = 0xe000_edb8;
const MPU_MAIR0: u32 = 0xe000_edc0;
const MPU_MAIR1: u32 = 0xe000_edc4;
const CCR: u32 = 0xe000_ed14;
const FPCCR: u32 = 0xe000_ef34;

/// What an `EXC_RETURN` value asks for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExcReturn {
    /// Return to Handler mode rather than Thread mode.
    pub to_handler: bool,
    /// Unstack from, and return to, the process stack.
    pub use_psp: bool,
    /// The frame is the extended one carrying `S0`–`S15` and `FPSCR`.
    pub fp_frame: bool,
}

/// Decode an Armv8-M `EXC_RETURN`, or `None` for a value this core may not
/// return through (the caller raises `INVPC`).
///
/// Bits `[31:7]` are the `0xFF` prefix and RES1; bit 5 `DCRS` must be set,
/// because no frame here carries the callee-saved registers; bit 1 is RES0.
/// Bit 6 `S` and bit 0 `ES` must both be clear on a part without the
/// Security Extension, which has only one security state, and both set on a
/// [`Security::SecureOnly`](super::Security::SecureOnly) part, whose
/// exceptions are all taken to, and return to, Secure state. Handler mode on
/// the process stack (`Mode` clear, `SPSEL` set) does not exist.
#[must_use]
pub const fn decode_exc_return(value: u32, secure: bool) -> Option<ExcReturn> {
    const FIXED_MASK: u32 = 0xffff_ff80 | exc_return::S | (1 << 5) | (1 << 1) | exc_return::ES;
    let fixed = 0xffff_ff80
        | (1 << 5)
        | if secure {
            exc_return::S | exc_return::ES
        } else {
            0
        };
    if value & FIXED_MASK != fixed {
        return None;
    }
    let to_handler = value & (1 << 3) == 0;
    let use_psp = value & (1 << 2) != 0;
    if to_handler && use_psp {
        return None;
    }
    Some(ExcReturn {
        to_handler,
        use_psp,
        fp_frame: value & exc_return::FP_FRAME == 0,
    })
}

/// The `EXC_RETURN` an Armv8-M exception entry puts in `LR`. On a
/// [`Security::SecureOnly`](super::Security::SecureOnly) part (`secure`) the
/// exception is taken from and to Secure state, so `S` and `ES` are set —
/// which gives `0xFFFFFFF1`, `0xFFFFFFF9` and `0xFFFFFFFD`, the ARMv7-M
/// values.
#[must_use]
pub const fn encode_exc_return(
    from_handler: bool,
    from_psp: bool,
    fp_frame: bool,
    secure: bool,
) -> u32 {
    let base = if from_handler {
        exc_return::V8_HANDLER_MSP
    } else if from_psp {
        exc_return::V8_THREAD_PSP
    } else {
        exc_return::V8_THREAD_MSP
    };
    let base = if secure {
        base | exc_return::S | exc_return::ES
    } else {
        base
    };
    if fp_frame {
        base & !exc_return::FP_FRAME
    } else {
        base
    }
}

/// Which enabled PMSAv8 regions an address falls in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MpuMatch {
    /// None: the background region decides.
    None,
    /// Exactly one, and which.
    One(u8),
    /// Two or more — a MemManage fault for any access.
    Overlap,
}

impl Sys {
    /// The same block on an Armv8-M Mainline part: PMSAv8, the v8-M `CCR`
    /// with its reset value of `0x00000201`, and the v8-M `FPCCR` bits.
    #[must_use]
    pub fn with_v8m(mut self) -> Sys {
        self.v8m = true;
        self.ccr = CCR_RES1;
        self
    }

    /// The registers whose v8-M meaning differs from v7-M's, or `None` to let
    /// the shared map answer.
    pub(super) fn read_word_v8m(&self, word: u32) -> Option<u32> {
        Some(match word {
            CCR => self.ccr,
            FPCCR => self.fpccr,
            MPU_TYPE..=MPU_MAIR1 if self.mpu_regions == 0 => 0,
            MPU_TYPE => u32::from(self.mpu_regions) << 8,
            MPU_CTRL => self.mpu_ctrl,
            MPU_RNR => self.mpu_rnr,
            MPU_RBAR..=MPU_RLAR_A3 => {
                let (n, limit) = self.mpu_alias(word);
                if limit {
                    self.mpu_rasr[n]
                } else {
                    self.mpu_rbar[n]
                }
            }
            MPU_MAIR0 => self.mpu_mair[0],
            MPU_MAIR1 => self.mpu_mair[1],
            _ => return None,
        })
    }

    /// The writes whose v8-M meaning differs, returning whether this handled
    /// it.
    pub(super) fn write_word_v8m(&mut self, word: u32, value: u32) -> bool {
        match word {
            CCR => self.ccr = (value & CCR_WRITABLE) | CCR_RES1,
            FPCCR => {
                if self.fp_present() {
                    self.fpccr = value & FPCCR_WRITABLE;
                }
            }
            MPU_TYPE..=MPU_MAIR1 if self.mpu_regions == 0 => {}
            MPU_TYPE => {}
            MPU_CTRL => self.mpu_ctrl = value & 0x7,
            MPU_RNR => self.mpu_rnr = value & 0xff,
            MPU_RBAR..=MPU_RLAR_A3 => {
                let (n, limit) = self.mpu_alias(word);
                if limit {
                    // `PXN`, bit 4, is Armv8.1-M; RES0 here.
                    self.mpu_rasr[n] = value & !0x10;
                } else {
                    self.mpu_rbar[n] = value;
                }
            }
            MPU_MAIR0 => self.mpu_mair[0] = value,
            MPU_MAIR1 => self.mpu_mair[1] = value,
            // Below `0xE000EDBC` there is nothing else in the MPU block; the
            // two words between `RLAR_A3` and `MAIR0` are reserved.
            _ => return false,
        }
        true
    }

    /// Which region an `RBAR`/`RLAR` address (or alias) names, and whether it
    /// is the limit register.
    ///
    /// `RBAR`/`RLAR` reach region `RNR`; alias *n* reaches `RNR[7:2]:n`.
    fn mpu_alias(&self, word: u32) -> (usize, bool) {
        let offset = word - MPU_RBAR;
        let alias = offset / 8;
        let region = if alias == 0 {
            self.mpu_rnr
        } else {
            (self.mpu_rnr & !3) | alias
        };
        ((region as usize) & (MPU_REGIONS - 1), offset % 8 == 4)
    }

    /// The enabled PMSAv8 regions `addr` falls in.
    #[must_use]
    pub fn mpu_match_v8(&self, addr: u32) -> MpuMatch {
        let mut found = MpuMatch::None;
        for n in 0..usize::from(self.mpu_regions).min(MPU_REGIONS) {
            let rlar = self.mpu_rasr[n];
            if rlar & 1 == 0 {
                continue;
            }
            let base = self.mpu_rbar[n] & !0x1f;
            let limit = rlar | 0x1f;
            if addr < base || addr > limit {
                continue;
            }
            found = match found {
                MpuMatch::None => MpuMatch::One(n as u8),
                _ => return MpuMatch::Overlap,
            };
        }
        found
    }

    /// Whether the PMSAv8 MPU permits `access` at `addr`.
    #[must_use]
    pub fn mpu_permits_v8(
        &self,
        addr: u32,
        access: Access,
        privileged: bool,
        priority: i32,
    ) -> bool {
        if self.mpu_regions == 0 || self.mpu_ctrl & 1 == 0 {
            return true;
        }
        if priority < 0 && self.mpu_ctrl & 0b10 == 0 {
            return true;
        }
        let n = match self.mpu_match_v8(addr) {
            MpuMatch::None => return privileged && self.mpu_ctrl & 0b100 != 0,
            MpuMatch::Overlap => return false,
            MpuMatch::One(n) => usize::from(n),
        };
        let rbar = self.mpu_rbar[n];
        if access == Access::Fetch && rbar & 1 != 0 {
            return false;
        }
        let read_only = rbar & (1 << 2) != 0;
        let any_privilege = rbar & (1 << 1) != 0;
        if !privileged && !any_privilege {
            return false;
        }
        !(read_only && access == Access::Write)
    }

    /// The `TT_RESP` word `TT` (or, with `unpriv`, `TTT`) returns for `addr`
    /// (DDI 0553, `TT` and the `TT_RESP` payload).
    ///
    /// `MREGION`/`MRVALID` (bits 7:0 and 16) name the one MPU region the
    /// address falls in — valid only when the MPU is enabled, exactly one
    /// region matches, and the instruction ran privileged, so unprivileged
    /// code cannot map out the MPU. `R` and `RW` (bits 18 and 19) say whether
    /// a read, and a read and a write, at the queried privilege would be
    /// permitted. Every security field — `SREGION`, `SRVALID`, `NSR`, `NSRW`,
    /// `S`, `IRVALID`, `IREGION` — is zero, there being no Security
    /// Extension.
    #[must_use]
    pub fn tt_response(
        &self,
        addr: u32,
        running_privileged: bool,
        unpriv: bool,
        priority: i32,
    ) -> u32 {
        let mut resp = 0u32;
        if running_privileged
            && self.mpu_regions != 0
            && self.mpu_ctrl & 1 != 0
            && let MpuMatch::One(n) = self.mpu_match_v8(addr)
        {
            resp |= u32::from(n) | (1 << 16);
        }
        let as_privileged = running_privileged && !unpriv;
        let r = self.mpu_permits_v8(addr, Access::Read, as_privileged, priority);
        let w = self.mpu_permits_v8(addr, Access::Write, as_privileged, priority);
        if r {
            resp |= 1 << 18;
        }
        if r && w {
            resp |= 1 << 19;
        }
        resp
    }
}
