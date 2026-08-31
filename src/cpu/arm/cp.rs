//! The coprocessor seam, and the memory-translation seam CP15 lives behind.
//!
//! An ARM926EJ-S is a core *plus* a CP15 that owns the MMU, the caches, the
//! tightly-coupled memories and a pile of SoC-specific control bits. None of
//! that belongs in a generic core: which translation table walk, which cache
//! policy and which TCM layout a part has is exactly the thing that differs
//! between the SoC that consumes this crate and the next one. `ROADMAP.md`
//! §15's first invariant — the core must not know what devices exist — points
//! the same way.
//!
//! So the core ships two traits and a default implementation of each:
//!
//! | Trait | What it is | Default |
//! | --- | --- | --- |
//! | [`Coprocessor`] | `MCR`/`MRC`/`CDP`/`LDC`/`STC`/`MCRR`/`MRRC` for one coprocessor number | none attached; every coprocessor instruction is Undefined |
//! | [`Mmu`] | virtual to physical, plus where the vectors live | [`FlatMmu`], an identity map that never faults |
//!
//! A downstream SoC crate implements both on one type — that is what a real
//! CP15 is — and attaches it with
//! [`Arm::attach_coprocessor`](super::Arm::attach_coprocessor) and
//! [`Arm::attach_mmu`](super::Arm::attach_mmu). For bring-up before that
//! exists, [`Cp15Stub`] answers the handful of registers that boot code
//! touches and reports no MMU.
//!
//! # Why a fault is data rather than an error type
//!
//! [`Fault`] carries an ARM fault-status encoding (ARM ARM B4.6) because the
//! CP15 that raised it is also the thing that has to publish it in `FSR` and
//! `FAR`, and the core is only the courier. [`Mmu::report_abort`] is that
//! hand-off; the core calls it on every abort it takes, including the ones the
//! address space raised rather than the MMU.
//!
//! # Sources
//!
//! ARM ARM (DDI 0100) A2.9 "Coprocessors", A4.1.19/30/32 for `CDP`, `MRC` and
//! `MCR`, B2 "The System Control Coprocessor" for the CP15 register layout the
//! stub implements, and B4.6 for the fault-status encodings.

use alloc::fmt;

use crate::core::sync::{AtomicU32, Ordering};

/// Which register a coprocessor instruction names.
///
/// The same five fields describe `MCR`, `MRC` and `CDP`, which is why one type
/// serves all three (ARM ARM A4.1.32's `<opcode_1>`, `<CRn>`, `<CRm>`,
/// `<opcode_2>`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CpOp {
    /// The coprocessor number, `0..=15`.
    pub cp: u8,
    /// The first opcode field: three bits for `MCR`/`MRC`, four for `CDP`.
    pub opc1: u8,
    /// The destination coprocessor register, for `CDP` only.
    pub crd: u8,
    /// The first coprocessor register operand.
    pub crn: u8,
    /// The second coprocessor register operand.
    pub crm: u8,
    /// The second opcode field.
    pub opc2: u8,
}

/// What an `LDC` or `STC` names.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CpTransfer {
    /// The coprocessor number.
    pub cp: u8,
    /// The coprocessor register.
    pub crd: u8,
    /// The `N` bit — a coprocessor-defined "long" flag.
    pub long: bool,
    /// The eight-bit field, which is a word offset in the indexed forms and a
    /// coprocessor option in the unindexed one (ARM ARM A5.5.5).
    pub option: u8,
}

/// Why a coprocessor refused an instruction.
///
/// Deliberately a single variant. The architecture gives a coprocessor exactly
/// one way to decline — the instruction becomes Undefined and the core takes
/// the exception (ARM ARM A2.9.1) — and inventing more would be inventing
/// hardware.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum CpFault {
    /// The coprocessor does not implement this encoding.
    Undefined,
}

/// The result of a coprocessor operation.
pub type CpResult<T = ()> = core::result::Result<T, CpFault>;

/// What a coprocessor asks the core to do afterwards.
///
/// Empty by default, and `#[non_exhaustive]` so a later addition is not a
/// breaking change for implementors that construct it with
/// [`CpEffect::default`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[non_exhaustive]
pub struct CpEffect {
    /// Stop fetching until an interrupt arrives.
    ///
    /// This is how `MCR p15, 0, Rd, c7, c0, 4` — "wait for interrupt" — is
    /// implemented: the core has no idea what that register means, and the
    /// CP15 that does cannot reach into the core to halt it.
    pub halt: bool,
}

impl CpEffect {
    /// No effect beyond the register write itself.
    pub const NONE: CpEffect = CpEffect { halt: false };

    /// Halt the core until an interrupt is asserted.
    pub const HALT: CpEffect = CpEffect { halt: true };
}

/// One coprocessor, as the core sees it.
///
/// Every method defaults to [`CpFault::Undefined`] except the two that a
/// system coprocessor actually needs, so implementing CP15 means writing
/// [`Coprocessor::mcr`] and [`Coprocessor::mrc`] and nothing else.
pub trait Coprocessor: Send + Sync + fmt::Debug {
    /// `MCR`: move an ARM register into a coprocessor register.
    ///
    /// # Errors
    ///
    /// [`CpFault::Undefined`] if the coprocessor does not implement the
    /// encoding, which the core turns into an Undefined Instruction exception.
    fn mcr(&self, op: CpOp, value: u32) -> CpResult<CpEffect> {
        let _ = (op, value);
        Err(CpFault::Undefined)
    }

    /// `MRC`: move a coprocessor register into an ARM register.
    ///
    /// Writing `R15` with an `MRC` moves bits 31..28 of the result into the
    /// `N`, `Z`, `C` and `V` flags instead; the core does that, not the
    /// coprocessor.
    ///
    /// # Errors
    ///
    /// [`CpFault::Undefined`] if the coprocessor does not implement the
    /// encoding.
    fn mrc(&self, op: CpOp) -> CpResult<u32> {
        let _ = op;
        Err(CpFault::Undefined)
    }

    /// `CDP`: an operation entirely internal to the coprocessor.
    ///
    /// # Errors
    ///
    /// [`CpFault::Undefined`] if the coprocessor does not implement the
    /// encoding.
    fn cdp(&self, op: CpOp) -> CpResult<CpEffect> {
        let _ = op;
        Err(CpFault::Undefined)
    }

    /// `MCRR`: move an ARM register pair into the coprocessor (ARMv5TE).
    ///
    /// # Errors
    ///
    /// [`CpFault::Undefined`] if the coprocessor does not implement the
    /// encoding.
    fn mcrr(&self, cp: u8, opc: u8, crm: u8, value: u64) -> CpResult<CpEffect> {
        let _ = (cp, opc, crm, value);
        Err(CpFault::Undefined)
    }

    /// `MRRC`: move a coprocessor value into an ARM register pair (ARMv5TE).
    ///
    /// # Errors
    ///
    /// [`CpFault::Undefined`] if the coprocessor does not implement the
    /// encoding.
    fn mrrc(&self, cp: u8, opc: u8, crm: u8) -> CpResult<u64> {
        let _ = (cp, opc, crm);
        Err(CpFault::Undefined)
    }

    /// How many words this `LDC` or `STC` transfers.
    ///
    /// The architecture lets the coprocessor decide, and lets it decide one
    /// word at a time; this seam takes the simpler route of asking up front,
    /// which is what every ARMv5 coprocessor worth modelling does.
    ///
    /// # Errors
    ///
    /// [`CpFault::Undefined`] if the coprocessor does not implement the
    /// encoding.
    fn transfer_len(&self, op: CpTransfer) -> CpResult<u8> {
        let _ = op;
        Err(CpFault::Undefined)
    }

    /// Accept word `index` of an `LDC`.
    ///
    /// # Errors
    ///
    /// [`CpFault::Undefined`] if the coprocessor does not implement the
    /// encoding.
    fn write_word(&self, op: CpTransfer, index: u8, value: u32) -> CpResult {
        let _ = (op, index, value);
        Err(CpFault::Undefined)
    }

    /// Supply word `index` of an `STC`.
    ///
    /// # Errors
    ///
    /// [`CpFault::Undefined`] if the coprocessor does not implement the
    /// encoding.
    fn read_word(&self, op: CpTransfer, index: u8) -> CpResult<u32> {
        let _ = (op, index);
        Err(CpFault::Undefined)
    }
}

// ---------------------------------------------------------------------------
// Memory translation
// ---------------------------------------------------------------------------

/// What kind of access is being translated.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AccessKind {
    /// An instruction fetch. A fault here is a Prefetch Abort.
    Fetch,
    /// A data read. A fault here is a Data Abort.
    Read,
    /// A data write. A fault here is a Data Abort.
    Write,
}

impl AccessKind {
    /// Whether a fault on this access is a Prefetch Abort rather than a Data
    /// Abort.
    #[must_use]
    pub const fn is_fetch(self) -> bool {
        matches!(self, AccessKind::Fetch)
    }
}

/// An abort, in the encoding CP15's fault-status register uses.
///
/// The core never interprets these; it passes them to
/// [`Mmu::report_abort`] and takes the corresponding exception. The constants
/// are ARM ARM B4.6's table.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Fault {
    /// The four-bit fault status.
    pub status: u8,
    /// The four-bit domain, where the fault has one.
    pub domain: u8,
}

impl Fault {
    /// Alignment fault (`0b0001`).
    pub const ALIGNMENT: Fault = Fault {
        status: 0b0001,
        domain: 0,
    };
    /// Translation fault, section (`0b0101`).
    pub const TRANSLATION_SECTION: Fault = Fault {
        status: 0b0101,
        domain: 0,
    };
    /// Translation fault, page (`0b0111`).
    pub const TRANSLATION_PAGE: Fault = Fault {
        status: 0b0111,
        domain: 0,
    };
    /// External abort on a non-line-fetch (`0b1000`).
    ///
    /// This is what an unmapped or width-refusing address space becomes: the
    /// bus said no, and on ARM that is an external abort rather than the
    /// 6502's open bus.
    pub const EXTERNAL: Fault = Fault {
        status: 0b1000,
        domain: 0,
    };
    /// Permission fault, section (`0b1101`).
    pub const PERMISSION_SECTION: Fault = Fault {
        status: 0b1101,
        domain: 0,
    };
    /// Permission fault, page (`0b1111`).
    pub const PERMISSION_PAGE: Fault = Fault {
        status: 0b1111,
        domain: 0,
    };

    /// The same fault, attributed to a domain.
    #[must_use]
    pub const fn in_domain(mut self, domain: u8) -> Fault {
        self.domain = domain;
        self
    }

    /// The value CP15's fault-status register would hold: domain in bits 7..4,
    /// status in bits 3..0.
    #[must_use]
    pub const fn to_fsr(self) -> u32 {
        (((self.domain & 0xf) as u32) << 4) | ((self.status & 0xf) as u32)
    }
}

/// Virtual-to-physical translation, and the two system properties that go with
/// it.
///
/// Implemented by whatever models CP15. The core calls [`Mmu::translate`] on
/// every fetch and every data access, so an implementation that walks page
/// tables should cache — the core deliberately does not cache on its behalf,
/// because only the MMU knows when to invalidate.
pub trait Mmu: Send + Sync + fmt::Debug {
    /// Translate one address.
    ///
    /// `privileged` is the CPU's current privilege, already adjusted for the
    /// `LDRT`/`STRT` forms, which make a privileged access behave as an
    /// unprivileged one (ARM ARM A4.1.24).
    ///
    /// # Errors
    ///
    /// A [`Fault`] the core turns into a Prefetch or Data Abort depending on
    /// [`AccessKind`].
    fn translate(&self, va: u32, kind: AccessKind, privileged: bool) -> Result<u32, Fault>;

    /// Whether the exception vectors sit at `0xffff0000` rather than `0`.
    ///
    /// CP15's control register bit 13 (ARM ARM B2.1). It lives here rather
    /// than in the core's configuration because it is a *runtime* property
    /// that guest code flips, and the thing guest code flips it through is
    /// this object.
    fn high_vectors(&self) -> bool {
        false
    }

    /// Told about every abort the core takes, so `FSR` and `FAR` can be
    /// latched.
    ///
    /// Called for aborts this trait raised *and* for the ones the address
    /// space raised, which is the case an implementation would otherwise never
    /// hear about.
    fn report_abort(&self, va: u32, fault: Fault, kind: AccessKind) {
        let _ = (va, fault, kind);
    }
}

/// The default: a flat address space with no MMU and no faults.
///
/// This is what makes the core usable on its own — hand it an
/// [`AddressSpace`](crate::core::space::AddressSpace) and it runs, exactly as
/// an ARM7TDMI or a cacheless bring-up configuration does.
#[derive(Debug, Clone, Copy, Default)]
pub struct FlatMmu;

impl Mmu for FlatMmu {
    fn translate(&self, va: u32, _kind: AccessKind, _privileged: bool) -> Result<u32, Fault> {
        Ok(va)
    }
}

// ---------------------------------------------------------------------------
// A CP15 stub
// ---------------------------------------------------------------------------

/// Just enough CP15 to keep boot code from taking an Undefined Instruction
/// exception.
///
/// It answers the identification registers, remembers the control register,
/// and honours its `V` bit so a machine can move the vectors. Everything else
/// reads as zero and ignores writes. There is deliberately **no MMU here**:
/// this reports [`translate`](Mmu::translate) as an identity map whatever the
/// `M` bit says, because a translation table walk is the SoC's to implement
/// and a half-built one is worse than an honest absence.
///
/// A real part replaces this wholesale rather than extending it.
#[derive(Debug)]
pub struct Cp15Stub {
    main_id: u32,
    cache_type: u32,
    control: AtomicU32,
}

impl Cp15Stub {
    /// A stub reporting `main_id` as its CP15 c0 register.
    ///
    /// `0x41069265` is the ARM926EJ-S value (implementor `A`, architecture
    /// `5TEJ`, part `926`); a machine modelling something else passes its own.
    #[must_use]
    pub const fn new(main_id: u32) -> Cp15Stub {
        Cp15Stub {
            main_id,
            cache_type: 0,
            control: AtomicU32::new(0),
        }
    }

    /// The main ID register value an ARM926EJ-S reports.
    pub const ARM926EJS_ID: u32 = 0x4106_9265;

    /// The same stub with a cache-type register value.
    #[must_use]
    pub const fn with_cache_type(mut self, cache_type: u32) -> Cp15Stub {
        self.cache_type = cache_type;
        self
    }

    /// The current control register.
    #[must_use]
    pub fn control(&self) -> u32 {
        self.control.load(Ordering::Acquire)
    }
}

impl Default for Cp15Stub {
    fn default() -> Cp15Stub {
        Cp15Stub::new(Cp15Stub::ARM926EJS_ID)
    }
}

impl Coprocessor for Cp15Stub {
    fn mrc(&self, op: CpOp) -> CpResult<u32> {
        if op.cp != 15 || op.opc1 != 0 {
            return Err(CpFault::Undefined);
        }
        match (op.crn, op.crm, op.opc2) {
            (0, 0, 0) => Ok(self.main_id),
            (0, 0, 1) => Ok(self.cache_type),
            (1, 0, 0) => Ok(self.control()),
            // A real CP15 has forty more registers; reading zero from them is
            // a lie, but a quiet and harmless one, where an Undefined
            // Instruction exception during boot is neither.
            _ => Ok(0),
        }
    }

    fn mcr(&self, op: CpOp, value: u32) -> CpResult<CpEffect> {
        if op.cp != 15 || op.opc1 != 0 {
            return Err(CpFault::Undefined);
        }
        match (op.crn, op.crm, op.opc2) {
            (1, 0, 0) => {
                self.control.store(value, Ordering::Release);
                Ok(CpEffect::NONE)
            }
            // c7 c0 4 is "wait for interrupt" on every ARM9 part that has one.
            (7, 0, 4) => Ok(CpEffect::HALT),
            _ => Ok(CpEffect::NONE),
        }
    }
}

impl Mmu for Cp15Stub {
    fn translate(&self, va: u32, _kind: AccessKind, _privileged: bool) -> Result<u32, Fault> {
        Ok(va)
    }

    fn high_vectors(&self) -> bool {
        // Control register bit 13, the `V` bit (ARM ARM B2.1).
        self.control() & (1 << 13) != 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_stub_answers_the_id_registers_and_remembers_control() {
        let cp = Cp15Stub::default();
        let id = CpOp {
            cp: 15,
            opc1: 0,
            crd: 0,
            crn: 0,
            crm: 0,
            opc2: 0,
        };
        assert_eq!(cp.mrc(id), Ok(Cp15Stub::ARM926EJS_ID));

        let control = CpOp { crn: 1, ..id };
        assert_eq!(cp.mcr(control, 1 << 13), Ok(CpEffect::NONE));
        assert_eq!(cp.mrc(control), Ok(1 << 13));
        assert!(cp.high_vectors());
    }

    #[test]
    fn wait_for_interrupt_asks_the_core_to_halt() {
        let cp = Cp15Stub::default();
        let wfi = CpOp {
            cp: 15,
            opc1: 0,
            crd: 0,
            crn: 7,
            crm: 0,
            opc2: 4,
        };
        assert_eq!(cp.mcr(wfi, 0), Ok(CpEffect::HALT));
    }

    #[test]
    fn another_coprocessor_number_is_not_this_ones_business() {
        let cp = Cp15Stub::default();
        let op = CpOp {
            cp: 14,
            opc1: 0,
            crd: 0,
            crn: 0,
            crm: 0,
            opc2: 0,
        };
        assert_eq!(cp.mrc(op), Err(CpFault::Undefined));
    }

    #[test]
    fn a_flat_mmu_translates_nothing_and_faults_never() {
        let mmu = FlatMmu;
        assert_eq!(
            mmu.translate(0xdead_beef, AccessKind::Read, false),
            Ok(0xdead_beef)
        );
        assert!(!mmu.high_vectors());
    }

    #[test]
    fn fault_status_packs_the_domain_above_the_status() {
        assert_eq!(Fault::EXTERNAL.to_fsr(), 0b1000);
        assert_eq!(Fault::PERMISSION_PAGE.in_domain(3).to_fsr(), 0x3f);
    }
}
