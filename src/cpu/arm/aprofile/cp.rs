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
//! | [`Mmu`] | virtual to physical, plus the control bits the core reads | [`FlatMmu`], an identity map that never faults |
//!
//! A downstream SoC crate implements both on one type — that is what a real
//! CP15 is — and attaches it with
//! [`Arm::attach_coprocessor`](super::Arm::attach_coprocessor) and
//! [`Arm::attach_mmu`](super::Arm::attach_mmu).
//!
//! **The architecture's own CP15 now ships in the core**:
//! [`Cp15`](super::cp15::Cp15) is a VMSAv5 system control coprocessor and is
//! selected by a construction property ([`Config::system`](super::Config)), so
//! a `.machine` file asks for one by name. The traits here remain the seam for
//! the parts that are genuinely the SoC's — a coprocessor 14 debug unit, a
//! vendor MMU — and [`Cp15Stub`] remains for bring-up where even a real CP15
//! is more than is wanted.
//!
//! # Why the core keeps the TLB and the MMU does not
//!
//! [`Tlb`] lives here, beside the trait, and the *core* owns an instance of
//! it. An [`Mmu`] implementation is `&self` and `Sync`, so a cache inside one
//! needs a lock or an atomic on the hot path — a cost paid on every access, to
//! avoid a walk that happens on one access in a thousand. The core has
//! `&mut` to its own execution state and can index a plain array, so that is
//! where the cache goes. What the MMU owes the core instead is
//! [`Regime::generation`]: a number that changes whenever a cached translation
//! could have gone stale. `ROADMAP.md` §4.1 puts the page table and the
//! software TLB *above* `core::space` for the same reason, and this is that
//! position occupied.
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

    /// Which of the three per-kind halves of a [`Tlb`] this access uses.
    #[inline]
    const fn slot(self) -> usize {
        match self {
            AccessKind::Fetch => 0,
            AccessKind::Read => 1,
            AccessKind::Write => 2,
        }
    }
}

/// A guest-*virtual* address, as an instruction names it.
///
/// Distinct from [`Pa`] because confusing the two is, in `CLAUDE.md`'s words,
/// the classic emulator bug — and a page-table walk is where it happens: every
/// descriptor read is at a physical address, every table index comes from a
/// virtual one, and the two are the same width on a 32-bit ARM so the compiler
/// is the only thing that can tell them apart. The newtypes stop at this seam:
/// the interpreter works in `u32` above it, wrapping on the way in and
/// unwrapping on the way out, because threading them through every addressing
/// mode would be noise for a distinction that only matters here.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Va(pub u32);

/// A guest-*physical* address: what comes out of translation and goes onto the
/// bus. See [`Va`].
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Pa(pub u32);

impl fmt::Display for Va {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:#010x}", self.0)
    }
}

impl fmt::Display for Pa {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:#010x}", self.0)
    }
}

/// Physical memory, as a translation-table walk needs to see it.
///
/// The walk is the one part of an [`Mmu`] that reads guest memory, and it must
/// read it *physically* and without translating — a table walk that went back
/// through the MMU would not terminate. The core supplies this because the core
/// is what holds the address space; an `Mmu` that kept its own handle on one
/// would have to be told about every rebind, and the bus master's identity
/// would then live in two places.
pub trait PhysMem {
    /// Read one 32-bit descriptor.
    ///
    /// `None` is a bus error — an unmapped table — which the walker turns into
    /// [`Fault::EXTERNAL_L1`] or [`Fault::EXTERNAL_L2`] depending on where it
    /// happened.
    fn read_u32(&self, at: Pa) -> Option<u32>;
}

/// Everything about the MMU that the core caches for the length of one
/// instruction.
///
/// Sampled once per [`step`](super::Arm::step) rather than once per access.
/// Three of the four fields are read on *every* memory access and none of them
/// can change under the instruction reading them — the only thing that changes
/// them is an `MCR` to CP15, which is itself an instruction. Sampling per step
/// also reproduces the architectural rule that enabling the MMU takes effect no
/// earlier than the following instruction (ARM ARM B2.1 leaves the exact point
/// implementation-defined, which is why every ARM boot sequence follows the
/// `MCR` with a pipeline's worth of harmless instructions).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Regime {
    /// Changes whenever a translation the core cached could have gone stale.
    ///
    /// The core watches it for *change*, never for value, so an implementation
    /// may bump it as coarsely as it likes — an ARMv5 CP15 bumps it on every
    /// c8 TLB operation, on a `TTBR`, domain or `FCSE PID` write, and on an
    /// MMU enable or disable, which between them cover every architectural
    /// invalidation.
    pub generation: u32,
    /// Whether addresses need translating at all.
    ///
    /// `false` is the ARM926EJ-S out of reset, and it means more than an
    /// identity map: the core skips the TLB, the walk and the permission check
    /// entirely, so a machine with the MMU off costs what it did before this
    /// module existed.
    pub translating: bool,
    /// CP15 c1's `V` bit: the exception vectors are at `0xffff0000`.
    pub high_vectors: bool,
    /// CP15 c1's `A` bit: an unaligned access is a Data Abort, not a rotate.
    pub alignment_faults: bool,
}

impl Regime {
    /// No translation, low vectors, no alignment checking — an ARM with no
    /// system coprocessor.
    pub const FLAT: Regime = Regime {
        generation: 0,
        translating: false,
        high_vectors: false,
        alignment_faults: false,
    };
}

impl Default for Regime {
    fn default() -> Regime {
        Regime::FLAT
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
    /// Domain fault, section (`0b1001`) — the domain is "no access".
    pub const DOMAIN_SECTION: Fault = Fault {
        status: 0b1001,
        domain: 0,
    };
    /// Domain fault, page (`0b1011`).
    pub const DOMAIN_PAGE: Fault = Fault {
        status: 0b1011,
        domain: 0,
    };
    /// External abort on a first-level translation (`0b1100`).
    ///
    /// The bus refused the read of the *first-level descriptor* itself. A
    /// guest that sees this has pointed `TTBR` at nothing.
    pub const EXTERNAL_L1: Fault = Fault {
        status: 0b1100,
        domain: 0,
    };
    /// External abort on a second-level translation (`0b1110`).
    pub const EXTERNAL_L2: Fault = Fault {
        status: 0b1110,
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
    /// The control bits the core reads, sampled once per instruction.
    ///
    /// Defaults to [`Regime::FLAT`], which is what makes
    /// [`translate`](Mmu::translate) unreachable for an implementation that
    /// does not translate: the core never calls it.
    fn regime(&self) -> Regime {
        Regime::FLAT
    }

    /// Translate one address, walking whatever tables this MMU has.
    ///
    /// Called only when [`Regime::translating`] is set, and only on a miss in
    /// the core's [`Tlb`] — so an implementation should do the honest walk here
    /// and not cache: the core is already caching, and a second cache would
    /// need a lock the core does not.
    ///
    /// `privileged` is the CPU's current privilege, already adjusted for the
    /// `LDRT`/`STRT` forms, which make a privileged access behave as an
    /// unprivileged one (ARM ARM A4.1.24).
    ///
    /// # Errors
    ///
    /// A [`Fault`] the core turns into a Prefetch or Data Abort depending on
    /// [`AccessKind`].
    fn translate(
        &self,
        mem: &dyn PhysMem,
        va: Va,
        kind: AccessKind,
        privileged: bool,
    ) -> Result<Pa, Fault>;

    /// Resolve one address for a debugger, with no side effects at all.
    ///
    /// The question is *where is this mapped*, and nothing else: no access
    /// kind, no privilege, no permission check. See
    /// [`Device::debug_translate`](crate::core::device::Device::debug_translate)
    /// for why a debugger asks it that way, and note that `mem` here reads the
    /// tables with [`MemAttrs::DEBUG`](crate::core::space::MemAttrs::DEBUG).
    ///
    /// The default is the ordinary walk, asked as a privileged read. That is
    /// correct for any MMU in this family, because the seam makes it so:
    /// [`PhysMem`] is read-only, so a VMSAv5-shaped walk **cannot** write an
    /// accessed or dirty bit back even if the architecture had one — and
    /// VMSAv5 does not. What the default cannot do is see a page the
    /// permissions hide; an implementation that wants a debugger to see one
    /// anyway overrides this, as [`Cp15`](super::cp15::Cp15) does.
    ///
    /// The core never calls [`report_abort`](Mmu::report_abort) for a failure
    /// here, so an implementation must not latch one itself.
    ///
    /// # Errors
    ///
    /// A [`Fault`] describing why nothing is mapped. The caller is a debugger,
    /// so it reports the fault rather than taking an abort.
    fn translate_debug(&self, mem: &dyn PhysMem, va: Va) -> Result<Pa, Fault> {
        self.translate(mem, va, AccessKind::Read, true)
    }

    /// Told about every abort the core takes, so `FSR` and `FAR` can be
    /// latched.
    ///
    /// Called for aborts this trait raised *and* for the ones the address
    /// space raised, which is the case an implementation would otherwise never
    /// hear about.
    fn report_abort(&self, va: Va, fault: Fault, kind: AccessKind) {
        let _ = (va, fault, kind);
    }
}

/// The default: a flat address space with no MMU and no faults.
///
/// This is what makes the core usable on its own — hand it an
/// [`AddressSpace`](crate::core::space::AddressSpace) and it runs, exactly as
/// an ARM7TDMI or a cacheless bring-up configuration does.
///
/// It carries the two control bits anyway, seeded from the core's
/// [`Config`](super::Config), because **the installed MMU is what the core asks
/// about them** — there is no second answer ORed in from elsewhere. On real
/// silicon `VINITHI` and the alignment strap set the *reset value* of CP15 c1
/// and software owns them afterwards; modelling it as "the board's answer, or
/// CP15's, whichever says yes" would leave a board with `VINITHI` tied high
/// unable to move its vectors back down, which hardware can do.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FlatMmu {
    /// The exception vectors are at `0xffff0000`.
    pub high_vectors: bool,
    /// An unaligned access is a Data Abort.
    pub alignment_faults: bool,
}

impl FlatMmu {
    /// A flat map with low vectors and no alignment checking.
    #[must_use]
    pub const fn new() -> FlatMmu {
        FlatMmu {
            high_vectors: false,
            alignment_faults: false,
        }
    }
}

impl Mmu for FlatMmu {
    fn regime(&self) -> Regime {
        Regime {
            high_vectors: self.high_vectors,
            alignment_faults: self.alignment_faults,
            ..Regime::FLAT
        }
    }

    fn translate(
        &self,
        _mem: &dyn PhysMem,
        va: Va,
        _kind: AccessKind,
        _privileged: bool,
    ) -> Result<Pa, Fault> {
        Ok(Pa(va.0))
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
    fn regime(&self) -> Regime {
        let control = self.control();
        Regime {
            // Control register bits 13 and 1, the `V` and `A` bits
            // (ARM ARM B2.1). The `M` bit is deliberately *not* read: this stub
            // has no translation tables, and pretending otherwise is the
            // half-built MMU its documentation refuses to be.
            high_vectors: control & (1 << 13) != 0,
            alignment_faults: control & (1 << 1) != 0,
            ..Regime::FLAT
        }
    }

    fn translate(
        &self,
        _mem: &dyn PhysMem,
        va: Va,
        _kind: AccessKind,
        _privileged: bool,
    ) -> Result<Pa, Fault> {
        Ok(Pa(va.0))
    }
}

// ---------------------------------------------------------------------------
// The software TLB
// ---------------------------------------------------------------------------

/// How many entries each of the three halves of the TLB holds.
///
/// Direct-mapped and a power of two, so a lookup is a mask, a compare and an
/// add. `ROADMAP.md` §9 calls that the fast path's shape; this is the
/// interpreter's version of it.
pub const TLB_ENTRIES: usize = 256;

/// How much address space one TLB entry covers, in bits.
///
/// **One kibibyte, not four.** VMSAv5's smallest page is the 1 KiB *tiny* page
/// (ARM ARM B4.3.2), so four consecutive kibibytes can have four different
/// translations and four different permissions. Keying at 4 KiB would mean a
/// tiny page silently answering for its three neighbours, which is the kind of
/// bug that only shows up in a guest nobody has run yet. Sections and large
/// pages simply occupy several entries, which costs entries and never
/// correctness.
pub const TLB_PAGE_BITS: u32 = 10;

/// One cached translation.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct Entry {
    /// Virtual page number, privilege and epoch mixed together, so a stale
    /// entry can never be mistaken for a hit.
    tag: u64,
    /// The physical base of the kibibyte this maps to.
    base: u32,
    /// Whether this slot holds anything.
    valid: bool,
}

/// The core's software TLB.
///
/// Derived state in the strict sense of `ROADMAP.md` §4.5: never serialized,
/// safe to throw away at any moment, and rebuilt by walking. A restored
/// snapshot comes back with an empty one, which is always correct and never
/// stale.
///
/// # Split by access kind
///
/// Three independent direct-mapped arrays, one per [`AccessKind`], for the
/// reason the RISC-V core's TLB is split the same way: an entry exists only
/// because a walk *for that kind of access* succeeded, so a cached store
/// translation has already passed the write-permission check and a cached fetch
/// has already passed the execute one. A single shared array would have to
/// re-check permissions on every hit, which is most of the walk's cost back
/// again. The tag carries the privilege for the same reason.
///
/// # Two things invalidate it
///
/// [`sync`](Tlb::sync) is handed the MMU's [`Regime::generation`] and the
/// address space's topology generation, and bumps a private epoch when either
/// changes. The first covers `TLBIALL` and every `TTBR`, domain or control
/// write; the second is `CLAUDE.md`'s rule that derived state dies with the
/// topology counter. The epoch is a `u64` counted here rather than the MMU's
/// `u32` counted there, so it cannot wrap in any run that finishes.
#[derive(Debug)]
pub struct Tlb {
    slots: [[Entry; TLB_ENTRIES]; 3],
    /// What the last [`sync`](Tlb::sync) saw, so a change can be detected.
    seen: (u32, u64),
    /// Bumped on every change; mixed into every tag.
    epoch: u64,
    hits: u64,
    misses: u64,
}

impl Default for Tlb {
    fn default() -> Tlb {
        Tlb::new()
    }
}

impl Tlb {
    /// An empty TLB.
    #[must_use]
    pub fn new() -> Tlb {
        Tlb {
            slots: [[Entry::default(); TLB_ENTRIES]; 3],
            seen: (0, 0),
            epoch: 1,
            hits: 0,
            misses: 0,
        }
    }

    /// Throw everything away, unconditionally.
    ///
    /// Used by reset and by a snapshot restore. The ordinary invalidation path
    /// is [`sync`](Tlb::sync), which costs one comparison and touches no entry.
    pub fn flush(&mut self) {
        self.epoch = self.epoch.wrapping_add(1);
        self.hits = 0;
        self.misses = 0;
    }

    /// Note the current MMU generation and address-space topology generation,
    /// invalidating everything if either moved.
    ///
    /// Called once per instruction, which is why it must be this cheap.
    #[inline]
    pub fn sync(&mut self, generation: u32, topology: u64) {
        if self.seen != (generation, topology) {
            self.seen = (generation, topology);
            self.epoch = self.epoch.wrapping_add(1);
        }
    }

    /// How many lookups hit and how many missed, for `rsemu` statistics.
    #[must_use]
    pub fn stats(&self) -> (u64, u64) {
        (self.hits, self.misses)
    }

    /// The tag for a page.
    #[inline]
    fn tag(&self, page: u32, privileged: bool) -> u64 {
        // Both inputs are multiplied rather than shifted into fields. The
        // page number needs it because the low-order bits that select the slot
        // would otherwise be the only bits distinguishing one page from
        // another; the epoch needs it because shifting a `u64` epoch into the
        // top of a `u64` tag throws away everything above the shift, and an
        // epoch that has passed that many invalidations would start colliding
        // with itself — which is a stale entry read as a hit.
        self.epoch.wrapping_mul(0xff51_afd7_ed55_8ccd)
            ^ u64::from(page).wrapping_mul(0x9e37_79b9_7f4a_7c15)
            ^ u64::from(privileged)
    }

    /// Look a page up. `None` is a miss, and the caller walks.
    #[inline]
    pub fn lookup(&mut self, kind: AccessKind, va: Va, privileged: bool) -> Option<Pa> {
        let page = va.0 >> TLB_PAGE_BITS;
        let slot = &self.slots[kind.slot()][(page as usize) & (TLB_ENTRIES - 1)];
        if slot.valid && slot.tag == self.tag(page, privileged) {
            let base = slot.base;
            self.hits += 1;
            Some(Pa(base | (va.0 & ((1 << TLB_PAGE_BITS) - 1))))
        } else {
            self.misses += 1;
            None
        }
    }

    /// Record a translation a walk just produced.
    #[inline]
    pub fn insert(&mut self, kind: AccessKind, va: Va, privileged: bool, pa: Pa) {
        let page = va.0 >> TLB_PAGE_BITS;
        let tag = self.tag(page, privileged);
        self.slots[kind.slot()][(page as usize) & (TLB_ENTRIES - 1)] = Entry {
            tag,
            base: pa.0 & !((1 << TLB_PAGE_BITS) - 1),
            valid: true,
        };
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
        assert!(cp.regime().high_vectors);
        assert!(
            !cp.regime().translating,
            "the stub is honest about having no page tables"
        );
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

    /// Physical memory that refuses everything: a flat MMU never reads it.
    #[derive(Debug)]
    struct NoMem;

    impl PhysMem for NoMem {
        fn read_u32(&self, _at: Pa) -> Option<u32> {
            None
        }
    }

    #[test]
    fn a_flat_mmu_translates_nothing_and_faults_never() {
        let mmu = FlatMmu::new();
        assert_eq!(
            mmu.translate(&NoMem, Va(0xdead_beef), AccessKind::Read, false),
            Ok(Pa(0xdead_beef))
        );
        assert_eq!(mmu.regime(), Regime::FLAT);
    }

    #[test]
    fn a_flat_mmu_carries_the_cores_two_straps() {
        let mmu = FlatMmu {
            high_vectors: true,
            alignment_faults: true,
        };
        let regime = mmu.regime();
        assert!(regime.high_vectors);
        assert!(regime.alignment_faults);
        assert!(!regime.translating, "a strap is not a page table");
    }

    #[test]
    fn fault_status_packs_the_domain_above_the_status() {
        assert_eq!(Fault::EXTERNAL.to_fsr(), 0b1000);
        assert_eq!(Fault::PERMISSION_PAGE.in_domain(3).to_fsr(), 0x3f);
        assert_eq!(Fault::DOMAIN_SECTION.in_domain(15).to_fsr(), 0xf9);
    }

    #[test]
    fn the_tlb_answers_what_it_was_told_and_only_that() {
        let mut tlb = Tlb::new();
        tlb.insert(AccessKind::Read, Va(0x0001_2345), true, Pa(0x8000_0345));

        // The same kibibyte, the same privilege: a hit, with the offset kept.
        assert_eq!(
            tlb.lookup(AccessKind::Read, Va(0x0001_2345), true),
            Some(Pa(0x8000_0345))
        );
        assert_eq!(
            tlb.lookup(AccessKind::Read, Va(0x0001_2000), true),
            Some(Pa(0x8000_0000)),
            "another byte of the same kibibyte translates through the same entry"
        );

        // A different access kind, a different privilege, and the kibibyte
        // next door are all misses: each would need its own walk.
        assert_eq!(tlb.lookup(AccessKind::Write, Va(0x0001_2345), true), None);
        assert_eq!(tlb.lookup(AccessKind::Read, Va(0x0001_2345), false), None);
        assert_eq!(tlb.lookup(AccessKind::Read, Va(0x0001_2800), true), None);
    }

    #[test]
    fn a_tiny_page_does_not_answer_for_its_neighbours() {
        // The bug this granularity exists to prevent: with a 4 KiB key, the
        // second kibibyte of a 4 KiB region would hit the first one's entry.
        let mut tlb = Tlb::new();
        tlb.insert(AccessKind::Read, Va(0x0000_0000), true, Pa(0x1000_0000));
        assert_eq!(tlb.lookup(AccessKind::Read, Va(0x0000_0400), true), None);
    }

    #[test]
    fn either_generation_moving_empties_the_tlb() {
        for (generation, topology) in [(1u32, 0u64), (0, 1)] {
            let mut tlb = Tlb::new();
            tlb.sync(0, 0);
            tlb.insert(AccessKind::Fetch, Va(0x4000), true, Pa(0x9000));
            assert!(tlb.lookup(AccessKind::Fetch, Va(0x4000), true).is_some());

            tlb.sync(generation, topology);
            assert_eq!(
                tlb.lookup(AccessKind::Fetch, Va(0x4000), true),
                None,
                "generation {generation}, topology {topology} left a stale entry"
            );

            // And a sync that changes nothing keeps what is there.
            tlb.insert(AccessKind::Fetch, Va(0x4000), true, Pa(0x9000));
            tlb.sync(generation, topology);
            assert!(tlb.lookup(AccessKind::Fetch, Va(0x4000), true).is_some());
        }
    }

    #[test]
    fn the_tlb_counts_its_hits_and_misses() {
        let mut tlb = Tlb::new();
        assert_eq!(tlb.lookup(AccessKind::Read, Va(0), true), None);
        tlb.insert(AccessKind::Read, Va(0), true, Pa(0));
        assert!(tlb.lookup(AccessKind::Read, Va(0), true).is_some());
        assert_eq!(tlb.stats(), (1, 1));
        tlb.flush();
        assert_eq!(tlb.stats(), (0, 0));
        assert_eq!(tlb.lookup(AccessKind::Read, Va(0), true), None);
    }
}
