//! CP15: the ARMv5 system control coprocessor, and the VMSAv5 table walk.
//!
//! An ARM926EJ-S is a core *plus* this. It owns the MMU — the translation table
//! base, the domain model, the fault registers — the exception vector base, the
//! alignment check, and the identification registers a guest probes to find out
//! what it is running on. Everything a `.machine` file needs to boot code that
//! turns virtual memory on is here.
//!
//! # How a machine file asks for one
//!
//! `cp15 = "arm926ejs"` on the `cpu.arm` object, and nothing else. It is a
//! **construction property of the core**, in the same way the 6502's `variant`
//! is: the property is read by [`Arm::from_props`](super::Arm::from_props),
//! which builds a [`Cp15`] and installs it as both the core's coprocessor 15
//! and its [`Mmu`]. No new plumbing exists for it, and none was needed —
//! `Device::export` did not have to grow a fourth shape, because CP15 is not
//! one device reaching another. It is part of the CPU, and the ARM ARM says so
//! by putting it in the architecture rather than in a SoC's manual.
//!
//! What stays behind the [`Coprocessor`] and [`Mmu`] traits is what genuinely
//! *is* the SoC's: a vendor coprocessor 14, a debug unit, an MMU that is not
//! VMSAv5. The seam did not move; it was simply not the thing standing between
//! a machine file and a page table.
//!
//! # What is modelled and what is not
//!
//! | Register | State |
//! | --- | --- |
//! | c0 identification | main ID, cache type, TCM status |
//! | c1 control | `M`, `A`, `S`, `R` and `V` are live; `B` is seeded from the core's configured byte order and then stored, because which way round a region is read is settled when it is mapped; `C`, `W`, `I`, `Z` and `RR` are stored and inert |
//! | c2 translation table base | live |
//! | c3 domain access control | live, all sixteen domains |
//! | c5 fault status | data and instruction, latched on every abort |
//! | c6 fault address | live |
//! | c7 cache operations | accepted; the caches are not modelled, so they are no-ops that cannot be observed as wrong |
//! | c8 TLB operations | live — they are what invalidates the core's TLB |
//! | c9 lockdown, TCM regions | stored, inert |
//! | c13 FCSE PID | live |
//!
//! **No cache model, no write buffer, no TCMs.** Those are the parts of an
//! ARM926EJ-S that are invisible to a correct program and expensive to model,
//! and `exec`'s timing model already says it evaluates against zero-wait-state
//! memory. A `c7` clean-and-invalidate on a machine with no cache is a no-op
//! that no guest can catch us at; a *lie* would be reporting cache lines dirty.
//! `MRC c7, c10, 3` — "test and clean" — therefore reports the cache already
//! clean, so the standard clean loop terminates on its first pass.
//!
//! # Sources
//!
//! *ARM Architecture Reference Manual* (ARM DDI 0100), part B: B2 "The System
//! Control Coprocessor" for the register map and the c1 bit assignments, B3 for
//! the FCSE, B4.3 for the first- and second-level descriptor formats and the
//! walk, B4.4 for the domain model, B4.5 for the access-permission encodings
//! against the `S` and `R` bits, and B4.6 for the fault-status values and the
//! order the checks happen in. The identification values and the `c7 c10 3`
//! behaviour are from the *ARM926EJ-S Technical Reference Manual* (ARM DDI
//! 0198), chapter 2. No emulator source of any licence was consulted
//! (`ROADMAP.md` §1).

use alloc::fmt;

use crate::core::error::Result;
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU32, Ordering};
use crate::core::value::Endian;

use super::Config;
use super::cp::{
    AccessKind, Coprocessor, CpEffect, CpFault, CpOp, CpResult, Fault, Mmu, Pa, PhysMem, Regime, Va,
};

/// CP15 c1, the control register (ARM ARM B2.1).
///
/// Only the bits an ARMv5 defines are named. A bit this core does not act on
/// is still stored and read back, because a guest that writes the whole
/// register and reads it again is entitled to see what it wrote.
pub mod control {
    /// `M`, bit 0: enable the MMU.
    pub const M: u32 = 1 << 0;
    /// `A`, bit 1: fault on an unaligned access instead of rotating.
    pub const A: u32 = 1 << 1;
    /// `C`, bit 2: enable the data cache. Stored; no cache is modelled.
    pub const C: u32 = 1 << 2;
    /// `W`, bit 3: enable the write buffer. Stored; not modelled.
    pub const W: u32 = 1 << 3;
    /// `B`, bit 7: big-endian memory system.
    ///
    /// Seeded from [`Config::endian`](super::super::Config) and readable, but
    /// **inert**: the core's byte order is a construction property, and a guest
    /// that flips this bit does not change how the next load is assembled. A
    /// bootloader sets it once before it does anything, which is why that has
    /// never mattered; a guest that switched endianness mid-run would need it
    /// to be live.
    pub const B: u32 = 1 << 7;
    /// `S`, bit 8: the system protection bit, read by the `AP == 0b00` rules.
    pub const S: u32 = 1 << 8;
    /// `R`, bit 9: the ROM protection bit, likewise.
    pub const R: u32 = 1 << 9;
    /// `I`, bit 12: enable the instruction cache. Stored; not modelled.
    pub const I: u32 = 1 << 12;
    /// `V`, bit 13: exception vectors at `0xffff0000`.
    pub const V: u32 = 1 << 13;

    /// Bits 4, 5 and 6, which an ARMv5 requires to read as one.
    ///
    /// They were `P`, `D` and `L` on an ARMv3 and are "should be one" from
    /// ARMv4 onwards, which is why a boot sequence that does a read-modify-write
    /// of this register leaves them set without meaning to.
    pub const READ_AS_ONE: u32 = 0b111 << 4;

    /// Which bits a write can change. Everything above bit 15 is reserved on an
    /// ARM926EJ-S and reads as zero.
    pub const WRITABLE: u32 = 0xffff;
}

/// One of the four access-permission outcomes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Allow {
    /// No access at all.
    None,
    /// Reads and instruction fetches.
    Read,
    /// Everything.
    Write,
}

impl Allow {
    /// Whether this permits `kind`.
    ///
    /// An ARMv5 has no execute-never bit, so a fetch needs exactly what a read
    /// needs (ARM ARM B4.5 — the table's columns are "read" and "write").
    const fn permits(self, kind: AccessKind) -> bool {
        match kind {
            AccessKind::Write => matches!(self, Allow::Write),
            _ => matches!(self, Allow::Read | Allow::Write),
        }
    }
}

/// Which level of the table a fault happened at, so it can name itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Level {
    /// A section: the first-level descriptor was the leaf.
    Section,
    /// A page: the walk reached a second-level descriptor.
    Page,
}

impl Level {
    const fn domain_fault(self) -> Fault {
        match self {
            Level::Section => Fault::DOMAIN_SECTION,
            Level::Page => Fault::DOMAIN_PAGE,
        }
    }

    const fn permission_fault(self) -> Fault {
        match self {
            Level::Section => Fault::PERMISSION_SECTION,
            Level::Page => Fault::PERMISSION_PAGE,
        }
    }
}

/// The ARMv5 system control coprocessor.
///
/// Every register is an atomic rather than a field behind a lock. The core
/// samples [`Mmu::regime`] once per instruction and the walk reads two or three
/// of these per miss, so the hot path must not take a lock — and there is no
/// invariant *between* two CP15 registers that a lock would be protecting:
/// each is independently writable by one `MCR`, which is exactly what an
/// atomic gives.
#[derive(Debug)]
pub struct Cp15 {
    main_id: u32,
    cache_type: u32,
    tcm_status: u32,
    /// What [`reset`](Cp15::reset) puts back in c1 — the straps, not zero.
    reset_control: u32,
    control: AtomicU32,
    /// c2: the translation table base.
    ttbr: AtomicU32,
    /// c3: two bits of access per domain, sixteen domains.
    domains: AtomicU32,
    /// c5 opcode 0: the data fault status register.
    dfsr: AtomicU32,
    /// c5 opcode 1: the instruction fault status register.
    ifsr: AtomicU32,
    /// c6: the fault address register.
    far: AtomicU32,
    /// c9: cache lockdown (data, instruction) and TCM region (data,
    /// instruction). Stored so a guest reads back what it wrote; inert.
    c9: [AtomicU32; 4],
    /// c13: the FCSE process identifier, in bits 31..25.
    fcse_pid: AtomicU32,
    /// Bumped by anything that could invalidate a cached translation.
    generation: AtomicU32,
}

impl Cp15 {
    /// The main ID register an ARM926EJ-S reports.
    ///
    /// Implementor `A` (ARM), variant 1, architecture `0b0110` (ARMv5TEJ), part
    /// number `0x926`, revision 5 (ARM926EJ-S TRM 2.3.1).
    pub const ARM926EJS_ID: u32 = 0x4106_9265;

    /// The cache type register for the 16 KiB / 16 KiB ARM926EJ-S.
    ///
    /// Decoded against the ARM ARM B2.4 format: `ctype = 0b1110`, separate
    /// instruction and data caches, and each of them 16 KiB, 4-way associative
    /// with an eight-word (32-byte) line. **No cache is modelled**; this is what
    /// a guest that asks is told, and it is told the truth about the part rather
    /// than about us, because software uses these fields to compute the *stride*
    /// of a maintenance loop and a wrong stride is a wrong loop even when every
    /// operation in it is a no-op.
    pub const ARM926EJS_CACHE_TYPE: u32 = 0x1d15_2152;

    /// A CP15 for an ARM926EJ-S, with `cfg`'s straps as its reset state.
    ///
    /// `high-vectors` becomes c1's `V` bit and `alignment-faults` its `A` bit —
    /// which is what `VINITHI` and the alignment strap do on real silicon: they
    /// set the *reset value*, and software owns the bit afterwards.
    #[must_use]
    pub fn arm926ejs(cfg: &Config) -> Cp15 {
        let mut control = control::READ_AS_ONE;
        if cfg.high_vectors {
            control |= control::V;
        }
        if cfg.alignment_faults {
            control |= control::A;
        }
        if cfg.endian == Endian::Big {
            control |= control::B;
        }
        Cp15 {
            main_id: Cp15::ARM926EJS_ID,
            cache_type: Cp15::ARM926EJS_CACHE_TYPE,
            // No tightly-coupled memory: an ARM926EJ-S built without TCMs
            // reports zero sizes here (ARM926EJ-S TRM 2.3.3).
            tcm_status: 0,
            reset_control: control,
            control: AtomicU32::new(control),
            ttbr: AtomicU32::new(0),
            domains: AtomicU32::new(0),
            dfsr: AtomicU32::new(0),
            ifsr: AtomicU32::new(0),
            far: AtomicU32::new(0),
            c9: [
                AtomicU32::new(0),
                AtomicU32::new(0),
                AtomicU32::new(0),
                AtomicU32::new(0),
            ],
            fcse_pid: AtomicU32::new(0),
            generation: AtomicU32::new(0),
        }
    }

    /// Return every register to its reset value.
    pub fn reset(&self) {
        self.control.store(self.reset_control, Ordering::Release);
        self.ttbr.store(0, Ordering::Release);
        self.domains.store(0, Ordering::Release);
        self.dfsr.store(0, Ordering::Release);
        self.ifsr.store(0, Ordering::Release);
        self.far.store(0, Ordering::Release);
        for reg in &self.c9 {
            reg.store(0, Ordering::Release);
        }
        self.fcse_pid.store(0, Ordering::Release);
        self.invalidate();
    }

    /// c1, the control register.
    #[must_use]
    pub fn control(&self) -> u32 {
        self.control.load(Ordering::Acquire)
    }

    /// c2, the translation table base.
    #[must_use]
    pub fn ttbr(&self) -> u32 {
        self.ttbr.load(Ordering::Acquire)
    }

    /// c3, the domain access control register.
    #[must_use]
    pub fn domains(&self) -> u32 {
        self.domains.load(Ordering::Acquire)
    }

    /// c5 opcode 0 and opcode 1: the data and instruction fault statuses.
    #[must_use]
    pub fn fault_status(&self) -> (u32, u32) {
        (
            self.dfsr.load(Ordering::Acquire),
            self.ifsr.load(Ordering::Acquire),
        )
    }

    /// c6, the fault address register.
    #[must_use]
    pub fn fault_address(&self) -> u32 {
        self.far.load(Ordering::Acquire)
    }

    /// c13, the FCSE process identifier.
    #[must_use]
    pub fn fcse_pid(&self) -> u32 {
        self.fcse_pid.load(Ordering::Acquire)
    }

    /// Whether the MMU is enabled — c1's `M` bit.
    #[must_use]
    pub fn mmu_enabled(&self) -> bool {
        self.control() & control::M != 0
    }

    /// Tell the core's TLB that everything it cached may be wrong.
    fn invalidate(&self) {
        self.generation.fetch_add(1, Ordering::AcqRel);
    }

    /// The modified virtual address `va` becomes, under the FCSE.
    ///
    /// The Fast Context Switch Extension relocates the bottom 32 MiB of the
    /// virtual address space to `PID:0` so that two processes linked at the same
    /// address do not need a TLB flush between them (ARM ARM B3.1). The
    /// register holds the PID in bits 31..25, already shifted, so this is an
    /// `or` rather than a multiply; addresses at or above 32 MiB are untouched.
    ///
    /// It applies whether or not the MMU is enabled, because B2.1's pipeline is
    /// VA → (FCSE) → MVA → (MMU) → PA and disabling the MMU only makes the
    /// second arrow an identity. With the reset PID of zero it is an identity
    /// too, which is every guest that has never heard of the FCSE.
    #[inline]
    fn mva(&self, va: Va) -> u32 {
        let pid = self.fcse_pid.load(Ordering::Acquire) & 0xfe00_0000;
        if pid != 0 && va.0 < 0x0200_0000 {
            va.0 | pid
        } else {
            va.0
        }
    }

    /// Whether domain `domain` permits this access without a permission check,
    /// needs one, or refuses outright (ARM ARM B4.4).
    fn domain_check(
        &self,
        domain: u8,
        ap: u32,
        kind: AccessKind,
        privileged: bool,
        level: Level,
    ) -> core::result::Result<(), Fault> {
        let access = (self.domains() >> (2 * u32::from(domain))) & 0b11;
        match access {
            // Manager: no permission check at all.
            0b11 => Ok(()),
            // Client: the descriptor's access permissions decide.
            0b01 => {
                if self.permits(ap, kind, privileged) {
                    Ok(())
                } else {
                    Err(level.permission_fault().in_domain(domain))
                }
            }
            // 0b00 is "no access". 0b10 is reserved and UNPREDICTABLE; treating
            // it as no access is the safe reading, and it is the one that makes
            // a guest notice it wrote a reserved value.
            _ => Err(level.domain_fault().in_domain(domain)),
        }
    }

    /// Whether a two-bit `AP` field permits this access (ARM ARM B4.5).
    fn permits(&self, ap: u32, kind: AccessKind, privileged: bool) -> bool {
        let control = self.control();
        let (privileged_allow, user_allow) = match ap & 0b11 {
            // `AP == 0b00` defers to the `S` and `R` bits in c1, which is the
            // whole reason those bits exist: they change the meaning of every
            // descriptor in the tables at once, which is how an OS makes its
            // entire address space read-only for a moment.
            0b00 => match (control & control::S != 0, control & control::R != 0) {
                (false, false) => (Allow::None, Allow::None),
                (true, false) => (Allow::Read, Allow::None),
                (false, true) => (Allow::Read, Allow::Read),
                // S and R both set is UNPREDICTABLE. Refusing is the reading
                // that cannot silently let a guest through a check it meant to
                // fail.
                (true, true) => (Allow::None, Allow::None),
            },
            0b01 => (Allow::Write, Allow::None),
            0b10 => (Allow::Write, Allow::Read),
            _ => (Allow::Write, Allow::Write),
        };
        let allow = if privileged {
            privileged_allow
        } else {
            user_allow
        };
        allow.permits(kind)
    }

    /// Decode a second-level descriptor and finish the translation.
    fn second_level(
        &self,
        descriptor: u32,
        mva: u32,
        domain: u8,
        kind: AccessKind,
        privileged: bool,
    ) -> core::result::Result<Pa, Fault> {
        // The four `AP` fields sit at bits 11..4, two bits each. Which one
        // applies depends on the page size, because they divide the page into
        // four equal subpages (ARM ARM B4.3.2).
        let subpage = |index: u32| (descriptor >> (4 + 2 * index)) & 0b11;
        let (ap, base, offset_mask) = match descriptor & 0b11 {
            // Nothing is mapped in this page. The domain travels with it: the
            // first-level descriptor was valid, so unlike a section
            // translation fault this one has a domain to name (ARM ARM B4.6 —
            // the `FSR` domain field is UNPREDICTABLE only for an alignment
            // fault and for the faults that happen at the first level).
            0b00 => return Err(Fault::TRANSLATION_PAGE.in_domain(domain)),
            // A large page: 64 KiB, four 16 KiB subpages.
            0b01 => (
                subpage((mva >> 14) & 0b11),
                descriptor & 0xffff_0000,
                0x0000_ffff,
            ),
            // A small page: 4 KiB, four 1 KiB subpages.
            0b10 => (
                subpage((mva >> 10) & 0b11),
                descriptor & 0xffff_f000,
                0x0000_0fff,
            ),
            // A tiny page: 1 KiB, and the only one with a single `AP` field —
            // there is nothing left to subdivide. The architecture defines it
            // only inside a *fine* second-level table; one found in a coarse
            // table is UNPREDICTABLE and is decoded here rather than refused,
            // because refusing would invent a fault the manual does not name.
            _ => (subpage(0), descriptor & 0xffff_fc00, 0x0000_03ff),
        };
        self.domain_check(domain, ap, kind, privileged, Level::Page)?;
        Ok(Pa(base | (mva & offset_mask)))
    }
}

impl Coprocessor for Cp15 {
    fn mrc(&self, op: CpOp) -> CpResult<u32> {
        if op.cp != 15 || op.opc1 != 0 {
            return Err(CpFault::Undefined);
        }
        Ok(match (op.crn, op.crm, op.opc2) {
            (0, 0, 1) => self.cache_type,
            (0, 0, 2) => self.tcm_status,
            // Every other opcode in c0 c0 reads the main ID; the architecture
            // says an unimplemented identification register returns it rather
            // than zero, so that software probing for one does not conclude the
            // part has no ID at all (ARM ARM B2.3).
            (0, 0, _) => self.main_id,
            (1, 0, 0) => self.control(),
            (2, 0, 0) => self.ttbr(),
            (3, 0, 0) => self.domains(),
            (5, 0, 0) => self.dfsr.load(Ordering::Acquire),
            (5, 0, 1) => self.ifsr.load(Ordering::Acquire),
            (6, 0, 0) => self.far.load(Ordering::Acquire),
            // "Test and clean the data cache", and its clean-and-invalidate
            // sibling. Both report their result in the `Z` flag, and the
            // idiomatic loop is `1: mrc p15,0,r15,c7,c10,3; bne 1b`. With no
            // cache there is never anything left dirty, so `Z` is set on the
            // first pass and the loop exits (ARM926EJ-S TRM 2.3.9).
            (7, 10 | 14, 3) => 0x4000_0000,
            (9, 0, 0) => self.c9[0].load(Ordering::Acquire),
            (9, 0, 1) => self.c9[1].load(Ordering::Acquire),
            (9, 1, 0) => self.c9[2].load(Ordering::Acquire),
            (9, 1, 1) => self.c9[3].load(Ordering::Acquire),
            (13, 0, 0) => self.fcse_pid(),
            // Everything else reads as zero. The architecture calls this
            // UNPREDICTABLE and real parts answer with whatever the bus last
            // carried; zero is the deterministic choice, and an Undefined
            // Instruction exception in the middle of a boot sequence probing
            // for a feature would be worse than a quiet zero.
            _ => 0,
        })
    }

    fn mcr(&self, op: CpOp, value: u32) -> CpResult<CpEffect> {
        if op.cp != 15 || op.opc1 != 0 {
            return Err(CpFault::Undefined);
        }
        match (op.crn, op.crm, op.opc2) {
            (1, 0, 0) => {
                self.control.store(
                    (value & control::WRITABLE) | control::READ_AS_ONE,
                    Ordering::Release,
                );
                // `M`, `S`, `R` and `A` all change what a cached translation
                // would have decided, and the `V` bit moves the vectors.
                self.invalidate();
            }
            (2, 0, 0) => {
                // Bits 13..0 of the base are "should be zero" — the table is
                // 16 KiB aligned — and masking them here means a guest that
                // leaves rubbish in them still gets the table it meant.
                self.ttbr.store(value & 0xffff_c000, Ordering::Release);
                self.invalidate();
            }
            (3, 0, 0) => {
                self.domains.store(value, Ordering::Release);
                self.invalidate();
            }
            (5, 0, 0) => self.dfsr.store(value & 0xff, Ordering::Release),
            (5, 0, 1) => self.ifsr.store(value & 0xff, Ordering::Release),
            (6, 0, 0) => self.far.store(value, Ordering::Release),
            // c7 c0 4 is "wait for interrupt" on every ARM9 part that has one.
            (7, 0, 4) => return Ok(CpEffect::HALT),
            // Every other c7 operation is cache or write-buffer maintenance.
            // Accepted and ignored: see the module documentation on why a
            // no-op is honest here and a lie would not be.
            (7, _, _) => {}
            // c8 is the TLB. Invalidate-all, invalidate-by-entry, and the
            // instruction- and data-only forms all land here; the core has one
            // unified TLB, so all of them empty all of it. Over-invalidating is
            // architecturally free — a TLB is allowed to lose an entry at any
            // time — where under-invalidating is a stale mapping.
            (8, _, _) => self.invalidate(),
            (9, 0, 0) => self.c9[0].store(value, Ordering::Release),
            (9, 0, 1) => self.c9[1].store(value, Ordering::Release),
            (9, 1, 0) => self.c9[2].store(value, Ordering::Release),
            (9, 1, 1) => self.c9[3].store(value, Ordering::Release),
            (13, 0, 0) => {
                self.fcse_pid.store(value & 0xfe00_0000, Ordering::Release);
                self.invalidate();
            }
            _ => {}
        }
        Ok(CpEffect::NONE)
    }
}

impl Mmu for Cp15 {
    fn regime(&self) -> Regime {
        let control = self.control();
        Regime {
            generation: self.generation.load(Ordering::Acquire),
            // The FCSE relocates addresses with the MMU off as well, so it
            // counts as translating even though no table is walked.
            translating: control & control::M != 0 || self.fcse_pid() != 0,
            high_vectors: control & control::V != 0,
            alignment_faults: control & control::A != 0,
        }
    }

    /// The VMSAv5 walk: at most two descriptor reads, in the order the manual
    /// checks them (ARM ARM B4.3).
    ///
    /// The order is the whole content of the fault-status register. A guest's
    /// abort handler reads `FSR` and decides between "grow the stack", "copy on
    /// write" and "kill the process" from that value alone, so a translation
    /// fault reported where the manual calls for a domain fault sends a kernel
    /// down the wrong branch — which is why the checks below are written out one
    /// at a time rather than folded together.
    fn translate(
        &self,
        mem: &dyn PhysMem,
        va: Va,
        kind: AccessKind,
        privileged: bool,
    ) -> core::result::Result<Pa, Fault> {
        let mva = self.mva(va);
        if self.control() & control::M == 0 {
            // The FCSE brought us here; there is no table to walk.
            return Ok(Pa(mva));
        }

        // The first-level table is 4096 entries indexed by the top twelve bits
        // of the address, so it is 16 KiB and `TTBR` is 16 KiB aligned.
        let first = self.ttbr() | ((mva >> 20) << 2);
        let descriptor = mem.read_u32(Pa(first)).ok_or(Fault::EXTERNAL_L1)?;
        let domain = ((descriptor >> 5) & 0xf) as u8;

        match descriptor & 0b11 {
            // Nothing is mapped in this megabyte.
            0b00 => Err(Fault::TRANSLATION_SECTION),
            // A coarse second-level table: 256 entries of 4 KiB each, so it is
            // 1 KiB and 1 KiB aligned, indexed by bits 19..12.
            0b01 => {
                let second = (descriptor & 0xffff_fc00) | (((mva >> 12) & 0xff) << 2);
                let entry = mem
                    .read_u32(Pa(second))
                    .ok_or_else(|| Fault::EXTERNAL_L2.in_domain(domain))?;
                self.second_level(entry, mva, domain, kind, privileged)
            }
            // A section: one megabyte, no second level.
            0b10 => {
                let ap = (descriptor >> 10) & 0b11;
                self.domain_check(domain, ap, kind, privileged, Level::Section)?;
                Ok(Pa((descriptor & 0xfff0_0000) | (mva & 0x000f_ffff)))
            }
            // A fine second-level table: 1024 entries of 1 KiB each, so it is
            // 4 KiB and 4 KiB aligned, indexed by bits 19..10.
            _ => {
                let second = (descriptor & 0xffff_f000) | (((mva >> 10) & 0x3ff) << 2);
                let entry = mem
                    .read_u32(Pa(second))
                    .ok_or_else(|| Fault::EXTERNAL_L2.in_domain(domain))?;
                self.second_level(entry, mva, domain, kind, privileged)
            }
        }
    }

    fn report_abort(&self, va: Va, fault: Fault, kind: AccessKind) {
        if kind.is_fetch() {
            // An ARM926EJ-S has an instruction fault *status* register
            // (c5 with opcode_2 = 1, ARM926EJ-S TRM 2.3.5) and no instruction
            // fault *address* register: a prefetch abort handler recovers the
            // address from `R14_abt`, which is why one was never needed.
            self.ifsr.store(fault.to_fsr(), Ordering::Release);
        } else {
            self.dfsr.store(fault.to_fsr(), Ordering::Release);
            // The FAR holds the *modified* virtual address, which is what the
            // MMU saw.
            self.far.store(self.mva(va), Ordering::Release);
        }
    }
}

impl Cp15 {
    /// Write the architectural registers into a snapshot.
    ///
    /// The generation counter is **not** written: it exists only to invalidate
    /// the core's TLB, which is derived state that a restore throws away
    /// anyway, and a counter in the chunk would make two identical machines
    /// hash differently for having taken different numbers of TLB flushes to
    /// get there.
    ///
    /// # Errors
    ///
    /// If the sink refuses a write.
    pub fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        w.write_u32(self.control())?;
        w.write_u32(self.ttbr())?;
        w.write_u32(self.domains())?;
        w.write_u32(self.dfsr.load(Ordering::Acquire))?;
        w.write_u32(self.ifsr.load(Ordering::Acquire))?;
        w.write_u32(self.far.load(Ordering::Acquire))?;
        for reg in &self.c9 {
            w.write_u32(reg.load(Ordering::Acquire))?;
        }
        w.write_u32(self.fcse_pid())?;
        Ok(())
    }

    /// Restore what [`save`](Cp15::save) wrote.
    ///
    /// # Errors
    ///
    /// If the chunk is short.
    pub fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        self.control
            .store(r.read_u32()? | control::READ_AS_ONE, Ordering::Release);
        self.ttbr.store(r.read_u32()?, Ordering::Release);
        self.domains.store(r.read_u32()?, Ordering::Release);
        self.dfsr.store(r.read_u32()?, Ordering::Release);
        self.ifsr.store(r.read_u32()?, Ordering::Release);
        self.far.store(r.read_u32()?, Ordering::Release);
        for reg in &self.c9 {
            reg.store(r.read_u32()?, Ordering::Release);
        }
        self.fcse_pid.store(r.read_u32()?, Ordering::Release);
        self.invalidate();
        Ok(())
    }
}

impl fmt::Display for Cp15 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let (dfsr, ifsr) = self.fault_status();
        write!(
            f,
            "cp15 c1={:#010x} c2={:#010x} c3={:#010x} dfsr={dfsr:#04x} ifsr={ifsr:#04x} \
             far={:#010x}",
            self.control(),
            self.ttbr(),
            self.domains(),
            self.fault_address(),
        )
    }
}

#[cfg(test)]
mod tests {
    use alloc::vec;
    use alloc::vec::Vec;

    use super::*;

    /// A flat physical memory for the walker to read descriptors out of.
    #[derive(Debug)]
    struct Ram(Vec<u32>);

    impl Ram {
        fn new() -> Ram {
            // 64 KiB of words, which is room for a 16 KiB first-level table at
            // zero and several second-level tables above it.
            Ram(vec![0; 0x1_0000])
        }

        fn put(&mut self, at: u32, value: u32) {
            self.0[(at / 4) as usize] = value;
        }
    }

    impl PhysMem for Ram {
        fn read_u32(&self, at: Pa) -> Option<u32> {
            self.0.get((at.0 / 4) as usize).copied()
        }
    }

    fn cp15() -> Cp15 {
        Cp15::arm926ejs(&Config::ARM926EJS)
    }

    fn op(crn: u8, crm: u8, opc2: u8) -> CpOp {
        CpOp {
            cp: 15,
            opc1: 0,
            crd: 0,
            crn,
            crm,
            opc2,
        }
    }

    /// Turn the MMU on with the table at `ttbr` and every domain a client.
    fn enable(cp: &Cp15, ttbr: u32) {
        cp.mcr(op(2, 0, 0), ttbr).unwrap();
        cp.mcr(op(3, 0, 0), 0x5555_5555).unwrap();
        cp.mcr(op(1, 0, 0), cp.control() | control::M).unwrap();
    }

    #[test]
    fn it_identifies_itself_as_an_arm926ejs() {
        let cp = cp15();
        assert_eq!(cp.mrc(op(0, 0, 0)), Ok(Cp15::ARM926EJS_ID));
        assert_eq!(cp.mrc(op(0, 0, 1)), Ok(Cp15::ARM926EJS_CACHE_TYPE));
        assert_eq!(cp.mrc(op(0, 0, 2)), Ok(0));
        // Another coprocessor number is not this one's business.
        assert_eq!(
            cp.mrc(CpOp {
                cp: 14,
                ..op(0, 0, 0)
            }),
            Err(CpFault::Undefined)
        );
    }

    #[test]
    fn the_control_register_keeps_its_should_be_one_bits() {
        let cp = cp15();
        assert_eq!(cp.control(), control::READ_AS_ONE);
        cp.mcr(op(1, 0, 0), 0).unwrap();
        assert_eq!(
            cp.control(),
            control::READ_AS_ONE,
            "writing zero must not clear bits the architecture reads as one"
        );
        cp.mcr(op(1, 0, 0), 0xffff_ffff).unwrap();
        assert_eq!(
            cp.control(),
            control::WRITABLE,
            "the reserved half of the register reads as zero"
        );
    }

    #[test]
    fn the_straps_become_the_reset_value_of_c1() {
        let cfg = Config {
            high_vectors: true,
            alignment_faults: true,
            endian: Endian::Big,
            ..Config::ARM926EJS
        };
        let cp = Cp15::arm926ejs(&cfg);
        assert_eq!(
            cp.control(),
            control::READ_AS_ONE | control::V | control::A | control::B
        );
        let regime = cp.regime();
        assert!(regime.high_vectors);
        assert!(regime.alignment_faults);
        assert!(!regime.translating, "a strap does not enable the MMU");

        // And software owns them afterwards, which a strap ORed in from the
        // side would prevent.
        cp.mcr(op(1, 0, 0), cp.control() & !control::V).unwrap();
        assert!(!cp.regime().high_vectors);
    }

    #[test]
    fn a_section_translates_and_keeps_the_offset() {
        let mut ram = Ram::new();
        // Virtual megabyte 1 -> physical megabyte 0x40, AP = 0b11, domain 0.
        ram.put(4, 0x0400_0000 | (0b11 << 10) | 0b10);
        let cp = cp15();
        enable(&cp, 0);

        assert_eq!(
            cp.translate(&ram, Va(0x0010_1234), AccessKind::Read, false),
            Ok(Pa(0x0400_1234))
        );
    }

    #[test]
    fn an_unmapped_megabyte_is_a_section_translation_fault() {
        let ram = Ram::new();
        let cp = cp15();
        enable(&cp, 0);
        assert_eq!(
            cp.translate(&ram, Va(0x0010_0000), AccessKind::Read, false),
            Err(Fault::TRANSLATION_SECTION)
        );
    }

    #[test]
    fn a_small_page_translates_through_a_coarse_table() {
        let mut ram = Ram::new();
        // First level: virtual megabyte 2 -> a coarse table at 0x8000,
        // domain 3.
        ram.put(8, 0x0000_8000 | (3 << 5) | 0b01);
        // Second level: virtual page 5 of that megabyte -> physical 0x2000,
        // every subpage read/write.
        ram.put(0x8000 + 5 * 4, 0x0000_2000 | 0xff0 | 0b10);
        let cp = cp15();
        enable(&cp, 0);

        assert_eq!(
            cp.translate(&ram, Va(0x0020_5abc), AccessKind::Write, true),
            Ok(Pa(0x0000_2abc))
        );
        // The page next door is not mapped, and the fault names the domain
        // the first-level descriptor put it in.
        assert_eq!(
            cp.translate(&ram, Va(0x0020_6000), AccessKind::Read, true),
            Err(Fault::TRANSLATION_PAGE.in_domain(3))
        );
    }

    #[test]
    fn a_large_page_selects_its_subpage_from_bits_15_and_14() {
        let mut ram = Ram::new();
        ram.put(0, 0x0000_8000 | 0b01);
        // A 64 KiB page at physical 0x1_0000, with subpage 0 read-only for
        // everyone (AP 0b10 -> privileged write, user read) and subpage 1
        // read/write.
        let descriptor = 0x0001_0000 | (0b10 << 4) | (0b11 << 6) | 0b01;
        // A large page occupies sixteen consecutive coarse-table entries.
        for index in 0..16 {
            ram.put(0x8000 + index * 4, descriptor);
        }
        let cp = cp15();
        enable(&cp, 0);

        // Subpage 0: a user write is refused, a user read is not.
        assert_eq!(
            cp.translate(&ram, Va(0x0000_0004), AccessKind::Read, false),
            Ok(Pa(0x0001_0004))
        );
        assert_eq!(
            cp.translate(&ram, Va(0x0000_0004), AccessKind::Write, false),
            Err(Fault::PERMISSION_PAGE)
        );
        // Subpage 1 starts 16 KiB in and permits both.
        assert_eq!(
            cp.translate(&ram, Va(0x0000_4004), AccessKind::Write, false),
            Ok(Pa(0x0001_4004))
        );
    }

    #[test]
    fn a_tiny_page_has_one_permission_field_and_a_kibibyte_of_reach() {
        let mut ram = Ram::new();
        // A fine second-level table at 0x9000.
        ram.put(0, 0x0000_9000 | 0b11);
        // Tiny page index 2 of that megabyte -> physical 0x3000, AP 0b01.
        ram.put(0x9000 + 2 * 4, 0x0000_3000 | (0b01 << 4) | 0b11);
        let cp = cp15();
        enable(&cp, 0);

        assert_eq!(
            cp.translate(&ram, Va(0x0000_0801), AccessKind::Write, true),
            Ok(Pa(0x0000_3001))
        );
        assert_eq!(
            cp.translate(&ram, Va(0x0000_0801), AccessKind::Read, false),
            Err(Fault::PERMISSION_PAGE),
            "AP 0b01 gives an unprivileged access nothing"
        );
        // The kibibyte above it is a different entry, and is empty.
        assert_eq!(
            cp.translate(&ram, Va(0x0000_0c00), AccessKind::Read, true),
            Err(Fault::TRANSLATION_PAGE)
        );
    }

    #[test]
    fn a_domain_with_no_access_refuses_what_its_permissions_would_allow() {
        let mut ram = Ram::new();
        // Domain 5, AP 0b11 — the descriptor permits everything.
        ram.put(0, (0b11 << 10) | (5 << 5) | 0b10);
        let cp = cp15();
        enable(&cp, 0);
        // Domain 5 is a client: permitted.
        assert!(cp.translate(&ram, Va(0), AccessKind::Write, true).is_ok());

        // Domain 5 to "no access": refused, and the fault names the domain.
        cp.mcr(op(3, 0, 0), 0x5555_5555 & !(0b11 << 10)).unwrap();
        assert_eq!(
            cp.translate(&ram, Va(0), AccessKind::Write, true),
            Err(Fault::DOMAIN_SECTION.in_domain(5))
        );

        // And a manager skips the permission check entirely, which is how a
        // domain with AP 0b00 is still reachable.
        cp.mcr(op(3, 0, 0), 0xffff_ffff).unwrap();
        // The same section in domain 5, but with `AP == 0b00`.
        ram.put(0, (5 << 5) | 0b10);
        assert!(cp.translate(&ram, Va(0), AccessKind::Write, false).is_ok());
    }

    #[test]
    fn the_s_and_r_bits_reinterpret_every_ap_zero_descriptor() {
        let mut ram = Ram::new();
        // A section in domain 0 with `AP == 0b00`, whose meaning is
        // whatever `S` and `R` say it is.
        ram.put(0, 0b10);
        let cp = cp15();
        enable(&cp, 0);

        // Neither: nobody may touch it.
        assert!(cp.translate(&ram, Va(0), AccessKind::Read, true).is_err());

        // S alone: privileged read.
        cp.mcr(op(1, 0, 0), cp.control() | control::S).unwrap();
        assert!(cp.translate(&ram, Va(0), AccessKind::Read, true).is_ok());
        assert!(cp.translate(&ram, Va(0), AccessKind::Read, false).is_err());
        assert!(cp.translate(&ram, Va(0), AccessKind::Write, true).is_err());

        // R alone: everybody reads.
        cp.mcr(op(1, 0, 0), (cp.control() & !control::S) | control::R)
            .unwrap();
        assert!(cp.translate(&ram, Va(0), AccessKind::Read, false).is_ok());
        assert!(cp.translate(&ram, Va(0), AccessKind::Write, true).is_err());

        // Both is UNPREDICTABLE, and we refuse.
        cp.mcr(op(1, 0, 0), cp.control() | control::S).unwrap();
        assert!(cp.translate(&ram, Va(0), AccessKind::Read, true).is_err());
    }

    #[test]
    fn a_table_the_bus_refuses_is_an_external_abort_naming_its_level() {
        let mut ram = Ram::new();
        let cp = cp15();
        // A first-level table off the end of memory.
        enable(&cp, 0xffff_c000);
        assert_eq!(
            cp.translate(&ram, Va(0), AccessKind::Read, true),
            Err(Fault::EXTERNAL_L1)
        );

        // And a first-level descriptor pointing at a second-level table that
        // is off the end, in domain 7.
        enable(&cp, 0);
        ram.put(0, 0xfff0_0000 | (7 << 5) | 0b01);
        assert_eq!(
            cp.translate(&ram, Va(0), AccessKind::Read, true),
            Err(Fault::EXTERNAL_L2.in_domain(7))
        );
    }

    #[test]
    fn the_fcse_relocates_the_bottom_thirty_two_megabytes() {
        let ram = Ram::new();
        let cp = cp15();
        assert!(!cp.regime().translating, "pid 0 translates nothing");

        cp.mcr(op(13, 0, 0), 0x0400_0000).unwrap();
        assert_eq!(cp.fcse_pid(), 0x0400_0000);
        assert!(
            cp.regime().translating,
            "a non-zero pid relocates even with the MMU off"
        );
        // Below 32 MiB: relocated. At or above: untouched.
        assert_eq!(
            cp.translate(&ram, Va(0x0000_1000), AccessKind::Read, true),
            Ok(Pa(0x0400_1000))
        );
        assert_eq!(
            cp.translate(&ram, Va(0x0200_0000), AccessKind::Read, true),
            Ok(Pa(0x0200_0000))
        );
    }

    #[test]
    fn every_invalidating_write_moves_the_generation() {
        let cp = cp15();
        let mut last = cp.regime().generation;
        for (crn, crm, opc2, value) in [
            (1u8, 0u8, 0u8, 1u32),
            (2, 0, 0, 0x4000),
            (3, 0, 0, 1),
            (8, 7, 0, 0),
            (8, 5, 0, 0),
            (8, 6, 1, 0),
            (13, 0, 0, 0x0200_0000),
        ] {
            cp.mcr(op(crn, crm, opc2), value).unwrap();
            let now = cp.regime().generation;
            assert_ne!(now, last, "c{crn} c{crm} {opc2} did not invalidate");
            last = now;
        }
        // And a write that changes no translation does not.
        cp.mcr(op(9, 0, 0), 0xffff).unwrap();
        assert_eq!(cp.regime().generation, last);
    }

    #[test]
    fn an_abort_latches_the_status_and_the_address() {
        let cp = cp15();
        cp.report_abort(
            Va(0xdead_beef),
            Fault::PERMISSION_PAGE.in_domain(2),
            AccessKind::Write,
        );
        assert_eq!(cp.fault_status().0, 0x2f);
        assert_eq!(cp.fault_address(), 0xdead_beef);

        // A prefetch abort takes the other status register and leaves the
        // address alone: ARMv5 has no instruction fault address register.
        cp.report_abort(Va(0x1000), Fault::TRANSLATION_SECTION, AccessKind::Fetch);
        assert_eq!(cp.fault_status(), (0x2f, 0x05));
        assert_eq!(cp.fault_address(), 0xdead_beef);
    }

    #[test]
    fn wait_for_interrupt_asks_the_core_to_halt() {
        assert_eq!(cp15().mcr(op(7, 0, 4), 0), Ok(CpEffect::HALT));
    }

    #[test]
    fn test_and_clean_reports_the_cache_already_clean() {
        // Otherwise `1: mrc p15,0,r15,c7,c10,3; bne 1b` never returns.
        let cp = cp15();
        assert_eq!(cp.mrc(op(7, 10, 3)), Ok(0x4000_0000));
        assert_eq!(cp.mrc(op(7, 14, 3)), Ok(0x4000_0000));
    }

    #[test]
    fn reset_puts_the_straps_back_and_nothing_else() {
        let cp = cp15();
        enable(&cp, 0x1_0000);
        cp.mcr(op(13, 0, 0), 0x0200_0000).unwrap();
        cp.report_abort(Va(1), Fault::EXTERNAL, AccessKind::Read);

        cp.reset();
        assert_eq!(cp.control(), control::READ_AS_ONE);
        assert_eq!(cp.ttbr(), 0);
        assert_eq!(cp.domains(), 0);
        assert_eq!(cp.fcse_pid(), 0);
        assert_eq!(cp.fault_status(), (0, 0));
        assert_eq!(cp.fault_address(), 0);
        assert!(!cp.mmu_enabled());
    }
}
