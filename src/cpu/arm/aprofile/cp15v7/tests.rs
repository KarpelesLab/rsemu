//! Tests for the ARMv7 CP15 and the VMSAv7 short-descriptor walk.
//!
//! The unit tests drive [`Cp15v7`] through its traits with a word-array
//! physical memory; the end-to-end ones build a Cortex-A9 [`Arm`] and run
//! hand-assembled guest code (encodings from `arm-none-eabi-as -march=armv7-a`,
//! written as words with the source in a comment, as `aprofile/tests.rs`
//! does).

use alloc::vec;
use alloc::vec::Vec;

use super::*;
use crate::core::device::{Device, ResetKind};
use crate::core::space::{AddressSpace, MemAttrs, RamStore, Region};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::value::Width;
use crate::cpu::arm::aprofile::{Arm, CLASS, Mode, System};

// ---------------------------------------------------------------------------
// Unit harness
// ---------------------------------------------------------------------------

/// A flat physical memory for the walker to read descriptors out of.
#[derive(Debug)]
struct Ram(Vec<u32>);

impl Ram {
    fn new() -> Ram {
        // 128 KiB: a 16 KiB first-level table at 0x4000, a second one at
        // 0x8000 for TTBR1, and second-level tables above.
        Ram(vec![0; 0x8000])
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

const L1: u32 = 0x4000;
const L1_HIGH: u32 = 0x8000;
const L2: u32 = 0xc000;

fn cp() -> Cp15v7 {
    Cp15v7::cortex_a9(&Config::CORTEX_A9)
}

fn op(opc1: u8, crn: u8, crm: u8, opc2: u8) -> CpOp {
    CpOp {
        cp: 15,
        opc1,
        crd: 0,
        crn,
        crm,
        opc2,
        privileged: true,
    }
}

fn user(op: CpOp) -> CpOp {
    CpOp {
        privileged: false,
        ..op
    }
}

const SCTLR: CpOp = CpOp {
    cp: 15,
    opc1: 0,
    crd: 0,
    crn: 1,
    crm: 0,
    opc2: 0,
    privileged: true,
};

/// A section descriptor: `pa`'s megabyte, `AP[2:0]`, domain, XN.
fn section(pa: u32, ap: u32, domain: u32, xn: bool) -> u32 {
    (pa & 0xfff0_0000)
        | (((ap >> 2) & 1) << 15)
        | ((ap & 3) << 10)
        | (domain << 5)
        | (u32::from(xn) << 4)
        | 0b10
}

/// A small-page descriptor.
fn small(pa: u32, ap: u32, xn: bool) -> u32 {
    (pa & 0xffff_f000) | (((ap >> 2) & 1) << 9) | ((ap & 3) << 4) | 0b10 | u32::from(xn)
}

/// TTBR0 at `L1`, every domain a client, the MMU on.
fn enable(cp: &Cp15v7) {
    cp.mcr(op(0, 2, 0, 0), L1).unwrap();
    cp.mcr(op(0, 3, 0, 0), 0x5555_5555).unwrap();
    cp.mcr(SCTLR, cp.sctlr() | sctlr::M).unwrap();
}

fn t(
    cp: &Cp15v7,
    ram: &Ram,
    va: u32,
    kind: AccessKind,
    privileged: bool,
) -> core::result::Result<u32, Fault> {
    cp.translate(ram, Va(va), kind, privileged).map(|pa| pa.0)
}

// ---------------------------------------------------------------------------
// Identification and the plain registers
// ---------------------------------------------------------------------------

#[test]
fn it_identifies_itself_as_a_cortex_a9() {
    let cp = cp();
    assert_eq!(cp.mrc(op(0, 0, 0, 0)), Ok(0x413f_c090));
    // Unimplemented c0 c0 slots alias MIDR.
    assert_eq!(cp.mrc(op(0, 0, 0, 4)), Ok(0x413f_c090));
    assert_eq!(cp.mrc(op(0, 0, 0, 1)), Ok(id::CTR));
    assert_eq!(cp.mrc(op(0, 0, 0, 2)), Ok(0), "no TCMs");
    assert_eq!(cp.mrc(op(0, 0, 0, 3)), Ok(0), "a unified TLB");
    assert_eq!(
        cp.mrc(op(0, 0, 0, 5)),
        Ok(0x8000_0000),
        "cpu 0 of cluster 0"
    );
    assert_eq!(cp.mrc(op(0, 0, 1, 4)), Ok(id::ID_MMFR0));
    assert_eq!(
        cp.mrc(op(0, 0, 2, 0)).unwrap() & 0x0f00_0000,
        0,
        "no divider"
    );
    // The reserved feature-ID space reads as zero rather than trapping.
    assert_eq!(cp.mrc(op(0, 0, 5, 3)), Ok(0));
    assert_eq!(cp.mrc(op(1, 0, 0, 1)), Ok(id::CLIDR));
    // And the ID registers refuse a write.
    assert_eq!(cp.mcr(op(0, 0, 0, 0), 0), Err(CpFault::Undefined));
    assert_eq!(cp.mcr(op(0, 0, 1, 0), 0), Err(CpFault::Undefined));
}

#[test]
fn csselr_picks_which_cache_ccsidr_describes() {
    let cp = cp();
    cp.mcr(op(2, 0, 0, 0), 0).unwrap();
    let data = cp.mrc(op(1, 0, 0, 0)).unwrap();
    // 32-byte lines, four ways, 256 sets: 32 KiB.
    assert_eq!(data & 7, 1);
    assert_eq!((data >> 3) & 0x3ff, 3);
    assert_eq!((data >> 13) & 0x7fff, 255);
    cp.mcr(op(2, 0, 0, 0), 1).unwrap();
    assert_eq!(cp.mrc(op(1, 0, 0, 0)), Ok(id::CCSIDR_INSN));
    // CLIDR's LoC is 1, so a set/way loop only ever asks about level 0.
    assert_eq!((id::CLIDR >> 24) & 7, 1);
}

#[test]
fn mpidr_and_cbar_come_from_the_construction_properties() {
    let cfg = Config {
        cpu_id: 2,
        cluster_id: 1,
        periphbase: 0xf000_0000,
        ..Config::CORTEX_A9
    };
    let cp = Cp15v7::cortex_a9(&cfg);
    assert_eq!(cp.mrc(op(0, 0, 0, 5)), Ok(0x8000_0102));
    assert_eq!(cp.mrc(op(4, 15, 0, 0)), Ok(0xf000_0000));
    assert_eq!(
        cp.mcr(op(4, 15, 0, 0), 0),
        Err(CpFault::Undefined),
        "CBAR is read-only"
    );
}

#[test]
fn sctlr_resets_to_the_a9_value_with_the_straps_and_keeps_its_fixed_bits() {
    let cp = cp();
    assert_eq!(cp.sctlr(), 0x00c5_0078);
    let strapped = Cp15v7::cortex_a9(&Config {
        high_vectors: true,
        endian: Endian::Big,
        ..Config::CORTEX_A9
    });
    assert_eq!(strapped.sctlr(), 0x00c5_0078 | sctlr::V | sctlr::EE);
    let regime = strapped.regime();
    assert!(regime.high_vectors && regime.big_endian_exceptions);
    assert!(!regime.translating);

    cp.mcr(SCTLR, 0).unwrap();
    assert_eq!(
        cp.sctlr(),
        sctlr::READ_AS_ONE,
        "the RAO bits survive a zero"
    );
    cp.mcr(SCTLR, u32::MAX).unwrap();
    assert_eq!(cp.sctlr(), sctlr::READ_AS_ONE | sctlr::WRITABLE);
    let regime = cp.regime();
    assert!(regime.translating && regime.alignment_faults && regime.thumb_exceptions);
    assert!(regime.unaligned, "ARMv7 always does unaligned accesses");
}

#[test]
fn vbar_and_cpacr_reach_the_regime() {
    let cp = cp();
    cp.mcr(op(0, 12, 0, 0), 0x8000_1234).unwrap();
    assert_eq!(
        cp.regime().vector_base,
        0x8000_1220,
        "VBAR is 32-byte aligned"
    );
    // The A9 preset has VFPv3-D32 and no NEON: cp10/11 and D32DIS are
    // writable, ASEDIS reads as one.
    assert_eq!(cp.cpacr(), 1 << 31);
    cp.mcr(op(0, 1, 0, 2), u32::MAX).unwrap();
    assert_eq!(cp.cpacr(), 0xc0f0_0000);
    assert_eq!(cp.regime().cp_access, 0xc0f0_0000);
    cp.mcr(op(0, 1, 0, 2), 0).unwrap();
    assert_eq!(cp.cpacr(), 1 << 31);
}

#[test]
fn the_performance_monitor_stores_but_does_not_count() {
    let cp = cp();
    assert_eq!(
        cp.mrc(op(0, 9, 12, 0)).unwrap() >> 11 & 0x1f,
        6,
        "six counters"
    );
    cp.mcr(op(0, 9, 12, 1), 0x8000_0003).unwrap();
    cp.mcr(op(0, 9, 12, 2), 0x1).unwrap();
    assert_eq!(cp.mrc(op(0, 9, 12, 1)), Ok(0x8000_0002));
    cp.mcr(op(0, 9, 12, 5), 3).unwrap();
    cp.mcr(op(0, 9, 13, 2), 99).unwrap();
    assert_eq!(cp.mrc(op(0, 9, 13, 2)), Ok(99));
    cp.mcr(op(0, 9, 12, 0), 1 << 1).unwrap();
    assert_eq!(
        cp.mrc(op(0, 9, 13, 2)),
        Ok(0),
        "PMCR.P resets the event counters"
    );
}

#[test]
fn unknown_encodings_and_write_only_reads_are_undefined() {
    let cp = cp();
    assert_eq!(cp.mrc(op(0, 11, 0, 0)), Err(CpFault::Undefined));
    assert_eq!(
        cp.mrc(op(0, 7, 5, 0)),
        Err(CpFault::Undefined),
        "ICIALLU is write-only"
    );
    assert_eq!(cp.mrc(op(0, 8, 7, 0)), Err(CpFault::Undefined));
    // The v5 "test and clean" does not exist on v7.
    assert_eq!(cp.mrc(op(0, 7, 10, 3)), Err(CpFault::Undefined));
    // The retired CP15 WFI is a no-op, not a halt.
    assert_eq!(cp.mcr(op(0, 7, 0, 4), 0), Ok(CpEffect::NONE));
    // FCSE absent: RAZ/WI.
    cp.mcr(op(0, 13, 0, 0), 0x0200_0000).unwrap();
    assert_eq!(cp.mrc(op(0, 13, 0, 0)), Ok(0));
    // Another coprocessor is not this one's business.
    assert_eq!(
        cp.mrc(CpOp {
            cp: 14,
            ..op(0, 0, 0, 0)
        }),
        Err(CpFault::Undefined)
    );
}

// ---------------------------------------------------------------------------
// Privilege
// ---------------------------------------------------------------------------

#[test]
fn user_mode_reaches_only_the_thread_ids_and_the_barriers() {
    let cp = cp();
    cp.mcr(op(0, 13, 0, 3), 0x1234).unwrap();
    // TPIDRURO: readable, not writable, from User mode.
    assert_eq!(cp.mrc(user(op(0, 13, 0, 3))), Ok(0x1234));
    assert_eq!(cp.mcr(user(op(0, 13, 0, 3)), 0), Err(CpFault::Undefined));
    // TPIDRURW: both.
    cp.mcr(user(op(0, 13, 0, 2)), 0x55).unwrap();
    assert_eq!(cp.mrc(user(op(0, 13, 0, 2))), Ok(0x55));
    // The three barriers.
    for (crm, opc2) in [(5, 4), (10, 4), (10, 5)] {
        assert_eq!(cp.mcr(user(op(0, 7, crm, opc2)), 0), Ok(CpEffect::NONE));
    }
    // And nothing else: not the ID registers, not SCTLR, not TPIDRPRW, not a
    // cache operation, not a TLB operation.
    for o in [
        op(0, 0, 0, 0),
        op(0, 1, 0, 0),
        op(0, 13, 0, 4),
        op(0, 2, 0, 0),
    ] {
        assert_eq!(cp.mrc(user(o)), Err(CpFault::Undefined), "{o:?}");
    }
    assert_eq!(cp.mcr(user(op(0, 7, 5, 0)), 0), Err(CpFault::Undefined));
    assert_eq!(cp.mcr(user(op(0, 8, 7, 0)), 0), Err(CpFault::Undefined));
}

#[test]
fn pmuserenr_opens_the_performance_monitor_to_user_mode() {
    let cp = cp();
    assert_eq!(cp.mrc(user(op(0, 9, 13, 0))), Err(CpFault::Undefined));
    assert_eq!(
        cp.mrc(user(op(0, 9, 14, 0))),
        Ok(0),
        "PMUSERENR itself is readable"
    );
    assert_eq!(cp.mcr(user(op(0, 9, 14, 0)), 1), Err(CpFault::Undefined));
    cp.mcr(op(0, 9, 14, 0), 1).unwrap();
    assert_eq!(cp.mrc(user(op(0, 9, 13, 0))), Ok(0));
}

// ---------------------------------------------------------------------------
// The walk
// ---------------------------------------------------------------------------

#[test]
fn a_section_and_a_supersection_translate() {
    let mut ram = Ram::new();
    ram.put(L1 + (0x001 << 2), section(0x4000_0000, 0b011, 0, false));
    // A supersection occupies sixteen consecutive first-level entries.
    let ss = 0x8000_0000 | (1 << 18) | (0b11 << 10) | 0b10;
    for i in 0..16 {
        ram.put(L1 + ((0x100 + i) << 2), ss);
    }
    let cp = cp();
    enable(&cp);
    assert_eq!(
        t(&cp, &ram, 0x0012_3456, AccessKind::Read, false),
        Ok(0x4002_3456)
    );
    assert_eq!(
        t(&cp, &ram, 0x10ab_cdef, AccessKind::Write, true),
        Ok(0x80ab_cdef)
    );
    assert_eq!(
        t(&cp, &ram, 0x0020_0000, AccessKind::Read, true),
        Err(Fault::TRANSLATION_SECTION)
    );
}

#[test]
fn small_and_large_pages_translate_through_a_second_level_table() {
    let mut ram = Ram::new();
    ram.put(L1, L2 | (2 << 5) | 0b01);
    ram.put(L2 + (5 << 2), small(0x0003_0000, 0b011, false));
    let large = 0x0005_0000 | (0b11 << 4) | 0b01;
    for i in 0..16 {
        ram.put(L2 + ((0x10 + i) << 2), large);
    }
    let cp = cp();
    enable(&cp);
    assert_eq!(
        t(&cp, &ram, 0x0000_5abc, AccessKind::Write, false),
        Ok(0x0003_0abc)
    );
    assert_eq!(
        t(&cp, &ram, 0x0001_7abc, AccessKind::Read, false),
        Ok(0x0005_7abc)
    );
    assert_eq!(
        t(&cp, &ram, 0x0000_6000, AccessKind::Read, true),
        Err(Fault::TRANSLATION_PAGE.in_domain(2)),
        "a page fault names the domain the first level gave it"
    );
}

#[test]
fn ttbcr_n_splits_the_address_space_between_the_two_bases() {
    let mut ram = Ram::new();
    // TTBR0's table: megabyte 1 -> 0x1000_0000.
    ram.put(L1 + (0x001 << 2), section(0x1000_0000, 0b011, 0, false));
    // TTBR1's table, indexed by the full VA[31:20]: megabyte 0xc00.
    ram.put(
        L1_HIGH + (0xc00 << 2),
        section(0x2000_0000, 0b011, 0, false),
    );
    let cp = cp();
    enable(&cp);
    cp.mcr(op(0, 2, 0, 1), L1_HIGH).unwrap();

    // N = 0: TTBR1 is never used, so 0xc000_0000 walks TTBR0's empty entry.
    assert_eq!(
        t(&cp, &ram, 0xc000_0000, AccessKind::Read, true),
        Err(Fault::TRANSLATION_SECTION)
    );
    // N = 2: the bottom gigabyte is TTBR0's, the rest TTBR1's.
    cp.mcr(op(0, 2, 0, 2), 2).unwrap();
    assert_eq!(
        t(&cp, &ram, 0x0010_0004, AccessKind::Read, true),
        Ok(0x1000_0004)
    );
    assert_eq!(
        t(&cp, &ram, 0xc000_0004, AccessKind::Read, true),
        Ok(0x2000_0004)
    );
    // PD1 disables the TTBR1 walk; PD0 the TTBR0 one.
    cp.mcr(op(0, 2, 0, 2), 2 | (1 << 5)).unwrap();
    assert_eq!(
        t(&cp, &ram, 0xc000_0004, AccessKind::Read, true),
        Err(Fault::TRANSLATION_SECTION)
    );
    assert!(t(&cp, &ram, 0x0010_0004, AccessKind::Read, true).is_ok());
    cp.mcr(op(0, 2, 0, 2), 2 | (1 << 4)).unwrap();
    assert!(t(&cp, &ram, 0x0010_0004, AccessKind::Read, true).is_err());
}

#[test]
fn a_shrunken_ttbr0_table_is_indexed_by_fewer_bits() {
    // N = 2: TTBR0's table is 4 KiB, 4 KiB aligned, indexed by VA[29:20].
    let mut ram = Ram::new();
    let small_table = 0x3000;
    ram.put(
        small_table + (0x3ff << 2),
        section(0x0400_0000, 0b011, 0, false),
    );
    let cp = cp();
    enable(&cp);
    cp.mcr(op(0, 2, 0, 0), small_table).unwrap();
    cp.mcr(op(0, 2, 0, 2), 2).unwrap();
    assert_eq!(
        t(&cp, &ram, 0x3ff0_0010, AccessKind::Read, true),
        Ok(0x0400_0010)
    );
}

#[test]
fn the_ap_matrix_decides_privileged_and_user_access() {
    use AccessKind::{Read, Write};
    // (AP[2:0], privileged read, privileged write, user read, user write)
    let table = [
        (0b000, false, false, false, false),
        (0b001, true, true, false, false),
        (0b010, true, true, true, false),
        (0b011, true, true, true, true),
        (0b100, false, false, false, false),
        (0b101, true, false, false, false),
        (0b110, true, false, true, false),
        (0b111, true, false, true, false),
    ];
    for (ap, pr, pw, ur, uw) in table {
        let mut ram = Ram::new();
        ram.put(L1, section(0x0010_0000, ap, 0, false));
        let cp = cp();
        enable(&cp);
        let ok = |kind, privileged| t(&cp, &ram, 0x10, kind, privileged).is_ok();
        assert_eq!(
            (
                ok(Read, true),
                ok(Write, true),
                ok(Read, false),
                ok(Write, false)
            ),
            (pr, pw, ur, uw),
            "AP {ap:03b}"
        );
        if !pr {
            assert_eq!(
                t(&cp, &ram, 0x10, Read, true),
                Err(Fault::PERMISSION_SECTION),
                "AP {ap:03b}"
            );
        }
    }
}

#[test]
fn execute_never_refuses_a_fetch_and_nothing_else() {
    let mut ram = Ram::new();
    ram.put(L1, section(0, 0b011, 0, true));
    ram.put(L1 + 4, L2 | 0b01);
    ram.put(L2, small(0x5000, 0b011, true));
    let cp = cp();
    enable(&cp);
    assert!(t(&cp, &ram, 0x100, AccessKind::Read, false).is_ok());
    assert_eq!(
        t(&cp, &ram, 0x100, AccessKind::Fetch, true),
        Err(Fault::PERMISSION_SECTION)
    );
    assert_eq!(
        t(&cp, &ram, 0x0010_0000, AccessKind::Fetch, true),
        Err(Fault::PERMISSION_PAGE)
    );
    // A manager domain skips the permission attributes, XN among them.
    cp.mcr(op(0, 3, 0, 0), 0b11).unwrap();
    assert!(t(&cp, &ram, 0x100, AccessKind::Fetch, true).is_ok());
}

#[test]
fn domains_can_refuse_or_skip_the_permission_check() {
    let mut ram = Ram::new();
    ram.put(L1, section(0, 0b011, 5, false));
    ram.put(L1 + 4, section(0x0010_0000, 0b000, 6, false));
    let cp = cp();
    enable(&cp);
    // Domain 5 no access: a domain fault naming it.
    cp.mcr(op(0, 3, 0, 0), 0x5555_5555 & !(0b11 << 10)).unwrap();
    assert_eq!(
        t(&cp, &ram, 0, AccessKind::Read, true),
        Err(Fault::DOMAIN_SECTION.in_domain(5))
    );
    // Domain 6 manager: AP 0b000 does not matter.
    cp.mcr(op(0, 3, 0, 0), 0b11 << 12).unwrap();
    assert!(t(&cp, &ram, 0x0010_0000, AccessKind::Write, false).is_ok());
}

#[test]
fn with_afe_a_clear_access_flag_faults_and_ap_is_the_simplified_model() {
    let mut ram = Ram::new();
    // AP[0] = 0: the access flag is clear.
    ram.put(L1, section(0, 0b010, 3, false));
    // AP = 0b011 under AFE: AP[2:1] = 01, read/write at both levels.
    ram.put(L1 + 4, section(0x0010_0000, 0b011, 0, false));
    // AP = 0b111 under AFE: read-only at both levels.
    ram.put(L1 + 8, section(0x0020_0000, 0b111, 0, false));
    ram.put(L1 + 12, L2 | 0b01);
    ram.put(L2, small(0x7000, 0b000, false));
    let cp = cp();
    enable(&cp);
    // Without AFE the first section is AP 0b010: user read-only.
    assert!(t(&cp, &ram, 0, AccessKind::Read, false).is_ok());

    cp.mcr(SCTLR, cp.sctlr() | sctlr::AFE).unwrap();
    assert_eq!(
        t(&cp, &ram, 0, AccessKind::Read, true),
        Err(Fault::ACCESS_FLAG_SECTION.in_domain(3))
    );
    // The access flag is checked before the domain: even a manager faults.
    cp.mcr(op(0, 3, 0, 0), u32::MAX).unwrap();
    assert_eq!(
        t(&cp, &ram, 0x0030_0000, AccessKind::Read, true),
        Err(Fault::ACCESS_FLAG_PAGE)
    );
    cp.mcr(op(0, 3, 0, 0), 0x5555_5555).unwrap();
    assert!(t(&cp, &ram, 0x0010_0000, AccessKind::Write, false).is_ok());
    assert!(t(&cp, &ram, 0x0020_0000, AccessKind::Read, false).is_ok());
    assert!(t(&cp, &ram, 0x0020_0000, AccessKind::Write, true).is_err());
}

#[test]
fn an_unreadable_table_is_an_external_abort_naming_its_level() {
    let mut ram = Ram::new();
    let cp = cp();
    enable(&cp);
    cp.mcr(op(0, 2, 0, 0), 0xffff_c000).unwrap();
    assert_eq!(
        t(&cp, &ram, 0, AccessKind::Read, true),
        Err(Fault::EXTERNAL_L1)
    );
    cp.mcr(op(0, 2, 0, 0), L1).unwrap();
    ram.put(L1, 0xfff0_0000 | (7 << 5) | 0b01);
    assert_eq!(
        t(&cp, &ram, 0, AccessKind::Read, true),
        Err(Fault::EXTERNAL_L2.in_domain(7))
    );
}

#[test]
fn the_debug_walk_sees_through_every_check() {
    let mut ram = Ram::new();
    // No access for anyone, XN, access flag clear, domain no-access.
    ram.put(L1, section(0x0040_0000, 0b000, 9, true));
    let cp = cp();
    enable(&cp);
    cp.mcr(SCTLR, cp.sctlr() | sctlr::AFE).unwrap();
    cp.mcr(op(0, 3, 0, 0), 0).unwrap();
    let before = (cp.data_fault(), cp.instruction_fault());
    assert_eq!(cp.translate_debug(&ram, Va(0x44)), Ok(Pa(0x0040_0044)));
    assert_eq!((cp.data_fault(), cp.instruction_fault()), before);
    // And an empty entry is still reported as unmapped.
    assert!(cp.translate_debug(&ram, Va(0x0010_0000)).is_err());
}

// ---------------------------------------------------------------------------
// Fault registers and the VA-to-PA operations
// ---------------------------------------------------------------------------

#[test]
fn a_data_abort_latches_dfsr_with_wnr_and_dfar() {
    let cp = cp();
    cp.report_abort(
        Va(0xdead_beef),
        Fault::PERMISSION_PAGE.in_domain(2),
        AccessKind::Write,
    );
    assert_eq!(cp.data_fault(), ((1 << 11) | 0x2f, 0xdead_beef));
    cp.report_abort(Va(0x1000), Fault::ACCESS_FLAG_SECTION, AccessKind::Read);
    assert_eq!(cp.data_fault(), (0x03, 0x1000));
    // A fault with FS[4] set puts it in bit 10.
    let wide = Fault {
        status: 0b1_0110,
        domain: 0,
    };
    cp.report_abort(Va(4), wide, AccessKind::Read);
    assert_eq!(cp.data_fault().0, (1 << 10) | 0b0110);
}

#[test]
fn a_prefetch_abort_latches_ifsr_and_ifar_without_a_domain() {
    let cp = cp();
    cp.report_abort(
        Va(0xc000_0000),
        Fault::PERMISSION_SECTION.in_domain(4),
        AccessKind::Fetch,
    );
    assert_eq!(cp.instruction_fault(), (0x0d, 0xc000_0000));
    assert_eq!(cp.data_fault(), (0, 0), "the data side is untouched");
    // And the registers read back through MRC.
    assert_eq!(cp.mrc(op(0, 5, 0, 1)), Ok(0x0d));
    assert_eq!(cp.mrc(op(0, 6, 0, 2)), Ok(0xc000_0000));
}

#[test]
fn ats1c_operations_leave_the_answer_in_par() {
    let mut ram = Ram::new();
    ram.put(L1, section(0x0050_0000, 0b001, 0, false)); // privileged only
    let cp = cp();
    // MMU off: the flat map.
    cp.mcr_with(op(0, 7, 8, 0), 0x1234_5678, &ram).unwrap();
    assert_eq!(cp.par(), 0x1234_5000);

    enable(&cp);
    let before = cp.data_fault();
    // ATS1CPR: fine.
    cp.mcr_with(op(0, 7, 8, 0), 0x0000_0abc, &ram).unwrap();
    assert_eq!(cp.par(), 0x0050_0000);
    // ATS1CUR: a permission fault, reported in PAR and nowhere else.
    cp.mcr_with(op(0, 7, 8, 2), 0x0000_0abc, &ram).unwrap();
    assert_eq!(cp.par(), 1 | (0b1101 << 1));
    assert_eq!(cp.data_fault(), before, "an ATS operation takes no abort");
    // An unmapped address: a section translation fault.
    cp.mcr_with(op(0, 7, 8, 1), 0x0100_0000, &ram).unwrap();
    assert_eq!(cp.par(), 1 | (0b0101 << 1));
    // A supersection says so.
    for i in 0..16 {
        ram.put(
            L1 + ((0x200 + i) << 2),
            0x9000_0000 | (1 << 18) | (0b11 << 10) | 0b10,
        );
    }
    cp.mcr_with(op(0, 7, 8, 0), 0x2012_3456, &ram).unwrap();
    assert_eq!(cp.par(), 0x9012_3000 | 0b10);
}

#[test]
fn every_translation_affecting_write_moves_the_generation() {
    let cp = cp();
    let mut last = cp.regime().generation;
    for (o, value) in [
        (SCTLR, 1),
        (op(0, 2, 0, 0), 0x4000),
        (op(0, 2, 0, 1), 0x8000),
        (op(0, 2, 0, 2), 1),
        (op(0, 3, 0, 0), 1),
        (op(0, 13, 0, 1), 7),
        (op(0, 8, 7, 0), 0),
        (op(0, 8, 7, 1), 0),
        (op(0, 8, 7, 2), 0),
        (op(0, 8, 5, 0), 0),
        (op(0, 8, 6, 3), 0),
        (op(0, 8, 3, 0), 0),
    ] {
        cp.mcr(o, value).unwrap();
        let now = cp.regime().generation;
        assert_ne!(now, last, "{o:?} did not invalidate");
        last = now;
    }
    // Cache maintenance and the attribute remap do not.
    cp.mcr(op(0, 7, 14, 1), 0).unwrap();
    cp.mcr(op(0, 10, 2, 0), 0).unwrap();
    assert_eq!(cp.regime().generation, last);
}

#[test]
fn an_inner_shareable_tlb_operation_reaches_the_joined_peers() {
    let a = cp();
    let b = cp();
    let c = cp();
    a.join(&b);
    a.join(&b); // twice is harmless
    let (gb, gc) = (b.regime().generation, c.regime().generation);
    // A local operation stays local.
    a.mcr(op(0, 8, 7, 0), 0).unwrap();
    assert_eq!(b.regime().generation, gb);
    // TLBIALLIS reaches b and not the unjoined c.
    a.mcr(op(0, 8, 3, 0), 0).unwrap();
    assert_ne!(b.regime().generation, gb);
    assert_eq!(c.regime().generation, gc);
    // And the other way round.
    let ga = a.regime().generation;
    b.mcr(op(0, 8, 3, 2), 0).unwrap();
    assert_ne!(a.regime().generation, ga);
}

#[test]
fn reset_puts_every_register_back() {
    let cp = cp();
    enable(&cp);
    cp.mcr(op(0, 12, 0, 0), 0x100).unwrap();
    cp.mcr(op(0, 13, 0, 4), 9).unwrap();
    cp.report_abort(Va(1), Fault::EXTERNAL, AccessKind::Read);
    cp.reset();
    assert!(!cp.mmu_enabled());
    assert_eq!(cp.sctlr(), 0x00c5_0078);
    assert_eq!((cp.ttbr0(), cp.dacr(), cp.vbar()), (0, 0, 0));
    assert_eq!(cp.data_fault(), (0, 0));
    assert_eq!(cp.mrc(op(0, 10, 2, 0)), Ok(id::PRRR));
}

// ---------------------------------------------------------------------------
// Inside a core
// ---------------------------------------------------------------------------

const RAM_SIZE: u64 = 0x2_0000;

/// A Cortex-A9 with 128 KiB of RAM at zero, having run its reset sequence.
fn a9(cfg: Config) -> (Arm, Arc<AddressSpace>) {
    let space = Arc::new(AddressSpace::new("cpu", 32));
    space
        .topology()
        .map(Region::ram("ram", Arc::new(RamStore::new(RAM_SIZE))), 0)
        .unwrap();
    let cpu = Arm::new(cfg);
    cpu.attach_space(Arc::clone(&space));
    cpu.step(); // the reset sequence
    (cpu, space)
}

fn poke(space: &AddressSpace, at: u32, words: &[u32]) {
    for (i, word) in words.iter().enumerate() {
        space
            .write(
                u64::from(at) + 4 * i as u64,
                Width::U32,
                u64::from(*word),
                MemAttrs::DEFAULT,
            )
            .unwrap();
    }
}

#[test]
fn guest_code_builds_a_table_turns_the_mmu_on_and_runs_through_a_remap() {
    let (cpu, space) = a9(Config::CORTEX_A9);
    // The table at 0x8000: megabyte 0 identity, and 0xc00 -> physical 0.
    poke(&space, 0x8000, &[section(0, 0b011, 0, false)]);
    poke(
        &space,
        0x8000 + (0xc00 << 2),
        &[section(0, 0b011, 0, false)],
    );
    poke(&space, 0x2000, &[0xcafe_f00d]);
    // The data abort vector: `b .`
    poke(&space, 0x10, &[0xeaff_fffe]);
    poke(
        &space,
        0x1000,
        &[
            0xe3a0_0902, // mov r0, #0x8000
            0xee02_0f10, // mcr p15, 0, r0, c2, c0, 0   ; TTBR0
            0xe3a0_1000, // mov r1, #0
            0xee02_1f50, // mcr p15, 0, r1, c2, c0, 2   ; TTBCR
            0xe3a0_1001, // mov r1, #1
            0xee03_1f10, // mcr p15, 0, r1, c3, c0, 0   ; DACR: domain 0 client
            0xee11_2f10, // mrc p15, 0, r2, c1, c0, 0
            0xe382_2001, // orr r2, r2, #1
            0xee01_2f10, // mcr p15, 0, r2, c1, c0, 0   ; SCTLR.M
            0xee07_0f95, // mcr p15, 0, r0, c7, c5, 4   ; CP15ISB
            0xe3a0_3103, // mov r3, #0xc0000000
            0xe383_3a02, // orr r3, r3, #0x2000
            0xe593_4000, // ldr r4, [r3]                ; through the remap
            0xe3a0_5103, // mov r5, #0xc0000000
            0xe285_5d41, // add r5, r5, #0x1040
            0xe12f_ff15, // bx r5                       ; execute through it
        ],
    );
    poke(
        &space,
        0x1040,
        &[
            0xe3a0_6077, // mov r6, #0x77
            0xe3a0_8101, // mov r8, #0x40000000
            0xe598_7000, // ldr r7, [r8]                ; unmapped: data abort
            0xeaff_fffe, // b .
        ],
    );
    cpu.set_pc(0x1000);
    for _ in 0..22 {
        cpu.step();
    }
    assert_eq!(cpu.reg(4), 0xcafe_f00d, "the load went through the remap");
    assert_eq!(cpu.reg(6), 0x77, "the fetch did too");
    assert_eq!(cpu.mode(), Mode::ABORT);
    assert_eq!(cpu.pc(), 0x10);
    assert_eq!(
        cpu.reg(14),
        0xc000_1048 + 8,
        "the aborting instruction ran at its virtual address"
    );
    let cp15 = cpu.cp15v7().expect("a Cortex-A9 has its CP15");
    assert_eq!(cp15.data_fault(), (0x005, 0x4000_0000));
    assert!(cp15.mmu_enabled());
    // A debugger sees the remap, and the identity map beside it.
    assert_eq!(cpu.translate_debug(0xc000_1040), Some(0x1040));
    assert_eq!(cpu.translate_debug(0x4000_0000), None);
}

#[test]
fn user_mode_cp15_access_is_undefined_except_tpidruro() {
    // mrc p15, 0, r0, c13, c0, 3 — TPIDRURO, readable from User mode.
    let (cpu, space) = a9(Config::CORTEX_A9);
    cpu.cp15v7().unwrap().mcr(op(0, 13, 0, 3), 0xabcd).unwrap();
    poke(&space, 0x1000, &[0xee1d_0f70]);
    cpu.set_pc(0x1000);
    cpu.set_cpsr(u32::from(Mode::USER.0));
    cpu.step();
    assert_eq!(cpu.mode(), Mode::USER);
    assert_eq!(cpu.reg(0), 0xabcd);

    for insn in [
        0xee0d_0f70, // mcr p15, 0, r0, c13, c0, 3  — TPIDRURO is read-only here
        0xee11_0f10, // mrc p15, 0, r0, c1, c0, 0   — SCTLR is PL1-only
        0xee9f_0f10, // mrc p15, 4, r0, c15, c0, 0  — so is CBAR
    ] {
        let (cpu, space) = a9(Config::CORTEX_A9);
        poke(&space, 0x1000, &[insn]);
        cpu.set_pc(0x1000);
        cpu.set_cpsr(u32::from(Mode::USER.0));
        cpu.step();
        assert_eq!(cpu.mode(), Mode::UNDEFINED, "{insn:#010x}");
    }
}

#[test]
fn guest_code_reads_mpidr_and_cbar() {
    let cfg = Config {
        cpu_id: 3,
        periphbase: 0xf000_0000,
        ..Config::CORTEX_A9
    };
    let (cpu, space) = a9(cfg);
    poke(
        &space,
        0x1000,
        &[
            0xee9f_0f10, // mrc p15, 4, r0, c15, c0, 0
            0xee10_1fb0, // mrc p15, 0, r1, c0, c0, 5
        ],
    );
    cpu.set_pc(0x1000);
    cpu.step();
    cpu.step();
    assert_eq!((cpu.reg(0), cpu.reg(1)), (0xf000_0000, 0x8000_0003));
}

#[test]
fn guest_ats1cpr_walks_the_real_tables() {
    let (cpu, space) = a9(Config::CORTEX_A9);
    poke(&space, 0x8000, &[section(0, 0b011, 0, false)]);
    poke(
        &space,
        0x8000 + (0x123 << 2),
        &[section(0x0010_0000, 0b011, 0, false)],
    );
    let cp15 = Arc::clone(cpu.cp15v7().unwrap());
    enable_at(&cp15, 0x8000);
    poke(
        &space,
        0x1000,
        &[
            0xee07_0f18, // mcr p15, 0, r0, c7, c8, 0   ; ATS1CPR r0
            0xee17_0f14, // mrc p15, 0, r0, c7, c4, 0   ; PAR
        ],
    );
    cpu.set_pc(0x1000);
    cpu.set_reg(0, 0x1234_5678);
    cpu.step();
    cpu.step();
    assert_eq!(cpu.reg(0), 0x0014_5000);
}

fn enable_at(cp: &Cp15v7, table: u32) {
    cp.mcr(op(0, 2, 0, 0), table).unwrap();
    cp.mcr(op(0, 3, 0, 0), 0x5555_5555).unwrap();
    cp.mcr(SCTLR, cp.sctlr() | sctlr::M).unwrap();
}

#[test]
fn isr_reads_the_cores_interrupt_inputs() {
    let (cpu, _space) = a9(Config::CORTEX_A9);
    let cp15 = cpu.cp15v7().unwrap();
    assert_eq!(cp15.mrc(op(0, 12, 1, 0)), Ok(0));
    cpu.set_irq(true);
    assert_eq!(cp15.mrc(op(0, 12, 1, 0)), Ok(1 << 7));
    cpu.set_fiq(true);
    assert_eq!(cp15.mrc(op(0, 12, 1, 0)), Ok((1 << 7) | (1 << 6)));
}

// ---------------------------------------------------------------------------
// The device surface
// ---------------------------------------------------------------------------

fn snapshot(cpu: &Arm) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("cpu", CLASS.name).unwrap();
    let mut writer = StateWriter::new(shape);
    {
        let mut chunk = writer.chunk("cpu", CLASS.name, CLASS.version).unwrap();
        cpu.save(&mut chunk).unwrap();
    }
    writer.to_vec().unwrap()
}

fn restore(cpu: &Arm, bytes: &[u8]) {
    let reader = StateReader::new(bytes).unwrap();
    let chunk = reader
        .load("cpu", CLASS.name, CLASS.version, &Migrations::new())
        .unwrap();
    cpu.load(&mut chunk.reader()).unwrap();
}

/// FNV-1a, which is enough to say "the same bytes".
fn hash(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf2_9ce4_8422_2325, |h, b| {
        (h ^ u64::from(*b)).wrapping_mul(0x0100_0000_01b3)
    })
}

#[test]
fn the_v7_cp15_round_trips_through_a_snapshot_with_an_identical_hash() {
    let (cpu, _space) = a9(Config::CORTEX_A9);
    let cp15 = Arc::clone(cpu.cp15v7().unwrap());
    enable_at(&cp15, 0x8000);
    for (o, value) in [
        (op(0, 2, 0, 1), 0x0001_c000),
        (op(0, 2, 0, 2), 2),
        (op(0, 12, 0, 0), 0x8000_0000),
        (op(0, 13, 0, 1), 0x42),
        (op(0, 13, 0, 2), 0x1111),
        (op(0, 13, 0, 3), 0x2222),
        (op(0, 13, 0, 4), 0x3333),
        (op(0, 1, 0, 2), 0x00f0_0000),
        (op(0, 1, 0, 1), 1 << 6),
        (op(0, 10, 2, 0), 0x1234),
        (op(0, 9, 12, 1), 0x8000_0001),
        (op(0, 15, 0, 0), 1),
    ] {
        cp15.mcr(o, value).unwrap();
    }
    cp15.report_abort(
        Va(0x4000_0000),
        Fault::TRANSLATION_SECTION,
        AccessKind::Write,
    );
    cp15.report_abort(Va(0xc000_0000), Fault::PERMISSION_PAGE, AccessKind::Fetch);
    let bytes = snapshot(&cpu);

    let (other, _space) = a9(Config::CORTEX_A9);
    restore(&other, &bytes);
    let again = snapshot(&other);
    assert_eq!(hash(&again), hash(&bytes));
    let restored = other.cp15v7().unwrap();
    assert_eq!(restored.ttbr1(), 0x0001_c000);
    assert_eq!(restored.vbar(), 0x8000_0000);
    assert!(restored.mmu_enabled());
    assert_eq!(restored.data_fault(), (0x805, 0x4000_0000));
    assert_eq!(other.tlb_stats(), (0, 0));

    // A cold reset puts it back to the part's reset state.
    other.reset(ResetKind::Cold);
    assert!(!other.cp15v7().unwrap().mmu_enabled());
}

#[test]
fn an_arm926_snapshot_is_the_same_bytes_it_always_was_and_a_v3_one_still_loads() {
    // The ARM926 path writes exactly what it wrote before the v7 CP15
    // existed, which is what makes the v3 -> v4 step the identity.
    let cpu = Arm::new(Config::ARM926EJS_MMU);
    let cp15 = cpu.cp15().unwrap();
    cp15.mcr(
        CpOp {
            cp: 15,
            opc1: 0,
            crd: 0,
            crn: 2,
            crm: 0,
            opc2: 0,
            privileged: true,
        },
        0x4000,
    )
    .unwrap();

    let mut shape = MachineShape::new();
    shape.add_device("cpu", CLASS.name).unwrap();
    let mut writer = StateWriter::new(shape);
    {
        // Written as a v3 chunk: the bytes a build before this one produced.
        let mut chunk = writer.chunk("cpu", CLASS.name, 3).unwrap();
        cpu.save(&mut chunk).unwrap();
    }
    let v3 = writer.to_vec().unwrap();

    let migrations = crate::machine::default_migrations().unwrap();
    let reader = StateReader::new(&v3).unwrap();
    let chunk = reader
        .load("cpu", CLASS.name, CLASS.version, &migrations)
        .expect("the v3 -> v4 step is registered");
    let restored = Arm::new(Config::ARM926EJS_MMU);
    restored.load(&mut chunk.reader()).unwrap();
    assert_eq!(restored.cp15().unwrap().ttbr(), 0x4000);
}

// A Cortex-A9 has VFP, and `from_props` refuses a VFP part in a build that
// cannot model one; without the feature the refusal is what is tested, below.
#[cfg(feature = "cpu-arm-aprofile-vfp")]
#[test]
fn the_cpu_property_brings_its_cp15_and_the_mp_properties_reach_it() {
    use crate::core::props::Props;

    let mut props = Props::new();
    props.insert("cpu", "cortex-a9");
    props.insert("cpu-id", 1u64);
    props.insert("cluster-id", 2u64);
    props.insert("periphbase", 0xf000_0000u64);
    let cpu = Arm::from_props(&props).expect("a Cortex-A9");
    assert_eq!(cpu.config().system, System::CortexA9);
    assert!(cpu.cp15().is_none());
    let cp15 = cpu.cp15v7().expect("its CP15 by default");
    assert_eq!(cp15.mpidr(), 0x8000_0201);
    assert_eq!(cp15.periphbase(), 0xf000_0000);

    // `cp15 = "none"` still takes it away.
    let mut bare = Props::new();
    bare.insert("cpu", "cortex-a9");
    bare.insert("cp15", "none");
    assert!(Arm::from_props(&bare).unwrap().cp15v7().is_none());

    // A CP15 for the other architecture is refused, both ways round.
    for (part, system) in [("arm926ejs", "cortex-a9"), ("cortex-a9", "arm926ejs")] {
        let mut wrong = Props::new();
        wrong.insert("cpu", part);
        wrong.insert("cp15", system);
        assert!(Arm::from_props(&wrong).is_err(), "{part} with {system}");
    }
    // And the name round-trips.
    assert_eq!(
        System::parse(System::CortexA9.as_str()),
        Some(System::CortexA9)
    );
}

#[test]
fn join_cluster_links_two_cores_tlbs() {
    let a = Arm::new(Config::CORTEX_A9);
    let b = Arm::new(Config {
        cpu_id: 1,
        ..Config::CORTEX_A9
    });
    a.join_cluster(&b);
    let before = b.cp15v7().unwrap().regime().generation;
    a.cp15v7().unwrap().mcr(op(0, 8, 3, 0), 0).unwrap();
    assert_ne!(b.cp15v7().unwrap().regime().generation, before);
}

#[cfg(not(feature = "cpu-arm-aprofile-vfp"))]
#[test]
fn a_cortex_a9_without_the_vfp_feature_is_refused_by_name() {
    use crate::core::props::Props;

    let mut props = Props::new();
    props.insert("cpu", "cortex-a9");
    let err = Arm::from_props(&props).expect_err("VFP is not compiled in");
    assert!(
        format!("{err}").contains("cpu-arm-aprofile-vfp"),
        "the error names the missing feature: {err}"
    );
}
