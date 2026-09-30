//! Tests for the ARMv6, ARMv6K, v6T2 and ARMv7 additions to the A32 core.
//!
//! Every encoding here was produced by assembling the instruction in the
//! comment with GNU `as` (`-march=armv7-a`, plus `sec`, `mp` and `idiv` where
//! needed) and reading the word back — running a tool, not reading its source.
//! Expected results are worked from DDI 0406C's pseudocode by hand.
//!
//! Two things matter as much as the new instructions working: an ARMv5TE
//! configuration must not change *at all* (`an_armv5_part_decodes_exactly_as_it_always_did`),
//! and an instruction a part lacks must take the Undefined Instruction
//! exception rather than execute.

use alloc::format;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::Device;
use crate::core::space::{AddressSpace, RamStore, Region};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

use super::cp::{AccessKind, Fault, FlatMmu, Mmu, Pa, PhysMem, Regime, Va};
use super::isa::{self, Insn};
use super::monitor::GlobalMonitor;
use super::*;

// ---------------------------------------------------------------------------
// Harness
// ---------------------------------------------------------------------------

const RAM: usize = 0x2_0000;
const CODE: u32 = 0x1000;

/// A Cortex-A9-architecture core on 128 KiB of RAM at zero, plus a small page
/// at the high vectors.
struct V7 {
    cpu: Arc<Arm>,
    ram: Arc<RamStore>,
}

impl V7 {
    fn with_config(cfg: Config) -> V7 {
        let ram = Arc::new(RamStore::new(RAM as u64));
        let space = AddressSpace::new("cpu", 32);
        space
            .topology()
            .map(Region::ram("ram", Arc::clone(&ram)), 0)
            .expect("ram maps");
        let cpu = Arc::new(Arm::new(cfg));
        cpu.attach_space(Arc::new(space));
        V7 { cpu, ram }
    }

    fn a9() -> V7 {
        V7::with_config(Config {
            arch: Arch::CORTEX_A9,
            ..Config::ARM926EJS
        })
    }

    /// The A9's instruction set with no VFP, for the tests that measure the
    /// ARMv6 trailer alone: a VFP part appends its register file too, and
    /// only in a build with the feature.
    fn a9_integer() -> V7 {
        V7::with_config(Config {
            arch: a9_integer_arch(),
            ..Config::ARM926EJS
        })
    }

    fn v5() -> V7 {
        V7::with_config(Config::ARM926EJS)
    }

    fn poke(&self, addr: u32, word: u32) {
        for (i, b) in word.to_le_bytes().iter().enumerate() {
            self.ram.write_u8(u64::from(addr) + i as u64, *b).unwrap();
        }
    }

    fn peek(&self, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = self.ram.read_u8(u64::from(addr) + i as u64).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    fn peek_byte(&self, addr: u32) -> u8 {
        self.ram.read_u8(u64::from(addr)).unwrap()
    }

    /// Load `words` at `CODE`, run the reset sequence, and start at `CODE` in
    /// System mode with every interrupt masked.
    fn run(&self, words: &[u32]) {
        for (i, w) in words.iter().enumerate() {
            self.poke(CODE + 4 * i as u32, *w);
        }
        self.cpu.step();
        self.cpu
            .set_cpsr(u32::from(Mode::SYSTEM.0) | psr::I | psr::F);
        self.cpu.set_pc(CODE);
    }

    fn steps(&self, n: usize) {
        for _ in 0..n {
            self.cpu.step();
        }
    }

    fn r(&self, i: u8) -> u32 {
        self.cpu.reg(i)
    }

    fn set(&self, i: u8, v: u32) {
        self.cpu.set_reg(i, v);
    }

    fn cpsr(&self) -> u32 {
        self.cpu.cpsr()
    }

    /// Whether the last step took the Undefined Instruction exception.
    fn undefined(&self) -> bool {
        self.cpu.mode() == Mode::UNDEFINED && self.cpu.pc() == 0x04
    }
}

/// Run one instruction on a Cortex-A9 with the given registers set first.
fn one(word: u32, regs: &[(u8, u32)]) -> V7 {
    let m = V7::a9();
    m.run(&[word]);
    for &(r, v) in regs {
        m.set(r, v);
    }
    m.cpu.step();
    m
}

fn text(word: u32) -> String {
    format!("{}", isa::decode_for(&Arch::CORTEX_A9, word))
}

// ---------------------------------------------------------------------------
// ARMv5 is unchanged
// ---------------------------------------------------------------------------

/// A digest of the ARMv5TE decoder's output — `Debug` and `Display` both —
/// over a million pseudo-random words and a sweep of every `op1`/`op2`
/// combination, taken from the build *before* any ARMv6 decoding existed.
/// If this moves, an ARM926EJ-S decodes something differently than it did.
#[test]
fn an_armv5_part_decodes_exactly_as_it_always_did() {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    let mut feed = |s: &str| {
        for b in s.bytes() {
            h ^= u64::from(b);
            h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
    };
    let mut x: u32 = 0x1234_5678;
    for _ in 0..(1u32 << 20) {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        let d = isa::decode_for(&Arch::V5TE, x);
        feed(&format!("{d:?}|{d}"));
    }
    for hi in 0..256u32 {
        for lo in 0..16u32 {
            for mid in [0u32, 0x000f_ff0f, 0x0005_a30c, 0x000f_0f0f] {
                for cond in [0xeu32, 0xf] {
                    let w = (cond << 28) | (hi << 20) | (lo << 4) | mid;
                    let d = isa::decode(w);
                    feed(&format!("{d:?}|{d}"));
                }
            }
        }
    }
    assert_eq!(h, 0xf439_9759_264f_28dc);
}

#[test]
fn a_v5_part_takes_undefined_on_what_it_lacks() {
    for word in [
        0xe6bf_0f31u32, // rev r0, r1
        0xe301_0234,    // movw r0, #0x1234
        0xe191_0f9f,    // ldrex r0, [r1]
        0xf57f_f05f,    // dmb sy
        0xe710_f211,    // sdiv r0, r1, r2
        0xe12f_ff20,    // bxj r0
    ] {
        let m = V7::v5();
        m.run(&[word]);
        m.cpu.step();
        assert!(m.undefined(), "{word:08x} should be UNDEFINED on ARMv5TE");
    }
}

#[test]
fn a_v5_part_keeps_the_older_meaning_where_there_was_one() {
    // `wfi` is `MSR CPSR_, #3` to an ARMv5 part: an MSR that writes nothing.
    let m = V7::v5();
    m.run(&[0xe320_f003]);
    m.cpu.step();
    assert!(!m.cpu.is_halted());
    assert_eq!(m.cpu.pc(), CODE + 4);
    // And `umaal` is a `MUL` with a stray bit 22.
    assert!(matches!(isa::decode(0xe041_0392).insn, Insn::Mul { .. }));
}

#[test]
fn a_part_without_the_divider_has_no_sdiv() {
    // Cortex-A9 has no hardware divide (Cortex-A9 TRM 1.3).
    assert_eq!(text(0xe710_f211), "UNDEFINED ; 0xe710f211");
    let arch = Arch {
        ext: Extensions {
            idiv_arm: true,
            ..Arch::CORTEX_A9.ext
        },
        ..Arch::CORTEX_A9
    };
    assert_eq!(
        format!("{}", isa::decode_for(&arch, 0xe710_f211)),
        "SDIV r0, r1, r2"
    );
}

// ---------------------------------------------------------------------------
// Disassembly
// ---------------------------------------------------------------------------

#[test]
fn every_new_encoding_disassembles_in_ual() {
    let cases: &[(u32, &str)] = &[
        (0xe6bf_0f31, "REV r0, r1"),
        (0xe6bf_0fb1, "REV16 r0, r1"),
        (0xe6ff_0fb1, "REVSH r0, r1"),
        (0xe6ff_0f31, "RBIT r0, r1"),
        (0xe6af_0471, "SXTB r0, r1, ROR #8"),
        (0xe6bf_0871, "SXTH r0, r1, ROR #16"),
        (0xe68f_0071, "SXTB16 r0, r1"),
        (0xe6ef_0c71, "UXTB r0, r1, ROR #24"),
        (0xe6ff_0071, "UXTH r0, r1"),
        (0xe6cf_0071, "UXTB16 r0, r1"),
        (0xe6a2_0071, "SXTAB r0, r2, r1"),
        (0xe6b2_0071, "SXTAH r0, r2, r1"),
        (0xe682_0071, "SXTAB16 r0, r2, r1"),
        (0xe6e2_0471, "UXTAB r0, r2, r1, ROR #8"),
        (0xe6f2_0071, "UXTAH r0, r2, r1"),
        (0xe6c2_0071, "UXTAB16 r0, r2, r1"),
        (0xe681_0fb2, "SEL r0, r1, r2"),
        (0xe611_0f12, "SADD16 r0, r1, r2"),
        (0xe611_0f32, "SASX r0, r1, r2"),
        (0xe611_0f52, "SSAX r0, r1, r2"),
        (0xe611_0f72, "SSUB16 r0, r1, r2"),
        (0xe611_0f92, "SADD8 r0, r1, r2"),
        (0xe611_0ff2, "SSUB8 r0, r1, r2"),
        (0xe621_0f12, "QADD16 r0, r1, r2"),
        (0xe631_0f92, "SHADD8 r0, r1, r2"),
        (0xe651_0f92, "UADD8 r0, r1, r2"),
        (0xe661_0f72, "UQSUB16 r0, r1, r2"),
        (0xe671_0f32, "UHASX r0, r1, r2"),
        (0xe6a7_0011, "SSAT r0, #8, r1"),
        (0xe6a0_0191, "SSAT r0, #1, r1, LSL #3"),
        (0xe6bf_0051, "SSAT r0, #32, r1, ASR #32"),
        (0xe6e8_0011, "USAT r0, #8, r1"),
        (0xe6e0_0251, "USAT r0, #0, r1, ASR #4"),
        (0xe6a7_0f31, "SSAT16 r0, #8, r1"),
        (0xe6ef_0f31, "USAT16 r0, #15, r1"),
        (0xe681_0012, "PKHBT r0, r1, r2"),
        (0xe681_0812, "PKHBT r0, r1, r2, LSL #16"),
        (0xe681_0852, "PKHTB r0, r1, r2, ASR #16"),
        (0xe681_0052, "PKHTB r0, r1, r2, ASR #32"),
        (0xe700_f211, "SMUAD r0, r1, r2"),
        (0xe700_f231, "SMUADX r0, r1, r2"),
        (0xe700_f251, "SMUSD r0, r1, r2"),
        (0xe700_3211, "SMLAD r0, r1, r2, r3"),
        (0xe700_3271, "SMLSDX r0, r1, r2, r3"),
        (0xe741_0312, "SMLALD r0, r1, r2, r3"),
        (0xe741_0372, "SMLSLDX r0, r1, r2, r3"),
        (0xe750_f211, "SMMUL r0, r1, r2"),
        (0xe750_f231, "SMMULR r0, r1, r2"),
        (0xe750_3211, "SMMLA r0, r1, r2, r3"),
        (0xe750_32d1, "SMMLS r0, r1, r2, r3"),
        (0xe750_32f1, "SMMLSR r0, r1, r2, r3"),
        (0xe780_f211, "USAD8 r0, r1, r2"),
        (0xe780_3211, "USADA8 r0, r1, r2, r3"),
        (0xe041_0392, "UMAAL r0, r1, r2, r3"),
        (0xe060_3291, "MLS r0, r1, r2, r3"),
        (0xf10c_0080, "CPSID i"),
        (0xf108_01c0, "CPSIE aif"),
        (0xf10e_00d3, "CPSID if, #19"),
        (0xf102_001f, "CPS #31"),
        (0xf96d_0513, "SRSDB sp!, #19"),
        (0xf8cd_0512, "SRSIA sp, #18"),
        (0xf8bd_0a00, "RFEIA sp!"),
        (0xf910_0a00, "RFEDB r0"),
        (0xf101_0200, "SETEND BE"),
        (0xf101_0000, "SETEND LE"),
        (0xe191_0f9f, "LDREX r0, [r1]"),
        (0xe181_2f90, "STREX r2, r0, [r1]"),
        (0xe1d1_0f9f, "LDREXB r0, [r1]"),
        (0xe1f1_0f9f, "LDREXH r0, [r1]"),
        (0xe1b2_0f9f, "LDREXD r0, r1, [r2]"),
        (0xe1c1_3f90, "STREXB r3, r0, [r1]"),
        (0xe1e1_3f90, "STREXH r3, r0, [r1]"),
        (0xe1a2_3f90, "STREXD r3, r0, r1, [r2]"),
        (0xf57f_f01f, "CLREX"),
        (0xe320_f000, "NOP"),
        (0xe320_f001, "YIELD"),
        (0xe320_f002, "WFE"),
        (0xe320_f003, "WFI"),
        (0xe320_f004, "SEV"),
        (0xe320_f0f5, "DBG #5"),
        (0xe301_0234, "MOVW r0, #4660"),
        (0xe34a_0bcd, "MOVT r0, #43981"),
        (0xe7cb_021f, "BFC r0, #4, #8"),
        (0xe7cb_0411, "BFI r0, r1, #8, #4"),
        (0xe7bf_0051, "SBFX r0, r1, #0, #32"),
        (0xe7eb_0251, "UBFX r0, r1, #4, #12"),
        (0xe0f1_00b2, "LDRHT r0, [r1], #2"),
        (0xe031_00b2, "LDRHT r0, [r1], -r2"),
        (0xe0f1_00d1, "LDRSBT r0, [r1], #1"),
        (0xe071_00f2, "LDRSHT r0, [r1], #-2"),
        (0xe0e1_00b2, "STRHT r0, [r1], #2"),
        (0xf57f_f05f, "DMB SY"),
        (0xf57f_f05b, "DMB ISH"),
        (0xf57f_f04f, "DSB SY"),
        (0xf57f_f06f, "ISB SY"),
        (0xf4d0_f004, "PLI [r0, #4]"),
        (0xf650_f101, "PLI [r0, -r1, LSL #2]"),
        (0xf590_f004, "PLDW [r0, #4]"),
        (0xf790_f001, "PLDW [r0, r1]"),
        (0xe160_0070, "SMC #0"),
        (0xe12f_ff20, "BXJ r0"),
        (0xe7f0_00f0, "UNDEFINED ; 0xe7f000f0"),
        // A condition goes after the whole mnemonic.
        (0x06bf_0f31, "REVEQ r0, r1"),
    ];
    for &(word, want) in cases {
        assert_eq!(text(word), want, "{word:08x}");
    }
}

#[test]
fn the_listing_follows_the_configured_part() {
    let m = V7::a9();
    m.poke(0x100, 0xe6bf_0f31);
    let listing = m.cpu.disassemble_physical(0x100, 1, false);
    assert_eq!(format!("{}", listing[0]), "00000100: e6bf0f31  REV r0, r1");
    let v5 = V7::v5();
    v5.poke(0x100, 0xe6bf_0f31);
    let listing = v5.cpu.disassemble_physical(0x100, 1, false);
    assert!(format!("{}", listing[0]).contains("UNDEFINED"));
}

// ---------------------------------------------------------------------------
// Media instructions
// ---------------------------------------------------------------------------

#[test]
fn rev_family() {
    let m = one(0xe6bf_0f31, &[(1, 0x1234_5678)]); // rev r0, r1
    assert_eq!(m.r(0), 0x7856_3412);
    let m = one(0xe6bf_0fb1, &[(1, 0x1234_5678)]); // rev16 r0, r1
    assert_eq!(m.r(0), 0x3412_7856);
    let m = one(0xe6ff_0fb1, &[(1, 0x1234_5680)]); // revsh r0, r1
    assert_eq!(m.r(0), 0xffff_8056);
    let m = one(0xe6ff_0f31, &[(1, 0x0000_0003)]); // rbit r0, r1
    assert_eq!(m.r(0), 0xc000_0000);
}

#[test]
fn extends_with_rotation_and_accumulation() {
    let m = one(0xe6af_0471, &[(1, 0x0000_8000)]); // sxtb r0, r1, ror #8
    assert_eq!(m.r(0), 0xffff_ff80);
    let m = one(0xe6e2_0471, &[(1, 0x0000_ff00), (2, 1)]); // uxtab r0, r2, r1, ror #8
    assert_eq!(m.r(0), 0x100);
    let m = one(0xe682_0071, &[(1, 0x0080_00ff), (2, 0x0001_0001)]); // sxtab16
    assert_eq!(m.r(0), 0xff81_0000);
    let m = one(0xe6ff_0071, &[(1, 0xdead_beef)]); // uxth r0, r1
    assert_eq!(m.r(0), 0xbeef);
}

#[test]
fn parallel_arithmetic_sets_ge_and_sel_reads_it() {
    // uadd8 r0, r1, r2 then sel r3, r4, r5 (0xe684_3fb5)
    let m = V7::a9();
    m.run(&[0xe651_0f92, 0xe684_3fb5]);
    m.set(1, 0xff80_0102);
    m.set(2, 0x0180_0101);
    m.set(4, 0xaaaa_aaaa);
    m.set(5, 0x5555_5555);
    m.steps(2);
    assert_eq!(m.r(0), 0x0000_0203);
    assert_eq!((m.cpsr() & psr::GE) >> 16, 0b1100);
    assert_eq!(m.r(3), 0xaaaa_5555);
}

#[test]
fn saturating_parallel_forms_leave_ge_alone() {
    let m = V7::a9();
    m.run(&[0xe621_0f12]); // qadd16 r0, r1, r2
    m.cpu.set_cpsr(m.cpsr() | (0b1010 << 16));
    m.set(1, 0x7fff_8000);
    m.set(2, 0x0001_ffff);
    m.cpu.step();
    assert_eq!(m.r(0), 0x7fff_8000);
    assert_eq!((m.cpsr() & psr::GE) >> 16, 0b1010);
    assert_eq!(m.cpsr() & psr::Q, 0, "QADD16 does not touch Q");
}

#[test]
fn ssat_and_usat_at_their_boundaries() {
    // ssat r0, #8, r1
    let m = one(0xe6a7_0011, &[(1, 127)]);
    assert_eq!(m.r(0), 127);
    assert_eq!(m.cpsr() & psr::Q, 0);
    let m = one(0xe6a7_0011, &[(1, 128)]);
    assert_eq!(m.r(0), 127);
    assert_ne!(m.cpsr() & psr::Q, 0);
    let m = one(0xe6a7_0011, &[(1, (-129i32) as u32)]);
    assert_eq!(m.r(0), (-128i32) as u32);
    // ssat r0, #32, r1, asr #32: the shift fills with the sign.
    let m = one(0xe6bf_0051, &[(1, 0x8000_0000)]);
    assert_eq!(m.r(0), u32::MAX);
    assert_eq!(m.cpsr() & psr::Q, 0);
    // ssat r0, #1, r1, lsl #3: 1 << 3 does not fit in one signed bit.
    let m = one(0xe6a0_0191, &[(1, 1)]);
    assert_eq!(m.r(0), 0);
    assert_ne!(m.cpsr() & psr::Q, 0);
    // usat r0, #8, r1 with a negative input clamps to zero.
    let m = one(0xe6e8_0011, &[(1, 0xffff_ffff)]);
    assert_eq!(m.r(0), 0);
    assert_ne!(m.cpsr() & psr::Q, 0);
    // usat r0, #0, r1, asr #4: the only representable value is zero.
    let m = one(0xe6e0_0251, &[(1, 0x10)]);
    assert_eq!(m.r(0), 0);
    assert_ne!(m.cpsr() & psr::Q, 0);
    // ssat16 r0, #8, r1
    let m = one(0xe6a7_0f31, &[(1, 0x8000_0010)]);
    assert_eq!(m.r(0), 0xff80_0010);
    // usat16 r0, #15, r1
    let m = one(0xe6ef_0f31, &[(1, 0x7fff_ffff)]);
    assert_eq!(m.r(0), 0x7fff_0000);
}

#[test]
fn pkh_packs_halves() {
    let m = one(0xe681_0812, &[(1, 0x1111_2222), (2, 0x3333_4444)]); // pkhbt lsl #16
    assert_eq!(m.r(0), 0x4444_2222);
    let m = one(0xe681_0052, &[(1, 0x1111_2222), (2, 0x8000_0000)]); // pkhtb asr #32
    assert_eq!(m.r(0), 0x1111_ffff);
}

#[test]
fn dual_multiplies() {
    // smuad r0, r1, r2: 2*3 + 4*5
    let m = one(0xe700_f211, &[(1, 0x0004_0002), (2, 0x0005_0003)]);
    assert_eq!(m.r(0), 26);
    // smuadx: 2*5 + 4*3
    let m = one(0xe700_f231, &[(1, 0x0004_0002), (2, 0x0005_0003)]);
    assert_eq!(m.r(0), 22);
    // smusd: 2*3 - 4*5
    let m = one(0xe700_f251, &[(1, 0x0004_0002), (2, 0x0005_0003)]);
    assert_eq!(m.r(0), (-14i32) as u32);
    // smuad of two (-32768)^2 products overflows and sets Q.
    let m = one(0xe700_f211, &[(1, 0x8000_8000), (2, 0x8000_8000)]);
    assert_eq!(m.r(0), 0x8000_0000);
    assert_ne!(m.cpsr() & psr::Q, 0);
    // smlad r0, r1, r2, r3
    let m = one(0xe700_3211, &[(1, 0x0004_0002), (2, 0x0005_0003), (3, 100)]);
    assert_eq!(m.r(0), 126);
    // smlald r0, r1, r2, r3: r1:r0 += 2*3 + 4*5
    let m = one(
        0xe741_0312,
        &[(0, 0xffff_fff0), (1, 0), (2, 0x0004_0002), (3, 0x0005_0003)],
    );
    assert_eq!((m.r(1), m.r(0)), (1, 10));
}

#[test]
fn most_significant_word_multiplies() {
    // smmul r0, r1, r2: (0x40000000 * 4) >> 32 = 1
    let m = one(0xe750_f211, &[(1, 0x4000_0000), (2, 4)]);
    assert_eq!(m.r(0), 1);
    // smmulr rounds: 0x80000000 * 1 = -2^31 → top word -1, rounded 0.
    let m = one(0xe750_f231, &[(1, 0x8000_0000), (2, 1)]);
    assert_eq!(m.r(0), 0);
    let m = one(0xe750_f211, &[(1, 0x8000_0000), (2, 1)]);
    assert_eq!(m.r(0), u32::MAX);
    // smmls r0, r1, r2, r3: (r3 << 32) - r1*r2
    let m = one(0xe750_32d1, &[(1, 0x4000_0000), (2, 4), (3, 10)]);
    assert_eq!(m.r(0), 9);
}

#[test]
fn usad8_umaal_and_mls() {
    let m = one(0xe780_3211, &[(1, 0x0010_ff00), (2, 0x0100_00ff), (3, 1)]);
    assert_eq!(m.r(0), 1 + 1 + 16 + 255 + 255);
    // umaal r0, r1, r2, r3: r1:r0 = r2 * r3 + r1 + r0, at its maximum.
    let m = one(
        0xe041_0392,
        &[(0, u32::MAX), (1, u32::MAX), (2, u32::MAX), (3, u32::MAX)],
    );
    assert_eq!((m.r(1), m.r(0)), (u32::MAX, u32::MAX));
    // mls r0, r1, r2, r3: r3 - r1 * r2
    let m = one(0xe060_3291, &[(1, 6), (2, 7), (3, 100)]);
    assert_eq!(m.r(0), 58);
}

#[test]
fn divide_by_zero_is_zero_and_int_min_wraps() {
    let arch = Arch {
        ext: Extensions {
            idiv_arm: true,
            ..Arch::CORTEX_A9.ext
        },
        ..Arch::CORTEX_A9
    };
    let run = |word, n, d| {
        let m = V7::with_config(Config {
            arch,
            ..Config::ARM926EJS
        });
        m.run(&[word]);
        m.set(1, n);
        m.set(2, d);
        m.cpu.step();
        m.r(0)
    };
    assert_eq!(run(0xe710_f211, (-7i32) as u32, 2), (-3i32) as u32); // sdiv
    assert_eq!(run(0xe710_f211, 0x8000_0000, u32::MAX), 0x8000_0000);
    assert_eq!(run(0xe710_f211, 5, 0), 0);
    assert_eq!(run(0xe730_f211, u32::MAX, 2), 0x7fff_ffff); // udiv
    assert_eq!(run(0xe730_f211, 5, 0), 0);
}

// ---------------------------------------------------------------------------
// v6T2
// ---------------------------------------------------------------------------

#[test]
fn movw_then_movt_builds_a_constant() {
    let m = V7::a9();
    m.run(&[0xe301_0234, 0xe34a_0bcd]); // movw r0, #0x1234; movt r0, #0xabcd
    m.steps(2);
    assert_eq!(m.r(0), 0xabcd_1234);
}

#[test]
fn bitfields() {
    let m = one(0xe7cb_021f, &[(0, u32::MAX)]); // bfc r0, #4, #8
    assert_eq!(m.r(0), 0xffff_f00f);
    let m = one(0xe7cb_0411, &[(0, 0), (1, 0xffff_fffa)]); // bfi r0, r1, #8, #4
    assert_eq!(m.r(0), 0x0000_0a00);
    let m = one(0xe7bf_0051, &[(1, 0x8765_4321)]); // sbfx r0, r1, #0, #32
    assert_eq!(m.r(0), 0x8765_4321);
    let m = one(0xe7eb_0251, &[(1, 0x8765_4321)]); // ubfx r0, r1, #4, #12
    assert_eq!(m.r(0), 0x432);
    // sbfx r0, r1, #4, #4 (0xe7a3_0251): field 0x2 → 2; field 0xa → -6.
    let m = one(0xe7a3_0251, &[(1, 0x0000_00a0)]);
    assert_eq!(m.r(0), (-6i32) as u32);
    // An extract off the top (lsb 20, width 13) is refused rather than guessed.
    assert_eq!(text(0xe7ac_0a51), "UNDEFINED ; 0xe7ac0a51");
}

#[test]
fn unprivileged_halfword_loads_use_user_permissions() {
    // A privileged-only page: the MMU refuses unprivileged access to 0x8000.
    #[derive(Debug)]
    struct Kernel;
    impl Mmu for Kernel {
        fn regime(&self) -> Regime {
            Regime {
                translating: true,
                ..Regime::FLAT
            }
        }
        fn translate(
            &self,
            _mem: &dyn PhysMem,
            va: Va,
            _kind: AccessKind,
            privileged: bool,
        ) -> core::result::Result<Pa, Fault> {
            if !privileged && va.0 >= 0x8000 && va.0 < 0x9000 {
                Err(Fault::PERMISSION_PAGE)
            } else {
                Ok(Pa(va.0))
            }
        }
    }
    let m = V7::a9();
    m.cpu.attach_mmu(Arc::new(Kernel));
    m.poke(0x8000, 0x0000_1234);
    // ldrh r0, [r1], #2 succeeds; ldrht r0, [r1], #2 aborts.
    m.run(&[0xe0d1_00b2]);
    m.set(1, 0x8000);
    m.cpu.step();
    assert_eq!(m.r(0), 0x1234);
    assert_eq!(m.r(1), 0x8002);
    let m2 = V7::a9();
    m2.cpu.attach_mmu(Arc::new(Kernel));
    m2.run(&[0xe0f1_00b2]);
    m2.set(1, 0x8000);
    m2.cpu.step();
    assert_eq!(m2.cpu.mode(), Mode::ABORT);
    assert_eq!(m2.cpu.pc(), 0x10);
}

// ---------------------------------------------------------------------------
// ARMv7: ALUWritePC, barriers, hints
// ---------------------------------------------------------------------------

#[test]
fn mov_pc_interworks_on_v7_but_not_on_v5() {
    // mov pc, r0 with r0 odd.
    let m = one(0xe1a0_f000, &[(0, 0x2001)]);
    assert!(m.cpu.is_thumb());
    assert_eq!(m.cpu.pc(), 0x2000);
    let v5 = V7::v5();
    v5.run(&[0xe1a0_f000]);
    v5.set(0, 0x2001);
    v5.cpu.step();
    assert!(!v5.cpu.is_thumb());
}

#[test]
fn barriers_and_preloads_are_no_ops() {
    let m = V7::a9();
    m.run(&[
        0xf57f_f05f,
        0xf57f_f04f,
        0xf57f_f06f,
        0xf4d0_f004,
        0xf590_f004,
    ]);
    m.steps(5);
    assert_eq!(m.cpu.pc(), CODE + 20);
    assert_eq!(m.cpu.mode(), Mode::SYSTEM);
}

#[test]
fn smc_is_undefined_because_monitor_mode_is_not_modelled() {
    let m = one(0xe160_0070, &[]);
    assert!(m.undefined());
}

#[test]
fn wfi_halts_and_an_interrupt_wakes_it() {
    let m = V7::a9();
    m.run(&[0xe320_f003, 0xe3a0_0001]); // wfi; mov r0, #1
    m.cpu.step();
    assert!(m.cpu.is_halted());
    m.steps(3);
    assert!(m.cpu.is_halted());
    assert_eq!(m.cpu.pc(), CODE + 4);
    // A masked IRQ still wakes it, and execution carries on.
    m.cpu.set_irq(true);
    m.cpu.step();
    assert!(!m.cpu.is_halted());
    assert_eq!(m.r(0), 1);
}

#[test]
fn wfe_consumes_a_pending_event_or_sleeps_until_one() {
    let m = V7::a9();
    // sev; wfe; wfe; mov r0, #1
    m.run(&[0xe320_f004, 0xe320_f002, 0xe320_f002, 0xe3a0_0001]);
    m.steps(2);
    assert!(
        !m.cpu.is_halted(),
        "the SEV's event lets the first WFE through"
    );
    m.cpu.step();
    assert!(m.cpu.is_halted(), "the second has nothing to consume");
    m.steps(2);
    assert!(m.cpu.is_halted());
    m.cpu.send_event();
    m.cpu.step();
    assert!(!m.cpu.is_halted());
    assert_eq!(m.r(0), 1);
    assert!(!m.cpu.event_pending(), "waking consumed the event");
}

#[test]
fn an_event_does_not_wake_wfi() {
    let m = V7::a9();
    m.run(&[0xe320_f003]);
    m.cpu.step();
    m.cpu.send_event();
    m.steps(2);
    assert!(m.cpu.is_halted());
}

// ---------------------------------------------------------------------------
// Exclusives
// ---------------------------------------------------------------------------

#[test]
fn an_exclusive_pair_succeeds_and_clrex_breaks_one() {
    let m = V7::a9();
    // ldrex r0, [r1]; add r0, r0, #1; strex r2, r0, [r1]
    m.run(&[0xe191_0f9f, 0xe280_0001, 0xe181_2f90]);
    m.poke(0x4000, 41);
    m.set(1, 0x4000);
    m.steps(3);
    assert_eq!(m.r(2), 0, "STREX reports success");
    assert_eq!(m.peek(0x4000), 42);
    assert_eq!(m.cpu.exclusive_tag(), None, "and leaves the monitor open");

    // ldrex r0, [r1]; clrex; strex r2, r0, [r1]
    let m = V7::a9();
    m.run(&[0xe191_0f9f, 0xf57f_f01f, 0xe181_2f90]);
    m.poke(0x4000, 7);
    m.set(1, 0x4000);
    m.set(0, 0);
    m.steps(3);
    assert_eq!(m.r(2), 1, "STREX after CLREX fails");
    assert_eq!(m.peek(0x4000), 7, "and stores nothing");
}

#[test]
fn a_strex_without_a_ldrex_fails_and_a_different_granule_fails() {
    let m = one(0xe181_2f90, &[(0, 5), (1, 0x4000)]);
    assert_eq!(m.r(2), 1);
    assert_eq!(m.peek(0x4000), 0);
    let m = V7::a9();
    // ldrex r0, [r1]; strex r2, r0, [r3]
    m.run(&[0xe191_0f9f, 0xe183_2f90]);
    m.set(1, 0x4000);
    m.set(3, 0x4040);
    m.steps(2);
    assert_eq!(m.r(2), 1);
}

#[test]
fn an_exception_between_the_pair_breaks_it() {
    let m = V7::a9();
    // ldrex r0, [r1]; strex r2, r0, [r1] with an IRQ taken in between.
    m.run(&[0xe191_0f9f, 0xe181_2f90]);
    m.poke(0x18, 0xe25e_f004); // subs pc, lr, #4 at the IRQ vector
    m.set(1, 0x4000);
    m.cpu.set_cpsr(u32::from(Mode::SYSTEM.0) | psr::F);
    m.cpu.step();
    assert!(m.cpu.exclusive_tag().is_some());
    m.cpu.set_irq(true);
    m.cpu.step(); // take the IRQ
    assert_eq!(m.cpu.mode(), Mode::IRQ);
    assert_eq!(m.cpu.exclusive_tag(), None);
    m.cpu.set_irq(false);
    m.steps(2); // return, then the STREX
    assert_eq!(m.cpu.pc(), CODE + 8);
    assert_eq!(m.r(2), 1);
}

#[test]
fn byte_half_and_doubleword_exclusives() {
    let m = V7::a9();
    // ldrexd r4, r5, [r1]; strexd r3, r6, r7, [r1] (0xe1a1_3f96)
    m.run(&[0xe1b1_4f9f, 0xe1a1_3f96]);
    m.poke(0x4000, 0x1111_1111);
    m.poke(0x4004, 0x2222_2222);
    m.set(1, 0x4000);
    m.set(6, 0xaaaa_aaaa);
    m.set(7, 0xbbbb_bbbb);
    m.steps(2);
    assert_eq!((m.r(4), m.r(5)), (0x1111_1111, 0x2222_2222));
    assert_eq!(m.r(3), 0);
    assert_eq!((m.peek(0x4000), m.peek(0x4004)), (0xaaaa_aaaa, 0xbbbb_bbbb));

    let m = V7::a9();
    // ldrexb r0, [r1]; strexb r3, r2, [r1]
    m.run(&[0xe1d1_0f9f, 0xe1c1_3f92]);
    m.poke(0x4000, 0x0000_00ee);
    m.set(1, 0x4001);
    m.set(2, 0x55);
    m.steps(2);
    assert_eq!(m.r(0), 0);
    assert_eq!(m.r(3), 0);
    assert_eq!(m.peek_byte(0x4001), 0x55);
}

#[test]
fn a_misaligned_exclusive_faults_whatever_sctlr_a_says() {
    let m = one(0xe191_0f9f, &[(1, 0x4002)]); // ldrex r0, [r1]
    assert_eq!(m.cpu.mode(), Mode::ABORT);
    assert_eq!(m.cpu.pc(), 0x10);
    let m = one(0xe1b2_0f9f, &[(2, 0x4004)]); // ldrexd at a word, not doubleword
    assert_eq!(m.cpu.mode(), Mode::ABORT);
}

#[test]
fn a_global_monitor_can_veto_a_store_exclusive() {
    #[derive(Debug, Default)]
    struct Veto {
        marks: crate::core::sync::AtomicU32,
        stores: crate::core::sync::AtomicU32,
    }
    impl GlobalMonitor for Veto {
        fn mark(&self, _: crate::core::space::RequesterId, _: u64, _: u32) {
            self.marks
                .fetch_add(1, crate::core::sync::Ordering::Relaxed);
        }
        fn store_exclusive(&self, _: crate::core::space::RequesterId, _: u64, _: u32) -> bool {
            false
        }
        fn observe_store(&self, _: crate::core::space::RequesterId, _: u64, _: u32) {
            self.stores
                .fetch_add(1, crate::core::sync::Ordering::Relaxed);
        }
    }
    let veto = Arc::new(Veto::default());
    let m = V7::a9();
    m.cpu.attach_global_monitor(veto.clone());
    // ldrex r0, [r1]; strex r2, r0, [r1]; str r0, [r1]
    m.run(&[0xe191_0f9f, 0xe181_2f90, 0xe581_0000]);
    m.set(1, 0x4000);
    m.steps(3);
    assert_eq!(m.r(2), 1);
    assert_eq!(veto.marks.load(crate::core::sync::Ordering::Relaxed), 1);
    assert_eq!(veto.stores.load(crate::core::sync::Ordering::Relaxed), 1);
}

// ---------------------------------------------------------------------------
// PSR, CPS, SETEND, SRS, RFE, exception entry
// ---------------------------------------------------------------------------

#[test]
fn msr_honours_the_v6_writable_bits() {
    // msr cpsr_fsxc, r0 (0xe12f_f000) from User mode.
    let m = V7::a9();
    m.run(&[0xe12f_f000]);
    m.cpu.set_cpsr(u32::from(Mode::USER.0));
    m.set(0, 0xffff_ffff);
    m.cpu.step();
    // NZCVQ, GE and E only: no IT, no J, no A/I/F, no T, no mode.
    assert_eq!(
        m.cpsr(),
        0xf80f_0200 | u32::from(Mode::USER.0),
        "{:08x}",
        m.cpsr()
    );

    // Privileged: A, I, F and the mode too — still not T, J or IT.
    let m = V7::a9();
    m.run(&[0xe12f_f000]);
    m.set(0, !psr::MODE | u32::from(Mode::SUPERVISOR.0));
    m.cpu.step();
    assert_eq!(m.cpsr(), 0xf80f_03c0 | u32::from(Mode::SUPERVISOR.0));

    // An ARMv5 part keeps writing whole bytes, T included.
    let v5 = V7::v5();
    v5.run(&[0xe12f_f000]);
    v5.set(0, !psr::MODE | u32::from(Mode::SYSTEM.0));
    v5.cpu.step();
    assert_eq!(v5.cpsr(), 0xffff_ffff);
}

#[test]
fn cps_changes_masks_and_mode_only_when_privileged() {
    // cpsie i
    let m = one(0xf108_0080, &[]);
    assert_eq!(m.cpsr() & psr::I, 0);
    assert_ne!(m.cpsr() & psr::F, 0);
    // cpsid if, #0x13
    let m = V7::a9();
    m.run(&[0xf10e_00d3]);
    m.cpu.set_cpsr(u32::from(Mode::SYSTEM.0));
    m.cpu.step();
    assert_eq!(m.cpu.mode(), Mode::SUPERVISOR);
    assert_eq!(m.cpsr() & (psr::I | psr::F), psr::I | psr::F);
    // From User mode it is a NOP.
    let m = V7::a9();
    m.run(&[0xf108_00c0]);
    m.cpu.set_cpsr(u32::from(Mode::USER.0) | psr::I | psr::F);
    m.cpu.step();
    assert_eq!(m.cpsr() & (psr::I | psr::F), psr::I | psr::F);
    assert_eq!(m.cpu.pc(), CODE + 4);
}

#[test]
fn setend_makes_data_accesses_big_endian_but_not_fetches() {
    let m = V7::a9();
    // setend be; ldr r0, [r1]; ldrh r2, [r1]; str r0, [r3]; setend le; ldr r4, [r1]
    m.run(&[
        0xf101_0200,
        0xe591_0000,
        0xe1d1_20b0,
        0xe583_0000,
        0xf101_0000,
        0xe591_4000,
    ]);
    m.poke(0x4000, 0x4433_2211);
    m.set(1, 0x4000);
    m.set(3, 0x5000);
    m.steps(4);
    assert_ne!(m.cpsr() & psr::E, 0);
    assert_eq!(m.r(0), 0x1122_3344);
    assert_eq!(m.r(2), 0x1122);
    assert_eq!(m.peek(0x5000), 0x4433_2211, "a store swaps back");
    m.steps(2);
    assert_eq!(m.r(4), 0x4433_2211);
}

#[test]
fn exception_entry_on_v7_sets_a_and_follows_te_and_ee() {
    #[derive(Debug)]
    struct Sctlr;
    impl Mmu for Sctlr {
        fn regime(&self) -> Regime {
            Regime {
                vector_base: 0x8000,
                thumb_exceptions: true,
                big_endian_exceptions: true,
                unaligned: true,
                ..Regime::FLAT
            }
        }
        fn translate(
            &self,
            _: &dyn PhysMem,
            va: Va,
            _: AccessKind,
            _: bool,
        ) -> core::result::Result<Pa, Fault> {
            Ok(Pa(va.0))
        }
    }
    let m = V7::a9();
    m.run(&[0xef00_0000]); // svc #0
    m.cpu.attach_mmu(Arc::new(Sctlr));
    let before = u32::from(Mode::SYSTEM.0) | psr::GE | psr::E | (1 << 12);
    m.cpu.set_cpsr(before);
    m.cpu.step();
    assert_eq!(m.cpu.mode(), Mode::SUPERVISOR);
    assert_eq!(m.cpu.pc(), 0x8008, "VBAR + 8");
    let cpsr = m.cpsr();
    assert_ne!(cpsr & psr::T, 0, "TE");
    assert_ne!(cpsr & psr::E, 0, "EE");
    assert_eq!(cpsr & psr::IT, 0, "IT cleared");
    assert_eq!(cpsr & psr::A, 0, "SVC leaves A alone");
    assert_eq!(m.cpu.regs().spsr().unwrap(), before, "SPSR has it all");

    // An IRQ masks A.
    let m = V7::a9();
    m.run(&[0xe1a0_0000]);
    m.cpu.set_cpsr(u32::from(Mode::SYSTEM.0));
    m.cpu.set_irq(true);
    m.cpu.step();
    assert_eq!(m.cpu.mode(), Mode::IRQ);
    assert_ne!(m.cpsr() & psr::A, 0);
    // An undefined instruction does not.
    let m = one(0xe7f0_00f0, &[]);
    assert!(m.undefined());
    assert_eq!(m.cpsr() & psr::A, 0);
    // ARMv5 has no A bit to set.
    let v5 = V7::v5();
    v5.run(&[0xe1a0_0000]);
    v5.cpu.set_cpsr(u32::from(Mode::SYSTEM.0));
    v5.cpu.set_irq(true);
    v5.cpu.step();
    assert_eq!(v5.cpsr() & psr::A, 0);
}

#[test]
fn an_exception_return_restores_ge_e_and_it() {
    let m = V7::a9();
    m.run(&[0xe1b0_f00e]); // movs pc, lr
    m.cpu.set_cpsr(u32::from(Mode::SUPERVISOR.0));
    let mut regs = m.cpu.regs();
    regs.spsr[Mode::SUPERVISOR.spsr_index().unwrap()] =
        u32::from(Mode::USER.0) | psr::GE | psr::E | psr::Q;
    m.cpu.set_regs(regs);
    m.set(14, 0x2000);
    m.cpu.step();
    assert_eq!(m.cpu.mode(), Mode::USER);
    assert_eq!(
        m.cpsr(),
        u32::from(Mode::USER.0) | psr::GE | psr::E | psr::Q
    );
    assert_eq!(m.cpu.pc(), 0x2000);
}

#[test]
fn srs_then_rfe_round_trips_an_exception() {
    let m = V7::a9();
    // srsdb sp!, #0x13 ; rfeia sp!
    m.run(&[0xf96d_0513, 0xf8bd_0a00]);
    m.cpu.set_cpsr(u32::from(Mode::IRQ.0) | psr::I);
    let mut regs = m.cpu.regs();
    regs.spsr[Mode::IRQ.spsr_index().unwrap()] = u32::from(Mode::USER.0) | psr::C;
    regs.banked_sp_lr[Mode::SUPERVISOR.bank()][0] = 0x6000;
    m.cpu.set_regs(regs);
    m.set(14, 0x3000);
    m.cpu.step();
    assert_eq!(m.peek(0x5ff8), 0x3000, "LR at the lower address");
    assert_eq!(m.peek(0x5ffc), u32::from(Mode::USER.0) | psr::C);
    assert_eq!(
        m.cpu.regs().reg_in_mode(Mode::SUPERVISOR, 13),
        0x5ff8,
        "the target mode's SP moved"
    );
    // Now in Supervisor, RFE pops it back.
    m.cpu.set_cpsr(u32::from(Mode::SUPERVISOR.0) | psr::I);
    m.cpu.step();
    assert_eq!(m.cpu.mode(), Mode::USER);
    assert_eq!(m.cpsr(), u32::from(Mode::USER.0) | psr::C);
    assert_eq!(m.cpu.pc(), 0x3000);
    assert_eq!(m.cpu.regs().reg_in_mode(Mode::SUPERVISOR, 13), 0x6000);
}

// ---------------------------------------------------------------------------
// Unaligned access
// ---------------------------------------------------------------------------

#[test]
fn an_unaligned_ldr_is_a_real_access_on_v7_and_a_rotate_on_v5() {
    let setup = |m: &V7| {
        m.run(&[0xe591_0000, 0xe1d1_20b0]); // ldr r0, [r1]; ldrh r2, [r1]
        m.poke(0x4000, 0x4433_2211);
        m.poke(0x4004, 0x8877_6655);
        m.set(1, 0x4001);
        m.steps(2);
    };
    let m = V7::a9();
    setup(&m);
    assert_eq!(m.r(0), 0x5544_3322);
    assert_eq!(m.r(2), 0x3322);
    let v5 = V7::v5();
    setup(&v5);
    assert_eq!(v5.r(0), 0x1144_3322, "ARMv5 rotates the aligned word");
}

#[test]
fn an_unaligned_str_writes_the_bytes_it_names() {
    let m = one(0xe581_0000, &[(0, 0xaabb_ccdd), (1, 0x4003)]); // str r0, [r1]
    assert_eq!(m.peek(0x4000), 0xdd00_0000);
    assert_eq!(m.peek(0x4004), 0x00aa_bbcc);
    let m = one(0xe1c1_00b0, &[(0, 0x1234), (1, 0x4001)]); // strh r0, [r1]
    assert_eq!(m.peek(0x4000), 0x0012_3400);
}

#[test]
fn ldm_ldrd_and_swp_still_need_alignment_on_v7() {
    for (word, regs) in [
        (0xe891_0003u32, [(1u8, 0x4002u32)]), // ldmia r1, {r0, r1}
        (0xe1c1_00d0, [(1, 0x4002)]),         // ldrd r0, r1, [r1]
        (0xe101_0092, [(1, 0x4002)]),         // swp r0, r2, [r1]
    ] {
        let m = one(word, &regs);
        assert_eq!(m.cpu.mode(), Mode::ABORT, "{word:08x}");
        assert_eq!(m.cpu.pc(), 0x10);
    }
    // LDRD needs a word, not a doubleword.
    let m = one(0xe1c1_20d0, &[(1, 0x4004)]); // ldrd r2, r3, [r1]
    assert_eq!(m.cpu.mode(), Mode::SYSTEM);
}

#[test]
fn sctlr_a_still_faults_an_unaligned_ldr() {
    let m = V7::with_config(Config {
        arch: Arch::CORTEX_A9,
        alignment_faults: true,
        ..Config::ARM926EJS
    });
    m.run(&[0xe591_0000]);
    m.set(1, 0x4001);
    m.cpu.step();
    assert_eq!(m.cpu.mode(), Mode::ABORT);
}

// ---------------------------------------------------------------------------
// Snapshot
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

fn restore(cpu: &Arm, bytes: &[u8], migrations: &Migrations) {
    let reader = StateReader::new(bytes).unwrap();
    let chunk = reader
        .load("cpu", CLASS.name, CLASS.version, migrations)
        .unwrap();
    cpu.load(&mut chunk.reader()).unwrap();
}

#[test]
fn the_monitor_and_event_survive_a_snapshot() {
    let m = V7::a9();
    m.run(&[0xe191_0f9f, 0xe320_f004, 0xe181_2f90]); // ldrex; sev; strex
    m.set(1, 0x4000);
    m.steps(2);
    let bytes = snapshot(&m.cpu);

    let other = V7::a9();
    restore(&other.cpu, &bytes, &Migrations::new());
    assert_eq!(other.cpu.exclusive_tag(), Some(0x4000));
    assert!(other.cpu.event_pending());
    assert_eq!(snapshot(&other.cpu), bytes, "save, load, save is stable");
    other.cpu.step();
    assert_eq!(other.r(2), 0, "the STREX after the restore still succeeds");
}

/// A `cpu.arm` v3 chunk of a Cortex-A9-architecture core, written by the
/// build before this change (r0 = 0x42, SP_svc = 0x8000, System mode with `C`,
/// PC = 0x1004).
const V3_FIXTURE: &str = "5253454d55534e5001000000000000000100000000000000030000000000000063707507000000000000006370752e61726d00000000000000000000000000000000000000000000000001030000000000000063707507000000000000006370752e61726d03000000d600000000000000420000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000041000001f000020000000000000000000000000000000000000000000000000008000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000010000000000000000000000000000000000000000000000000000000000";

fn a9_integer_arch() -> Arch {
    let mut arch = Arch::CORTEX_A9;
    arch.ext.vfp = None;
    arch
}

#[test]
fn a_v3_snapshot_loads_through_the_shipped_migration() {
    let bytes: Vec<u8> = (0..V3_FIXTURE.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&V3_FIXTURE[i..i + 2], 16).unwrap())
        .collect();
    let migrations = crate::machine::default_migrations().unwrap();
    // No VFP: a v3 build had none, so no v3 chunk carries a register file.
    let cpu = Arm::new(Config {
        arch: a9_integer_arch(),
        ..Config::ARM926EJS
    });
    restore(&cpu, &bytes, &migrations);
    assert_eq!(cpu.reg(0), 0x42);
    assert_eq!(cpu.regs().reg_in_mode(Mode::SUPERVISOR, 13), 0x8000);
    assert_eq!(cpu.pc(), 0x1004);
    assert_eq!(cpu.cpsr(), 0x1f | psr::C);
    assert_eq!(cpu.exclusive_tag(), None);
    assert!(!cpu.event_pending());
    // And the ARMv5 core reads the same v3 bytes as it always did.
    let v5 = Arm::new(Config::ARM926EJS);
    restore(&v5, &bytes, &migrations);
    assert_eq!(v5.regs(), cpu.regs());
}

#[test]
fn an_armv5_chunk_has_no_v6_trailer() {
    let v5 = V7::v5();
    v5.run(&[]);
    let a9 = V7::a9_integer();
    a9.run(&[]);
    assert_eq!(snapshot(&a9.cpu).len(), snapshot(&v5.cpu).len() + 7);
}

#[test]
fn flat_mmu_still_reports_no_vector_base() {
    assert_eq!(FlatMmu::new().regime().vector_base, 0);
}

#[test]
fn the_reset_pin_resets_cp15_too() {
    // SCTLR.V moves the vectors to 0xffff0000; a pulse on the reset input
    // must put them back, as it puts the MMU enable back, or a guest that
    // resets itself from a high-vector kernel resets into nowhere.
    let m = V7::with_config(Config::CORTEX_A9);
    let sctlr = m.cpu.cp15v7().expect("a Cortex-A9 has its CP15").sctlr();
    m.run(&[0xee01_0f10]); // mcr p15, 0, r0, c1, c0, 0
    m.set(0, sctlr | (1 << 13));
    m.steps(1);
    assert_ne!(m.cpu.cp15v7().unwrap().sctlr() & (1 << 13), 0, "V set");
    m.cpu.lines.request_reset();
    m.cpu.step();
    assert_eq!(
        m.cpu.cp15v7().unwrap().sctlr() & (1 << 13),
        0,
        "V back to its strap"
    );
    assert_eq!(
        m.cpu.pc(),
        0,
        "the reset vector is fetched from the low base"
    );
    assert_eq!(m.cpu.mode(), Mode::SUPERVISOR);
}
