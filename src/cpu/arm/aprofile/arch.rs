//! Which architecture a core implements: a version plus a set of
//! independently selectable extensions.
//!
//! `ROADMAP.md` §6.1.1 is the long form of why this is a lattice and not a
//! ladder. The short form: ARM versions do not nest. Thumb-2 arrives in v6T2,
//! *after* v6 and v6K, which lack it; the Security Extensions and the
//! Multiprocessing Extensions are independently present or absent within one
//! version; VFP is optional everywhere. So a decode site never asks
//! `version >= V6`. It asks the one question it means — "does this part have
//! the v6 media instructions?" — and the answer is a field here.
//!
//! [`Version`] still exists, because a handful of behaviours genuinely are the
//! version's rather than an extension's: what an unaligned `LDR` does, and
//! whether a data-processing write to the PC interworks. Those read it through
//! the named helpers below rather than by comparing it.
//!
//! # Sources
//!
//! *ARM Architecture Reference Manual, ARMv7-A and ARMv7-R edition* (ARM DDI
//! 0406C), A1.3 ("Architecture versions, profiles, and variants"), A1.4
//! ("Architecture extensions") and appendix D ("Differences between ARMv6 and
//! ARMv7") and appendix E ("ARMv4 and ARMv5 differences"); *Cortex-A9
//! Technical Reference Manual* (ARM DDI 0388) chapter 1 for which options the
//! part has.

/// An architecture version.
///
/// A `#[repr(transparent)]` newtype with named constants, not an enum
/// (CLAUDE.md, "Type conventions"). Deliberately **not** `PartialOrd`: the
/// versions are not a chain, and an ordering would invite exactly the
/// `>= V6` comparison this module exists to prevent.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Version(pub u8);

impl Version {
    /// ARMv4T: ARM7TDMI.
    pub const V4T: Version = Version(0x4b);
    /// ARMv5TE(J): ARM926EJ-S.
    pub const V5TE: Version = Version(0x5e);
    /// ARMv6: ARM1136.
    pub const V6: Version = Version(0x60);
    /// ARMv6K: ARM11 MPCore, ARM1176.
    pub const V6K: Version = Version(0x6b);
    /// ARMv6T2: ARM1156T2.
    pub const V6T2: Version = Version(0x62);
    /// ARMv7: Cortex-A8, Cortex-A9.
    pub const V7: Version = Version(0x70);

    /// The name a debugger or an error message prints.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self.0 {
            0x4b => "ARMv4T",
            0x5e => "ARMv5TE",
            0x60 => "ARMv6",
            0x6b => "ARMv6K",
            0x62 => "ARMv6T2",
            0x70 => "ARMv7",
            _ => "ARM?",
        }
    }
}

/// Which VFP register file and instruction set a part has, if any.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Vfp {
    /// The VFP architecture version: 2 or 3.
    pub version: u8,
    /// Thirty-two double registers rather than sixteen (VFPv3-D32).
    pub d32: bool,
}

impl Vfp {
    /// VFPv2, as on ARM1136JF-S and ARM1176JZF-S.
    pub const V2: Vfp = Vfp {
        version: 2,
        d32: false,
    };
    /// VFPv3-D16.
    pub const V3_D16: Vfp = Vfp {
        version: 3,
        d32: false,
    };
    /// VFPv3-D32, which is what a Cortex-A9 with NEON has.
    pub const V3_D32: Vfp = Vfp {
        version: 3,
        d32: true,
    };
}

/// The independently selectable parts of the architecture.
///
/// Total and un-`cfg`'d (`ROADMAP.md` §6.1.1): every field exists in every
/// build, so `Arch` has one shape and a downstream crate can name a preset
/// portably. Whether the *code* behind a field was compiled in is a separate
/// question, answered at construction.
#[allow(clippy::struct_excessive_bools)] // It is a set of independent flags.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Extensions {
    /// The 16-bit Thumb instruction set (T).
    pub thumb: bool,
    /// The DSP extensions (E): `QADD`, `SMLA<x><y>`, `LDRD`/`STRD`, `PLD`.
    pub dsp: bool,
    /// The ARMv6 additions to A32: the media instructions (`REV`, the
    /// extends, `SEL`, the parallel add/subtract family, `SSAT`/`USAT`,
    /// `PKH`, the dual multiplies, `UMAAL`), `CPS`, `SRS`/`RFE`, `SETEND`,
    /// `LDREX`/`STREX`, and unaligned access support.
    pub v6: bool,
    /// The ARMv6K additions: `LDREXB`/`LDREXH`/`LDREXD` and the matching
    /// stores, `CLREX`, and the `YIELD`/`WFE`/`WFI`/`SEV` hints.
    pub v6k: bool,
    /// Thumb-2 (v6T2): the 32-bit Thumb encodings and `IT`, plus the A32
    /// instructions that arrived with it — `MOVW`/`MOVT`, `BFC`/`BFI`,
    /// `SBFX`/`UBFX`, `RBIT`, `MLS`, and the unprivileged halfword and
    /// signed-byte loads and stores.
    pub thumb2: bool,
    /// The ARMv7 additions: `DMB`/`DSB`/`ISB`, `PLI`, `DBG`, and the
    /// interworking data-processing write to the PC.
    pub v7: bool,
    /// The Multiprocessing Extensions: `PLDW`, and `MPIDR` reporting a
    /// cluster.
    pub mp: bool,
    /// The Security Extensions (TrustZone): Monitor mode, `SMC`, and the
    /// banked CP15 registers.
    pub security: bool,
    /// `SDIV`/`UDIV` in the Thumb instruction set.
    pub idiv_thumb: bool,
    /// `SDIV`/`UDIV` in the ARM instruction set.
    pub idiv_arm: bool,
    /// The VFP floating-point extension, if present.
    pub vfp: Option<Vfp>,
    /// Advanced SIMD (NEON). Requires [`vfp`](Extensions::vfp).
    pub neon: bool,
}

impl Extensions {
    /// Nothing beyond the base instruction set.
    pub const NONE: Extensions = Extensions {
        thumb: false,
        dsp: false,
        v6: false,
        v6k: false,
        thumb2: false,
        v7: false,
        mp: false,
        security: false,
        idiv_thumb: false,
        idiv_arm: false,
        vfp: None,
        neon: false,
    };
}

/// A core's architecture: its version and its extensions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Arch {
    /// The architecture version.
    pub version: Version,
    /// The extensions this part has.
    pub ext: Extensions,
}

impl Arch {
    /// ARMv5TE, as implemented by an ARM926EJ-S: Thumb and the DSP extensions.
    /// Jazelle is not modelled; `BXJ` behaves as `BX`, which is what the
    /// architecture permits of a trivial Jazelle implementation.
    pub const V5TE: Arch = Arch {
        version: Version::V5TE,
        ext: Extensions {
            thumb: true,
            dsp: true,
            ..Extensions::NONE
        },
    };

    /// A Cortex-A9 MPCore (ARMv7-A) as the R-Car H1 builds it: Thumb-2, the
    /// Security and Multiprocessing Extensions, VFPv3-D32 and NEON, no
    /// hardware divide (Cortex-A9 TRM 1.1 and 1.3).
    ///
    /// **NEON is off** until this core implements Advanced SIMD. The part has
    /// it, but a preset that claims an extension the interpreter cannot
    /// execute would tell the guest (through `MVFR1`) to use instructions that
    /// then trap. Absent-and-reported-absent is honest; present-and-broken is
    /// not.
    pub const CORTEX_A9: Arch = Arch {
        version: Version::V7,
        ext: Extensions {
            thumb: true,
            dsp: true,
            v6: true,
            v6k: true,
            thumb2: true,
            v7: true,
            mp: true,
            security: true,
            idiv_thumb: false,
            idiv_arm: false,
            vfp: Some(Vfp::V3_D32),
            neon: false,
        },
    };

    /// Whether an unaligned word or halfword access is performed as such
    /// (when `SCTLR.U` or ARMv7 says so) rather than rotated.
    ///
    /// ARMv6 introduced unaligned support behind `SCTLR.U`; ARMv7 made it the
    /// only behaviour (DDI 0406C A3.2.1, appendix D12.3). Whether the bit is
    /// set is the MMU's to report, so this is only "can the part do it at
    /// all".
    #[must_use]
    pub const fn has_unaligned(self) -> bool {
        self.ext.v6
    }

    /// Whether a data-processing instruction writing the PC in ARM state
    /// interworks, the way `BX` does (`ALUWritePC`, DDI 0406C A2.3.2). ARMv7
    /// only; on earlier parts it is a plain branch.
    #[must_use]
    pub const fn alu_write_pc_interworks(self) -> bool {
        self.ext.v7
    }
}

impl Default for Arch {
    fn default() -> Arch {
        Arch::V5TE
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_presets_say_what_their_parts_have() {
        // Read through a binding: asserting on a `const` field directly is a
        // compile-time fact clippy rightly calls pointless, but the question
        // here is what the presets *say*, and that is what a test pins.
        let (v5, a9) = (Arch::V5TE, Arch::CORTEX_A9);
        assert!(!v5.ext.v6);
        assert!(!v5.has_unaligned());
        assert!(a9.ext.thumb2);
        assert!(a9.alu_write_pc_interworks());
        assert_eq!(a9.ext.vfp, Some(Vfp::V3_D32));
        assert!(!a9.ext.idiv_arm, "Cortex-A9 has no divider");
    }
}
