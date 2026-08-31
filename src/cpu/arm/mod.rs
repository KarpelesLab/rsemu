//! The ARM family.
//!
//! This module is the family, not a core. It always compiles and links
//! nothing; each core underneath it is a separate Cargo feature, and a build
//! that wants an ARM926EJ-S does not pay for a Cortex-M4.
//!
//! # Why the split is by profile
//!
//! ARM is cut here **by profile rather than by architecture version**, because
//! that is where the machine actually changes shape:
//!
//! | | [`aprofile`] | `v7m` |
//! |---|---|---|
//! | instruction sets | A32 and Thumb | Thumb-2 only |
//! | registers | seven modes, banked | one bank, two stack pointers |
//! | status | `CPSR`/`SPSR` | `xPSR` |
//! | system regs | CP15, a coprocessor | a memory-mapped SCB |
//! | exceptions | eight vectors at a fixed base | a relocatable NVIC table |
//!
//! Versions *within* a profile share all of that, so they share a core and
//! differ by a construction property. ARMv6 adds ten things to ARMv5TE — media
//! SIMD, `LDREX`/`STREX`, VMSAv6, TrustZone among them — while keeping all ten
//! of the rows above, so it belongs in [`aprofile`] rather than in a module of
//! its own that would be a near-copy. ARMv7E-M keeps none of them.
//!
//! # Why a version is not an ordering
//!
//! The obvious model, a `Version` enum compared with `>=`, is wrong: ARM
//! versions are a lattice of independently optional extensions, not a chain.
//! DSP is in ARMv5TE but not in plain ARMv5; Thumb-2 arrives in v6T2, *after*
//! v6 and v6K, which lack it; ARM1176JZF-S has TrustZone and VFPv2 where
//! ARM1136J-S, also ARMv6, has neither. So a core takes a profile, a version,
//! an independently selectable extension set and a memory model, and the
//! public surface is a named part — `Config::ARM926EJS`, not a hand-assembled
//! set of flags.
//!
//! Guests probe for features by executing an instruction and catching the
//! `UNDEF`, so an instruction the configured part does not have must trap
//! rather than execute. That is a property of the instance, which is why it
//! cannot live in a Cargo feature: features are additive and unified across a
//! whole compilation, and ROADMAP.md §2 promises machines that run an
//! ARM1176JZF-S beside a Cortex-A8 in one binary. Features decide what is
//! *compiled*; the configuration decides what an *instance* does.
//!
//! ROADMAP.md §6.1 and §6.1.1 carry the long form.

#[cfg(feature = "cpu-arm-aprofile")]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-aprofile")))]
pub mod aprofile;

#[cfg(feature = "cpu-arm-v7m")]
#[cfg_attr(docsrs, doc(cfg(feature = "cpu-arm-v7m")))]
pub mod v7m;
