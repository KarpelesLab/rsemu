//! PSCI: the firmware interface a kernel calls to switch the machine off.
//!
//! # Source
//!
//! *Arm Power State Coordination Interface*, ARM DEN 0022 (issue D describes
//! PSCI 1.1), for the function identifiers, the return codes and the version
//! encoding; and *SMC Calling Convention*, ARM DEN 0028, for the shape of a
//! function identifier — bit 31 is *fast call*, bit 30 selects the 32- or
//! 64-bit calling convention, and bits 29:24 are the owning entity, of which
//! `4` is the Standard Secure Service that PSCI belongs to. Both are published
//! by Arm for implementers. The identifiers and codes below were cross-checked
//! against **Trusted Firmware-A** (`include/lib/psci/psci.h`, BSD-3-Clause)
//! and the **ARM boot-wrapper** (`include/psci.h`, BSD-3-Clause), which are
//! permissive and agree with the specification exactly.
//!
//! # Why this is in the core and not in `dev/arm/`
//!
//! Because `SMC` and `HVC` are *instructions*. A PSCI call is not an address
//! and there is nothing for an address space to dispatch: the guest puts a
//! function id in `X0`, executes one instruction, and expects a result in `X0`
//! when it returns. The only place that can happen is beside the register
//! file.
//!
//! What is *not* here is what the board does about it. `SYSTEM_OFF` and
//! `SYSTEM_RESET` leave the core on a wire, and
//! [`dev::arm::power`](crate::dev::arm::power) is one board's answer to them;
//! another board could answer differently, and a core that decided for itself
//! would have taken that choice away.
//!
//! # Why `SMC` works on a core with no EL3
//!
//! Architecturally, `SMC` is UNDEFINED when EL3 is not implemented, and
//! `cpu.arm.a64` implements EL0 and EL1 only —
//! [`Config::id_aa64pfr0`](super::Config::id_aa64pfr0) says so and a guest can
//! read it. So `psci = "smc"` is the board asserting something the ID
//! registers do not: *there is a monitor here, it is not modelled as an
//! exception level, and it answers these calls*. That is exactly what a
//! machine with firmware in ROM looks like from EL1, and it is why the conduit
//! is a **construction property with `none` as an available value** rather
//! than something the core always does. A board that says `psci = "none"`
//! keeps the architectural answer, and `SMC` is UNDEFINED on it.
//!
//! The honest alternative — implementing EL3 — is a second stack pointer, a
//! second vector table, `SCR_EL3`, and a whole exception level whose only
//! inhabitant would be forty lines of `match`. `docs/platforms/arm64-virt.md`
//! records the trade.
//!
//! # What is implemented, and what is refused
//!
//! Everything a single-processor kernel calls, and nothing else. The calls
//! that bring up a second core — `CPU_ON`, `AFFINITY_INFO` — are answered but
//! not *performed*: this board has one core, so `CPU_ON` for a processor that
//! does not exist is `INVALID_PARAMETERS`, which is the specification's own
//! answer and is what a kernel that reads the device tree will never ask.
//! `PSCI_FEATURES` reports exactly the set below, so a kernel discovers the
//! gap rather than falling into it.

use super::sysreg::El;

/// Which instruction a board's guests call firmware with.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Conduit {
    /// Neither: `SMC` and `HVC` are UNDEFINED, which is the architectural
    /// answer for a core with no EL3 and no EL2.
    #[default]
    None,
    /// `SMC`, which is what a kernel running at EL1 below a monitor uses.
    Smc,
    /// `HVC`, which is what a kernel running at EL1 below a hypervisor uses.
    Hvc,
}

impl Conduit {
    /// The conduit a machine file's `psci` property names.
    #[must_use]
    pub fn by_name(name: &str) -> Option<Conduit> {
        match name {
            "none" => Some(Conduit::None),
            "smc" => Some(Conduit::Smc),
            "hvc" => Some(Conduit::Hvc),
            _ => None,
        }
    }

    /// The name a machine file writes for this conduit.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Conduit::None => "none",
            Conduit::Smc => "smc",
            Conduit::Hvc => "hvc",
        }
    }

    /// The names a machine file may write.
    pub const NAMES: &'static [&'static str] = &["none", "smc", "hvc"];
}

/// The function identifiers this core answers (DEN 0022 §5.1).
///
/// The 32-bit and 64-bit forms of one function differ only in bit 30, and a
/// caller may use either where both exist — so the dispatch below masks that
/// bit off after checking the argument width, rather than listing both.
pub mod fid {
    /// `PSCI_VERSION`.
    pub const VERSION: u32 = 0x8400_0000;
    /// `CPU_SUSPEND` (SMC32; the SMC64 form is `0xc400_0001`).
    pub const CPU_SUSPEND: u32 = 0x8400_0001;
    /// `CPU_OFF`. There is no 64-bit form: it takes no arguments.
    pub const CPU_OFF: u32 = 0x8400_0002;
    /// `CPU_ON` (SMC32; the SMC64 form is `0xc400_0003`).
    pub const CPU_ON: u32 = 0x8400_0003;
    /// `AFFINITY_INFO` (SMC32; the SMC64 form is `0xc400_0004`).
    pub const AFFINITY_INFO: u32 = 0x8400_0004;
    /// `MIGRATE_INFO_TYPE`.
    pub const MIGRATE_INFO_TYPE: u32 = 0x8400_0006;
    /// `SYSTEM_OFF`.
    pub const SYSTEM_OFF: u32 = 0x8400_0008;
    /// `SYSTEM_RESET`.
    pub const SYSTEM_RESET: u32 = 0x8400_0009;
    /// `PSCI_FEATURES`.
    pub const FEATURES: u32 = 0x8400_000a;

    /// Bit 30 of a function id: set for the 64-bit calling convention.
    pub const SMC64: u32 = 1 << 30;
}

/// The return codes (DEN 0022 table 6). Every one is a 32-bit *signed*
/// integer, which is why they are declared as `i32` and sign-extended into
/// `X0` — a kernel comparing `x0` against `-2` gets nothing useful from a
/// zero-extended `0xfffffffe`.
pub mod ret {
    /// The call did what was asked.
    pub const SUCCESS: i32 = 0;
    /// This implementation does not have that function.
    pub const NOT_SUPPORTED: i32 = -1;
    /// An argument was out of range or named something that does not exist.
    pub const INVALID_PARAMETERS: i32 = -2;
    /// The caller is not allowed to do that.
    pub const DENIED: i32 = -3;
    /// The processor named is already on.
    pub const ALREADY_ON: i32 = -4;
}

/// The version this core reports: PSCI 1.0.
///
/// Bits 31:16 are the major version and bits 15:0 the minor one (DEN 0022
/// §5.1.1), so 1.0 is `0x0001_0000` — **not** `0x0000_0100`, which is the
/// mistake that makes a kernel decide it is talking to PSCI 0.0 and give up.
pub const VERSION: u32 = 0x0001_0000;

/// What the core should do after a call, beyond writing `X0`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Effect {
    /// Nothing: the call was answered and the guest carries on.
    None,
    /// The board should switch the machine off.
    Poweroff,
    /// The board should restart the machine.
    Reboot,
}

/// What a call produced: the value for `X0`, and what the board must do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Outcome {
    /// The value to write back into `X0`.
    pub x0: u64,
    /// What has to happen outside the core.
    pub effect: Effect,
}

impl Outcome {
    /// A plain result with nothing for the board to do.
    #[must_use]
    const fn value(x0: u64) -> Outcome {
        Outcome {
            x0,
            effect: Effect::None,
        }
    }

    /// An error code, sign-extended as the specification requires.
    #[must_use]
    const fn error(code: i32) -> Outcome {
        Outcome::value(code as i64 as u64)
    }
}

/// Service a call made with `x0`-`x3` as the guest left them.
///
/// `el` is the exception level the call was made from: PSCI is a firmware
/// interface and EL0 has no business calling it, so an unprivileged call is
/// refused rather than answered. `cpus` is how many processors this machine
/// has, which is what makes `CPU_ON` for processor 1 on a single-core board an
/// honest `INVALID_PARAMETERS`.
#[must_use]
pub fn call(el: El, cpus: u64, x: [u64; 4]) -> Outcome {
    if el != El::El1 {
        // DEN 0022 §5.2.1: a call from an unprivileged level is not a PSCI
        // call at all. `NOT_SUPPORTED` rather than a fault, because the
        // conduit instruction itself is what would have faulted.
        return Outcome::error(ret::NOT_SUPPORTED);
    }
    let raw = x[0] as u32;
    // Fold the 64-bit convention onto the 32-bit id: the two forms of one
    // function differ only in bit 30 and do the same thing.
    let fid = raw & !fid::SMC64;
    match fid {
        fid::VERSION => Outcome::value(u64::from(VERSION)),
        fid::SYSTEM_OFF => Outcome {
            x0: ret::SUCCESS as u64,
            effect: Effect::Poweroff,
        },
        fid::SYSTEM_RESET => Outcome {
            x0: ret::SUCCESS as u64,
            effect: Effect::Reboot,
        },
        fid::CPU_OFF => {
            // The last processor cannot switch itself off and leave the
            // machine running: DEN 0022 makes that DENIED, and a kernel that
            // gets it prints "CPU_OFF returned -3" and stops trying, which is
            // the right outcome on a board with one core.
            Outcome::error(ret::DENIED)
        }
        fid::CPU_ON => {
            let target = x[1];
            if affinity_index(target, cpus).is_some() {
                // The processor exists and is already running, because on this
                // board every processor is.
                Outcome::error(ret::ALREADY_ON)
            } else {
                Outcome::error(ret::INVALID_PARAMETERS)
            }
        }
        fid::AFFINITY_INFO => match affinity_index(x[1], cpus) {
            // 0 is `ON`, which is the only state a processor on this board is
            // ever in (DEN 0022 §5.1.4).
            Some(_) => Outcome::value(0),
            None => Outcome::error(ret::INVALID_PARAMETERS),
        },
        // 2 is `TOS_NOT_PRESENT_MP`: there is no trusted OS to migrate, which
        // is what stops a kernel looking for one.
        fid::MIGRATE_INFO_TYPE => Outcome::value(2),
        fid::FEATURES => {
            let asked = (x[1] as u32) & !fid::SMC64;
            if implemented(asked) {
                // Zero means "implemented, with no feature flags", which is
                // the answer for every function here.
                Outcome::value(0)
            } else {
                Outcome::error(ret::NOT_SUPPORTED)
            }
        }
        // `CPU_SUSPEND` is answered rather than implemented: a kernel that
        // called it and was told SUCCESS would expect to have been suspended
        // and resumed, and this core does neither.
        _ => Outcome::error(ret::NOT_SUPPORTED),
    }
}

/// Whether `fid` is one of the functions [`call`] answers.
#[must_use]
pub fn implemented(fid: u32) -> bool {
    matches!(
        fid,
        fid::VERSION
            | fid::CPU_OFF
            | fid::CPU_ON
            | fid::AFFINITY_INFO
            | fid::MIGRATE_INFO_TYPE
            | fid::SYSTEM_OFF
            | fid::SYSTEM_RESET
            | fid::FEATURES
    )
}

/// Which processor an `MPIDR_EL1`-shaped affinity value names, if any.
///
/// This board numbers its processors in `Aff0` from zero, which is what
/// `arm.boot` describes in the device tree, so the index is the low byte and
/// every other affinity level must be zero. A target with `Aff1` set names a
/// second cluster, which this board does not have.
#[must_use]
pub fn affinity_index(target: u64, cpus: u64) -> Option<u64> {
    if target & !0xffu64 != 0 {
        return None;
    }
    (target < cpus).then_some(target)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at_el1(x0: u64, x1: u64) -> Outcome {
        call(El::El1, 1, [x0, x1, 0, 0])
    }

    #[test]
    fn the_version_is_the_one_a_kernel_looks_for() {
        // Major in the top half: 1.0 is 0x00010000. The transposed spelling
        // reads as version 0.65536 and is the classic way to get a kernel to
        // decide PSCI is not there.
        assert_eq!(at_el1(u64::from(fid::VERSION), 0).x0, 0x0001_0000);
        assert_eq!(VERSION >> 16, 1);
        assert_eq!(VERSION & 0xffff, 0);
    }

    #[test]
    fn the_two_calling_conventions_reach_the_same_function() {
        // Bit 30 selects SMC64, and a kernel uses whichever it compiled for.
        let a = at_el1(u64::from(fid::AFFINITY_INFO), 0);
        let b = at_el1(u64::from(fid::AFFINITY_INFO | fid::SMC64), 0);
        assert_eq!(a, b);
        assert_eq!(a.x0, 0, "processor 0 is ON");
    }

    #[test]
    fn system_off_asks_the_board_rather_than_deciding() {
        let out = at_el1(u64::from(fid::SYSTEM_OFF), 0);
        assert_eq!(out.effect, Effect::Poweroff);
        assert_eq!(out.x0, 0);
        assert_eq!(
            at_el1(u64::from(fid::SYSTEM_RESET), 0).effect,
            Effect::Reboot
        );
    }

    #[test]
    fn an_error_is_sign_extended_the_way_the_specification_says() {
        // `NOT_SUPPORTED` is -1 as a 32-bit signed integer, and a kernel
        // compares `x0` against it as a 64-bit negative number.
        let out = at_el1(0x8400_00ff, 0);
        assert_eq!(out.x0, u64::MAX, "-1, sign-extended");
        let out = at_el1(u64::from(fid::CPU_ON), 7);
        assert_eq!(out.x0 as i64, i64::from(ret::INVALID_PARAMETERS));
    }

    #[test]
    fn features_reports_exactly_what_call_answers() {
        for fid in [
            fid::VERSION,
            fid::CPU_OFF,
            fid::SYSTEM_OFF,
            fid::SYSTEM_RESET,
            fid::FEATURES,
        ] {
            assert_eq!(
                at_el1(u64::from(fid::FEATURES), u64::from(fid)).x0,
                0,
                "{fid:#x} is answered, so PSCI_FEATURES must say so"
            );
        }
        // And one that is not: a kernel must discover the gap here rather
        // than by calling it.
        assert_eq!(
            at_el1(u64::from(fid::FEATURES), u64::from(fid::CPU_SUSPEND)).x0 as i64,
            i64::from(ret::NOT_SUPPORTED)
        );
        assert!(!implemented(fid::CPU_SUSPEND));
    }

    #[test]
    fn a_second_processor_on_a_one_processor_board_does_not_exist() {
        assert_eq!(affinity_index(0, 1), Some(0));
        assert_eq!(affinity_index(1, 1), None);
        assert_eq!(affinity_index(1, 2), Some(1));
        // A target naming another cluster is not this board's processor 0.
        assert_eq!(affinity_index(0x100, 2), None);
        assert_eq!(
            call(El::El1, 2, [u64::from(fid::CPU_ON), 1, 0, 0]).x0 as i64,
            i64::from(ret::ALREADY_ON)
        );
    }

    #[test]
    fn an_unprivileged_call_is_refused() {
        // PSCI is a firmware interface; a thread has no business calling it.
        assert_eq!(
            call(El::El0, 1, [u64::from(fid::SYSTEM_OFF), 0, 0, 0]),
            Outcome::error(ret::NOT_SUPPORTED)
        );
    }

    #[test]
    fn a_conduit_round_trips_through_its_name() {
        for name in Conduit::NAMES {
            let conduit = Conduit::by_name(name).expect("a name this core accepts");
            assert_eq!(conduit.as_str(), *name);
        }
        assert_eq!(Conduit::by_name("psci"), None);
        assert_eq!(Conduit::default(), Conduit::None);
    }
}
