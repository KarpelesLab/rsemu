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
//! Everything a kernel calls to bring a machine up and take it down:
//! `PSCI_VERSION`, `SYSTEM_OFF`, `SYSTEM_RESET`, `MIGRATE_INFO_TYPE`,
//! `PSCI_FEATURES`, and — on a board that put its processors in a
//! [`Cluster`] — `CPU_ON`, `CPU_OFF` and `AFFINITY_INFO` for real.
//! `CPU_SUSPEND` is refused rather than answered, because a kernel told
//! `SUCCESS` would expect to have been suspended and resumed and this core
//! does neither. `PSCI_FEATURES` reports exactly the set [`implemented`]
//! names, so a kernel discovers the gap rather than falling into it.
//!
//! # The roster, and why `call` is not a pure function any more
//!
//! `CPU_ON` is the one call whose whole point is to reach a **sibling**: the
//! processor executing the `SMC` must change another processor's state. There
//! is no route from a core to its siblings through the address space — a
//! processor is not a memory-mapped device — so the boards's processors meet
//! by name in a [`Cluster`], a `HostKind::rendezvous` host object in the
//! build's `HostObjects`, exactly the way every local APIC on a `pc` board
//! meets on one `apic.bus`.
//!
//! What crosses is four atomics on the sibling's [`Lines`], for
//! the same reason `Lines` exists at all: the sender is holding its own
//! `BUS`-ranked execution lock when it writes them, and taking the sibling's
//! would be the deadlock the ranked order exists to prevent. The sibling
//! notices at its own next instruction boundary. The roster's own lock ranks
//! under `BUS` and is released before anything is done with what it held.
//!
//! A board that names no cluster keeps the old answers, which were honest for
//! it: every processor it has is running, so `AFFINITY_INFO` is `ON`,
//! `CPU_ON` is `ALREADY_ON`, and `CPU_OFF` is `DENIED` because the last
//! processor cannot switch itself off and leave the machine running.

use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;

use crate::core::error::Result;
use crate::core::hosts::{HostKind, HostObjects};
use crate::core::props::Props;
use crate::core::sync::{LockRank, Mutex};

use super::Lines;
use super::sysreg::El;

/// The kind a processor cluster is filed under in a build's `HostObjects`.
pub const CLUSTER_KIND: HostKind = HostKind::rendezvous("arm-cluster");

/// The cluster name a core gets when a machine description does not say.
pub const DEFAULT_CLUSTER: &str = "cluster";

/// Where the roster's lock sits in the ranked order.
///
/// **Below [`LockRank::BUS`] and above [`LockRank::DEVICE`]**, and forced
/// rather than chosen for the same reason `apic.bus`'s roster is: a CPU holds
/// a `BUS`-ranked lock across the instruction it is executing, and this is
/// reached from inside one. It ranks above `DEVICE` because it is released
/// before the caller touches what it held — which here is a sibling's
/// [`Lines`], and those are atomics with no lock of their own at all.
pub const CLUSTER_RANK: LockRank = LockRank::new(0x4c62);

/// The processors on one board, as `CPU_ON` can see them.
///
/// Held **weakly**: the machine owns the cores and a roster merely refers to
/// them (`ROADMAP.md` §4.3's weak edge). A `Weak` that no longer upgrades is a
/// processor that has been dropped, which is a processor `CPU_ON` must not
/// find.
#[derive(Debug)]
pub struct Cluster {
    members: Mutex<Vec<(u64, Weak<Lines>)>>,
}

impl Default for Cluster {
    /// Written out rather than derived, because a derived one would build the
    /// roster's lock at [`LockRank::LEAF`] and a `LEAF` lock nests under
    /// anything — which is the opposite of what [`CLUSTER_RANK`] says and
    /// exactly the kind of silent disagreement the ranked order exists to
    /// catch.
    fn default() -> Cluster {
        Cluster::new()
    }
}

impl Cluster {
    /// An empty roster.
    #[must_use]
    pub fn new() -> Cluster {
        Cluster {
            members: Mutex::with_rank(CLUSTER_RANK, Vec::new()),
        }
    }

    /// Put a processor on the roster under the affinity its `MPIDR_EL1` names.
    ///
    /// Returns `false` if a live processor already holds that affinity: two
    /// cores with one `MPIDR_EL1` is a board that cannot mean anything, and
    /// the caller reports it rather than picking one. (It is also the defect a
    /// copied-and-pasted `object cpu1` produces, which is why it is checked
    /// here rather than left to the guest to discover.)
    pub fn join(&self, affinity: u64, lines: Weak<Lines>) -> bool {
        let mut members = self.members.lock();
        // A processor that has been dropped is not one anything can find, so
        // its entry goes rather than accumulating: a test that builds and
        // discards machines in a loop shares one `HostObjects` with all of
        // them, and a roster that only ever grew would be a leak measured in
        // machines.
        members.retain(|(_, other)| other.strong_count() > 0);
        if members.iter().any(|(id, _)| *id == affinity) {
            return false;
        }
        members.push((affinity, lines));
        true
    }

    /// The processor at `affinity`, if this board has one.
    ///
    /// The lock is released before the answer is handed back, explicitly: the
    /// caller is about to write another core's interrupt lines.
    #[must_use]
    pub fn find(&self, affinity: u64) -> Option<Arc<Lines>> {
        let members = self.members.lock();
        let found = members
            .iter()
            .find(|(id, _)| *id == affinity)
            .and_then(|(_, lines)| lines.upgrade());
        drop(members);
        found
    }

    /// How many processors are on the roster.
    #[must_use]
    pub fn len(&self) -> usize {
        self.members
            .lock()
            .iter()
            .filter(|(_, lines)| lines.strong_count() > 0)
            .count()
    }

    /// Whether the roster is empty, which is what a board with no `cluster`
    /// property has.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// How many of them are powered on.
    ///
    /// What makes `CPU_OFF` on the last one `DENIED`: a machine whose every
    /// processor is off is a machine nothing can ever start again, and DEN
    /// 0022 §5.1.2 makes refusing it the implementation's job.
    #[must_use]
    pub fn powered(&self) -> usize {
        let members = self.members.lock();
        let live: Vec<Arc<Lines>> = members.iter().filter_map(|(_, l)| l.upgrade()).collect();
        drop(members);
        live.iter().filter(|lines| lines.powered()).count()
    }

    /// The cluster `name` refers to in `hosts`, creating it on first mention.
    ///
    /// The **host** side of the rendezvous.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if another kind of host object already holds
    /// that name.
    pub fn open(hosts: &HostObjects, name: &str) -> Result<Arc<Cluster>> {
        hosts.open(CLUSTER_KIND, name, Cluster::new)
    }

    /// The cluster `name` refers to in the build these properties belong to.
    ///
    /// The **device** side, called from `new(props)`. A `Props` that belongs
    /// to no build gets a private cluster, so a core a unit test constructed
    /// by hand still works and simply meets nobody.
    ///
    /// # Errors
    ///
    /// As [`Cluster::open`].
    pub fn attach(props: &Props, name: &str) -> Result<Arc<Cluster>> {
        props.host(CLUSTER_KIND, name, Cluster::new)
    }
}

/// The affinity an `MPIDR_EL1` value names: `Aff0`, `Aff1`, `Aff2` and
/// `Aff3`, and nothing else.
///
/// Bit 31 is RES1, bit 30 is `U` and bit 24 is `MT` (DDI 0487 D17.2.100).
/// None of the three is part of a processor's identity, and a `CPU_ON` whose
/// `target_cpu` carried bit 31 — because whoever wrote the machine file
/// copied `MPIDR_EL1` verbatim — must still find the processor it names.
#[must_use]
pub const fn affinity_of(mpidr: u64) -> u64 {
    mpidr & 0x0000_00ff_00ff_ffff
}

/// The affinity a `CPU_ON` or `AFFINITY_INFO` argument names, if it is a
/// legal one.
///
/// DEN 0022 §5.1.3: `target_cpu` is an `MPIDR_EL1`-shaped value in which
/// every bit outside the four affinity fields is zero. A caller that sets one
/// is not naming a processor, and `INVALID_PARAMETERS` is the answer — not a
/// masked-off guess at what it meant.
#[must_use]
pub const fn target_affinity(target: u64) -> Option<u64> {
    if target & !0x0000_00ff_00ff_ffffu64 != 0 {
        None
    } else {
        Some(target)
    }
}

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

/// The processors this call can see: the board's roster, and the caller's own
/// lines.
///
/// A struct rather than three more arguments because the three are one fact —
/// *who is on this board and which of them is asking* — and because a
/// `cluster` of `None` and a `cpus` of 2 is the combination whose meaning has
/// to be stated once: a board that declared two processors and put neither on
/// a roster, which is what `secondary = "spin-table"` is.
#[derive(Debug, Clone, Copy)]
pub struct Siblings<'a> {
    /// The board's roster, if the machine file gave the processors a cluster.
    pub cluster: Option<&'a Cluster>,
    /// How many processors the board says it has, for a board with no roster.
    pub cpus: u64,
    /// The calling processor's own interrupt lines, which is what `CPU_OFF`
    /// switches off.
    pub me: &'a Lines,
}

/// Service a call made with `x0`-`x3` as the guest left them.
///
/// `el` is the exception level the call was made from: PSCI is a firmware
/// interface and EL0 has no business calling it, so an unprivileged call is
/// refused rather than answered.
///
/// **Not a pure function.** `CPU_ON` writes the target's start request into
/// its [`Lines`] and `CPU_OFF` clears the caller's own `powered` flag; both
/// are atomics, written with no lock held but the caller's own, and both are
/// read by their owner at its next instruction boundary. Everything a *board*
/// must do is still returned in [`Outcome::effect`] rather than done here.
pub fn call(el: El, siblings: Siblings<'_>, x: [u64; 4]) -> Outcome {
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
    // The target of `CPU_ON` and `AFFINITY_INFO`, resolved once: `None` on a
    // board with no roster, or when `x1` names no processor this board has.
    let target = || {
        siblings
            .cluster
            .zip(target_affinity(x[1]))
            .and_then(|(c, a)| c.find(a))
    };
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
        fid::CPU_OFF => match siblings.cluster {
            // The last powered processor cannot switch itself off and leave
            // the machine running: DEN 0022 §5.1.2 makes that DENIED, and a
            // kernel that gets it prints "CPU_OFF returned -3" and stops
            // trying, which is the right outcome on a board with one core.
            Some(cluster) if cluster.powered() > 1 => {
                siblings.me.power_off();
                // DEN 0022: `CPU_OFF` does not return on success. The value is
                // written into `X0` anyway and the core stops before it can
                // read it — which is the honest shape of "does not return"
                // when the caller is an interpreter that must return
                // *something* to its own step loop.
                Outcome::value(ret::SUCCESS as u64)
            }
            _ => Outcome::error(ret::DENIED),
        },
        fid::CPU_ON => match siblings.cluster {
            Some(_) => match target() {
                None => Outcome::error(ret::INVALID_PARAMETERS),
                Some(lines) if lines.powered() => Outcome::error(ret::ALREADY_ON),
                Some(lines) => {
                    // The entry point is `x2` and the context id `x3`, and the
                    // started processor enters with `X0 = context_id`
                    // (DEN 0022 §5.1.3). Applied by the target itself, at its
                    // own next instruction boundary.
                    lines.request_start(x[2], x[3]);
                    Outcome::value(ret::SUCCESS as u64)
                }
            },
            // No roster: every processor this board has is running, so the
            // only two answers are "that one is already on" and "there is no
            // such processor".
            None => {
                if affinity_index(x[1], siblings.cpus).is_some() {
                    Outcome::error(ret::ALREADY_ON)
                } else {
                    Outcome::error(ret::INVALID_PARAMETERS)
                }
            }
        },
        // 0 is `ON` and 1 is `OFF` (DEN 0022 §5.1.4). This used to be an
        // unconditional 0, which was only honest because every processor was
        // running; a kernel reads it in a loop after `CPU_ON` and after
        // `CPU_OFF`, and a constant `ON` makes an offline processor look hung.
        fid::AFFINITY_INFO => match siblings.cluster {
            Some(_) => match target() {
                Some(lines) => Outcome::value(u64::from(!lines.powered())),
                None => Outcome::error(ret::INVALID_PARAMETERS),
            },
            None => match affinity_index(x[1], siblings.cpus) {
                Some(_) => Outcome::value(0),
                None => Outcome::error(ret::INVALID_PARAMETERS),
            },
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

    /// A board with no roster: what every one-processor machine is, and what
    /// `arm64-virt-smp` was until `CPU_ON` landed.
    fn alone(cpus: u64) -> Lines {
        let _ = cpus;
        Lines::default()
    }

    fn at_el1(x0: u64, x1: u64) -> Outcome {
        let me = alone(1);
        call(
            El::El1,
            Siblings {
                cluster: None,
                cpus: 1,
                me: &me,
            },
            [x0, x1, 0, 0],
        )
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
            {
                let me = alone(2);
                call(
                    El::El1,
                    Siblings {
                        cluster: None,
                        cpus: 2,
                        me: &me,
                    },
                    [u64::from(fid::CPU_ON), 1, 0, 0],
                )
                .x0 as i64
            },
            i64::from(ret::ALREADY_ON)
        );
    }

    #[test]
    fn an_unprivileged_call_is_refused() {
        // PSCI is a firmware interface; a thread has no business calling it.
        assert_eq!(
            {
                let me = alone(1);
                call(
                    El::El0,
                    Siblings {
                        cluster: None,
                        cpus: 1,
                        me: &me,
                    },
                    [u64::from(fid::SYSTEM_OFF), 0, 0, 0],
                )
            },
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

    /// A board whose processors can see each other, and the four calls that
    /// only mean anything on one.
    ///
    /// `CPU_ON` for a processor that is off starts it; for one that is on it
    /// is `ALREADY_ON`; for an affinity nobody has it is `INVALID_PARAMETERS`.
    /// What "starts it" means here is the *request* — the entry point and the
    /// context id land in the target's own lines, and the target applies them
    /// itself at its next instruction boundary.
    #[test]
    fn cpu_on_reaches_a_sibling_through_the_roster() {
        let cluster = Cluster::new();
        let boot = Arc::new(Lines::default());
        let second = Arc::new(Lines::default());
        second.set_powered(false);
        assert!(cluster.join(0, Arc::downgrade(&boot)));
        assert!(cluster.join(1, Arc::downgrade(&second)));
        assert_eq!(cluster.len(), 2);
        assert_eq!(cluster.powered(), 1, "only the boot processor is running");

        let on = |x: [u64; 4]| {
            call(
                El::El1,
                Siblings {
                    cluster: Some(&cluster),
                    cpus: 2,
                    me: &boot,
                },
                x,
            )
        };

        // Before: affinity 1 is OFF, which is 1 and not the constant 0 this
        // call used to answer.
        assert_eq!(on([u64::from(fid::AFFINITY_INFO), 1, 0, 0]).x0, 1);
        assert_eq!(on([u64::from(fid::AFFINITY_INFO), 0, 0, 0]).x0, 0);

        // `CPU_ON(1, entry, context)`.
        let out = on([u64::from(fid::CPU_ON), 1, 0x4020_0000, 0xdead_beef]);
        assert_eq!(out.x0 as i64, i64::from(ret::SUCCESS));
        assert_eq!(out.effect, Effect::None, "nothing for the board to do");
        assert!(second.powered(), "and it is on now");
        assert_eq!(on([u64::from(fid::AFFINITY_INFO), 1, 0, 0]).x0, 0);
        assert_eq!(second.take_start(), Some((0x4020_0000, 0xdead_beef)));
        assert_eq!(second.take_start(), None, "taken once and only once");

        // A second `CPU_ON` for a processor that is on.
        assert_eq!(
            on([u64::from(fid::CPU_ON), 1, 0x4020_0000, 0]).x0 as i64,
            i64::from(ret::ALREADY_ON)
        );
        // And one for an affinity this board has not got.
        assert_eq!(
            on([u64::from(fid::CPU_ON), 2, 0x4020_0000, 0]).x0 as i64,
            i64::from(ret::INVALID_PARAMETERS)
        );
        assert_eq!(
            on([u64::from(fid::AFFINITY_INFO), 2, 0, 0]).x0 as i64,
            i64::from(ret::INVALID_PARAMETERS)
        );
    }

    /// `CPU_OFF` switches the *caller* off — the half a spin table cannot do
    /// at all — and the last processor standing is refused.
    #[test]
    fn cpu_off_stops_the_caller_and_never_the_last_one() {
        let cluster = Cluster::new();
        let boot = Arc::new(Lines::default());
        let second = Arc::new(Lines::default());
        cluster.join(0, Arc::downgrade(&boot));
        cluster.join(1, Arc::downgrade(&second));
        let off = |me: &Lines| {
            call(
                El::El1,
                Siblings {
                    cluster: Some(&cluster),
                    cpus: 2,
                    me,
                },
                [u64::from(fid::CPU_OFF), 0, 0, 0],
            )
        };
        // Two are running, so the second one may stop.
        assert_eq!(off(&second).x0 as i64, i64::from(ret::SUCCESS));
        assert!(!second.powered());
        assert_eq!(cluster.powered(), 1);
        // The last one may not: a machine with every processor off is a
        // machine nothing can start again.
        assert_eq!(off(&boot).x0 as i64, i64::from(ret::DENIED));
        assert!(boot.powered(), "and it is still running");
    }

    /// A board with no roster keeps every answer it had, which is the whole
    /// of what a spin-table board and `a64-mini` rely on.
    #[test]
    fn a_board_with_no_roster_answers_the_way_it_always_did() {
        let me = Lines::default();
        let ask = |cpus: u64, x: [u64; 4]| {
            call(
                El::El1,
                Siblings {
                    cluster: None,
                    cpus,
                    me: &me,
                },
                x,
            )
        };
        assert_eq!(ask(2, [u64::from(fid::AFFINITY_INFO), 1, 0, 0]).x0, 0);
        assert_eq!(
            ask(2, [u64::from(fid::CPU_ON), 1, 0, 0]).x0 as i64,
            i64::from(ret::ALREADY_ON)
        );
        assert_eq!(
            ask(1, [u64::from(fid::CPU_ON), 1, 0, 0]).x0 as i64,
            i64::from(ret::INVALID_PARAMETERS)
        );
        assert_eq!(
            ask(2, [u64::from(fid::CPU_OFF), 0, 0, 0]).x0 as i64,
            i64::from(ret::DENIED)
        );
        assert!(me.powered(), "and nothing switched it off");
    }

    /// An `MPIDR_EL1` value is not an affinity: bit 31 is RES1 and is set on
    /// every real one, and a `CPU_ON` `target_cpu` that carries it is not
    /// naming a processor.
    #[test]
    fn an_affinity_is_the_four_fields_and_nothing_else() {
        // What a machine file writes for processor 1 of `arm64-virt-smp`.
        assert_eq!(affinity_of(0x8000_0001), 1);
        assert_eq!(affinity_of(0x8000_0000), 0);
        // Aff1 and Aff2 survive; the `U` and `MT` bits do not.
        assert_eq!(affinity_of(0x8100_0203), 0x0203);
        assert_eq!(affinity_of(0x0000_00ff_00ff_ffff), 0x0000_00ff_00ff_ffff);
        // A target with a bit outside the affinity fields names nothing.
        assert_eq!(target_affinity(0x8000_0001), None);
        assert_eq!(target_affinity(1), Some(1));
        assert_eq!(target_affinity(0x0100_0000), None, "bits 31:24 are zero");
        assert_eq!(target_affinity(0x0000_0100_0000_0000), None, "and 63:40");

        // And the roster is keyed on the affinity, so a core whose `MPIDR` has
        // bit 31 set is still found by a kernel's `CPU_ON(1, …)`.
        let cluster = Cluster::new();
        let lines = Arc::new(Lines::default());
        cluster.join(affinity_of(0x8000_0001), Arc::downgrade(&lines));
        assert!(cluster.find(1).is_some());
        assert!(cluster.find(0x8000_0001).is_none());
    }

    /// Two processors with one `MPIDR_EL1` is a board that cannot mean
    /// anything, and the roster says so rather than picking one.
    #[test]
    fn one_affinity_belongs_to_one_processor() {
        let cluster = Cluster::new();
        let a = Arc::new(Lines::default());
        let b = Arc::new(Lines::default());
        assert!(cluster.join(7, Arc::downgrade(&a)));
        assert!(!cluster.join(7, Arc::downgrade(&b)));
        // A processor that has been dropped is not one `CPU_ON` can find, and
        // its affinity is free again — the roster holds a `Weak` precisely so
        // that a machine torn down and rebuilt does not accumulate ghosts.
        drop(a);
        assert!(cluster.find(7).is_none());
        assert_eq!(cluster.len(), 0);
        assert!(cluster.join(7, Arc::downgrade(&b)));
    }
}
