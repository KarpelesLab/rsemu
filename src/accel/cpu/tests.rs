//! Tests for the accelerated `cpu.x86` device.
//!
//! Everything that needs `/dev/kvm` skips without it; everything that does not
//! is written so that it does not, because a decision that can only be checked
//! on a host with a hypervisor is a decision that will not be checked.

use super::*;
use crate::core::props::Props;
use crate::core::wire::{Level, WireIdAllocator};

/// A host with a hypervisor, or `None`.
///
/// A `/dev/kvm` that is present and unusable is a failure rather than a skip,
/// for the reason `accel::kvm`'s own suite gives.
fn cpus() -> Option<Arc<AccelCpus>> {
    match AccelCpus::open(ThreadingMode::Parallel) {
        Ok(host) => Some(host),
        Err(e) if e.is_unavailable() => None,
        Err(e) => panic!("/dev/kvm is present but unusable: {e}"),
    }
}

#[test]
fn a_deterministic_threading_mode_is_refused_before_anything_is_opened() {
    // Structural, and checkable on a host with no hypervisor at all: the mode
    // is looked at *before* `/dev/kvm`, so the refusal is the one a caller
    // gets rather than "no KVM here".
    let refused = AccelCpus::open(ThreadingMode::Deterministic).unwrap_err();
    assert!(matches!(refused, AccelError::Nondeterministic(_)));
    assert!(!refused.is_unavailable());
    assert!(alloc::string::ToString::to_string(&refused).contains("not"));
}

#[test]
fn the_install_rank_is_outside_the_lock_the_vm_takes_under_it() {
    // `Vm::slots` is at `MACHINE` and is acquired while this one is held, so
    // this one has to be strictly outside it or the debug rank check fires —
    // which it did, the first time this module was written.
    assert!(INSTALL_RANK < LockRank::MACHINE);
}

#[test]
fn an_nmi_is_latched_once_per_rising_edge() {
    let latch = NmiLatch::default();
    assert!(!latch.take(), "nothing is owed out of construction");

    latch.set(true);
    assert!(latch.take(), "the edge is owed");
    assert!(!latch.take(), "and taken exactly once");

    // A level that stays high is not a second edge.
    latch.set(true);
    assert!(!latch.take());

    // It becomes one after the line drops.
    latch.set(false);
    latch.set(true);
    assert!(latch.take());
}

#[test]
fn an_nmi_pin_wire_ors_its_sources() {
    let ids = WireIdAllocator::new();
    let (a, b) = (ids.alloc(), ids.alloc());
    let latch = Arc::new(NmiLatch::default());
    let pin = NmiPin {
        latch: Arc::clone(&latch),
        inputs: FanIn::new(&[a, b]),
    };

    pin.set_level(a, 0, Level::High);
    assert!(latch.take(), "one driver asserting asserts the pin");
    // The second driver asserting is not a new edge, because the net was
    // already high — the wired-OR the PC's parity checker and coprocessor
    // share.
    pin.set_level(b, 0, Level::High);
    assert!(!latch.take());
    // And the first releasing does not drop the net while the second holds it.
    pin.set_level(a, 0, Level::Low);
    pin.set_level(b, 0, Level::Low);
    pin.set_level(a, 0, Level::High);
    assert!(latch.take(), "a genuine second edge");
}

#[test]
fn a_processor_reports_the_class_and_the_chunk_version_the_machine_file_named() {
    let Some(host) = cpus() else { return };
    let cpu = host.construct(&Props::new()).expect("a preset part");
    // Both halves of the interchange: the realizer refuses a constructor that
    // builds a device of another class, and a snapshot chunk is keyed by the
    // class name and its version. Neither may drift from `cpu::x86`'s.
    assert_eq!(cpu.class().name, "cpu.x86");
    assert_eq!(cpu.class().version, crate::cpu::x86::CLASS.version);
    assert_eq!(
        cpu.id(),
        0,
        "the first one built is the bootstrap processor"
    );
    assert!(cpu.vcpu().is_none(), "nothing observable before `bind`");
    assert!(!cpu.is_halted());
    assert!(!cpu.is_stopped());
    assert_eq!(cpu.entries(), 0);
    assert_eq!(cpu.interpreted(), 0);
}

/// A processor with no memory map behind it decides it cannot fetch in
/// hardware, which is the safe direction for that decision to fail in.
///
/// Nothing has been installed here — no `bind`, so no slots — and the
/// predicate is therefore false for every address. What that buys is that the
/// interpreter runs rather than the guest being entered at an address that
/// would come straight back out as an opaque internal error.
#[test]
fn a_processor_with_no_slots_installed_will_not_fetch_in_hardware() {
    let Some(host) = cpus() else { return };
    let cpu = host.construct(&Props::new()).expect("a preset part");
    assert!(host.plan().is_none(), "nothing is installed before `bind`");
    assert!(
        !cpu.fetch_in_hardware(),
        "with no memory slot anywhere, every fetch belongs to the interpreter"
    );
}

#[test]
fn the_properties_a_machine_file_writes_reach_the_core_underneath() {
    let Some(host) = cpus() else { return };
    let props = Props::new()
        .with("model", "80386")
        .with("engine", "interp")
        .with("iospace", "port");
    let cpu = host.construct(&props).expect("an 80386");
    assert_eq!(
        cpu.shell().config().variant,
        crate::cpu::x86::Variant::I80386
    );
    assert_eq!(cpu.shell().io_space_name(), "port");

    // And a property nothing accepts is still refused, because the reader is
    // the same one: a typo silently ignored is an afternoon lost.
    let bad = Props::new().with("engien", "kvm");
    assert!(host.construct(&bad).is_err());
}

#[test]
fn identifiers_follow_construction_order_so_the_first_processor_is_the_bootstrap_one() {
    let Some(host) = cpus() else { return };
    let a = host.construct(&Props::new()).expect("cpu0");
    let b = host.construct(&Props::new()).expect("cpu1");
    assert_eq!((a.id(), b.id()), (0, 1));
    let built = host.cpus();
    assert_eq!(built.len(), 2);
    assert_eq!(built[0].id(), 0);
    assert_eq!(built[1].id(), 1);
}

#[test]
fn the_host_table_does_not_keep_the_machines_devices_alive() {
    let Some(host) = cpus() else { return };
    {
        let _cpu = host.construct(&Props::new()).expect("cpu0");
        assert_eq!(host.cpus().len(), 1);
    }
    // The machine owns its devices; this table refers to them. A host object
    // that outlived the machine and kept its processors would keep the whole
    // board — and its vCPU file descriptors — alive with it.
    assert!(host.cpus().is_empty());
}

#[test]
fn replacing_the_class_leaves_every_other_binding_alone() {
    let Some(host) = cpus() else { return };
    let mut bindings = crate::machine::Bindings::new();
    bindings
        .bind("cpu.x86", |props| {
            Ok(Arc::new(crate::cpu::x86::X86::from_props_defaulting(
                props,
                Variant::I80486,
            )?) as Arc<dyn Instance>)
        })
        .expect("the interpreter's own binding");
    bindings
        .bind("ram", |_| unreachable!("never constructed here"))
        .expect("a second class");

    host.install(&mut bindings);
    assert_eq!(bindings.len(), 2, "one class replaced, none added or lost");
    let built =
        bindings.get("cpu.x86").expect("still bound")(&Props::new()).expect("and constructs");
    assert_eq!(built.class().name, "cpu.x86");
    // It is *this* module's device now, which is visible in the only place a
    // caller can see it without a downcast: the host table gained an entry.
    assert_eq!(host.cpus().len(), 1);
}
