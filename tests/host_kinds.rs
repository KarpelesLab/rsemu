//! Every host-object kind in the tree, and what the record/replay seal does
//! about it (`ROADMAP.md` §4.5).
//!
//! [`HostKind`] carries a role, and the role decides whether a sealed
//! host-object table demands a channel for objects filed under that kind. The
//! roles are declared one per module, by the module that owns the kind, and
//! `core::hosts` cannot see any of them — it is never feature-gated and a NIC
//! is. So the inventory lives here, where `--all-features` can see the lot.
//!
//! Two things are pinned, and only the second needs an argument.
//!
//! **What each kind is.** Marking a real input as a rendezvous would switch the
//! seal off for it silently: the board would build, the recording would be
//! short, and nothing would say so — which is the exact failure this mechanism
//! exists to make visible. A row here is cheap and a wrong role is not.
//!
//! **That the three `pad` declarations agree.** The NES, the Game Boy and the
//! Master System each declare `pad` in their own module, and a [`HostKind`]'s
//! identity is its *name alone* — a kind that compared its role too would file
//! `HostKind::new("pad")` and `HostKind::door("pad", …)` under two different
//! slots of one table and every lookup would miss. That is the right identity,
//! and the price of it is that two modules can disagree about the role of a
//! name they share. Nothing in the type system catches that, so this does.

#![cfg(feature = "std")]

use rsemu::core::hosts::HostKind;

/// Every kind this build ships, with the role its module declared.
fn inventory() -> Vec<(&'static str, HostKind)> {
    let mut kinds: Vec<(&'static str, HostKind)> = vec![("capture", HostKind::CAPTURE)];

    // Doors: host input, as `(instant, payload)`.
    kinds.push(("chardev", rsemu::host::chardev::ports::KIND));
    #[cfg(feature = "dev-nes-io")]
    kinds.push(("nes pad", rsemu::dev::nes::input::pads::KIND));
    #[cfg(feature = "dev-gb")]
    kinds.push(("gb pad", rsemu::dev::gb::joypad::pads::KIND));
    #[cfg(feature = "dev-sms")]
    kinds.push(("sms pad", rsemu::dev::sms::io::pads::KIND));
    #[cfg(feature = "dev-net")]
    kinds.push(("netdev", rsemu::dev::net::link::ports::KIND));

    // Rendezvous: two ends of one build finding each other.
    #[cfg(feature = "bus-pci")]
    kinds.push(("pci-bus", rsemu::bus::pci::buses::KIND));
    #[cfg(feature = "bus-usb")]
    kinds.push(("usb-bus", rsemu::bus::usb::buses::KIND));
    #[cfg(feature = "bus-i2c")]
    kinds.push(("i2c-bus", rsemu::bus::i2c::buses::KIND));
    #[cfg(feature = "bus-spi")]
    kinds.push(("spi-bus", rsemu::bus::spi::buses::KIND));
    #[cfg(feature = "dev-ata-disk")]
    kinds.push(("ata-bay", rsemu::dev::ata::bays::KIND));
    #[cfg(feature = "dev-sd-card")]
    kinds.push(("sd-slot", rsemu::dev::sd::slots::KIND));
    #[cfg(feature = "dev-pc-floppy")]
    kinds.push(("floppy-drive", rsemu::dev::pc::fdc::drives::KIND));
    #[cfg(feature = "dev-pc-apic")]
    kinds.push(("apic-bus", rsemu::dev::pc::apic::bus::KIND));
    #[cfg(feature = "dev-riscv")]
    kinds.push(("signal", rsemu::dev::riscv::syscon::signals::KIND));
    #[cfg(feature = "dev-riscv")]
    kinds.push(("riscv.dt", rsemu::dev::riscv::dt::KIND));

    // Pulled: host bytes the guest asks for a sector at a time.
    #[cfg(feature = "dev-medium")]
    kinds.push(("medium", rsemu::dev::medium::KIND));

    kinds
}

/// Which names are doors. Everything else this build ships is not.
const DOORS: &[&str] = &["chardev", "pad", "netdev"];

#[test]
fn every_kind_this_build_ships_declares_the_role_it_has() {
    for (what, kind) in inventory() {
        let expected = DOORS.contains(&kind.as_str());
        assert_eq!(
            kind.is_door(),
            expected,
            "`{what}` is filed under `{kind}`, which this build declares as a {}; \
             a wrong role here is a recording that is quietly missing a stream",
            if kind.is_door() { "door" } else { "rendezvous" }
        );
    }
}

#[test]
fn a_door_that_is_shipped_can_say_where_a_recorded_payload_goes() {
    // The enforcement is that a door with no sink refuses to build under a
    // recording. That is a fine diagnostic for a *new* kind and a bug report
    // for one already in the tree, so no kind here may be in that state.
    for (what, kind) in inventory() {
        if !kind.is_door() {
            continue;
        }
        assert!(
            kind.sink_factory().is_some(),
            "`{what}` is a door with no `sink()`: a board that opens one would refuse \
             to build under `--record-input`"
        );
    }
}

#[test]
fn the_three_consoles_agree_about_what_a_pad_is() {
    // A `HostKind` is its name, so these three are one key in one table. They
    // must therefore be one *role* as well, and nothing but this checks it.
    let pads: Vec<(&'static str, HostKind)> = inventory()
        .into_iter()
        .filter(|(_, kind)| kind.as_str() == "pad")
        .collect();
    let Some((first_what, first)) = pads.first() else {
        return; // a build with no console at all
    };
    for (what, kind) in &pads {
        assert_eq!(
            kind.is_door(),
            first.is_door(),
            "`{what}` and `{first_what}` both declare the `pad` kind and disagree about \
             whether it is a door; they are one key in one host-object table"
        );
        assert_eq!(*kind, *first, "and they are the same key");
    }
}
