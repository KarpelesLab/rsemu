//! The snapshot upgrade table this build ships (`ROADMAP.md` §4.5).
//!
//! A class version with no migration step is decoration: bumping it orphans
//! every snapshot an earlier build wrote, cleanly and with a message naming the
//! gap, but orphaned all the same. [`core::state`](crate::core::state) has the
//! mechanism — [`Migrations`] and the `vN -> vN+1` chain — and this module is
//! where the steps are collected so that
//! [`Machine::load`](crate::machine::Machine::load) can run them without the
//! caller arranging anything. That matters because the load path with real
//! users on it is `rsemu_load` in `crate::wasm`, reached from a button in
//! `web/src/session.js`: nobody there is in a position to build a table.
//!
//! # Why a flat list rather than a chain through every `mod.rs`
//!
//! The registry and the bindings are assembled by chaining `dev::…::register`
//! calls down the module tree, and this deliberately is not. A migration table
//! is a thing to *read*: "which classes can carry an old save state forward,
//! and how far" should be answerable by opening one file, and the answer is
//! short enough to be a list for a long time yet. A chain would also put a
//! `migrations` function in every subsystem's `mod.rs` whether or not it has a
//! single step to contribute, which is a lot of empty scaffolding for a table
//! with one entry in it.
//!
//! # The rule this file exists to enforce
//!
//! **Bump a class version and register its step in the same commit.** Not the
//! next one: the encoding is fresh in the author's mind exactly once, and a
//! step written later is written from a diff. [`DeviceClass`]'s `version` field
//! says the same thing at the point where the number lives.
//!
//! # When a bump should *not* get a step
//!
//! Migrations are for snapshots that exist, and writing one costs more than it
//! looks. Of the 97 classes an all-features build registers, **eighteen ship
//! above v1 with no step behind them** — 35 single-version hops, the deepest
//! being `cpu.x86` at v8 and `cpu.i8086` at v6, then `cpu.mos6502` and
//! `nes.ppu` at v4. Writing those 35 is not the obvious good it sounds like.
//!
//! Two things have to be true before a step is worth having, and the second is
//! the binding one:
//!
//! 1. **Snapshots at the old version plausibly exist.** For roughly half of the
//!    eighteen they might: the browser build's `demo` feature set carries the
//!    NES, Game Boy, SMS, Apple 1 and PC/AT boards, and `rsemu_save` there is a
//!    button.
//! 2. **The old encoding can be reproduced faithfully enough to test the step.**
//!    §4.5 asks for a cross-version load from a *fixture*, and for a version
//!    nobody can still run a build of, the fixture would have to be synthesised
//!    from the same reading of `git log` that produced the step. The test would
//!    then confirm the author's belief about the old bytes rather than an old
//!    build's behaviour — and it is exactly the belief that is likely to be
//!    wrong.
//!
//! A wrong step is worse than no step. Today a bumped class refuses the old
//! chunk and names the gap; a step that misreads a v5 register file loads
//! happily and hands the guest a CPU whose flags are somebody's guess. So the
//! eighteen stay orphaned, deliberately, and what changes is the rule for the
//! next bump rather than the treatment of the last thirty-five.
//!
//! There is a second reason not to read too much into those numbers, and it is
//! worth knowing before anyone spends a week on retroactive steps: for a save
//! state in the wild, class versions are not the binding constraint. A snapshot
//! also carries a [`MachineShape`](crate::core::state::MachineShape), which is
//! device classes at instance paths plus region layout, and that has **no**
//! migration mechanism at all — adding a device to a board, or moving a region,
//! orphans every old save state of it however complete the version table is.
//! Boards change more often than chunk encodings do.
//!
//! That is deliberate rather than a second gap to fill, and it should not be
//! filled: [`core::state`](crate::core::state)'s "The shape has no migration,
//! and should not" makes the argument — a shape step would have to invent the
//! state of a device the guest's own drivers were never told about, and it is
//! not keyable anyway, since a snapshot header carries no board name and no
//! board revision to key it on.
//! `tests/crosshost_snapshot.rs`'s
//! `a_board_that_gained_a_device_refuses_its_older_save_states` pins the
//! refusal, and pins that it names the device.
//!
//! [`DeviceClass`]: crate::core::device::DeviceClass

use crate::core::error::Result;
use crate::core::state::Migrations;

/// Every upgrade step the classes in this build register.
///
/// [`Machine::load`](crate::machine::Machine::load) calls this, so an ordinary
/// load already migrates. A caller with steps of its own — a tool converting a
/// snapshot, a test — should start from this table and add to it rather than
/// build one from scratch, or it loses the ones shipped here.
///
/// Cheap enough to build per load: a `BTreeMap` of function pointers, on a path
/// that is about to walk every byte of guest RAM.
///
/// # Errors
///
/// [`crate::Error::State`] if two classes register the same step, which is a
/// bug in this file rather than in any snapshot.
pub fn default_migrations() -> Result<Migrations> {
    #[allow(unused_mut)]
    let mut migrations = Migrations::new();
    #[cfg(feature = "dev-flash-spinor")]
    crate::dev::flash::spinor::migrations(&mut migrations)?;
    Ok(migrations)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The classes with steps, so the tests below have something to ask about.
    ///
    /// `Migrations` deliberately does not enumerate its own classes — nothing
    /// but a test wants that — so the list is repeated here, and
    /// `the_named_classes_all_register_something` keeps it honest.
    const CLASSES: &[&str] = &[
        #[cfg(feature = "dev-flash-spinor")]
        crate::dev::flash::spinor::CLASS_NAME,
    ];

    #[test]
    fn the_table_builds() {
        // The only way this fails is a duplicate registration, and the only
        // place that can happen is the list in this file — so the test guards
        // that list, not `Migrations`.
        default_migrations().expect("no class registers a step twice");
    }

    /// Every step registered here is a single-version hop with no gap.
    ///
    /// `Migrations::upgrade` walks `from`, `from + 1`, … and fails on the first
    /// version it has no step for, so a class that registers v1 and v3 but not
    /// v2 can carry nothing at all. That is a hole a test can see and a reader
    /// cannot.
    #[test]
    fn every_class_has_an_unbroken_chain() {
        let table = default_migrations().expect("the table builds");
        for class in CLASSES {
            let steps = table.steps_for(class);
            for pair in steps.windows(2) {
                assert_eq!(
                    pair[1],
                    pair[0] + 1,
                    "`{class}` has no step from v{}",
                    pair[0] + 1
                );
            }
        }
    }

    /// Every class named above really has a step, so the list cannot rot into
    /// a set of names that no longer registers anything.
    #[test]
    fn the_named_classes_all_register_something() {
        let table = default_migrations().expect("the table builds");
        for class in CLASSES {
            assert!(
                !table.steps_for(class).is_empty(),
                "`{class}` is listed as having migrations and registers none"
            );
        }
    }

    /// A class's chain reaches the version its [`DeviceClass`] declares.
    ///
    /// The failure this catches is the one the whole file is about: somebody
    /// bumps `STATE_VERSION` again and does not add the step, so the chain
    /// stops one short and every older snapshot is orphaned again — silently,
    /// because nothing else looks at both numbers.
    ///
    /// [`DeviceClass`]: crate::core::device::DeviceClass
    #[cfg(feature = "dev-flash-spinor")]
    #[test]
    fn the_spi_flash_chain_reaches_the_shipping_version() {
        let table = default_migrations().expect("the table builds");
        let class = &crate::dev::flash::spinor::CLASS;
        let steps = table.steps_for(class.name);
        let top = steps.last().copied().expect("spinor registers steps");
        assert_eq!(
            top + 1,
            class.version,
            "`{}` ships v{} and its migration chain stops at v{}: register the \
             v{top}->v{} step in the commit that bumped it",
            class.name,
            class.version,
            top + 1,
            top + 1
        );
        assert_eq!(steps.first().copied(), Some(1), "the chain starts at v1");
    }
}
