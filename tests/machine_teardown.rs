//! Is a machine freed when it is dropped?
//!
//! Nothing in the tree asked. A machine is a graph of `Arc`s — devices,
//! regions, wires, the scheduler's slots — and a cycle among them is silent:
//! every test builds a machine, uses it and lets the process exit. The first
//! thing that built and dropped one in a loop under LeakSanitizer
//! (`fuzz/fuzz_targets/machine_mmio.rs`) found that seven of the eight boards
//! it tried were never freed, megabytes at a time. An embedder that lets a user
//! switch machines — the browser demo does — keeps every one it ever built.
//!
//! So: realize every board this build ships, keep a `Weak` to each device and
//! each address space, drop the machine, and see which of them are still
//! alive.
//!
//! # What it found, and what it cannot see
//!
//! Every lazily-advanced device and its scheduler slot owned each other, so
//! every board with a timer in it kept its devices for ever: 241 devices
//! across the catalog, with the fix in `core::sched` (`Drop for Scheduler`)
//! taken out. That is what this test pins.
//!
//! It sees a cycle that passes through the device's own `Arc` — the one the
//! machine holds — and **not** one closed entirely *beneath* it. `st.i2c` is
//! the example: its inner `Shared` owns the wire engine, which owns the slave
//! face, which owns `Shared`. The `Arc<dyn Device>` dies on time and the state
//! behind it never does. Only a leak checker sees that, and
//! `machine_mmio`'s board table lists the boards where one still does.
//!
//! # The ledger
//!
//! [`KNOWN_LEAKS`] is for staging a cycle that cannot be fixed in the commit
//! that found it. It is checked in **both** directions: a device that leaks
//! and is not listed fails the test, and so does a listed one that has
//! stopped leaking, so the fix and the deletion land together and the list
//! only ever shrinks. It is empty, which is where it belongs.

use std::sync::{Arc, Weak};

use rsemu::core::device::Device;
use rsemu::core::space::AddressSpace;
use rsemu::machine::Machine;
use rsemu::machine::catalog;

/// Device classes that survive their machine, and why.
///
/// Keyed by class rather than by board: one cycle in `st.i2c` shows up on
/// every board that carries one, and a list of boards would have to be
/// extended for each new machine file although nothing new was wrong.
///
/// A class is listed when an instance of it is still alive after the drop.
/// That is not always the class that *closes* the cycle — a device held by a
/// leaked neighbour survives with it — so the reason names the cycle, and the
/// classes under one reason come and go together.
const KNOWN_LEAKS: &[(&str, &str)] = &[];

/// Images to try in a media slot, smallest first.
///
/// A board has to realize, not run, so what a slot needs is only whatever its
/// loader checks: most accept nothing at all, a ROM socket wants its size, a
/// kernel loader wants a header it can recognise. Nothing here is a real
/// image; a board none of these satisfy is reported and skipped, and
/// [`MUST_REALIZE`] is what keeps "skipped" from quietly becoming "all".
fn candidates() -> Vec<Vec<u8>> {
    vec![
        Vec::new(),
        // A 68000 reset vector pair and `BRA .`, which is also ten bytes of
        // nothing in particular to every other loader.
        vec![0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x60, 0xfe],
        vec![0u8; 256],
        vec![0u8; 32 * 1024],
        vec![0u8; 128 * 1024],
        vec![0u8; 512 * 1024],
        ines_nrom(),
    ]
}

/// An iNES file for the simplest cartridge there is: mapper 0, one 16 KiB PRG
/// bank and one 8 KiB CHR bank, all zeroes.
///
/// The one slot the blank images cannot fill, because the NES reads its
/// cartridge's wiring out of the file's header (the 16 bytes at the front:
/// `NES<EOF>`, the PRG size in 16 KiB units, the CHR size in 8 KiB units).
/// Without it a build with only the NES in its catalog would realize nothing.
fn ines_nrom() -> Vec<u8> {
    let mut image = vec![0u8; 16 + 16 * 1024 + 8 * 1024];
    image[..6].copy_from_slice(&[b'N', b'E', b'S', 0x1a, 1, 1]);
    image
}

/// Boards that the candidates above are known to realize, where this build
/// has them. If one of these is in the catalog and does not come up, the test
/// has stopped testing and says so rather than passing on what is left.
const MUST_REALIZE: &[&str] = &[
    "nes-ntsc",
    "pc-at",
    "arm926",
    "stm32f407",
    "riscv-virt",
    "mac-plus",
    "amiga-a500",
];

/// Realize `name` with the first candidate image every slot accepts.
fn realize(name: &str, slots: &[&str]) -> Option<Machine> {
    for image in candidates() {
        let media: Vec<(&str, &[u8])> = slots.iter().map(|s| (*s, image.as_slice())).collect();
        if let Ok(machine) = catalog::build_catalog(name, &media) {
            return Some(machine);
        }
    }
    None
}

/// What outlived one machine.
struct Survivors {
    /// `(instance path, class name)` of every device still alive.
    devices: Vec<(String, &'static str)>,
    /// Names of the address spaces still alive.
    spaces: Vec<String>,
}

fn drop_and_look(machine: Machine) -> Survivors {
    let devices: Vec<(String, &'static str, Weak<dyn Device>)> = machine
        .devices()
        .iter()
        .map(|d| {
            (
                d.path().to_string(),
                d.class().name,
                Arc::downgrade(d.device()),
            )
        })
        .collect();
    let spaces: Vec<(String, Weak<AddressSpace>)> = machine
        .spaces()
        .iter()
        .map(|s| (s.name().to_string(), Arc::downgrade(s.space())))
        .collect();
    drop(machine);
    Survivors {
        devices: devices
            .into_iter()
            .filter(|(_, _, weak)| weak.strong_count() != 0)
            .map(|(path, class, _)| (path, class))
            .collect(),
        spaces: spaces
            .into_iter()
            .filter(|(_, weak)| weak.strong_count() != 0)
            .map(|(name, _)| name)
            .collect(),
    }
}

#[test]
fn every_shipped_machine_is_freed_when_it_is_dropped() {
    let mut realized = 0usize;
    let mut skipped = Vec::new();
    let mut unexpected = Vec::new();
    // Classes seen alive after a drop, for the other direction of the ledger.
    let mut leaked_classes = std::collections::BTreeSet::new();
    // Classes that were instantiated at all: a listed class this build never
    // built cannot be shown to have stopped leaking.
    let mut built_classes = std::collections::BTreeSet::new();

    for entry in catalog::machines() {
        let Some(machine) = realize(entry.name, entry.media) else {
            assert!(
                !MUST_REALIZE.contains(&entry.name),
                "`{}` no longer realizes from this test's stand-in images, so nothing \
                 about it was checked",
                entry.name
            );
            skipped.push(entry.name);
            continue;
        };
        realized += 1;
        for d in machine.devices() {
            built_classes.insert(d.class().name);
        }
        let survivors = drop_and_look(machine);
        for (path, class) in &survivors.devices {
            leaked_classes.insert(*class);
            if !KNOWN_LEAKS.iter().any(|(known, _)| known == class) {
                unexpected.push(format!("{}: `{path}` ({class})", entry.name));
            }
        }
        // A space is only ever kept alive by a device that is, so it is
        // reported with them rather than tracked in the ledger.
        if !survivors.spaces.is_empty() && survivors.devices.is_empty() {
            unexpected.push(format!(
                "{}: address space(s) {:?} outlived every device",
                entry.name, survivors.spaces
            ));
        }
    }

    println!(
        "machine teardown: {realized} board(s) realized and dropped; not realizable from \
         stand-in images: {skipped:?}"
    );
    assert!(
        unexpected.is_empty(),
        "these outlived the machine they belonged to — something they own, or that owns \
         them, is in a reference cycle:\n  {}",
        unexpected.join("\n  ")
    );

    let stale: Vec<&str> = KNOWN_LEAKS
        .iter()
        .map(|(class, _)| *class)
        .filter(|class| built_classes.contains(class) && !leaked_classes.contains(class))
        .collect();
    assert!(
        stale.is_empty(),
        "these classes are in KNOWN_LEAKS and are freed now — delete their entries: {stale:?}"
    );

    // What keeps this from passing on nothing is `MUST_REALIZE`, checked in
    // the loop: seven boards from seven families, each of which has to come
    // up wherever the catalog has it. There is deliberately no "at least one
    // board realized" here. This file is compiled under every feature on its
    // own by the sweep, and a build whose only board wants an image none of
    // the stand-ins is — which is what `machine-nes` was before the iNES one
    // — has nothing to ask and should say so, not fail.
    if realized == 0 && !catalog::machines().is_empty() {
        println!("machine teardown: nothing realized, so nothing was checked in this build");
    }
}
