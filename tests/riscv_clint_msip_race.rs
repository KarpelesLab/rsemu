//! Two harts writing one `msip` register at once must leave the line agreeing
//! with the register.
//!
//! This is the interleaving an SMP guest produces on every inter-processor
//! interrupt under `threading parallel`: the sender stores 1 to the target's
//! `msip` while the target, finishing the previous one, stores 0. The CLINT
//! changes the bit under its lock and drives the wire after releasing it
//! (`ROADMAP.md` §4.7's re-entrancy contract), so the two drives can land in
//! the opposite order to the two changes. The register then says 1 and the
//! line says 0, and because a store of the value `msip` already holds drives
//! nothing, no later write repairs it: the interrupt is lost for good and the
//! sender waits on an acknowledgement that never comes.
//!
//! *The RISC-V Instruction Set Manual*, Volume II, "Machine Interrupt
//! Registers": "MSIP is read-only in mip, and is written by accesses to
//! memory-mapped control registers, which are used to provide machine-level
//! interprocessor interrupts." The line *is* the register bit; they may not
//! part company.
//!
//! The window is a few host instructions wide, so this is a rate and not a
//! verdict: each run makes [`ROUNDS`] attempts at the interleaving and counts
//! the rounds that ended with the two disagreeing. Before the fix, 29 of 30
//! runs on a 64-core host counted at least one; the assertion is that there
//! are none.

#![cfg(all(feature = "dev-riscv", feature = "std"))]

use std::sync::Arc;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};

use rsemu::core::device::Device;
use rsemu::core::space::{AddressSpace, MemAttrs};
use rsemu::core::value::Width;
use rsemu::core::wire::{Level, Wire, WireId, WireIdAllocator, WireSink, WireSource};
use rsemu::dev::riscv::Clint;

/// How many times one run tries the interleaving.
const ROUNDS: u64 = 40_000;

/// A sink that records the last level it was told.
#[derive(Debug, Default)]
struct Probe {
    level: AtomicU32,
}

impl WireSink for Probe {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        self.level
            .store(u32::from(level.is_high()), Ordering::SeqCst);
    }
}

#[test]
fn the_msip_line_agrees_with_the_register_after_two_harts_write_it_at_once() {
    let clint = Arc::new(Clint::with_harts(1, 10_000_000));
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let probe = Arc::new(Probe::default());
    let wire = Wire::builder()
        .source(id)
        .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
        .build_shared();
    clint
        .connect("msip0", WireSource::new(wire, id))
        .expect("the block drives msip0");
    let space = AddressSpace::new("mem", 64);
    space
        .topology()
        .map(clint.region("regs").expect("the register block"), 0)
        .expect("an empty space has room");
    let space = Arc::new(space);

    // The round the two writers may start, and how many writes have finished.
    // Spun on rather than parked on: the window is a handful of instructions,
    // and a wake-up from a barrier is far wider than that.
    let go = Arc::new(AtomicU64::new(0));
    let done = Arc::new(AtomicU64::new(0));

    let mut disagreements = 0u64;
    std::thread::scope(|s| {
        for value in [0u64, 1] {
            let space = Arc::clone(&space);
            let go = Arc::clone(&go);
            let done = Arc::clone(&done);
            s.spawn(move || {
                for round in 1..=ROUNDS {
                    while go.load(Ordering::Acquire) < round {
                        std::hint::spin_loop();
                    }
                    space
                        .write(0, Width::U32, value, MemAttrs::DEFAULT)
                        .expect("msip0 takes a word write");
                    done.fetch_add(1, Ordering::AcqRel);
                }
            });
        }
        for round in 1..=ROUNDS {
            go.store(round, Ordering::Release);
            while done.load(Ordering::Acquire) < 2 * round {
                std::hint::spin_loop();
            }
            // Both writes have returned, so both drives have been delivered:
            // whatever the line says now is what it will go on saying.
            let line = probe.level.load(Ordering::SeqCst) != 0;
            if line != clint.msip(0) {
                disagreements += 1;
            }
        }
    });

    println!(
        "msip race: {disagreements} of {ROUNDS} rounds left the line disagreeing with the register"
    );
    assert_eq!(
        disagreements, 0,
        "the msip line parted company with the msip register"
    );
}
