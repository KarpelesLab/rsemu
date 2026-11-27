//! Two harts writing the PLIC at once must leave each external-interrupt line
//! agreeing with the registers.
//!
//! The PLIC decides which contexts to notify under its state lock and drives
//! the lines after releasing it (`ROADMAP.md` §4.7's re-entrancy contract), so
//! two concurrent register writes can drive in the opposite order to the one
//! they changed the state in. The line is then left saying what the *earlier*
//! state wanted: low with a source pending and enabled — a notification
//! nothing will deliver until some unrelated PLIC access recomputes it, which
//! on a board whose only interrupt source is the console is the next
//! keystroke — or high with nothing to claim.
//!
//! *RISC-V Platform-Level Interrupt Controller Specification*, "Operation":
//! "The PLIC core forwards an interrupt notification to one or more targets if
//! the targets have any pending interrupts enabled, and the priority of the
//! pending interrupts exceeds a per-target threshold." And "Interrupt
//! Notifications": "The value in EIP can change as a result of changes to
//! state in the PLIC core [...]. The value in EIP is communicated to the
//! destination target as an interrupt notification." The line is a function
//! of the registers, not of who drove it last.
//!
//! The window is a few host instructions wide, so this is a rate and not a
//! verdict: each run makes [`ROUNDS`] attempts and counts the rounds that
//! ended with the line and the registers disagreeing. The assertion is that
//! there are none.

#![cfg(all(feature = "dev-riscv", feature = "std"))]

use std::sync::Arc;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};

use rsemu::core::device::Device;
use rsemu::core::space::{AddressSpace, MemAttrs};
use rsemu::core::value::Width;
use rsemu::core::wire::{Level, Wire, WireId, WireIdAllocator, WireSink, WireSource};
use rsemu::dev::riscv::Plic;

/// How many times one run tries the interleaving.
const ROUNDS: u64 = 40_000;

/// The source the test asserts and leaves asserted.
const SOURCE: u32 = 10;

/// Context 0's enable word for sources 0..=31.
const ENABLE0: u64 = 0x2000;

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
fn the_external_line_agrees_with_the_registers_after_two_harts_write_them_at_once() {
    let plic = Arc::new(Plic::build(31, 1, true));
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let probe = Arc::new(Probe::default());
    let wire = Wire::builder()
        .source(id)
        .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
        .build_shared();
    plic.connect("meip0", WireSource::new(wire, id))
        .expect("the PLIC drives meip0");
    let space = AddressSpace::new("mem", 64);
    space
        .topology()
        .map(plic.region("regs").expect("the register block"), 0)
        .expect("an empty space has room");
    let space = Arc::new(space);
    let write = |offset: u64, value: u64| {
        space
            .write(offset, Width::U32, value, MemAttrs::DEFAULT)
            .expect("a word write is legal");
    };

    // One source, priority 1, asserted and never claimed: it stays pending for
    // the whole test, so the line is exactly "is it enabled in context 0".
    write(4 * u64::from(SOURCE), 1);
    plic.set_source(SOURCE, true);
    assert!(plic.is_pending(SOURCE));

    let go = Arc::new(AtomicU64::new(0));
    let done = Arc::new(AtomicU64::new(0));

    let mut disagreements = 0u64;
    std::thread::scope(|s| {
        for value in [0u64, 1 << SOURCE] {
            let space = Arc::clone(&space);
            let go = Arc::clone(&go);
            let done = Arc::clone(&done);
            s.spawn(move || {
                for round in 1..=ROUNDS {
                    while go.load(Ordering::Acquire) < round {
                        std::hint::spin_loop();
                    }
                    space
                        .write(ENABLE0, Width::U32, value, MemAttrs::DEFAULT)
                        .expect("the enable word takes a word write");
                    done.fetch_add(1, Ordering::AcqRel);
                }
            });
        }
        for round in 1..=ROUNDS {
            go.store(round, Ordering::Release);
            while done.load(Ordering::Acquire) < 2 * round {
                std::hint::spin_loop();
            }
            // Both writes have returned, so both drives have been delivered.
            let line = probe.level.load(Ordering::SeqCst) != 0;
            let enabled = space
                .read(ENABLE0, Width::U32, MemAttrs::DEFAULT)
                .expect("the enable word reads")
                & (1 << SOURCE)
                != 0;
            if line != enabled {
                disagreements += 1;
            }
        }
    });

    println!(
        "plic race: {disagreements} of {ROUNDS} rounds left the line disagreeing with the registers"
    );
    assert_eq!(
        disagreements, 0,
        "the external-interrupt line parted company with the registers"
    );
}
