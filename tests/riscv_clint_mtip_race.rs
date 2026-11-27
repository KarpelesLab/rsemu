//! Two harts writing one `mtimecmp` at once must leave the timer line agreeing
//! with the comparison.
//!
//! The sibling of `riscv_clint_msip_race.rs`, for the other line the CLINT
//! drives. The block changes `mtimecmp` under its lock and drives the wire
//! after releasing it (`ROADMAP.md` §4.7's re-entrancy contract), so two
//! writers — or a writer and the scheduler's `advance_to` — can drive in the
//! opposite order to the one they compared in. The line then says "pending"
//! while `mtime < mtimecmp`, or the reverse.
//!
//! *The RISC-V Instruction Set Manual*, Volume II, "Machine Timer Registers
//! (mtime and mtimecmp)": "A machine timer interrupt becomes pending whenever
//! mtime contains a value greater than or equal to mtimecmp, treating the
//! values as unsigned integers. The interrupt remains posted until mtimecmp
//! becomes greater than mtime (typically as a result of writing mtimecmp)."
//! The line *is* that comparison.
//!
//! Unlike `msip`, a wrong `mtip` does not last for ever: the next `advance_to`
//! recomputes every line. But "next" is a scheduler quantum away, and until
//! then a hart that has just pushed its deadline out takes a timer interrupt
//! it cancelled — or one that pulled its deadline in sleeps through it.
//!
//! The window is a few host instructions wide, so this is a rate and not a
//! verdict: each run makes [`ROUNDS`] attempts at the interleaving and counts
//! the rounds that ended with the two disagreeing. The assertion is that there
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

/// `mtimecmp` of hart 0.
const MTIMECMP0: u64 = 0x4000;

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

/// Wait for `ready`, spinning first and yielding after — see the `msip` test
/// for why both.
fn wait_until(ready: impl Fn() -> bool) {
    let mut spins = 0u32;
    while !ready() {
        if spins < 256 {
            spins += 1;
            std::hint::spin_loop();
        } else {
            std::thread::yield_now();
        }
    }
}

#[test]
fn the_mtip_line_agrees_with_the_comparison_after_two_harts_write_mtimecmp_at_once() {
    let clint = Arc::new(Clint::with_harts(1, 10_000_000));
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let probe = Arc::new(Probe::default());
    let wire = Wire::builder()
        .source(id)
        .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
        .build_shared();
    clint
        .connect("mtip0", WireSource::new(wire, id))
        .expect("the block drives mtip0");
    let space = AddressSpace::new("mem", 64);
    space
        .topology()
        .map(clint.region("regs").expect("the register block"), 0)
        .expect("an empty space has room");
    let space = Arc::new(space);

    let go = Arc::new(AtomicU64::new(0));
    let done = Arc::new(AtomicU64::new(0));

    let mut disagreements = 0u64;
    std::thread::scope(|s| {
        // One writer makes the timer pending (`mtime >= 0` always), the other
        // pushes the deadline out of reach.
        for value in [0u64, u64::MAX] {
            let space = Arc::clone(&space);
            let go = Arc::clone(&go);
            let done = Arc::clone(&done);
            s.spawn(move || {
                for round in 1..=ROUNDS {
                    wait_until(|| go.load(Ordering::Acquire) >= round);
                    space
                        .write(MTIMECMP0, Width::U64, value, MemAttrs::DEFAULT)
                        .expect("mtimecmp0 takes a doubleword write");
                    done.fetch_add(1, Ordering::AcqRel);
                }
            });
        }
        for round in 1..=ROUNDS {
            go.store(round, Ordering::Release);
            wait_until(|| done.load(Ordering::Acquire) >= 2 * round);
            // Both writes have returned, so both drives have been delivered:
            // whatever the line says now is what it will go on saying until
            // the scheduler next advances the block.
            let line = probe.level.load(Ordering::SeqCst) != 0;
            if line != (clint.mtime() >= clint.mtimecmp(0)) {
                disagreements += 1;
            }
        }
    });

    println!(
        "mtip race: {disagreements} of {ROUNDS} rounds left the line disagreeing with the comparison"
    );
    assert_eq!(
        disagreements, 0,
        "the mtip line parted company with mtime >= mtimecmp"
    );
}
