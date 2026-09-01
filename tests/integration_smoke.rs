//! Do the independently-built core modules actually compose?
//!
//! Every module in `core` was written in isolation. Their unit tests prove each
//! works alone; this proves they fit together, which is the thing isolated tests
//! structurally cannot show.

use rsemu::core::clock::{ClockForest, Rational};
use rsemu::core::space::{AddressSpace, MemAttrs, RamStore, Region};
use rsemu::core::value::Width;
use std::sync::Arc;

/// The NES shape in miniature: one crystal, two domains at a fixed integer
/// ratio, and RAM mirrored into a 16-bit space — the three mechanisms phase 3
/// leans on, exercised together rather than one at a time.
#[test]
fn a_clock_forest_and_an_address_space_coexist() {
    // Time. The NES master is 236250000/11 Hz, deliberately not an integer.
    let mut forest = ClockForest::new();
    let master = forest
        .add_oscillator("master", Rational::new(236_250_000, 11).unwrap())
        .unwrap();
    let cpu = forest.add_domain("cpu", master, 1, 12).unwrap();
    let ppu = forest.add_domain("ppu", master, 1, 4).unwrap();

    // Memory. 2 KiB of RAM mirrored across $0000-$1FFF.
    let space = AddressSpace::new("cpubus", 16);
    let ram = Region::ram("wram", Arc::new(RamStore::new(0x800)));
    let mirror = Region::mirror("wram-mirror", ram, 0x2000).unwrap();
    {
        let mut topo = space.topology();
        topo.map(mirror, 0x0000).unwrap();
        topo.rebuild();
    }

    let attrs = MemAttrs::default();
    space.write(0x0003, Width::U8, 0xa5, attrs).unwrap();

    // The mirror is real: one write appears in all four windows.
    for base in [0x0000u64, 0x0800, 0x1000, 0x1800] {
        let v = space.read(base + 3, Width::U8, attrs).unwrap();
        assert_eq!(v, 0xa5, "mirror at {base:#06x}");
    }

    // Time advances independently of memory, and the ratio is exact.
    forest.advance_domain(cpu, 1_000).unwrap();
    assert_eq!(forest.ticks(cpu).unwrap(), 1_000);
    assert_eq!(forest.ticks(ppu).unwrap(), 3_000);
}
