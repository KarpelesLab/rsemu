//! A Cortex-A9 board's CPU side through the machine layer: the MPCore private
//! region, a GICv1, an L2C-310 and an AMD-command-set boot flash.
//!
//! The unit tests beside each device drive it directly. What they cannot show
//! is that the machine file's `clock = cpuclk / 2` is the PERIPHCLK the timers
//! count, that the five windows of the private region land where DDI 0407 §1.5
//! puts them, and that a private timer's pin reaches its own processor's bank
//! of the interrupt controller. `machines/tests/a9-private.machine` is the
//! board.

#![cfg(all(
    feature = "cpu-arm-aprofile",
    feature = "dev-arm-mpcore",
    feature = "dev-arm-l2c310",
    feature = "dev-flash-cfi"
))]

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{BuildOptions, Machine, catalog};

const BOARD: &str = include_str!("../machines/tests/a9-private.machine");

const PRIV: u64 = 0xf000_0000;
const SCU: u64 = PRIV;
const GICC: u64 = PRIV + 0x100;
const TWD: u64 = PRIV + 0x600;
const GICD: u64 = PRIV + 0x1000;
const L2C: u64 = 0xf010_0000;

/// `B .` at the reset vector, so both processors idle with IRQs masked (they
/// come out of reset with CPSR.I set) while the test drives the bus.
fn parked() -> Vec<u8> {
    0xeaff_fffeu32.to_le_bytes().to_vec()
}

fn boot(tag: &str) -> Machine {
    let mut options = BuildOptions::new()
        .with_classes(catalog::classes())
        .with_bindings(catalog::bindings().expect("this build's bindings"));
    options.realize.media.insert("flash", parked());
    let registry = catalog::registry().expect("this build's registry");
    rsemu::machine::build(tag, BOARD, &registry, &options)
        .unwrap_or_else(|e| panic!("the board does not realize: {e}"))
}

fn load(m: &Machine, addr: u64, width: Width) -> u64 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, width, MemAttrs::DEFAULT)
        .unwrap_or_else(|e| panic!("{addr:#x}: {e:?}"))
}

fn store(m: &Machine, addr: u64, width: Width, value: u64) {
    m.space("mem")
        .expect("the memory space")
        .write(addr, width, value, MemAttrs::DEFAULT)
        .unwrap_or_else(|e| panic!("{addr:#x}: {e:?}"));
}

fn load32(m: &Machine, addr: u64) -> u32 {
    load(m, addr, Width::U32) as u32
}

fn store32(m: &Machine, addr: u64, value: u32) {
    store(m, addr, Width::U32, u64::from(value));
}

/// A 16-bit bus cycle at the flash's word address `word`.
fn flash_w(m: &Machine, word: u64, value: u16) {
    store(m, word * 2, Width::U16, u64::from(value));
}

fn flash_r(m: &Machine, word: u64) -> u16 {
    load(m, word * 2, Width::U16) as u16
}

#[test]
fn every_window_answers_where_the_trm_puts_it() {
    let m = boot("a9.map");
    // SCU Configuration: two processors, both SMP, 32 KiB tag RAMs.
    assert_eq!(load32(&m, SCU + 0x04), 0x0000_0531);
    // The GICv1 CPU interface's identification, at +0x1fc.
    assert_eq!(
        (load32(&m, GICC + 0xfc) >> 20),
        0x390,
        "ICCIIDR part number"
    );
    // ICDICTR: 96 ids, two CPU interfaces.
    assert_eq!(load32(&m, GICD + 0x004), 0x22);
    // The L2C-310, r3p1, 16 ways of 64 KiB.
    assert_eq!(load32(&m, L2C), 0x4100_00c6);
    assert_eq!(load32(&m, L2C + 0x104), 0x0207_0000);
    // And the reset vector, out of the flash's fast read path.
    assert_eq!(load32(&m, 0), 0xeaff_fffe);
}

#[test]
fn the_flash_speaks_the_amd_set_through_the_sixteen_bit_bus() {
    let m = boot("a9.flash");
    flash_w(&m, 0x555, 0xaa);
    flash_w(&m, 0x2aa, 0x55);
    flash_w(&m, 0x555, 0x90);
    assert_eq!(flash_r(&m, 0x00), 0x0001);
    assert_eq!(flash_r(&m, 0x01), 0x227e);
    assert_eq!(flash_r(&m, 0x0e), 0x2202);
    assert_eq!(flash_r(&m, 0x0f), 0x2201);
    flash_w(&m, 0, 0xf0);
    flash_w(&m, 0x55, 0x98);
    assert_eq!(flash_r(&m, 0x10), u16::from(b'Q'));
    assert_eq!(flash_r(&m, 0x13), 0x0002, "the AMD command set");
    flash_w(&m, 0, 0xf0);
    assert_eq!(load32(&m, 0), 0xeaff_fffe, "array mode again");
}

#[test]
fn a_private_timer_counts_periphclk_and_reaches_its_own_gic_bank() {
    let mut m = boot("a9.twd");
    // The distributor and CPU interface 0 on, ID 29 enabled in CPU 0's bank.
    store32(&m, GICD, 1);
    store32(&m, GICD + 0x100, 1 << 29);
    store32(&m, GICC, 1);
    store32(&m, GICC + 0x04, 0xf0);
    // 99 999 decrements of a 100 MHz PERIPHCLK: 999.99 us.
    store32(&m, TWD, 99_999);
    store32(&m, TWD + 0x08, 0b101); // enable, IRQ enable, prescaler 0

    // A round is atomic: a run that stops short of the expiry, which is the
    // round's natural end, executes nothing — so "not yet" is all a read can
    // say here, and it must say that.
    m.run_for(GlobalTime::from_nanos(999_000)).expect("runs");
    assert_eq!(load32(&m, TWD + 0x0c), 0, "not yet");
    assert_eq!(load32(&m, GICC + 0x18), 1023, "nothing pending");

    m.run_for(GlobalTime::from_nanos(1_000)).expect("runs");
    assert_eq!(load32(&m, TWD + 0x0c), 1, "the event flag");
    assert_eq!(load32(&m, TWD + 0x04), 0, "a one-shot stops at zero");
    assert_eq!(
        load32(&m, GICC + 0x18),
        29,
        "ID 29 is CPU 0's highest pending"
    );
    // Acknowledge at the timer and the line drops.
    store32(&m, TWD + 0x0c, 1);
    assert_eq!(load32(&m, GICC + 0x18), 1023);

    // And the rate: 2 ms of a 100 MHz PERIPHCLK is 200 000 decrements.
    store32(&m, TWD, 1_000_000);
    m.run_for(GlobalTime::from_nanos(2_000_000)).expect("runs");
    assert_eq!(
        load32(&m, TWD + 0x04),
        800_000,
        "cpuclk / 2 is what it counts"
    );
}

#[test]
fn each_processor_reaches_its_own_bank_through_the_bus() {
    // `processors = [cpu0, cpu1]` resolves to the requester ids the cores
    // stamp; an access carrying cpu1's reaches bank 1 of both the timers and
    // the GIC.
    let m = boot("a9.bank");
    let as_cpu = |path: &str| {
        MemAttrs::DEFAULT.with_requester(m.device(path).expect("a processor").requester())
    };
    let mem = m.space("mem").expect("the memory space");
    mem.write(TWD, Width::U32, 111, as_cpu("cpu0")).unwrap();
    mem.write(TWD, Width::U32, 222, as_cpu("cpu1")).unwrap();
    assert_eq!(mem.read(TWD, Width::U32, as_cpu("cpu0")).unwrap(), 111);
    assert_eq!(mem.read(TWD, Width::U32, as_cpu("cpu1")).unwrap(), 222);
    // The GIC's banked enables answer per processor at the same address.
    mem.write(GICD + 0x100, Width::U32, 1 << 29, as_cpu("cpu1"))
        .unwrap();
    let enabled = |who: &str| mem.read(GICD + 0x100, Width::U32, as_cpu(who)).unwrap() & (1 << 29);
    assert_eq!(enabled("cpu0"), 0);
    assert_ne!(enabled("cpu1"), 0);
}
