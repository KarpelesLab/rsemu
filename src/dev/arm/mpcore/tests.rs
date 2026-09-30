use super::*;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::wire::{Wire, WireId, WireIdAllocator, WireSink};

/// A sink that records the last level it was told and counts rising edges.
#[derive(Debug, Default)]
struct Probe {
    level: AtomicU32,
    edges: AtomicU32,
}

impl WireSink for Probe {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        let was = self
            .level
            .swap(u32::from(level.is_high()), Ordering::Relaxed);
        if was == 0 && level.is_high() {
            self.edges.fetch_add(1, Ordering::Relaxed);
        }
    }
}

impl Probe {
    fn high(&self) -> bool {
        self.level.load(Ordering::Relaxed) != 0
    }

    fn edges(&self) -> u32 {
        self.edges.load(Ordering::Relaxed)
    }
}

/// The requester ids two processors would be allocated — arbitrary, and not
/// processor numbers.
const CPU0: RequesterId = RequesterId(5);
const CPU1: RequesterId = RequesterId(9);

struct Bench {
    dev: A9MpCore,
    /// `probes[cpu][pin]`, in `PIN_NAMES` order.
    probes: Vec<[Arc<Probe>; 4]>,
}

fn bench(cpus: usize) -> Bench {
    let dev = A9MpCore::build(cpus, 32 * 1024, 0, 0);
    if cpus > 1 {
        assert!(dev.attach_processor(0, CPU0));
        assert!(dev.attach_processor(1, CPU1));
    }
    let ids = WireIdAllocator::new();
    let mut probes = Vec::new();
    for cpu in 0..cpus {
        let set: [Arc<Probe>; 4] = core::array::from_fn(|_| Arc::new(Probe::default()));
        for (pin, probe) in set.iter().enumerate() {
            let src = ids.alloc();
            let wire = Wire::builder()
                .source(src)
                .sink(Arc::clone(probe) as Arc<dyn WireSink>, 0)
                .build_shared();
            dev.connect(
                &format!("{}{cpu}", PIN_NAMES[pin]),
                WireSource::new(wire, src),
            )
            .expect("every processor has four pins");
        }
        probes.push(set);
    }
    Bench { dev, probes }
}

impl Bench {
    fn region(&self, name: &str) -> Arc<dyn MemOps> {
        ops(&self.dev.region(name).expect("a window"))
    }

    fn read_as(&self, window: &str, offset: u64, attrs: MemAttrs) -> u32 {
        let mut bytes = [0u8; 4];
        self.region(window)
            .read(offset, &mut bytes, attrs)
            .expect("a word read");
        u32::from_le_bytes(bytes)
    }

    fn write_as(&self, window: &str, offset: u64, value: u32, attrs: MemAttrs) {
        self.region(window)
            .write(offset, &value.to_le_bytes(), attrs)
            .expect("a word write");
    }

    fn read(&self, window: &str, offset: u64) -> u32 {
        self.read_as(window, offset, MemAttrs::DEFAULT)
    }

    fn write(&self, window: &str, offset: u64, value: u32) {
        self.write_as(window, offset, value, MemAttrs::DEFAULT);
    }

    fn on(&self, cpu: usize, pin: usize) -> bool {
        self.probes[cpu][pin].high()
    }
}

/// The register file behind a window.
fn ops(region: &RegionRef) -> Arc<dyn MemOps> {
    match region.kind() {
        crate::core::space::RegionKind::Io(ops) => Arc::clone(ops),
        _ => panic!("every window here is an I/O region"),
    }
}

fn as_cpu(requester: RequesterId) -> MemAttrs {
    MemAttrs::DEFAULT.with_requester(requester)
}

// -- the SCU ----------------------------------------------------------------

#[test]
fn the_scu_configuration_register_reflects_the_processor_count() {
    // Four processors, 32 KiB data caches: CPU number 3, all four SMP bits,
    // and tag RAM size 0b01 for each (DDI 0407 §2.2.2).
    let quad = A9MpCore::build(4, 32 * 1024, 0, 0);
    let config = {
        let mut b = [0u8; 4];
        ops(&quad.region("scu").unwrap())
            .read(0x04, &mut b, MemAttrs::DEFAULT)
            .unwrap();
        u32::from_le_bytes(b)
    };
    assert_eq!(config & 3, 3, "CPU number is processors - 1");
    assert_eq!((config >> 4) & 0xf, 0xf, "every processor in SMP");
    assert_eq!((config >> 8) & 0xff, 0b01_01_01_01, "32 KiB tag RAMs");

    let one = bench(1);
    assert_eq!(one.read("scu", 0x04), 0x0000_0110, "one processor");
    let small = A9MpCore::build(2, 16 * 1024, 0, 0);
    let mut b = [0u8; 4];
    ops(&small.region("scu").unwrap())
        .read(0x04, &mut b, MemAttrs::DEFAULT)
        .unwrap();
    assert_eq!(u32::from_le_bytes(b), 0x0000_0031, "16 KiB is 0b00");
}

#[test]
fn the_scu_stores_what_is_writable_and_nothing_else() {
    let b = bench(2);
    b.write("scu", 0x00, 0xffff_ffff);
    assert_eq!(b.read("scu", 0x00), SCU_CTRL_MASK);
    // Two power-mode bits per present processor, one byte each.
    b.write("scu", 0x08, 0xffff_ffff);
    assert_eq!(b.read("scu", 0x08), 0x0000_0303);
    // A byte write reaches one processor's byte and leaves the other alone.
    b.region("scu")
        .write(0x09, &[0x00], MemAttrs::DEFAULT)
        .unwrap();
    assert_eq!(b.read("scu", 0x08), 0x0000_0003);
    // A narrow write anywhere else is refused.
    assert!(
        b.region("scu")
            .write(0x00, &[1], MemAttrs::DEFAULT)
            .is_err()
    );
    // Invalidate All is write-only and complete at once.
    b.write("scu", 0x0c, 0xffff);
    assert_eq!(b.read("scu", 0x0c), 0);
    b.write("scu", 0x40, 0xe012_3456);
    assert_eq!(b.read("scu", 0x40), 0xe010_0000, "1 MiB granules");
    assert_eq!(b.read("scu", 0x50), 0b11, "every processor out of reset");
    assert_eq!(b.read("scu", 0x54), 0);
    b.write("scu", 0x54, 0xffff_ffff);
    assert_eq!(b.read("scu", 0x54), 0x333);
}

// -- the global timer ---------------------------------------------------------

#[test]
fn the_global_timer_counts_through_its_prescaler() {
    let b = bench(1);
    b.write("global", 0x00, 5);
    // Prescaler 3: one increment every four PERIPHCLK ticks.
    b.write("global", 0x08, GT_ENABLE | (3 << 8));
    b.dev.advance_to(40);
    assert_eq!(b.read("global", 0x00), 15);
    assert_eq!(b.read("global", 0x04), 0);
    b.dev.advance_to(43);
    assert_eq!(
        b.read("global", 0x00),
        15,
        "three ticks is not an increment"
    );
    b.dev.advance_to(44);
    assert_eq!(b.read("global", 0x00), 16);
    // A running counter ignores writes (DDI 0407 §4.4.1)...
    b.write("global", 0x00, 0);
    assert_eq!(b.read("global", 0x00), 16);
    // ...and a stopped one holds still and takes them.
    b.write("global", 0x08, 0);
    b.dev.advance_to(1_000);
    assert_eq!(b.read("global", 0x00), 16);
    b.write("global", 0x04, 1);
    assert_eq!(b.read("global", 0x04), 1);
}

#[test]
fn the_global_comparator_fires_and_auto_increments() {
    let b = bench(1);
    b.write("global", 0x10, 20);
    b.write("global", 0x14, 0);
    b.write("global", 0x18, 10);
    b.write(
        "global",
        0x08,
        GT_ENABLE | GT_COMP_ENABLE | GT_IRQ_ENABLE | GT_AUTO_INC | (3 << 8),
    );
    // Counter 20 at tick 80.
    assert_eq!(b.dev.next_event_tick(), Some(80));
    b.dev.advance_to(79);
    assert!(!b.on(0, PIN_GT));
    b.dev.advance_to(80);
    assert!(b.on(0, PIN_GT), "ID 27 raised");
    assert_eq!(b.read("global", 0x0c), 1, "event flag");
    assert_eq!(b.read("global", 0x10), 30, "comparator stepped by 10");
    assert_eq!(b.dev.next_event_tick(), Some(120));
    b.write("global", 0x0c, 1);
    assert!(!b.on(0, PIN_GT), "write-one-to-clear drops the line");
    b.dev.advance_to(120);
    assert!(b.on(0, PIN_GT));
    assert_eq!(b.probes[0][PIN_GT].edges(), 2);
    assert_eq!(b.read("global", 0x10), 40);
}

#[test]
fn a_comparator_behind_a_running_counter_fires_at_once() {
    // The r2p0-and-later rule: greater than or equal (DDI 0407 §4.4.1).
    let b = bench(1);
    b.write("global", 0x08, GT_ENABLE | GT_IRQ_ENABLE);
    b.dev.advance_to(100);
    b.write("global", 0x10, 50);
    assert!(!b.on(0, PIN_GT), "not while the comparator is disabled");
    b.write("global", 0x08, GT_ENABLE | GT_IRQ_ENABLE | GT_COMP_ENABLE);
    assert!(b.on(0, PIN_GT));
    // Without auto-increment it fires once, not again every tick.
    b.write("global", 0x0c, 1);
    b.dev.advance_to(200);
    assert!(!b.on(0, PIN_GT));
    assert_eq!(b.dev.next_event_tick(), None);
}

// -- the private timer --------------------------------------------------------

#[test]
fn a_one_shot_private_timer_expires_after_load_decrements_and_stays_at_zero() {
    let b = bench(1);
    b.write("private", 0x00, 99);
    assert_eq!(b.read("private", 0x04), 99, "a load write sets the counter");
    // Prescaler 1: two PERIPHCLK ticks per decrement, so 99 decrements are
    // 198 ticks.
    b.write("private", 0x08, CTRL_ENABLE | CTRL_IRQ_ENABLE | (1 << 8));
    assert_eq!(b.dev.next_event_tick(), Some(198));
    b.dev.advance_to(100);
    assert_eq!(b.read("private", 0x04), 49);
    b.dev.advance_to(197);
    assert!(!b.on(0, PIN_TWD));
    b.dev.advance_to(198);
    assert!(b.on(0, PIN_TWD), "ID 29 raised");
    assert_eq!(b.read("private", 0x0c), 1);
    assert_eq!(b.read("private", 0x04), 0);
    assert_eq!(b.dev.next_event_tick(), None, "one-shot");
    b.dev.advance_to(10_000);
    assert_eq!(b.read("private", 0x04), 0);
    b.write("private", 0x0c, 1);
    assert!(!b.on(0, PIN_TWD));
}

#[test]
fn an_auto_reload_private_timer_has_the_trm_period_exactly() {
    // ((PRESCALER + 1) × (Load + 1)) / PERIPHCLK: with prescaler 4 and load
    // 999, a period is 5 000 ticks (DDI 0407 §4.1.1).
    let b = bench(1);
    b.write("private", 0x00, 999);
    b.write(
        "private",
        0x08,
        CTRL_ENABLE | CTRL_AUTO_RELOAD | CTRL_IRQ_ENABLE | (4 << 8),
    );
    // The first expiry counts down from the loaded value.
    assert_eq!(b.dev.next_event_tick(), Some(999 * 5));
    let mut at = 999 * 5;
    for n in 1..=5u32 {
        b.dev.advance_to(at - 1);
        assert_eq!(b.probes[0][PIN_TWD].edges(), n - 1);
        b.dev.advance_to(at);
        assert_eq!(b.probes[0][PIN_TWD].edges(), n, "expiry {n} at {at}");
        b.write("private", 0x0c, 1);
        assert_eq!(b.dev.next_event_tick(), Some(at + 5_000));
        at += 5_000;
    }
    // Between expiries the counter reloads from Load after zero.
    b.dev.advance_to(at - 5_000 + 5);
    assert_eq!(b.read("private", 0x04), 999);
    // Many periods in one step collapse into one flag, and the phase is kept.
    b.dev.advance_to(at + 50_000);
    assert_eq!(b.read("private", 0x0c), 1);
    assert_eq!(b.dev.next_event_tick(), Some(at + 55_000));
}

// -- the watchdog -------------------------------------------------------------

#[test]
fn the_watchdog_disable_sequence_is_two_consecutive_words() {
    let b = bench(1);
    b.write("private", 0x28, CTRL_WD_MODE);
    assert_eq!(b.read("private", 0x28) & CTRL_WD_MODE, CTRL_WD_MODE);
    // Writing zero to the mode bit does nothing.
    b.write("private", 0x28, 0);
    assert_eq!(b.read("private", 0x28) & CTRL_WD_MODE, CTRL_WD_MODE);
    // A sequence broken by another write does nothing.
    b.write("private", 0x34, WD_DISABLE_1);
    b.write("private", 0x20, 5);
    b.write("private", 0x34, WD_DISABLE_2);
    assert_eq!(b.read("private", 0x28) & CTRL_WD_MODE, CTRL_WD_MODE);
    // So does one in the wrong order.
    b.write("private", 0x34, WD_DISABLE_2);
    b.write("private", 0x34, WD_DISABLE_1);
    assert_eq!(b.read("private", 0x28) & CTRL_WD_MODE, CTRL_WD_MODE);
    // The real thing, which the second word of the last attempt began.
    b.write("private", 0x34, WD_DISABLE_2);
    assert_eq!(
        b.read("private", 0x28) & CTRL_WD_MODE,
        0,
        "timer mode again"
    );
}

#[test]
fn a_watchdog_in_watchdog_mode_requests_a_reset_and_a_kick_postpones_it() {
    let b = bench(1);
    b.write("private", 0x20, 100);
    b.write(
        "private",
        0x28,
        CTRL_WD_MODE | CTRL_ENABLE | CTRL_IRQ_ENABLE,
    );
    b.dev.advance_to(60);
    // Kick: a load write reloads the counter.
    b.write("private", 0x20, 100);
    b.dev.advance_to(159);
    assert_eq!(b.probes[0][PIN_WDRESET].edges(), 0);
    b.dev.advance_to(160);
    assert_eq!(b.probes[0][PIN_WDRESET].edges(), 1, "WDRESETREQ pulsed");
    assert!(!b.on(0, PIN_WDRESET), "a pulse, not a level");
    assert!(!b.on(0, PIN_WDT), "no interrupt in watchdog mode");
    assert_eq!(b.read("private", 0x30), 1, "reset status");
    // The status survives a warm reset and not a cold one.
    b.dev.reset(ResetKind::Warm);
    assert_eq!(b.read("private", 0x30), 1);
    assert_eq!(b.read("private", 0x28), 0, "but the mode does not");
    b.dev.reset(ResetKind::Cold);
    assert_eq!(b.read("private", 0x30), 0);
}

#[test]
fn a_watchdog_in_timer_mode_is_a_second_timer_on_id_30() {
    let b = bench(1);
    b.write("private", 0x20, 9);
    b.write(
        "private",
        0x28,
        CTRL_ENABLE | CTRL_AUTO_RELOAD | CTRL_IRQ_ENABLE,
    );
    b.dev.advance_to(9);
    assert!(b.on(0, PIN_WDT));
    assert!(!b.on(0, PIN_TWD));
    b.write("private", 0x2c, 1);
    assert!(!b.on(0, PIN_WDT));
    b.dev.advance_to(19);
    assert!(b.on(0, PIN_WDT), "period of Load + 1");
}

// -- banking --------------------------------------------------------------------

#[test]
fn each_processor_sees_its_own_private_timer() {
    let b = bench(2);
    b.write_as("private", 0x00, 100, as_cpu(CPU0));
    b.write_as("private", 0x00, 50, as_cpu(CPU1));
    assert_eq!(b.read_as("private", 0x00, as_cpu(CPU0)), 100);
    assert_eq!(b.read_as("private", 0x00, as_cpu(CPU1)), 50);
    for who in [CPU0, CPU1] {
        b.write_as("private", 0x08, CTRL_ENABLE | CTRL_IRQ_ENABLE, as_cpu(who));
    }
    b.dev.advance_to(50);
    assert!(b.on(1, PIN_TWD), "processor 1's expired");
    assert!(!b.on(0, PIN_TWD), "processor 0's did not");
    assert_eq!(b.read_as("private", 0x04, as_cpu(CPU0)), 50);
    assert_eq!(b.read_as("private", 0x0c, as_cpu(CPU0)), 0);
    assert_eq!(b.read_as("private", 0x0c, as_cpu(CPU1)), 1);
    // Something that is not a processor sees processor 0's bank.
    assert_eq!(b.read("private", 0x00), 100);
    b.dev.advance_to(100);
    assert!(b.on(0, PIN_TWD));
}

#[test]
fn the_global_counter_is_shared_and_its_comparators_are_not() {
    let b = bench(2);
    b.write_as("global", 0x10, 10, as_cpu(CPU1));
    b.write_as(
        "global",
        0x08,
        GT_ENABLE | GT_COMP_ENABLE | GT_IRQ_ENABLE,
        as_cpu(CPU1),
    );
    assert_eq!(b.read_as("global", 0x08, as_cpu(CPU0)), GT_ENABLE);
    b.dev.advance_to(10);
    assert_eq!(b.read_as("global", 0x00, as_cpu(CPU0)), 10);
    assert!(b.on(1, PIN_GT));
    assert!(!b.on(0, PIN_GT));
}

#[test]
fn a_multiprocessor_block_that_names_no_processors_is_refused() {
    let e = A9MpCore::new(&Props::new().with("cpus", 2u64))
        .expect_err("no map")
        .to_string();
    assert!(e.contains("processors"), "{e}");
    assert!(A9MpCore::new(&Props::new().with("dcache", 24u64 * 1024)).is_err());
    assert!(A9MpCore::new(&Props::new().with("cpus", 5u64)).is_err());
    assert!(A9MpCore::new(&Props::new()).is_ok());
    assert!(!bench(2).dev.attach_processor(0, CPU1), "one bank per core");
}

// -- debug, snapshots ---------------------------------------------------------

#[test]
fn a_debug_read_moves_nothing_and_a_debug_write_is_refused() {
    let b = bench(1);
    b.write("private", 0x00, 10);
    b.write("private", 0x08, CTRL_ENABLE | CTRL_IRQ_ENABLE);
    b.dev.advance_to(10);
    let debug = MemAttrs {
        debug: true,
        ..MemAttrs::DEFAULT
    };
    assert_eq!(b.read_as("private", 0x0c, debug), 1);
    assert_eq!(b.read_as("private", 0x0c, debug), 1, "still set");
    assert!(b.on(0, PIN_TWD));
    assert!(
        b.region("private")
            .write(0x0c, &1u32.to_le_bytes(), debug)
            .is_err()
    );
    assert!(b.on(0, PIN_TWD), "a debugger cannot acknowledge");
}

fn snapshot(dev: &A9MpCore) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("mpcore", CLASS.name).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("mpcore", CLASS.name, CLASS.version).unwrap();
        dev.save(&mut chunk).unwrap();
    }
    w.to_vec().unwrap()
}

#[test]
fn state_round_trips_to_an_identical_hash() {
    let b = bench(2);
    b.write("scu", 0x00, 1);
    b.write_as("private", 0x00, 77, as_cpu(CPU1));
    b.write_as(
        "private",
        0x08,
        CTRL_ENABLE | CTRL_AUTO_RELOAD | CTRL_IRQ_ENABLE | (2 << 8),
        as_cpu(CPU1),
    );
    b.write_as("private", 0x28, CTRL_WD_MODE, as_cpu(CPU0));
    b.write_as("private", 0x34, WD_DISABLE_1, as_cpu(CPU0));
    b.write("global", 0x10, 1_000);
    b.write("global", 0x18, 7);
    b.write(
        "global",
        0x08,
        GT_ENABLE | GT_COMP_ENABLE | GT_AUTO_INC | (1 << 8),
    );
    // Mid-prescale, so the phase has to travel too.
    b.dev.advance_to(1_001);
    let bytes = snapshot(&b.dev);

    let restored = bench(2);
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("mpcore", CLASS.name, CLASS.version, &Migrations::new())
        .unwrap();
    restored.dev.load(&mut chunk.reader()).unwrap();
    assert_eq!(snapshot(&b.dev), snapshot(&restored.dev), "the state hash");
    assert_eq!(restored.dev.tick(), 1_001);
    assert_eq!(restored.dev.next_event_tick(), b.dev.next_event_tick());

    // And the two go on to do exactly the same thing — including finishing a
    // disable sequence that was half-written when the snapshot was taken.
    for dev in [&b, &restored] {
        dev.write_as("private", 0x34, WD_DISABLE_2, as_cpu(CPU0));
        dev.dev.advance_to(5_000);
    }
    assert_eq!(snapshot(&b.dev), snapshot(&restored.dev));
    assert_eq!(
        restored.read_as("private", 0x28, as_cpu(CPU0)) & CTRL_WD_MODE,
        0
    );
    assert_eq!(
        restored.read("global", 0x00),
        b.read("global", 0x00),
        "the same count"
    );
}

#[test]
fn the_count_down_arithmetic_matches_counting_one_at_a_time() {
    // The closed form against the definition, over every small case.
    for load in 0..6u32 {
        for count in 0..6u32 {
            for reload in [false, true] {
                let mut value = count;
                let mut fired = false;
                for n in 1..40u64 {
                    // One decrement by definition.
                    if value > 0 {
                        value -= 1;
                        if value == 0 {
                            fired = true;
                        }
                    } else if reload {
                        value = load;
                        if value == 0 {
                            fired = true;
                        }
                    }
                    assert_eq!(
                        count_down(count, n, load, reload),
                        (value, fired),
                        "load {load} count {count} reload {reload} after {n}"
                    );
                }
            }
        }
    }
}
