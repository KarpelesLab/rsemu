//! Tests for [`super`], the STM32 TIM.
//!
//! Every expectation here comes from ST RM0090 §17–§19, cited where it is not
//! obvious. No emulator source of any licence was consulted.

use super::*;

use alloc::vec::Vec;

use crate::core::props::Value;
use crate::core::registry::Registry;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::wire::{Wire, WireId};

/// A 16-bit general-purpose timer with four channels — a `TIM3`.
fn general() -> Tim {
    Tim::with_config(Config {
        variant: Variant::General,
        mask: 0xffff,
        channels: 4,
        arr_reset: 0,
    })
}

/// A 32-bit general-purpose timer — a `TIM2` or `TIM5`.
fn general32() -> Tim {
    Tim::with_config(Config {
        variant: Variant::General,
        mask: 0xffff_ffff,
        channels: 4,
        arr_reset: 0,
    })
}

/// An advanced timer — a `TIM1`.
fn advanced() -> Tim {
    Tim::with_config(Config {
        variant: Variant::Advanced,
        mask: 0xffff,
        channels: 4,
        arr_reset: 0,
    })
}

/// A basic timer — a `TIM6`.
fn basic() -> Tim {
    Tim::with_config(Config {
        variant: Variant::Basic,
        mask: 0xffff,
        channels: 0,
        arr_reset: 0,
    })
}

fn peek(tim: &Tim, offset: u64) -> u32 {
    let mut word = [0u8; 4];
    tim.shared
        .read(offset, &mut word, MemAttrs::DEFAULT)
        .expect("a word read is legal");
    u32::from_le_bytes(word)
}

fn peek_debug(tim: &Tim, offset: u64) -> u32 {
    let mut word = [0u8; 4];
    tim.shared
        .read(offset, &mut word, MemAttrs::DEBUG)
        .expect("a word read is legal");
    u32::from_le_bytes(word)
}

fn poke(tim: &Tim, offset: u64, value: u32) {
    tim.shared
        .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
        .expect("a word write is legal");
}

/// A wire with one source, so a pin has something to drive.
fn dummy_source(id: u64) -> WireSource {
    let wire_id = WireId::new(id);
    WireSource::new(Wire::builder().source(wire_id).build_shared(), wire_id)
}

/// Connect a pin and hand back the source, whose level a test can read.
fn wire(tim: &Tim, pin: &str, id: u64) -> WireSource {
    let source = dummy_source(id);
    Device::connect(tim, pin, source.clone()).expect("the pin exists");
    source
}

/// Drive an input pin: build the net its sink hangs off, and hand back the
/// source a test moves.
fn drive_input(tim: &Tim, pin: &str, id: u64) -> WireSource {
    let wire_id = WireId::new(id);
    let sink = Device::sink(tim, pin, &[wire_id]).expect("the pin exists");
    let wire = Wire::builder()
        .source(wire_id)
        .sink(sink.sink, sink.line)
        .build_shared();
    WireSource::new(wire, wire_id)
}

/// Wire `master`'s `TRGO` into `slave`'s `itrN` — the internal trigger
/// connection RM0090 Table 86 makes on the die and a board file makes here.
fn chain(master: &Tim, slave: &Tim, itr: &str, id: u64) {
    let wire_id = WireId::new(id);
    let sink = Device::sink(slave, itr, &[wire_id]).expect("the slave has that ITR");
    let wire = Wire::builder()
        .source(wire_id)
        .sink(sink.sink, sink.line)
        .build_shared();
    Device::connect(master, TRGO_PIN, WireSource::new(wire, wire_id)).expect("the master has TRGO");
}

/// Put channel `i` into input mode on its own `TIx`, with a filter and a
/// capture prescaler, and enable the capture.
fn configure_capture(tim: &Tim, i: usize, polarity: u32, filter: u32, prescaler: u32) {
    let offset = if i < 2 { OFF_CCMR1 } else { OFF_CCMR2 };
    let shift = 8 * (i as u32 % 2);
    let byte = CCS_TI_DIRECT | (prescaler << CCMR_ICPSC_SHIFT) | (filter << CCMR_ICF_SHIFT);
    let current = peek(tim, offset);
    poke(tim, offset, (current & !(0xff << shift)) | (byte << shift));
    let ccer = peek(tim, OFF_CCER) & !(0xf << (4 * i));
    poke(tim, OFF_CCER, ccer | ((CCER_CCE | polarity) << (4 * i)));
}

/// A sink that counts rising edges, for the output pins that pulse.
#[derive(Debug, Default)]
struct Pulses {
    count: AtomicU32,
}

impl Pulses {
    fn get(&self) -> u32 {
        self.count.load(Ordering::Relaxed)
    }
}

impl WireSink for Pulses {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        if level == Level::High {
            self.count.fetch_add(1, Ordering::Relaxed);
        }
    }
}

/// Count the pulses on one of this timer's output pins.
fn watch(tim: &Tim, pin: &str, id: u64) -> Arc<Pulses> {
    let counter = Arc::new(Pulses::default());
    let wire_id = WireId::new(id);
    let wire = Wire::builder()
        .source(wire_id)
        .sink(Arc::clone(&counter) as Arc<dyn WireSink>, 0)
        .build_shared();
    Device::connect(tim, pin, WireSource::new(wire, wire_id)).expect("the pin exists");
    counter
}

/// Start the counter, which is always the last thing firmware does.
fn start(tim: &Tim) {
    poke(tim, OFF_CR1, peek(tim, OFF_CR1) | CR1_CEN);
}

// ---------------------------------------------------------------------------
// The counter
// ---------------------------------------------------------------------------

#[test]
fn uif_sets_after_arr_plus_one_ticks_and_cnt_wraps() {
    let tim = general();
    poke(&tim, OFF_ARR, 9);
    start(&tim);

    for expected in 1..=9u32 {
        tim.advance_by(1);
        assert_eq!(peek(&tim, OFF_CNT), expected);
        assert_eq!(peek(&tim, OFF_SR) & SR_UIF, 0, "not yet");
    }
    // The tenth tick is the overflow: `CNT` returns to zero and `UIF` sets.
    tim.advance_by(1);
    assert_eq!(peek(&tim, OFF_CNT), 0);
    assert_eq!(peek(&tim, OFF_SR) & SR_UIF, SR_UIF);

    // And a period is ARR + 1 ticks, every time.
    poke(&tim, OFF_SR, !SR_UIF);
    tim.advance_by(9);
    assert_eq!(peek(&tim, OFF_SR) & SR_UIF, 0);
    tim.advance_by(1);
    assert_eq!(peek(&tim, OFF_SR) & SR_UIF, SR_UIF);
}

#[test]
fn the_prescaler_divides_the_counter_and_keeps_its_phase() {
    let tim = general();
    poke(&tim, OFF_ARR, 0xffff);
    poke(&tim, OFF_PSC, 3); // CK_INT / 4
    poke(&tim, OFF_EGR, EGR_UG); // load the shadow now
    poke(&tim, OFF_SR, 0);
    start(&tim);

    tim.advance_by(3);
    assert_eq!(peek(&tim, OFF_CNT), 0, "three of the four ticks");
    tim.advance_by(1);
    assert_eq!(peek(&tim, OFF_CNT), 1);
    // A span that straddles several divisions is exact, not rounded.
    tim.advance_by(10);
    assert_eq!(peek(&tim, OFF_CNT), 3, "11 of 14 ticks are three clocks");
}

#[test]
fn psc_is_applied_only_at_the_next_update_event() {
    let tim = general();
    poke(&tim, OFF_ARR, 3);
    start(&tim);
    assert_eq!(peek(&tim, OFF_PSC), 0);

    // Two ticks in, ask for a divide-by-four.
    tim.advance_by(2);
    assert_eq!(peek(&tim, OFF_CNT), 2);
    poke(&tim, OFF_PSC, 3);
    assert_eq!(peek(&tim, OFF_PSC), 3, "the preload register reads back");

    // The old ratio is still in force until the update event, so the counter
    // reaches ARR and wraps on the very next two ticks.
    tim.advance_by(2);
    assert_eq!(peek(&tim, OFF_CNT), 0);
    assert_eq!(peek(&tim, OFF_SR) & SR_UIF, SR_UIF);

    // Now divide-by-four is loaded: four ticks to each count.
    tim.advance_by(3);
    assert_eq!(peek(&tim, OFF_CNT), 0);
    tim.advance_by(1);
    assert_eq!(peek(&tim, OFF_CNT), 1);
}

#[test]
fn arr_is_shadowed_when_arpe_is_set_and_immediate_when_it_is_not() {
    // ARPE clear: the write lands at once, which is what makes a shortened
    // period take effect this cycle.
    let tim = general();
    poke(&tim, OFF_ARR, 99);
    start(&tim);
    tim.advance_by(3);
    poke(&tim, OFF_ARR, 4);
    tim.advance_by(1);
    assert_eq!(peek(&tim, OFF_CNT), 4);
    tim.advance_by(1);
    assert_eq!(peek(&tim, OFF_CNT), 0, "the shortened period took at once");
    assert_eq!(peek(&tim, OFF_SR) & SR_UIF, SR_UIF);

    // ARPE set: the write waits for the update event. This is the classic
    // emulation bug in this peripheral, so it is asserted directly.
    let tim = general();
    poke(&tim, OFF_CR1, CR1_ARPE);
    poke(&tim, OFF_ARR, 9);
    poke(&tim, OFF_EGR, EGR_UG);
    poke(&tim, OFF_SR, 0);
    start(&tim);

    tim.advance_by(3);
    poke(&tim, OFF_ARR, 4);
    assert_eq!(peek(&tim, OFF_ARR), 4, "the preload register reads back");
    // The period in force is still 10, so nothing happens at 4 or 5 …
    tim.advance_by(2);
    assert_eq!(peek(&tim, OFF_CNT), 5);
    assert_eq!(peek(&tim, OFF_SR) & SR_UIF, 0, "still the old ARR");
    // … and the counter runs all the way to 9 before wrapping.
    tim.advance_by(5);
    assert_eq!(peek(&tim, OFF_CNT), 0);
    assert_eq!(peek(&tim, OFF_SR) & SR_UIF, SR_UIF);

    // From now on the period is 5.
    poke(&tim, OFF_SR, 0);
    tim.advance_by(4);
    assert_eq!(peek(&tim, OFF_CNT), 4);
    tim.advance_by(1);
    assert_eq!(peek(&tim, OFF_CNT), 0);
    assert_eq!(peek(&tim, OFF_SR) & SR_UIF, SR_UIF);
}

#[test]
fn clearing_arpe_pushes_the_written_value_through_at_once() {
    let tim = general();
    poke(&tim, OFF_CR1, CR1_ARPE);
    poke(&tim, OFF_ARR, 99);
    poke(&tim, OFF_EGR, EGR_UG);
    poke(&tim, OFF_ARR, 4);
    start(&tim);
    tim.advance_by(3);
    assert_eq!(peek(&tim, OFF_CNT), 3, "the shadow still says 99");

    // "The new value is taken into account immediately" once preload is off.
    poke(&tim, OFF_CR1, CR1_CEN);
    tim.advance_by(1);
    assert_eq!(peek(&tim, OFF_CNT), 4);
    tim.advance_by(1);
    assert_eq!(peek(&tim, OFF_CNT), 0);
    assert_eq!(peek(&tim, OFF_SR) & SR_UIF, SR_UIF);
}

#[test]
fn an_auto_reload_dropped_below_the_counter_runs_the_period_out_the_long_way() {
    // The gotcha every STM32 forum thread is about: the comparator matches on
    // equality, so a counter already past the new `ARR` never meets it and runs
    // up to the width's own wrap before restarting. Nothing sets `UIF` on the
    // way round, which is why the missing interrupt looks like a dead timer.
    let tim = general();
    poke(&tim, OFF_ARR, 0xffff);
    start(&tim);
    tim.advance_by(100);
    poke(&tim, OFF_ARR, 50);
    tim.advance_by(1);
    assert_eq!(peek(&tim, OFF_CNT), 101, "it kept going up");
    assert_eq!(peek(&tim, OFF_SR) & SR_UIF, 0);
    tim.advance_by(0xffff - 101);
    assert_eq!(peek(&tim, OFF_CNT), 0xffff);
    tim.advance_by(1);
    assert_eq!(peek(&tim, OFF_CNT), 0, "the width wraps it, silently");
    assert_eq!(peek(&tim, OFF_SR) & SR_UIF, 0);
    tim.advance_by(51);
    assert_eq!(
        peek(&tim, OFF_SR) & SR_UIF,
        SR_UIF,
        "and now the comparator"
    );
}

#[test]
fn downcounting_reloads_arr_at_the_underflow() {
    let tim = general();
    poke(&tim, OFF_CR1, CR1_DIR);
    poke(&tim, OFF_ARR, 3);
    poke(&tim, OFF_EGR, EGR_UG);
    poke(&tim, OFF_SR, 0);
    assert_eq!(peek(&tim, OFF_CNT), 3, "UG loads ARR when counting down");
    start(&tim);

    for expected in [2u32, 1, 0] {
        tim.advance_by(1);
        assert_eq!(peek(&tim, OFF_CNT), expected);
        assert_eq!(peek(&tim, OFF_SR) & SR_UIF, 0);
    }
    tim.advance_by(1);
    assert_eq!(peek(&tim, OFF_CNT), 3, "the underflow reloads");
    assert_eq!(peek(&tim, OFF_SR) & SR_UIF, SR_UIF);
}

#[test]
fn center_aligned_mode_counts_up_then_down_and_sets_uif_at_both_ends_when_cms_is_11() {
    let tim = general();
    // CMS = 11: the flags are set on both the up and the down slope.
    poke(&tim, OFF_CR1, 0x3 << CR1_CMS_SHIFT);
    poke(&tim, OFF_ARR, 3);
    start(&tim);

    let mut seen: Vec<(u32, bool)> = Vec::new();
    for _ in 0..12 {
        poke(&tim, OFF_SR, 0);
        tim.advance_by(1);
        seen.push((peek(&tim, OFF_CNT), peek(&tim, OFF_SR) & SR_UIF != 0));
    }
    assert_eq!(
        seen,
        alloc::vec![
            (1, false),
            (2, false),
            (3, true), // the overflow, at the peak
            (2, false),
            (1, false),
            (0, true), // the underflow, at the trough
            (1, false),
            (2, false),
            (3, true),
            (2, false),
            (1, false),
            (0, true),
        ],
        "a center-aligned period is 2 * ARR counter clocks"
    );
}

#[test]
fn a_32_bit_tim2_does_not_wrap_at_16_bits() {
    let tim = general32();
    poke(&tim, OFF_ARR, 0x0002_0000);
    poke(&tim, OFF_CNT, 0x0000_fffe);
    start(&tim);
    tim.advance_by(4);
    assert_eq!(peek(&tim, OFF_CNT), 0x0001_0002);
    assert_eq!(
        peek(&tim, OFF_SR) & SR_UIF,
        0,
        "nothing happened at 16 bits"
    );

    // And a 16-bit part truncates the same write.
    let tim = general();
    poke(&tim, OFF_ARR, 0x0002_0000);
    assert_eq!(peek(&tim, OFF_ARR), 0);
}

#[test]
fn one_pulse_mode_stops_after_one_update() {
    let tim = general();
    poke(&tim, OFF_CR1, CR1_OPM);
    poke(&tim, OFF_ARR, 4);
    start(&tim);
    assert_eq!(peek(&tim, OFF_CR1) & CR1_CEN, CR1_CEN);

    tim.advance_by(5);
    assert_eq!(peek(&tim, OFF_CNT), 0);
    assert_eq!(peek(&tim, OFF_SR) & SR_UIF, SR_UIF);
    assert_eq!(peek(&tim, OFF_CR1) & CR1_CEN, 0, "the update cleared CEN");
    assert_eq!(tim.next_event(), None, "and nothing is pending");

    // A stopped counter stays put however long the machine runs.
    tim.advance_by(1000);
    assert_eq!(peek(&tim, OFF_CNT), 0);
}

#[test]
fn ug_reinitializes_the_counter_and_urs_decides_whether_uif_follows() {
    let tim = general();
    poke(&tim, OFF_ARR, 99);
    start(&tim);
    tim.advance_by(7);
    assert_eq!(peek(&tim, OFF_CNT), 7);

    poke(&tim, OFF_EGR, EGR_UG);
    assert_eq!(peek(&tim, OFF_CNT), 0);
    assert_eq!(peek(&tim, OFF_SR) & SR_UIF, SR_UIF);

    // With URS set, only a real over/underflow raises the flag — which is what
    // stops a `UG` used to load the prescaler from firing a spurious interrupt.
    poke(&tim, OFF_SR, 0);
    poke(&tim, OFF_CR1, CR1_CEN | CR1_URS);
    poke(&tim, OFF_PSC, 1);
    poke(&tim, OFF_EGR, EGR_UG);
    assert_eq!(peek(&tim, OFF_SR) & SR_UIF, 0, "URS suppressed it");
    // The shadow was still reloaded, so the new ratio is in force.
    tim.advance_by(1);
    assert_eq!(peek(&tim, OFF_CNT), 0);
    tim.advance_by(1);
    assert_eq!(peek(&tim, OFF_CNT), 1);
}

#[test]
fn udis_suppresses_the_update_entirely_but_ug_still_clears_the_counter() {
    let tim = general();
    poke(&tim, OFF_ARR, 4);
    poke(&tim, OFF_CR1, CR1_UDIS);
    start(&tim);
    tim.advance_by(5);
    assert_eq!(peek(&tim, OFF_CNT), 0, "the counter still wrapped");
    assert_eq!(peek(&tim, OFF_SR) & SR_UIF, 0, "but there was no update");

    // "However the counter and the prescaler are reinitialized if the UG bit
    // is set" — the shadows are not, so a buffered `ARR` stays where it was.
    tim.advance_by(3);
    poke(&tim, OFF_CR1, CR1_CEN | CR1_UDIS | CR1_ARPE);
    poke(&tim, OFF_ARR, 9);
    poke(&tim, OFF_EGR, EGR_UG);
    assert_eq!(peek(&tim, OFF_CNT), 0);
    assert_eq!(peek(&tim, OFF_SR) & SR_UIF, 0);
    tim.advance_by(5);
    assert_eq!(peek(&tim, OFF_CNT), 0, "still the old ARR of 4");
}

#[test]
fn the_repetition_counter_divides_the_update_rate() {
    let tim = advanced();
    poke(&tim, OFF_ARR, 3);
    poke(&tim, OFF_RCR, 2); // an update every three overflows
    poke(&tim, OFF_EGR, EGR_UG);
    poke(&tim, OFF_SR, 0);
    start(&tim);

    tim.advance_by(4);
    assert_eq!(peek(&tim, OFF_SR) & SR_UIF, 0, "one overflow of three");
    tim.advance_by(4);
    assert_eq!(peek(&tim, OFF_SR) & SR_UIF, 0, "two");
    tim.advance_by(4);
    assert_eq!(peek(&tim, OFF_SR) & SR_UIF, SR_UIF, "three: the update");

    // A general-purpose timer has no RCR, so every overflow is an update.
    let tim = general();
    poke(&tim, OFF_RCR, 2);
    assert_eq!(peek(&tim, OFF_RCR), 0, "the register is not there");
}

// ---------------------------------------------------------------------------
// The interrupt
// ---------------------------------------------------------------------------

#[test]
fn uie_raises_the_irq_wire_and_writing_zero_to_uif_lowers_it() {
    let tim = general();
    let irq = wire(&tim, IRQ_PIN, 1);
    poke(&tim, OFF_ARR, 4);
    poke(&tim, OFF_DIER, DIER_UIE);
    start(&tim);
    assert_eq!(irq.level(), Level::Low);

    tim.advance_by(5);
    assert_eq!(irq.level(), Level::High, "UIF and UIE together");

    // `rc_w0`: a zero clears the flag, a one leaves it alone.
    poke(&tim, OFF_SR, !SR_UIF);
    assert_eq!(irq.level(), Level::Low);

    // Without the enable the flag still sets, and the pin stays down.
    poke(&tim, OFF_DIER, 0);
    tim.advance_by(5);
    assert_eq!(peek(&tim, OFF_SR) & SR_UIF, SR_UIF);
    assert_eq!(irq.level(), Level::Low);
}

#[test]
fn a_compare_match_sets_ccxif_and_raises_the_line() {
    let tim = general();
    let irq = wire(&tim, IRQ_PIN, 1);
    poke(&tim, OFF_ARR, 9);
    poke(&tim, OFF_CCR1 + 4, 3); // CCR2
    poke(&tim, OFF_DIER, DIER_CC1IE << 1);
    start(&tim);

    tim.advance_by(2);
    assert_eq!(peek(&tim, OFF_SR) & (SR_CC1IF << 1), 0);
    assert_eq!(irq.level(), Level::Low);
    tim.advance_by(1);
    assert_eq!(peek(&tim, OFF_SR) & (SR_CC1IF << 1), SR_CC1IF << 1);
    assert_eq!(irq.level(), Level::High);

    poke(&tim, OFF_SR, !(SR_CC1IF << 1));
    assert_eq!(irq.level(), Level::Low);
}

#[test]
fn an_advanced_timer_splits_its_vectors_and_the_others_refuse_to() {
    let tim = advanced();
    let up = wire(&tim, IRQ_UP_PIN, 1);
    let cc = wire(&tim, IRQ_CC_PIN, 2);
    let irq = wire(&tim, IRQ_PIN, 3);
    poke(&tim, OFF_ARR, 9);
    poke(&tim, OFF_CCR1, 3);
    poke(&tim, OFF_DIER, DIER_UIE | DIER_CC1IE);
    start(&tim);

    tim.advance_by(3);
    assert_eq!(cc.level(), Level::High, "the compare vector");
    assert_eq!(up.level(), Level::Low);
    assert_eq!(irq.level(), Level::High, "and the combined line");

    poke(&tim, OFF_SR, !SR_CC1IF);
    tim.advance_by(7);
    assert_eq!(up.level(), Level::High, "the update vector");
    assert_eq!(cc.level(), Level::Low);

    // A general-purpose timer has one vector and says so.
    let tim = general();
    let err = Device::connect(&tim, IRQ_UP_PIN, dummy_source(4)).unwrap_err();
    assert!(err.to_string().contains("advanced"), "{err}");
}

// ---------------------------------------------------------------------------
// Output compare
// ---------------------------------------------------------------------------

/// Put channel `i` (zero-based) into `mode` with the given compare value and
/// enable its output.
fn configure_channel(tim: &Tim, i: usize, mode: u32, ccr: u32, preload: bool) {
    let offset = if i < 2 { OFF_CCMR1 } else { OFF_CCMR2 };
    let shift = 8 * (i as u32 % 2);
    let mut byte = mode << CCMR_OCM_SHIFT;
    if preload {
        byte |= CCMR_OCPE;
    }
    let current = peek(tim, offset);
    poke(tim, offset, (current & !(0xff << shift)) | (byte << shift));
    poke(tim, OFF_CCR1 + 4 * i as u64, ccr);
    poke(tim, OFF_CCER, peek(tim, OFF_CCER) | (CCER_CCE << (4 * i)));
}

#[test]
fn pwm_mode_1_drives_the_channel_wire_high_for_ccr_ticks_out_of_arr_plus_one() {
    let tim = general();
    let ch1 = wire(&tim, "ch1", 1);
    poke(&tim, OFF_ARR, 9);
    configure_channel(&tim, 0, OCM_PWM1, 3, false);
    start(&tim);

    // ARR = 9, CCR1 = 3: high while CNT is 0, 1 or 2, low from 3 to 9.
    for period in 0..3 {
        let mut levels = Vec::new();
        for _ in 0..10 {
            levels.push(ch1.level() == Level::High);
            tim.advance_by(1);
        }
        assert_eq!(
            levels,
            alloc::vec![
                true, true, true, false, false, false, false, false, false, false
            ],
            "period {period}"
        );
    }

    // A duty of zero is genuinely zero, and a compare above ARR is always on —
    // the two cases an edge-driven model gets wrong.
    poke(&tim, OFF_CCR1, 0);
    poke(&tim, OFF_EGR, EGR_UG);
    for _ in 0..10 {
        assert_eq!(ch1.level(), Level::Low);
        tim.advance_by(1);
    }
    poke(&tim, OFF_CCR1, 100);
    poke(&tim, OFF_EGR, EGR_UG);
    for _ in 0..10 {
        assert_eq!(ch1.level(), Level::High);
        tim.advance_by(1);
    }
}

#[test]
fn pwm_mode_2_is_the_inverse_and_ccxp_inverts_the_pin() {
    let tim = general();
    let ch1 = wire(&tim, "ch1", 1);
    poke(&tim, OFF_ARR, 3);
    configure_channel(&tim, 0, OCM_PWM2, 2, false);
    start(&tim);

    let mut levels = Vec::new();
    for _ in 0..4 {
        levels.push(ch1.level() == Level::High);
        tim.advance_by(1);
    }
    assert_eq!(levels, alloc::vec![false, false, true, true]);

    // `CCER.CCxP` inverts what reaches the pin, not `OCxREF`.
    poke(&tim, OFF_CCER, peek(&tim, OFF_CCER) | CCER_CCP);
    poke(&tim, OFF_EGR, EGR_UG);
    let mut levels = Vec::new();
    for _ in 0..4 {
        levels.push(ch1.level() == Level::High);
        tim.advance_by(1);
    }
    assert_eq!(levels, alloc::vec![true, true, false, false]);
}

#[test]
fn toggle_and_the_forced_modes_do_what_ocxm_says() {
    let tim = general();
    let ch1 = wire(&tim, "ch1", 1);
    poke(&tim, OFF_ARR, 3);
    configure_channel(&tim, 0, OCM_TOGGLE, 2, false);
    start(&tim);

    assert_eq!(ch1.level(), Level::Low);
    tim.advance_by(2);
    assert_eq!(ch1.level(), Level::High, "the first match");
    tim.advance_by(4);
    assert_eq!(ch1.level(), Level::Low, "the second");

    // Forcing is immediate and needs no match.
    configure_channel(&tim, 0, OCM_FORCE_ACTIVE, 2, false);
    assert_eq!(ch1.level(), Level::High);
    configure_channel(&tim, 0, OCM_FORCE_INACTIVE, 2, false);
    assert_eq!(ch1.level(), Level::Low);

    // Frozen holds whatever the latch has.
    configure_channel(&tim, 0, OCM_ACTIVE, 2, false);
    tim.advance_by(4);
    assert_eq!(ch1.level(), Level::High);
    configure_channel(&tim, 0, OCM_FROZEN, 2, false);
    tim.advance_by(8);
    assert_eq!(ch1.level(), Level::High, "the comparison has no effect");
}

#[test]
fn ocxpe_shadows_the_compare_register_until_the_update() {
    let tim = general();
    let ch1 = wire(&tim, "ch1", 1);
    poke(&tim, OFF_ARR, 9);
    configure_channel(&tim, 0, OCM_PWM1, 3, true);
    poke(&tim, OFF_EGR, EGR_UG);
    start(&tim);

    // Ask for a wider pulse two ticks into the period.
    tim.advance_by(2);
    poke(&tim, OFF_CCR1, 8);
    assert_eq!(peek(&tim, OFF_CCR1), 8, "the preload register reads back");
    tim.advance_by(1);
    assert_eq!(ch1.level(), Level::Low, "this period is still 3 wide");

    // The update event loads it; the next period is 8 wide.
    tim.advance_by(7);
    assert_eq!(peek(&tim, OFF_CNT), 0);
    let mut levels = Vec::new();
    for _ in 0..10 {
        levels.push(ch1.level() == Level::High);
        tim.advance_by(1);
    }
    assert_eq!(
        levels,
        alloc::vec![true, true, true, true, true, true, true, true, false, false]
    );
}

#[test]
fn a_disabled_channel_drives_nothing() {
    let tim = general();
    let ch1 = wire(&tim, "ch1", 1);
    poke(&tim, OFF_ARR, 3);
    configure_channel(&tim, 0, OCM_FORCE_ACTIVE, 0, false);
    assert_eq!(ch1.level(), Level::High);
    poke(&tim, OFF_CCER, 0);
    assert_eq!(ch1.level(), Level::Low);
}

#[test]
fn moe_clear_holds_an_advanced_timers_outputs_at_their_idle_level() {
    let tim = advanced();
    let ch1 = wire(&tim, "ch1", 1);
    let ch1n = wire(&tim, "ch1n", 2);
    poke(&tim, OFF_ARR, 3);
    configure_channel(&tim, 0, OCM_PWM1, 2, false);
    poke(&tim, OFF_CCER, peek(&tim, OFF_CCER) | CCER_CCNE);

    // With `MOE` clear the outputs sit at `CR2.OIS1` / `OIS1N`, which come up
    // as zero — so both are low, however the comparator is driving.
    start(&tim);
    assert_eq!(ch1.level(), Level::Low);
    assert_eq!(ch1n.level(), Level::Low);
    tim.advance_by(1);
    assert_eq!(ch1.level(), Level::Low, "MOE gates it");

    // A non-zero idle level is what a board with a low-side driver programs.
    poke(&tim, OFF_CR2, 1 << CR2_OIS_SHIFT); // OIS1 = 1
    assert_eq!(ch1.level(), Level::High);
    assert_eq!(ch1n.level(), Level::Low);

    // Enabling the main output hands the pins back to the comparator, and the
    // complementary output is the reference inverted.
    poke(&tim, OFF_BDTR, BDTR_MOE);
    poke(&tim, OFF_EGR, EGR_UG);
    assert_eq!(ch1.level(), Level::High, "CNT 0 < CCR 2");
    assert_eq!(ch1n.level(), Level::Low);
    tim.advance_by(2);
    assert_eq!(ch1.level(), Level::Low);
    assert_eq!(ch1n.level(), Level::High);

    // And clearing it puts them straight back to idle.
    poke(&tim, OFF_BDTR, 0);
    assert_eq!(ch1.level(), Level::High, "OIS1");
    assert_eq!(ch1n.level(), Level::Low);
}

#[test]
fn a_general_purpose_timer_has_no_moe_to_gate_with() {
    let tim = general();
    let ch1 = wire(&tim, "ch1", 1);
    configure_channel(&tim, 0, OCM_FORCE_ACTIVE, 0, false);
    assert_eq!(peek(&tim, OFF_BDTR), 0, "the register is not there");
    assert_eq!(ch1.level(), Level::High, "and nothing gates the output");
    assert!(Device::connect(&tim, "ch1n", dummy_source(9)).is_err());
}

// ---------------------------------------------------------------------------
// The scheduler seam
// ---------------------------------------------------------------------------

#[test]
fn the_next_event_lands_on_the_tick_the_update_is_due() {
    let tim = general();
    assert_eq!(tim.next_event(), None, "a stopped counter has nothing due");

    poke(&tim, OFF_ARR, 9);
    poke(&tim, OFF_PSC, 4);
    poke(&tim, OFF_EGR, EGR_UG);
    poke(&tim, OFF_SR, 0);
    start(&tim);
    // 10 counter clocks of five CK_INT each.
    assert_eq!(tim.next_event(), Some(50));

    tim.advance_by(7);
    assert_eq!(tim.next_event(), Some(50), "still the same instant");
    tim.advance_by(43);
    assert_eq!(peek(&tim, OFF_SR) & SR_UIF, SR_UIF);
    assert_eq!(tim.next_event(), Some(100));

    // A compare match is an event too: the wire has to change on its tick.
    poke(&tim, OFF_CCR1, 3);
    configure_channel(&tim, 0, OCM_PWM1, 3, false);
    assert_eq!(tim.next_event(), Some(65), "three counter clocks away");
}

#[test]
fn advancing_in_one_jump_matches_advancing_tick_by_tick() {
    let one = general();
    let many = general();
    for tim in [&one, &many] {
        poke(tim, OFF_ARR, 7);
        poke(tim, OFF_PSC, 2);
        poke(tim, OFF_EGR, EGR_UG);
        poke(tim, OFF_SR, 0);
        configure_channel(tim, 0, OCM_TOGGLE, 5, false);
        start(tim);
    }
    one.advance_by(1000);
    for _ in 0..1000 {
        many.advance_by(1);
    }
    assert_eq!(peek(&one, OFF_CNT), peek(&many, OFF_CNT));
    assert_eq!(peek(&one, OFF_SR), peek(&many, OFF_SR));
    assert_eq!(one.next_event(), many.next_event());
    // One lock at a time: `core::sync` ranks them, and two of the same rank
    // held at once is a violation whoever owns them.
    let one_regs = *one.shared.regs.lock();
    let many_regs = *many.shared.regs.lock();
    assert_eq!(
        one_regs.ocref, many_regs.ocref,
        "the toggle latch took the same number of matches either way"
    );
    assert_eq!(one_regs, many_regs);
}

#[test]
fn a_debug_read_advances_nothing_and_a_debug_write_is_refused() {
    let tim = general();
    poke(&tim, OFF_ARR, 9);
    start(&tim);
    tim.advance_by(4);

    let before = tim.current_tick();
    assert_eq!(peek_debug(&tim, OFF_CNT), 4);
    assert_eq!(peek_debug(&tim, OFF_SR), 0);
    assert_eq!(tim.current_tick(), before, "a debug read moved no time");

    let err = tim
        .shared
        .write(OFF_EGR, &EGR_UG.to_le_bytes(), MemAttrs::DEBUG)
        .unwrap_err();
    assert_eq!(err, BusError::BadAccess);
    assert_eq!(peek(&tim, OFF_CNT), 4, "and it changed nothing");
}

#[test]
fn half_word_access_reaches_both_halves_of_a_32_bit_counter() {
    let tim = general32();
    poke(&tim, OFF_ARR, 0xffff_ffff);
    poke(&tim, OFF_CNT, 0x1234_5678);

    let mut half = [0u8; 2];
    tim.shared
        .read(OFF_CNT, &mut half, MemAttrs::DEFAULT)
        .unwrap();
    assert_eq!(u16::from_le_bytes(half), 0x5678);
    tim.shared
        .read(OFF_CNT + 2, &mut half, MemAttrs::DEFAULT)
        .unwrap();
    assert_eq!(u16::from_le_bytes(half), 0x1234);

    // A half-word write leaves the other half alone.
    tim.shared
        .write(OFF_CNT + 2, &0xbeefu16.to_le_bytes(), MemAttrs::DEFAULT)
        .unwrap();
    assert_eq!(peek(&tim, OFF_CNT), 0xbeef_5678);

    // A byte access is refused rather than widened.
    let mut byte = [0u8; 1];
    assert!(
        tim.shared
            .read(OFF_CNT, &mut byte, MemAttrs::DEFAULT)
            .is_err()
    );
}

#[test]
fn a_null_auto_reload_blocks_the_counter() {
    // RM0090 §19.4.8: "The counter is blocked while the auto-reload value is
    // null." Without it a reset timer would ask for an event every tick.
    let tim = general();
    start(&tim);
    assert_eq!(tim.next_event(), None);
    tim.advance_by(1000);
    assert_eq!(peek(&tim, OFF_CNT), 0);
    assert_eq!(peek(&tim, OFF_SR) & SR_UIF, 0);
}

// ---------------------------------------------------------------------------
// The basic timer
// ---------------------------------------------------------------------------

#[test]
fn a_basic_timer_is_a_counter_and_nothing_else() {
    let tim = basic();
    assert_eq!(tim.channels(), 0);
    // `CCMR1`, `CCER`, `CCR1`, `RCR` and `BDTR` are not there: they read zero
    // and a write to one is ignored rather than faulting.
    for offset in [
        OFF_CCMR1, OFF_CCMR2, OFF_CCER, OFF_CCR1, OFF_RCR, OFF_BDTR, OFF_SMCR, OFF_DCR, OFF_DMAR,
    ] {
        poke(&tim, offset, 0xffff_ffff);
        assert_eq!(peek(&tim, offset), 0, "offset {offset:#x}");
    }
    // No slave controller either, so nothing an `SMCR` write could have set is
    // acted on: a basic timer's counter runs off `CK_INT` and nothing else.
    assert!(Device::sink(&tim, "itr0", &[WireId::new(3)]).is_none());
    // `CR1` keeps only the bits RM0090 §19.4.1 gives it — no `DIR`, no `CMS`,
    // no `CKD`.
    poke(&tim, OFF_CR1, 0xffff_ffff);
    assert_eq!(peek(&tim, OFF_CR1), CR1_MASK_BASIC);
    poke(&tim, OFF_CR1, CR1_CEN);

    poke(&tim, OFF_ARR, 4);
    let irq = wire(&tim, IRQ_PIN, 1);
    poke(&tim, OFF_DIER, DIER_UIE);
    poke(&tim, OFF_CR1, CR1_CEN);
    tim.advance_by(5);
    assert_eq!(peek(&tim, OFF_CNT), 0);
    assert_eq!(irq.level(), Level::High);
    assert!(Device::connect(&tim, "ch1", dummy_source(2)).is_err());
}

// ---------------------------------------------------------------------------
// Construction and the snapshot
// ---------------------------------------------------------------------------

#[test]
fn properties_name_a_real_part_or_are_refused() {
    let tim = Tim::new(&Props::new()).unwrap();
    assert_eq!(tim.variant(), Variant::General);
    assert_eq!(tim.channels(), 4);
    assert_eq!(tim.shared.cfg.mask, 0xffff);

    let props = Props::new()
        .with("variant", Value::from("basic"))
        .with("width", Value::from(16u64));
    let tim = Tim::new(&props).unwrap();
    assert_eq!(tim.variant(), Variant::Basic);
    assert_eq!(tim.channels(), 0, "a basic timer defaults to none");

    let props = Props::new()
        .with("variant", Value::from("advanced"))
        .with("channels", Value::from(2u64));
    assert_eq!(Tim::new(&props).unwrap().channels(), 2);

    for (name, value) in [
        ("width", Value::from(24u64)),
        ("channels", Value::from(5u64)),
        ("varient", Value::from("basic")),
    ] {
        assert!(
            Tim::new(&Props::new().with(name, value.clone())).is_err(),
            "{name} = {value:?}"
        );
    }
    // A basic timer with channels is not a part that exists.
    let props = Props::new()
        .with("variant", Value::from("basic"))
        .with("channels", Value::from(4u64));
    assert!(Tim::new(&props).is_err());
    // An auto-reload reset value has to fit the counter.
    let props = Props::new().with("arr-reset", Value::from(0x1_0000u64));
    assert!(Tim::new(&props).is_err());
    let props = Props::new()
        .with("width", Value::from(32u64))
        .with("arr-reset", Value::from(0xffff_ffffu64));
    assert_eq!(peek(&Tim::new(&props).unwrap(), OFF_ARR), 0xffff_ffff);

    assert_eq!(Variant::Basic.as_str(), "basic");
    assert_eq!(Variant::General.as_str(), "general");
    assert_eq!(Variant::Advanced.as_str(), "advanced");
}

#[test]
fn a_reset_returns_every_register_to_its_documented_value() {
    let tim = advanced();
    poke(&tim, OFF_ARR, 0x1234);
    poke(&tim, OFF_PSC, 7);
    poke(&tim, OFF_BDTR, BDTR_MOE);
    configure_channel(&tim, 0, OCM_PWM1, 3, false);
    start(&tim);
    tim.advance_by(9);

    Device::reset(&tim, ResetKind::Cold);
    for offset in [
        OFF_CR1, OFF_CR2, OFF_DIER, OFF_SR, OFF_CCMR1, OFF_CCER, OFF_CNT, OFF_PSC, OFF_ARR,
        OFF_RCR, OFF_CCR1, OFF_BDTR,
    ] {
        assert_eq!(peek(&tim, offset), 0, "offset {offset:#x}");
    }
    assert_eq!(tim.next_event(), None);
}

#[test]
fn a_snapshot_round_trips_to_identical_state() {
    let saved = advanced();
    poke(&saved, OFF_CR1, CR1_ARPE);
    poke(&saved, OFF_ARR, 19);
    poke(&saved, OFF_PSC, 3);
    poke(&saved, OFF_RCR, 1);
    poke(&saved, OFF_DIER, DIER_UIE | DIER_CC1IE);
    poke(&saved, OFF_BDTR, BDTR_MOE);
    configure_channel(&saved, 0, OCM_PWM1, 7, true);
    configure_channel(&saved, 1, OCM_TOGGLE, 11, false);
    poke(&saved, OFF_EGR, EGR_UG);
    start(&saved);
    // Land mid-prescaler and mid-period, with a latched toggle behind us: the
    // three pieces of state a lazy timer gets wrong if it saves only `CNT`.
    saved.advance_by(46);
    poke(&saved, OFF_ARR, 25);
    poke(&saved, OFF_CCR1, 9);

    // The input side: `SMCR`'s storage, `OR1`/`OR2`, a burst index off zero,
    // and — the one that a naive encoding drops — a filter sample part-way
    // through its hold time.
    poke(
        &saved,
        OFF_SMCR,
        (1 << SMCR_TS_SHIFT) | (0b0011 << SMCR_ETF_SHIFT) | SMCR_ETP,
    );
    poke(&saved, OFF_OR1, 0x5555);
    poke(&saved, OFF_OR2, 0xaaaa);
    poke(&saved, OFF_DCR, 13 | (2 << 8));
    poke(&saved, OFF_DMAR, 0);
    poke(
        &saved,
        OFF_CCMR2,
        CCS_TI_DIRECT | (0b0011 << CCMR_ICF_SHIFT),
    );
    poke(&saved, OFF_CCER, peek(&saved, OFF_CCER) | (CCER_CCE << 8));
    let ti3 = drive_input(&saved, "ti3", 77);
    ti3.set(Level::High);

    let mut shape = MachineShape::new();
    shape.add_device("tim1", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("tim1", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(&saved, &mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();

    let restored = advanced();
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("tim1", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(&restored, &mut chunk.reader()).unwrap();

    // The strongest form the assertion takes: saving the restored device again
    // has to produce byte-identical bytes, so nothing in the encoding was
    // dropped on the way through.
    let mut shape = MachineShape::new();
    shape.add_device("tim1", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("tim1", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(&restored, &mut chunk).unwrap();
    }
    assert_eq!(w.to_vec().unwrap(), bytes, "the chunk round-trips exactly");

    // Every architectural register, then the parts only a further run reveals.
    let regs: Vec<u32> = (0..26).map(|i| peek_debug(&saved, i * 4)).collect();
    let after: Vec<u32> = (0..26).map(|i| peek_debug(&restored, i * 4)).collect();
    assert_eq!(regs, after);
    let saved_regs = *saved.shared.regs.lock();
    let restored_regs = *restored.shared.regs.lock();
    assert_eq!(saved_regs, restored_regs);
    assert_eq!(saved.current_tick(), restored.current_tick());
    assert_eq!(saved.next_event(), restored.next_event());

    // The shadows, the prescaler phase, the toggle latch and the half-run
    // input filter all came across, so the two run identically from here —
    // including the capture the filter is about to make.
    saved.advance_by(500);
    restored.advance_by(500);
    let saved_regs = *saved.shared.regs.lock();
    let restored_regs = *restored.shared.regs.lock();
    assert_eq!(saved_regs, restored_regs);
    assert_eq!(saved.next_event(), restored.next_event());
}

#[test]
fn the_class_is_registrable_and_agrees_with_its_schema() {
    let mut reg = Registry::new();
    register(&mut reg).unwrap();
    assert!(register(&mut reg).is_err(), "twice is a collision");
    let device = reg.create(CLASS_NAME, &Props::new()).unwrap();
    assert_eq!(device.class().name, CLASS_NAME);

    let schema = schema();
    for pin in [
        IRQ_PIN,
        IRQ_UP_PIN,
        IRQ_CC_PIN,
        IRQ_TRG_PIN,
        TRGO_PIN,
        DMA_UP_PIN,
        DMA_TRG_PIN,
        "ch1",
        "ch4",
        "ch3n",
        "dma-ch1",
        "dma-ch4",
    ] {
        assert_eq!(schema.port_named(pin).expect(pin).dir, PortDir::Out);
    }
    for pin in ["ti1", "ti4", ETR_PIN, "itr0", "itr3"] {
        assert_eq!(schema.port_named(pin).expect(pin).dir, PortDir::In);
    }
    assert!(schema.port_named("ch5").is_none());
    assert!(schema.port_named("ch4n").is_none());
    assert!(schema.port_named("ti5").is_none());
    assert!(schema.port_named("itr4").is_none());

    // Every input the schema declares is one `sink` answers for, and nothing
    // else is — the two tables have to agree or a board file lies.
    let tim = general();
    for pin in ["ti1", "ti4", ETR_PIN, "itr0", "itr3"] {
        assert!(
            Device::sink(&tim, pin, &[WireId::new(1)]).is_some(),
            "{pin}"
        );
    }
    for pin in ["ti5", "itr4", "ch1", "nonsense"] {
        assert!(
            Device::sink(&tim, pin, &[WireId::new(1)]).is_none(),
            "{pin}"
        );
    }
    // A basic timer bonds neither, and a two-channel one has no `TI3`.
    assert!(Device::sink(&basic(), "etr", &[WireId::new(1)]).is_none());
    assert!(Device::sink(&basic(), "ti1", &[WireId::new(1)]).is_none());
    assert_eq!(input_pin("ti2"), Some(Input::Ti(1)));
    assert_eq!(input_pin("itr2"), Some(Input::Itr(2)));
    assert_eq!(input_pin("ti0"), None);

    assert!(Device::region(&tim, "").is_some());
    assert!(Device::region(&tim, "regs").is_some());
    assert!(Device::region(&tim, "fifo").is_none());
    assert!(Device::is_lazy(&tim));

    // The pin spellings the schema declares are the ones `connect` parses, and
    // there is exactly one spelling each.
    assert_eq!(channel_pin("ch1"), Some((0, false)));
    assert_eq!(channel_pin("ch3n"), Some((2, true)));
    assert_eq!(channel_pin("ch01"), None);
    assert_eq!(channel_pin("ch0"), None);
    assert_eq!(channel_pin("chn"), None);
}

// ---------------------------------------------------------------------------
// The machine seam
// ---------------------------------------------------------------------------

#[test]
fn a_board_binds_the_clock_and_the_scheduler_advances_the_counter() {
    use crate::core::clock::GlobalTime;
    use crate::core::value::Width;

    let mut options = crate::machine::BuildOptions::new();
    options.classes.insert(schema());
    for s in crate::machine::builtin::schemas() {
        options.classes.insert(s);
    }
    bind(&mut options.bindings).expect("nothing else claims st.tim");
    crate::machine::builtin::bind(&mut options.bindings).expect("ram");
    let mut registry = Registry::new();
    register(&mut registry).expect("nothing else claims st.tim");
    crate::machine::builtin::register(&mut registry).expect("ram");

    // The wiring a board writes, minus the `wire tim3.irq -> cpu.irq29` line
    // that needs a core to land on. `clock = hse * 21 / 2` is 84 MHz: an F4's
    // APB1 runs at 42 and RM0090 §7.2 doubles it for the timers.
    let text = concat!(
        "machine \"m\" {\n",
        "  osc hse = 8000000 Hz\n",
        "  space mem { width = 32 }\n",
        "  object sram \"ram\" { size = 4K }\n",
        "  object tim3 \"st.tim\" { clock = hse * 21 / 2, variant = \"general\" }\n",
        "  map mem 0x20000000 size 4K = sram\n",
        "  map mem 0x40000400 size 0x50 = tim3\n",
        "}\n"
    );
    let mut machine =
        crate::machine::build("t.machine", text, &registry, &options).expect("the board builds");

    let space = Arc::clone(machine.space("mem").expect("mem"));
    let base = 0x4000_0400u64;
    let poke32 = |offset: u64, value: u32| {
        space
            .write(
                base + offset,
                Width::U32,
                u64::from(value),
                MemAttrs::DEFAULT,
            )
            .expect("a word write reaches the timer");
    };
    let peek32 = |offset: u64| -> u32 {
        space
            .read(base + offset, Width::U32, MemAttrs::DEFAULT)
            .expect("a word read reaches the timer") as u32
    };

    // 84 MHz divided by 84 is a microsecond, and an update every 1000 of those
    // is the millisecond tick firmware actually asks a TIM for.
    poke32(OFF_PSC, 83);
    poke32(OFF_ARR, 999);
    poke32(OFF_EGR, EGR_UG);
    poke32(OFF_SR, 0);
    poke32(OFF_CR1, CR1_CEN);

    machine
        .run_for(GlobalTime::from_nanos(500_000))
        .expect("the machine runs");
    assert_eq!(peek32(OFF_SR) & SR_UIF, 0, "not due for another half");
    let cnt = peek32(OFF_CNT);
    assert!((495..=505).contains(&cnt), "half a millisecond in: {cnt}");

    machine
        .run_for(GlobalTime::from_nanos(500_100))
        .expect("the machine runs");
    assert_eq!(peek32(OFF_SR) & SR_UIF, SR_UIF, "the update landed");
}

// ---------------------------------------------------------------------------
// Input capture (RM0090 §17.3.5)
// ---------------------------------------------------------------------------

#[test]
fn input_capture_latches_cnt_on_the_selected_edge_and_sets_ccxof_on_the_second() {
    let tim = general();
    let ti1 = drive_input(&tim, "ti1", 1);
    poke(&tim, OFF_ARR, 0xffff);
    // `CC1S = 01`, no filter, no prescaler, rising edge.
    configure_capture(&tim, 0, 0, 0, 0);
    start(&tim);

    tim.advance_by(10);
    ti1.set(Level::High);
    assert_eq!(
        peek_debug(&tim, OFF_CCR1),
        10,
        "the counter at the instant of the edge"
    );
    assert_eq!(peek_debug(&tim, OFF_SR) & SR_CC1IF, SR_CC1IF);
    assert_eq!(
        peek_debug(&tim, OFF_SR) & SR_CC1OF,
        0,
        "one capture is not an over-capture"
    );

    // A falling edge is not the one `CC1NP:CC1P = 00` selects.
    ti1.set(Level::Low);
    tim.advance_by(5);
    assert_eq!(
        peek_debug(&tim, OFF_CCR1),
        10,
        "the wrong edge captures nothing"
    );

    // A second capture with `CC1IF` still set: the value is overwritten and
    // `CC1OF` records that one was lost.
    ti1.set(Level::High);
    assert_eq!(peek_debug(&tim, OFF_CCR1), 15);
    assert_eq!(peek_debug(&tim, OFF_SR) & SR_CC1OF, SR_CC1OF);

    // "CCxIF is cleared ... by reading the captured data stored in TIMx_CCRx",
    // and `CCxOF` is not: that one only clears on a write of zero.
    assert_eq!(peek(&tim, OFF_CCR1), 15);
    assert_eq!(peek_debug(&tim, OFF_SR) & SR_CC1IF, 0);
    assert_eq!(peek_debug(&tim, OFF_SR) & SR_CC1OF, SR_CC1OF);
    poke(&tim, OFF_SR, !SR_CC1OF);
    assert_eq!(peek_debug(&tim, OFF_SR) & SR_CC1OF, 0);
}

#[test]
fn a_capture_raises_the_irq_wire_and_a_debug_read_of_ccrx_does_not_clear_it() {
    let tim = general();
    let irq = wire(&tim, IRQ_PIN, 9);
    let ti1 = drive_input(&tim, "ti1", 1);
    poke(&tim, OFF_ARR, 0xffff);
    poke(&tim, OFF_DIER, DIER_CC1IE);
    configure_capture(&tim, 0, 0, 0, 0);
    start(&tim);

    tim.advance_by(7);
    ti1.set(Level::High);
    assert_eq!(irq.level(), Level::High);

    // Invariant 5: a debugger read may not pop a FIFO, and it may not clear a
    // capture flag either.
    assert_eq!(peek_debug(&tim, OFF_CCR1), 7);
    assert_eq!(irq.level(), Level::High, "a debug read changed nothing");
    assert_eq!(peek(&tim, OFF_CCR1), 7);
    assert_eq!(irq.level(), Level::Low, "a guest read did");
}

#[test]
fn the_input_filter_rejects_a_pulse_shorter_than_icxf_asks_for() {
    let tim = general();
    let ti1 = drive_input(&tim, "ti1", 1);
    poke(&tim, OFF_ARR, 0xffff);
    // `ICxF = 0011`: eight samples at `f_CK_INT`, so eight ticks of hold.
    configure_capture(&tim, 0, 0, 0b0011, 0);
    start(&tim);

    ti1.set(Level::High);
    tim.advance_by(4);
    ti1.set(Level::Low);
    tim.advance_by(20);
    assert_eq!(
        peek_debug(&tim, OFF_SR) & SR_CC1IF,
        0,
        "a four-tick glitch is not an edge"
    );

    ti1.set(Level::High);
    tim.advance_by(7);
    assert_eq!(
        peek_debug(&tim, OFF_SR) & SR_CC1IF,
        0,
        "seven ticks is not eight"
    );
    tim.advance_by(1);
    assert_eq!(peek_debug(&tim, OFF_SR) & SR_CC1IF, SR_CC1IF);
    assert_eq!(
        peek_debug(&tim, OFF_CCR1),
        32,
        "and the capture is stamped when the filter accepted, not when the pin moved"
    );
}

#[test]
fn a_filter_deadline_is_an_event_the_scheduler_is_told_about() {
    let tim = general();
    let ti1 = drive_input(&tim, "ti1", 1);
    poke(&tim, OFF_ARR, 0xffff);
    configure_capture(&tim, 0, 0, 0b0010, 0); // four samples at f_CK_INT
    // The counter is stopped, so the only thing that can happen is the filter.
    assert_eq!(tim.next_event(), None);
    ti1.set(Level::High);
    assert_eq!(tim.next_event(), Some(4), "four ticks from now");
}

#[test]
fn icxpsc_captures_every_fourth_edge_and_ccxe_resets_the_divider() {
    let tim = general();
    let ti1 = drive_input(&tim, "ti1", 1);
    poke(&tim, OFF_ARR, 0xffff);
    // `IC1PSC = 10`: one capture every four events. Both edges are events.
    configure_capture(&tim, 0, CCER_CCP | CCER_CCNP, 0, 0b10);
    start(&tim);

    for _ in 0..3 {
        tim.advance_by(1);
        ti1.set(Level::High);
        tim.advance_by(1);
        ti1.set(Level::Low);
    }
    // Six edges: one capture on the fourth, at tick 4.
    assert_eq!(peek_debug(&tim, OFF_CCR1), 4);

    // "the prescaler is reset as soon as CC1E = 0" — RM0090 §17.4.7. The two
    // edges already banked are forgotten, so the next capture is four away.
    poke(&tim, OFF_CCER, 0);
    poke(&tim, OFF_CCER, CCER_CCE | CCER_CCP | CCER_CCNP);
    for _ in 0..2 {
        tim.advance_by(1);
        ti1.set(Level::High);
        tim.advance_by(1);
        ti1.set(Level::Low);
    }
    assert_eq!(
        peek_debug(&tim, OFF_CCR1),
        10,
        "four fresh edges, at tick 10"
    );
}

#[test]
fn ccxs_10_captures_the_other_input_of_the_pair() {
    let tim = general();
    let ti2 = drive_input(&tim, "ti2", 2);
    poke(&tim, OFF_ARR, 0xffff);
    // `CC1S = 10`: channel 1 captures `TI2`.
    poke(&tim, OFF_CCMR1, CCS_TI_INDIRECT);
    poke(&tim, OFF_CCER, CCER_CCE);
    start(&tim);

    tim.advance_by(12);
    ti2.set(Level::High);
    assert_eq!(peek_debug(&tim, OFF_CCR1), 12);
}

// ---------------------------------------------------------------------------
// The slave-mode controller (RM0090 §17.3.15)
// ---------------------------------------------------------------------------

#[test]
fn trgo_from_one_timer_clocks_another_in_external_clock_mode_1() {
    let master = general();
    let slave = general();
    chain(&master, &slave, "itr0", 1);

    // The master pulses `TRGO` on every update: `MMS = 010`.
    poke(&master, OFF_CR2, MMS_UPDATE << CR2_MMS_SHIFT);
    poke(&master, OFF_ARR, 3);
    start(&master);
    // The slave counts those pulses: `SMS = 111`, `TS = 000` (`ITR0`).
    poke(&slave, OFF_ARR, 0xffff);
    poke(&slave, OFF_SMCR, SMS_EXT1);
    start(&slave);

    assert_eq!(
        slave.next_event(),
        None,
        "an externally clocked timer has no event of its own to schedule"
    );

    for _ in 0..16 {
        master.advance_by(1);
    }
    assert_eq!(peek(&master, OFF_CNT), 0, "four whole periods");
    assert_eq!(
        peek(&slave, OFF_CNT),
        4,
        "one slave count per master period"
    );
    assert_eq!(
        peek_debug(&slave, OFF_SR) & SR_TIF,
        SR_TIF,
        "and a trigger flag with it"
    );

    // The same span in one jump delivers the same four pulses: the count is
    // state, not an edge that a long advance can swallow.
    let master = general();
    let slave = general();
    chain(&master, &slave, "itr0", 2);
    poke(&master, OFF_CR2, MMS_UPDATE << CR2_MMS_SHIFT);
    poke(&master, OFF_ARR, 3);
    start(&master);
    poke(&slave, OFF_ARR, 0xffff);
    poke(&slave, OFF_SMCR, SMS_EXT1);
    start(&slave);
    master.advance_by(16);
    assert_eq!(peek(&slave, OFF_CNT), 4);

    // And the slave's own prescaler still divides what arrives.
    poke(&slave, OFF_PSC, 1);
    poke(&slave, OFF_EGR, EGR_UG);
    poke(&slave, OFF_SR, 0);
    master.advance_by(16);
    assert_eq!(
        peek(&slave, OFF_CNT),
        2,
        "PSC = 1 halves the external clock too"
    );
}

#[test]
fn mms_enable_exports_the_counter_enable_as_a_level() {
    let tim = general();
    let trgo = wire(&tim, TRGO_PIN, 3);
    poke(&tim, OFF_ARR, 9);
    poke(&tim, OFF_CR2, MMS_ENABLE << CR2_MMS_SHIFT);
    assert_eq!(trgo.level(), Level::Low);
    start(&tim);
    assert_eq!(trgo.level(), Level::High);
    poke(&tim, OFF_CR1, 0);
    assert_eq!(trgo.level(), Level::Low);
}

#[test]
fn gated_mode_runs_the_counter_only_while_trgi_is_high() {
    let tim = general();
    let itr0 = drive_input(&tim, "itr0", 1);
    poke(&tim, OFF_ARR, 0xffff);
    poke(&tim, OFF_SMCR, SMS_GATED);
    start(&tim);

    tim.advance_by(10);
    assert_eq!(peek(&tim, OFF_CNT), 0, "the gate is shut");
    assert_eq!(tim.next_event(), None, "and nothing is due while it is");

    itr0.set(Level::High);
    assert_eq!(
        peek_debug(&tim, OFF_SR) & SR_TIF,
        SR_TIF,
        "gated mode sets TIF when the counter starts"
    );
    tim.advance_by(10);
    assert_eq!(peek(&tim, OFF_CNT), 10);

    poke(&tim, OFF_SR, 0);
    itr0.set(Level::Low);
    assert_eq!(
        peek_debug(&tim, OFF_SR) & SR_TIF,
        SR_TIF,
        "and again when it stops"
    );
    tim.advance_by(10);
    assert_eq!(peek(&tim, OFF_CNT), 10, "stopped, and not reset");
}

#[test]
fn reset_mode_reinitializes_the_counter_on_a_trigger_edge() {
    let tim = general();
    let itr1 = drive_input(&tim, "itr1", 1);
    poke(&tim, OFF_ARR, 0xffff);
    // `TS = 001` selects `ITR1`.
    poke(&tim, OFF_SMCR, SMS_RESET | (1 << SMCR_TS_SHIFT));
    start(&tim);

    tim.advance_by(100);
    assert_eq!(peek(&tim, OFF_CNT), 100);
    itr1.set(Level::High);
    assert_eq!(peek(&tim, OFF_CNT), 0);
    assert_eq!(
        peek_debug(&tim, OFF_SR) & (SR_UIF | SR_TIF),
        SR_UIF | SR_TIF,
        "a reset is an update event as well as a trigger"
    );
    tim.advance_by(5);
    assert_eq!(peek(&tim, OFF_CNT), 5, "and it carries on from zero");
}

#[test]
fn trigger_mode_starts_a_stopped_counter_and_never_stops_it() {
    let tim = general();
    let itr0 = drive_input(&tim, "itr0", 1);
    poke(&tim, OFF_ARR, 0xffff);
    poke(&tim, OFF_SMCR, SMS_TRIGGER);
    // `CEN` deliberately stays clear: the trigger is what sets it.

    tim.advance_by(10);
    assert_eq!(peek(&tim, OFF_CNT), 0);
    itr0.set(Level::High);
    assert_eq!(
        peek(&tim, OFF_CR1) & CR1_CEN,
        CR1_CEN,
        "the trigger set CEN"
    );
    tim.advance_by(10);
    assert_eq!(peek(&tim, OFF_CNT), 10);
    itr0.set(Level::Low);
    tim.advance_by(10);
    assert_eq!(peek(&tim, OFF_CNT), 20, "and nothing stops it again");
}

#[test]
fn ti1f_ed_triggers_on_both_edges_of_the_first_input() {
    let tim = general();
    let ti1 = drive_input(&tim, "ti1", 1);
    poke(&tim, OFF_ARR, 0xffff);
    poke(&tim, OFF_CCMR1, CCS_TI_DIRECT);
    // `TS = 100` is the edge detector, and reset mode is what firmware uses it
    // for: a pulse of either sign restarts the measurement.
    poke(&tim, OFF_SMCR, SMS_RESET | (TS_TI1F_ED << SMCR_TS_SHIFT));
    start(&tim);

    tim.advance_by(30);
    ti1.set(Level::High);
    assert_eq!(peek(&tim, OFF_CNT), 0);
    tim.advance_by(20);
    ti1.set(Level::Low);
    assert_eq!(peek(&tim, OFF_CNT), 0, "the falling edge resets it too");
}

#[test]
fn ece_clocks_the_counter_from_etr_with_etps_dividing_and_etp_choosing_the_edge() {
    let tim = general();
    let etr = drive_input(&tim, "etr", 1);
    poke(&tim, OFF_ARR, 0xffff);
    poke(&tim, OFF_SMCR, SMCR_ECE);
    start(&tim);
    assert_eq!(
        tim.next_event(),
        None,
        "external clock mode 2 schedules nothing"
    );

    for _ in 0..4 {
        etr.set(Level::High);
        etr.set(Level::Low);
    }
    assert_eq!(peek(&tim, OFF_CNT), 4, "one count per rising edge");

    // `ETPS = 01`: every second edge.
    poke(&tim, OFF_SMCR, SMCR_ECE | (1 << SMCR_ETPS_SHIFT));
    for _ in 0..4 {
        etr.set(Level::High);
        etr.set(Level::Low);
    }
    assert_eq!(peek(&tim, OFF_CNT), 6);

    // `ETP`: the falling edge becomes the active one.
    poke(&tim, OFF_SMCR, SMCR_ECE | SMCR_ETP);
    etr.set(Level::High);
    assert_eq!(
        peek(&tim, OFF_CNT),
        6,
        "with ETP a rising edge is the idle one"
    );
    etr.set(Level::Low);
    assert_eq!(peek(&tim, OFF_CNT), 7);
}

#[test]
fn encoder_mode_3_counts_four_per_quadrature_cycle_in_the_phase_s_direction() {
    let tim = general();
    let ti1 = drive_input(&tim, "ti1", 1);
    let ti2 = drive_input(&tim, "ti2", 2);
    poke(&tim, OFF_ARR, 0xffff);
    // `CC1S = CC2S = 01`, and `SMS = 011`.
    poke(&tim, OFF_CCMR1, CCS_TI_DIRECT | (CCS_TI_DIRECT << 8));
    poke(&tim, OFF_SMCR, SMS_ENCODER3);
    start(&tim);

    // Forward: A leads B. Four counts up per cycle, one per edge.
    for _ in 0..3 {
        ti1.set(Level::High);
        ti2.set(Level::High);
        ti1.set(Level::Low);
        ti2.set(Level::Low);
    }
    assert_eq!(peek(&tim, OFF_CNT), 12);
    assert_eq!(peek(&tim, OFF_CR1) & CR1_DIR, 0, "DIR reads back as up");

    // Reverse: B leads A, and every edge counts the other way.
    for _ in 0..3 {
        ti2.set(Level::High);
        ti1.set(Level::High);
        ti2.set(Level::Low);
        ti1.set(Level::Low);
    }
    assert_eq!(peek(&tim, OFF_CNT), 0);
    assert_eq!(peek(&tim, OFF_CR1) & CR1_DIR, CR1_DIR, "and as down");

    // "DIR is read only when the timer is configured in ... Encoder mode."
    poke(&tim, OFF_CR1, peek(&tim, OFF_CR1) & !CR1_DIR);
    assert_eq!(
        peek(&tim, OFF_CR1) & CR1_DIR,
        CR1_DIR,
        "the write is ignored"
    );
}

#[test]
fn encoder_mode_1_counts_on_ti2_alone_and_mode_2_on_ti1_alone() {
    // Mode 1 counts `TI2FP2`'s edges and mode 2 counts `TI1FP1`'s, so each of
    // them is half of mode 3: two counts per quadrature cycle rather than four.
    for (sms, ignored) in [(SMS_ENCODER1, "ti1"), (SMS_ENCODER2, "ti2")] {
        let tim = general();
        let ti1 = drive_input(&tim, "ti1", 1);
        let ti2 = drive_input(&tim, "ti2", 2);
        poke(&tim, OFF_ARR, 0xffff);
        poke(&tim, OFF_CCMR1, CCS_TI_DIRECT | (CCS_TI_DIRECT << 8));
        poke(&tim, OFF_SMCR, sms);
        start(&tim);

        // The input this mode does not count on, moving on its own.
        let quiet = if ignored == "ti1" { &ti1 } else { &ti2 };
        for _ in 0..3 {
            quiet.set(Level::High);
            quiet.set(Level::Low);
        }
        assert_eq!(
            peek(&tim, OFF_CNT),
            0,
            "{ignored}'s edges do not clock this mode"
        );

        // Three forward quadrature cycles, `A` leading `B`.
        for _ in 0..3 {
            ti1.set(Level::High);
            ti2.set(Level::High);
            ti1.set(Level::Low);
            ti2.set(Level::Low);
        }
        assert_eq!(
            peek(&tim, OFF_CNT),
            6,
            "two counts a cycle, not mode 3's four"
        );
    }
}

#[test]
fn a_chain_of_timers_wired_into_a_ring_terminates() {
    // Two timers each clocking the other is a board file nobody should write,
    // and the part would simply oscillate. What must not happen is an
    // unbounded stack — `MAX_TRGO_DEPTH`.
    let a = general();
    let b = general();
    chain(&a, &b, "itr0", 1);
    chain(&b, &a, "itr0", 2);
    for t in [&a, &b] {
        poke(t, OFF_CR2, MMS_UPDATE << CR2_MMS_SHIFT);
        poke(t, OFF_ARR, 1);
        poke(t, OFF_SMCR, SMS_EXT1);
        start(t);
    }
    // `a` is externally clocked, so nothing moves until something pokes it.
    poke(&a, OFF_EGR, EGR_UG);
    // Reaching here at all is the assertion.
    assert!(peek(&a, OFF_CNT) <= 1);
}

// ---------------------------------------------------------------------------
// `DCR`/`DMAR`, and the DMA request lines
// ---------------------------------------------------------------------------

#[test]
fn dmar_is_a_window_on_the_register_dba_names_and_the_index_wraps_at_dbl() {
    let tim = general();
    poke(&tim, OFF_ARR, 0x1234);
    poke(&tim, OFF_CCR1, 0x0011);
    poke(&tim, OFF_CCR1 + 4, 0x0022);
    // `DBA` = 11 (`ARR`, at 0x2c), `DBL` = 2: a three-register burst.
    poke(&tim, OFF_DCR, 11 | (2 << 8));
    assert_eq!(peek(&tim, OFF_DMAR), 0x1234, "ARR");
    assert_eq!(
        peek(&tim, OFF_DMAR),
        0,
        "RCR, which a general timer does not have"
    );
    assert_eq!(peek(&tim, OFF_DMAR), 0x0011, "CCR1");
    assert_eq!(peek(&tim, OFF_DMAR), 0x1234, "and back to the start");

    // A debug read looks through the window without moving the index along.
    assert_eq!(peek_debug(&tim, OFF_DMAR), 0, "RCR, and it stays there");
    assert_eq!(peek_debug(&tim, OFF_DMAR), 0);
    assert_eq!(peek(&tim, OFF_DMAR), 0);

    // And a write lands on the register the window is over.
    poke(&tim, OFF_DCR, 13); // `CCR1`, one transfer
    poke(&tim, OFF_DMAR, 0x00ff);
    assert_eq!(peek_debug(&tim, OFF_CCR1), 0x00ff);
}

#[test]
fn the_dma_request_lines_pulse_once_per_event_and_only_when_dier_says_so() {
    let tim = general();
    let up = watch(&tim, DMA_UP_PIN, 5);
    let ch1 = watch(&tim, "dma-ch1", 6);
    let trg = watch(&tim, DMA_TRG_PIN, 7);

    poke(&tim, OFF_ARR, 3);
    configure_channel(&tim, 0, OCM_FROZEN, 2, false);
    start(&tim);

    // Eight ticks of `ARR = 3` is two updates and two compare matches, and
    // with `DIER` clear not one of them asks for anything.
    for _ in 0..8 {
        tim.advance_by(1);
    }
    assert_eq!(up.get(), 0, "UDE is clear");
    assert_eq!(ch1.get(), 0, "and so is CC1DE");

    poke(&tim, OFF_DIER, DIER_UDE | DIER_CC1DE | DIER_TDE);
    for _ in 0..8 {
        tim.advance_by(1);
    }
    assert_eq!(up.get(), 2, "one pulse per update event");
    assert_eq!(ch1.get(), 2, "one per compare match");
    assert_eq!(trg.get(), 0, "and nothing triggered anything");

    // `EGR.TG` is a trigger event by software, and it asks too.
    poke(&tim, OFF_EGR, EGR_TG);
    assert_eq!(trg.get(), 1);
}

/// A board with a timer, a DMA controller, and the one wire between them.
///
/// `TIM3`'s update request drives `DMA1` stream 0, which walks a table in RAM
/// into `TIM3_DMAR` — and `DCR` puts that window over `CCR1`. This is the
/// "DMA burst mode" of RM0090 §17.4.17-§17.4.18, and it is the whole reason `DCR`
/// and
/// `DMAR` exist.
#[cfg(feature = "dev-stm32-dma")]
const TIM_DMA_BOARD: &str = r#"
machine "tim-dma" {
  osc clk = 8000000 Hz
  space mem { width = 32 }
  object ram  "ram" { size = 4K }
  object tim3 "st.tim" { clock = clk, variant = "general", channels = 4 }
  object dma1 "st.dma" { clock = clk, space = mem, variant = "stream" }
  map mem 0x20000000 size 4K   = ram
  map mem 0x40000400 size 0x64 = tim3
  map mem 0x40026000 size 0xd0 = dma1
  wire tim3.dma-up -> dma1.req0
}
"#;

#[cfg(feature = "dev-stm32-dma")]
#[test]
fn a_timer_update_drives_a_dma_burst_end_to_end() {
    use crate::core::clock::GlobalTime;
    use crate::core::value::Width;
    use crate::dev::stm32::dma;

    let mut options = crate::machine::BuildOptions::new();
    options.classes.insert(schema());
    options.classes.insert(dma::schema());
    for s in crate::machine::builtin::schemas() {
        options.classes.insert(s);
    }
    bind(&mut options.bindings).expect("st.tim");
    dma::bind(&mut options.bindings).expect("st.dma");
    crate::machine::builtin::bind(&mut options.bindings).expect("ram");
    let mut registry = Registry::new();
    register(&mut registry).expect("st.tim");
    dma::register(&mut registry).expect("st.dma");
    crate::machine::builtin::register(&mut registry).expect("ram");

    let mut machine =
        match crate::machine::build("tim-dma.machine", TIM_DMA_BOARD, &registry, &options) {
            Ok(m) => m,
            Err(e) => panic!("{e}"),
        };

    let space = Arc::clone(machine.space("mem").expect("mem"));
    let ram = 0x2000_0000u64;
    let tim = 0x4000_0400u64;
    let dma_regs = 0x4002_6000u64;
    let poke32 = |addr: u64, value: u32| {
        space
            .write(addr, Width::U32, u64::from(value), MemAttrs::DEFAULT)
            .expect("mapped");
    };
    let peek32 = |addr: u64| -> u32 {
        space
            .read(addr, Width::U32, MemAttrs::DEFAULT)
            .expect("mapped") as u32
    };

    // The duty-cycle table firmware wants pushed into `CCR1`, one per period.
    for (i, value) in [11u32, 22, 33, 44].into_iter().enumerate() {
        poke32(ram + 4 * i as u64, value);
    }

    // `DCR`: `DBA` = 13 words from `CR1`, which is `CCR1` at 0x34; `DBL` = 0,
    // so each request moves exactly one register.
    poke32(tim + OFF_DCR, 13);
    poke32(tim + OFF_ARR, 3);
    poke32(tim + OFF_DIER, DIER_UDE);

    // Stream 0: memory to peripheral, 32-bit both sides, `MINC`, four items,
    // pointed at `TIM3_DMAR`.
    poke32(dma_regs + 0x10 + 8, (tim + OFF_DMAR) as u32); // S0PAR
    poke32(dma_regs + 0x10 + 0x0c, ram as u32); // S0M0AR
    poke32(dma_regs + 0x10 + 4, 4); // S0NDTR
    poke32(
        dma_regs + 0x10,
        (1 << 6) | (2 << 11) | (2 << 13) | (1 << 10) | 1,
    );

    assert_eq!(peek32(tim + 0x34), 0, "nothing has moved yet");
    poke32(tim + OFF_CR1, CR1_CEN);

    // Four periods of four ticks at 8 MHz is 2 µs; give it a little more.
    machine
        .run_for(GlobalTime::from_nanos(3_000))
        .expect("the machine runs");

    assert_eq!(
        peek32(dma_regs + 0x10 + 4),
        0,
        "every item of the table went out"
    );
    assert_eq!(
        peek32(tim + 0x34),
        44,
        "and `CCR1` holds the last of them, written through `DMAR`"
    );
}

#[test]
fn uifremap_copies_uif_into_cnt_bit_31() {
    let tim = general32();
    poke(&tim, OFF_ARR, 3);
    poke(&tim, OFF_CR1, CR1_UIFREMAP);
    start(&tim);
    tim.advance_by(4);
    assert_eq!(peek_debug(&tim, OFF_SR) & SR_UIF, SR_UIF);
    assert_eq!(peek(&tim, OFF_CNT), 0x8000_0000, "CNT = 0 with UIF on top");
    poke(&tim, OFF_SR, 0);
    assert_eq!(peek(&tim, OFF_CNT), 0);
    poke(&tim, OFF_CR1, peek(&tim, OFF_CR1) & !CR1_UIFREMAP);
    tim.advance_by(4);
    assert_eq!(
        peek(&tim, OFF_CNT) & 0x8000_0000,
        0,
        "and not when it is off"
    );
}

#[test]
fn or1_and_or2_read_back_what_was_written() {
    let tim = general();
    poke(&tim, OFF_OR1, 0xdead_beef);
    poke(&tim, OFF_OR2, 0x0bad_f00d);
    assert_eq!(peek_debug(&tim, OFF_OR1), 0xdead_beef);
    assert_eq!(peek_debug(&tim, OFF_OR2), 0x0bad_f00d);
    Device::reset(&tim, ResetKind::Cold);
    assert_eq!(peek_debug(&tim, OFF_OR1), 0);
}
