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
fn center_aligned_mode_counts_up_then_down_and_sets_uif_at_both_ends() {
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
    // RM0090 §19.4.3: "The counter is blocked while the auto-reload value is
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
    for offset in [OFF_CCMR1, OFF_CCMR2, OFF_CCER, OFF_CCR1, OFF_RCR, OFF_BDTR] {
        poke(&tim, offset, 0xffff_ffff);
        assert_eq!(peek(&tim, offset), 0, "offset {offset:#x}");
    }
    // `CR1` keeps only the five bits RM0090 §19.4.1 gives it — no `DIR`, no
    // `CMS`, no `CKD`.
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
    let regs: Vec<u32> = (0..20).map(|i| peek_debug(&saved, i * 4)).collect();
    let after: Vec<u32> = (0..20).map(|i| peek_debug(&restored, i * 4)).collect();
    assert_eq!(regs, after);
    let saved_regs = *saved.shared.regs.lock();
    let restored_regs = *restored.shared.regs.lock();
    assert_eq!(saved_regs, restored_regs);
    assert_eq!(saved.current_tick(), restored.current_tick());
    assert_eq!(saved.next_event(), restored.next_event());

    // The shadows, the prescaler phase and the toggle latch all came across, so
    // the two run identically from here.
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
    for pin in [IRQ_PIN, IRQ_UP_PIN, IRQ_CC_PIN, "ch1", "ch4", "ch3n"] {
        assert_eq!(schema.port_named(pin).expect(pin).dir, PortDir::Out);
    }
    assert!(schema.port_named("ch5").is_none());
    assert!(schema.port_named("ch4n").is_none());

    let tim = general();
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
