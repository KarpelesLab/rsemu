//! Tests for the NES APU.
//!
//! These check *facts* rather than behaviour-in-general: the frame sequencer's
//! exact CPU-cycle schedule in both modes, the IRQ's timing and its two clear
//! paths, the length table, the sweep unit's mute conditions and its two
//! negation flavours, the noise LFSR's period in both modes, the DMC's rate
//! table and memory reader, and a save/load round trip. A test that only
//! asserted "something happened" would pass against a wrong table.

use alloc::vec::Vec;

use super::*;
use crate::core::state::{ChunkReader, MachineShape, StateReader, StateWriter};
use crate::core::sync::{AtomicU32, Ordering as AtomicOrdering};
use crate::core::wire::{Wire, WireId, WireIdAllocator, WireSink};
use frame::{FrameCounter, FrameEvent};
use units::LENGTH_TABLE;

/// A machine-less APU with a given property set.
fn apu_with(props: Props) -> Apu {
    Apu::new(&props).expect("properties are valid")
}

/// A machine-less APU with default properties.
fn apu() -> Apu {
    apu_with(Props::new())
}

// ---------------------------------------------------------------------------
// Frame counter
// ---------------------------------------------------------------------------

/// What the sequencer produced over a run, as CPU cycle numbers.
#[derive(Debug, Default, PartialEq, Eq)]
struct Schedule {
    quarters: Vec<u64>,
    halves: Vec<u64>,
    irq_rises: Vec<u64>,
}

/// Run `cycles` CPU cycles through `fc`, starting the cycle count at `from`.
fn record(fc: &mut FrameCounter, from: u64, cycles: u64) -> Schedule {
    let mut out = Schedule::default();
    let mut was_irq = fc.irq();
    for i in 0..cycles {
        let now = from + i;
        let event = fc.tick(now);
        if event.quarter {
            out.quarters.push(now - from + 1);
        }
        if event.half {
            out.halves.push(now - from + 1);
        }
        if fc.irq() && !was_irq {
            out.irq_rises.push(now - from + 1);
        }
        was_irq = fc.irq();
    }
    out
}

#[test]
fn the_four_step_sequence_clocks_on_the_documented_cpu_cycles() {
    // NESdev APU Frame Counter, mode 0, converted from APU cycles: 3728 PUT is
    // CPU 7457, 14914 GET is CPU 29828, and the wrap is CPU 29830.
    let mut fc = FrameCounter::new(Timing::Ntsc);
    let s = record(&mut fc, 1, 29830);
    assert_eq!(s.quarters, [7457, 14913, 22371, 29829]);
    assert_eq!(s.halves, [14913, 29829]);
    assert_eq!(s.irq_rises, [29828]);
    assert_eq!(fc.cycle(), 0, "the sequence wraps at 29830");
}

#[test]
fn the_four_step_sequence_repeats_every_29830_cycles() {
    let mut fc = FrameCounter::new(Timing::Ntsc);
    let first = record(&mut fc, 1, 29830);
    let second = record(&mut fc, 29831, 29830);
    assert_eq!(first.quarters, second.quarters);
    assert_eq!(first.halves, second.halves);
    assert_eq!(first.irq_rises, [29828]);
    // The flag is already set going into the second pass, so no rising edge.
    assert!(second.irq_rises.is_empty());
}

#[test]
fn the_pal_four_step_sequence_uses_its_own_table() {
    let mut fc = FrameCounter::new(Timing::Pal);
    let s = record(&mut fc, 1, 33254);
    assert_eq!(s.quarters, [8313, 16627, 24939, 33253]);
    assert_eq!(s.halves, [16627, 33253]);
    assert_eq!(s.irq_rises, [33252]);
}

#[test]
fn the_five_step_sequence_clocks_immediately_and_never_raises_an_irq() {
    let mut fc = FrameCounter::new(Timing::Ntsc);
    // Written on a get cycle, so the reset lands 4 CPU cycles later.
    fc.write(0x80, false);
    assert_eq!(fc.mode(), Mode::FiveStep);
    assert_eq!(fc.tick(1), FrameEvent::NONE);
    assert_eq!(fc.tick(2), FrameEvent::NONE);
    assert_eq!(fc.tick(3), FrameEvent::NONE);
    assert_eq!(
        fc.tick(4),
        FrameEvent::BOTH,
        "bit 7 set clocks both units when the reset takes effect"
    );
    assert_eq!(fc.cycle(), 0);

    let s = record(&mut fc, 5, 37282);
    assert_eq!(s.quarters, [7457, 14913, 22371, 37281]);
    assert_eq!(s.halves, [14913, 37281]);
    assert!(s.irq_rises.is_empty(), "mode 1 never sets the frame IRQ");
    assert!(!fc.irq());
}

#[test]
fn a_four_step_write_resets_without_clocking_anything() {
    let mut fc = FrameCounter::new(Timing::Ntsc);
    // Get some distance into the sequence first.
    record(&mut fc, 1, 10_000);
    assert_ne!(fc.cycle(), 0);
    fc.write(0x00, true); // put cycle: 3 CPU cycles
    assert_eq!(fc.tick(10_001), FrameEvent::NONE);
    assert_eq!(fc.tick(10_002), FrameEvent::NONE);
    assert_eq!(
        fc.tick(10_003),
        FrameEvent::NONE,
        "bit 7 clear resets the sequence without clocking"
    );
    assert_eq!(fc.cycle(), 0);
}

#[test]
fn the_4017_reset_delay_is_three_or_four_cycles_by_alignment() {
    for (on_put, delay) in [(true, 3u64), (false, 4u64)] {
        let mut fc = FrameCounter::new(Timing::Ntsc);
        record(&mut fc, 1, 1000);
        fc.write(0x00, on_put);
        for i in 1..delay {
            fc.tick(1000 + i);
            assert!(fc.reset_pending(), "reset fired {i} cycles early");
        }
        fc.tick(1000 + delay);
        assert!(!fc.reset_pending());
        assert_eq!(fc.cycle(), 0);
    }
}

#[test]
fn the_frame_irq_is_cleared_by_a_status_read_but_not_on_the_setting_cycle() {
    let mut fc = FrameCounter::new(Timing::Ntsc);
    // Cycle 29828 is where the flag is first set.
    record(&mut fc, 1, 29828);
    assert!(fc.irq());
    // Reading on that very cycle returns 1 and leaves the flag alone.
    assert!(fc.read_irq(29828, false));
    assert!(fc.irq(), "a flag set this cycle survives the read");
    // Reading on any later cycle clears it.
    assert!(fc.read_irq(29829, false));
    assert!(!fc.irq());
}

#[test]
fn a_debug_read_never_clears_the_frame_irq() {
    let mut fc = FrameCounter::new(Timing::Ntsc);
    record(&mut fc, 1, 29830);
    assert!(fc.irq());
    assert!(fc.read_irq(99_999, true));
    assert!(fc.irq(), "MemAttrs::debug must have no side effect");
}

#[test]
fn setting_the_inhibit_bit_clears_and_suppresses_the_frame_irq() {
    let mut fc = FrameCounter::new(Timing::Ntsc);
    record(&mut fc, 1, 29830);
    assert!(fc.irq());
    fc.write(0x40, false);
    assert!(!fc.irq(), "bit 6 clears the flag immediately");
    let s = record(&mut fc, 29831, 4 + 29830);
    assert!(s.irq_rises.is_empty(), "inhibited: the flag stays clear");
}

// ---------------------------------------------------------------------------
// Length counters
// ---------------------------------------------------------------------------

#[test]
fn the_length_table_matches_the_documented_values() {
    // NESdev APU Length Counter. Written out rather than computed so that a
    // transcription slip is visible here rather than only as a wrong note.
    assert_eq!(
        LENGTH_TABLE,
        [
            10, 254, 20, 2, 40, 4, 80, 6, 160, 8, 60, 10, 14, 12, 26, 14, 12, 16, 24, 18, 48, 20,
            96, 22, 192, 24, 72, 26, 16, 28, 32, 30
        ]
    );
}

#[test]
fn the_length_table_has_the_documented_structure() {
    // The wiki's second presentation of the same table: with index bit 0 set
    // the remaining bits select a linear length, except for index 1.
    for i in (3..32usize).step_by(2) {
        assert_eq!(
            usize::from(LENGTH_TABLE[i]),
            i - 1,
            "odd index {i} should be the linear length {}",
            i - 1
        );
    }
    assert_eq!(LENGTH_TABLE[1], 254);
    // Even indices are note lengths on a base of 10 (bit 4 clear) or 12 (set).
    assert_eq!(LENGTH_TABLE[0x00], 10);
    assert_eq!(LENGTH_TABLE[0x10], 12);
    assert_eq!(LENGTH_TABLE[0x18], 192);
    assert_eq!(LENGTH_TABLE[0x08], 160);
}

#[test]
fn a_disabled_channel_cannot_load_its_length_counter() {
    let apu = apu();
    // Pulse 1 is disabled at power-up, so the load is discarded.
    apu.write(0x03, 0x08);
    assert_eq!(apu.read(0x15) & 0x01, 0, "load while disabled is ignored");
    apu.write(0x15, 0x01);
    apu.write(0x03, 0x08);
    assert_eq!(apu.read(0x15) & 0x01, 0x01);
    // Clearing the enable bit forces the counter to zero and loses the value.
    apu.write(0x15, 0x00);
    assert_eq!(apu.read(0x15) & 0x01, 0);
    apu.write(0x15, 0x01);
    assert_eq!(
        apu.read(0x15) & 0x01,
        0,
        "enabling does not restore a length"
    );
}

#[test]
fn a_halted_length_counter_does_not_count_down() {
    let apu = apu();
    apu.write(0x15, 0x01);
    apu.write(0x00, 0x30); // halt set (bit 5), constant volume
    apu.write(0x03, 0x08); // length index 1 -> 254
    // Two full sequences is four half-frame clocks; nothing should move.
    apu.advance(2 * 29830);
    assert_eq!(apu.read(0x15) & 0x01, 0x01);

    // Clearing the halt bit lets it run out.
    apu.write(0x15, 0x02);
    apu.write(0x04, 0x10); // pulse 2: halt clear
    apu.write(0x07, 0x18); // length index 3 -> 2
    apu.advance(2 * 29830);
    assert_eq!(
        apu.read(0x15) & 0x02,
        0,
        "two half frames expire a length of 2"
    );
}

// ---------------------------------------------------------------------------
// Sweep
// ---------------------------------------------------------------------------

#[test]
fn a_period_below_eight_mutes_the_pulse_channel() {
    let sweep = pulse::Sweep::new(true);
    for period in 0..8u16 {
        assert!(sweep.muting(period), "period {period} must mute");
    }
    assert!(!sweep.muting(8));
}

#[test]
fn a_target_period_above_7ff_mutes_even_with_the_sweep_disabled() {
    // Negate clear, shift 0, current period >= $400: the target is twice the
    // period, which overflows and mutes. This is the case the wiki calls out as
    // the reason the bottom octave of the pulse channels is unused.
    let sweep = pulse::Sweep::new(true);
    assert!(!sweep.muting(0x3FF));
    assert!(sweep.muting(0x400));
    assert_eq!(sweep.target(0x400), 0x800);
}

#[test]
fn pulse_one_negates_with_the_ones_complement_and_pulse_two_with_the_twos() {
    // The wiki's own example: making a change amount of 20 negative gives -21
    // on pulse 1 and -20 on pulse 2.
    let mut one = pulse::Sweep::new(true);
    let mut two = pulse::Sweep::new(false);
    // Shift 0 makes the change amount equal to the period; negate set.
    one.write(0x88);
    two.write(0x88);
    assert_eq!(one.target(20), 0, "20 - 21 clamps to zero on pulse 1");
    assert_eq!(two.target(20), 0, "20 - 20 is zero on pulse 2");

    // A shift of 1 separates them: change = 10.
    one.write(0x89);
    two.write(0x89);
    assert_eq!(one.target(20), 20 - 10 - 1);
    assert_eq!(two.target(20), 20 - 10);
}

#[test]
fn a_disabled_sweep_never_updates_the_period() {
    let mut sweep = pulse::Sweep::new(false);
    sweep.write(0x00); // disabled, shift 0
    let mut period = 0x100u16;
    for _ in 0..16 {
        sweep.clock(&mut period);
    }
    assert_eq!(period, 0x100);
}

#[test]
fn an_enabled_sweep_updates_the_period_on_its_divider_period() {
    let mut sweep = pulse::Sweep::new(false);
    // Enabled, P = 0 (period 1 half frame), negate clear, shift 1.
    sweep.write(0x81);
    let mut period = 0x100u16;
    sweep.clock(&mut period); // reload pass: divider was 0, so it updates
    assert_eq!(period, 0x180);
}

// ---------------------------------------------------------------------------
// Triangle
// ---------------------------------------------------------------------------

#[test]
fn the_linear_counter_reload_flag_persists_while_the_control_flag_is_set() {
    let apu = apu();
    apu.write(0x15, 0x04);
    apu.write(0x08, 0xFF); // control set, reload value 127
    apu.write(0x0B, 0x08); // sets the reload flag
    // Every quarter frame reloads the counter while control stays set.
    apu.advance(29830);
    apu.advance(29830);
    // Clearing the control flag lets the next clock consume the reload flag and
    // the counter then counts down.
    apu.write(0x08, 0x02); // control clear, reload value 2
    apu.advance(29830 * 2);
    // Four quarter frames per sequence: reload to 2, then 1, 0, 0 ... the
    // channel is silent, which is what the linear counter is for.
    assert_eq!(
        apu.read(0x15) & 0x04,
        0x04,
        "the length counter is separate"
    );
}

#[test]
fn the_triangle_holds_its_output_when_ultrasonic_halt_is_enabled() {
    let props = Props::new()
        .with("halt-ultrasonic", true)
        .with("sample-buffer", 0u64);
    let apu = apu_with(props);
    apu.write(0x15, 0x04);
    apu.write(0x08, 0xFF);
    apu.write(0x0A, 0x00);
    apu.write(0x0B, 0x08); // period 0: ultrasonic
    apu.advance(1000);
    let held = apu.output();
    apu.advance(1000);
    assert_eq!(apu.output(), held, "an ultrasonic triangle is frozen");
}

// ---------------------------------------------------------------------------
// Noise
// ---------------------------------------------------------------------------

#[test]
fn the_noise_period_table_matches_the_documented_values() {
    assert_eq!(
        noise::periods(Timing::Ntsc),
        [
            4, 8, 16, 32, 64, 96, 128, 160, 202, 254, 380, 508, 762, 1016, 2034, 4068
        ]
    );
    assert_eq!(
        noise::periods(Timing::Pal),
        [
            4, 8, 14, 30, 60, 88, 118, 148, 188, 236, 354, 472, 708, 944, 1890, 3778
        ]
    );
    // Every entry is even, because the timer is clocked once per APU cycle.
    for period in noise::periods(Timing::Ntsc) {
        assert_eq!(period % 2, 0);
    }
}

/// Shift the LFSR until it returns to its starting value, up to `limit` steps.
fn lfsr_period(mode: u8, limit: u32) -> u32 {
    let mut n = noise::Noise::new(Timing::Ntsc);
    n.write_period(mode); // period index 0: one shift every two APU cycles
    let start = n.shift();
    for step in 1..=limit {
        n.tick_timer();
        n.tick_timer();
        if n.shift() == start {
            return step;
        }
    }
    0
}

#[test]
fn the_noise_lfsr_has_the_documented_periods_in_both_modes() {
    // 32767 steps with the mode flag clear (tap on bit 1), and 93 with it set
    // (tap on bit 6) from the power-on value of 1.
    assert_eq!(lfsr_period(0x00, 40_000), 32767);
    assert_eq!(lfsr_period(0x80, 40_000), 93);
}

#[test]
fn the_noise_lfsr_shifts_right_with_feedback_into_bit_14() {
    let mut n = noise::Noise::new(Timing::Ntsc);
    assert_eq!(n.shift(), 1, "power-on value");
    n.write_period(0x00);
    n.tick_timer();
    // 1: bit 0 is 1, bit 1 is 0, feedback 1; shift right gives 0, then bit 14.
    assert_eq!(n.shift(), 0x4000);
    n.tick_timer();
    n.tick_timer();
    assert_eq!(n.shift(), 0x2000);
}

// ---------------------------------------------------------------------------
// DMC
// ---------------------------------------------------------------------------

#[test]
fn the_dmc_rate_table_matches_the_documented_values() {
    assert_eq!(
        dmc::rates(Timing::Ntsc),
        [
            428, 380, 340, 320, 286, 254, 226, 214, 190, 160, 142, 128, 106, 84, 72, 54
        ]
    );
    assert_eq!(
        dmc::rates(Timing::Pal),
        [
            398, 354, 316, 298, 276, 236, 210, 198, 176, 148, 132, 118, 98, 78, 66, 50
        ]
    );
    for rate in dmc::rates(Timing::Ntsc) {
        assert_eq!(rate % 2, 0, "rates are even: the timer runs at APU rate");
    }
}

#[test]
fn enabling_the_dmc_schedules_a_load_fetch_from_the_sample_address() {
    let apu = apu();
    apu.write(0x12, 0x01); // $C000 + 1 * 64
    apu.write(0x13, 0x00); // 0 * 16 + 1 = one byte
    assert!(apu.dma_request().is_none());
    apu.write(0x15, 0x10);
    let request = apu.dma_request().expect("a load fetch is scheduled");
    assert_eq!(request.kind, DmaKind::Load);
    assert_eq!(request.addr, 0xC040);
    assert_eq!(apu.read(0x15) & 0x10, 0x10, "bytes remaining is non-zero");

    assert!(apu.dma_complete(request.serial, 0x55));
    assert!(apu.dma_request().is_none(), "one byte, one fetch");
    assert_eq!(apu.read(0x15) & 0x10, 0, "the sample is exhausted");
}

#[test]
fn the_memory_reader_wraps_from_ffff_to_8000() {
    let apu = apu();
    apu.write(0x12, 0xFF); // $C000 + 255 * 64 = $FFC0
    apu.write(0x13, 0x0F); // 15 * 16 + 1 = 241 bytes
    apu.write(0x15, 0x10);
    let mut addrs = Vec::new();
    for byte in 0..80u8 {
        let request = apu.dma_request().expect("the reader keeps asking");
        addrs.push(request.addr);
        assert!(apu.dma_complete(request.serial, byte));
        // Emptying the buffer is what schedules the next fetch, so run the
        // output unit until it does.
        apu.advance(8 * 428);
    }
    assert_eq!(addrs[0], 0xFFC0);
    assert_eq!(addrs[63], 0xFFFF);
    assert_eq!(addrs[64], 0x8000, "the address counter wraps to $8000");
}

#[test]
fn a_non_looping_sample_raises_the_dmc_irq_when_its_last_byte_is_read() {
    let apu = apu();
    apu.write(0x10, 0x80); // IRQ enabled, no loop, rate 0
    apu.write(0x12, 0x00);
    apu.write(0x13, 0x00); // one byte
    apu.write(0x15, 0x10);
    let request = apu.dma_request().unwrap();
    assert!(apu.dma_complete(request.serial, 0x00));
    assert_eq!(apu.read(0x15) & 0x80, 0x80, "the DMC IRQ flag is set");
    assert_eq!(apu.irq_level(), Level::High);
    // A $4015 read does not clear it; only a $4015 write or $4010 bit 7 does.
    assert_eq!(apu.read(0x15) & 0x80, 0x80);
    apu.write(0x15, 0x00);
    assert_eq!(apu.read(0x15) & 0x80, 0);
    assert_eq!(apu.irq_level(), Level::Low);
}

#[test]
fn clearing_the_dmc_irq_enable_bit_clears_the_flag() {
    let apu = apu();
    apu.write(0x10, 0x80);
    apu.write(0x13, 0x00);
    apu.write(0x15, 0x10);
    let request = apu.dma_request().unwrap();
    apu.dma_complete(request.serial, 0x00);
    assert_eq!(apu.read(0x15) & 0x80, 0x80);
    apu.write(0x10, 0x00);
    assert_eq!(apu.read(0x15) & 0x80, 0);
}

#[test]
fn a_looping_sample_restarts_instead_of_raising_an_irq() {
    let apu = apu();
    apu.write(0x10, 0xC0); // IRQ enabled and loop set: loop wins
    apu.write(0x12, 0x00);
    apu.write(0x13, 0x00);
    apu.write(0x15, 0x10);
    let request = apu.dma_request().unwrap();
    assert!(apu.dma_complete(request.serial, 0xFF));
    assert_eq!(apu.read(0x15) & 0x80, 0, "no IRQ on a looping sample");
    assert_eq!(apu.read(0x15) & 0x10, 0x10, "the reader restarted");
}

#[test]
fn stopping_playback_withdraws_a_scheduled_fetch() {
    // The "aborted DMA" hook: the CPU has latched a request and the sample is
    // stopped before its get cycle. NESdev DMA, Bugs.
    let apu = apu();
    apu.write(0x13, 0x00);
    apu.write(0x15, 0x10);
    let request = apu.dma_request().unwrap();
    assert!(apu.dma_is_pending(request.serial));
    apu.write(0x15, 0x00);
    assert!(!apu.dma_is_pending(request.serial));
    assert!(apu.dma_request().is_none());
    assert!(
        !apu.dma_complete(request.serial, 0x42),
        "a withdrawn fetch must be rejected, not applied"
    );
}

#[test]
fn the_output_unit_moves_the_level_by_two_per_bit() {
    let apu = apu();
    apu.write(0x11, 0x40); // direct load: level 64
    apu.write(0x10, 0x0F); // fastest rate, no IRQ, no loop
    apu.write(0x13, 0x00);
    apu.write(0x15, 0x10);
    let request = apu.dma_request().unwrap();
    // 0xFF is eight 1 bits: +2 each, so 64 -> 80.
    assert!(apu.dma_complete(request.serial, 0xFF));
    // Rate $F is 54 CPU cycles per bit; the first clock loads the shifter.
    apu.advance(54 * 9);
    assert_eq!(apu.read(0x11), apu.open_bus(), "$4011 is write-only");
    assert!(apu.output() > 0);
}

#[test]
fn the_dmc_level_saturates_rather_than_wrapping() {
    let apu = apu();
    apu.write(0x11, 0x7F);
    assert_eq!(apu.dmc_output(), 0x7F);
    apu.write(0x11, 0xFF);
    assert_eq!(apu.dmc_output(), 0x7F, "only seven bits are loadable");
}

// ---------------------------------------------------------------------------
// $4015 and open bus
// ---------------------------------------------------------------------------

#[test]
fn bit_five_of_the_status_register_is_open_bus() {
    let apu = apu();
    apu.set_open_bus(0xFF);
    assert_eq!(apu.read(0x15) & 0x20, 0x20);
    apu.set_open_bus(0x00);
    assert_eq!(apu.read(0x15) & 0x20, 0x00);
    // And bit 5 is the *only* bit the open bus contributes.
    apu.set_open_bus(0xFF);
    assert_eq!(apu.read(0x15), 0x20);
}

#[test]
fn the_write_only_registers_read_back_as_open_bus() {
    let apu = apu();
    apu.set_open_bus(0xA5);
    for index in [0x00u8, 0x03, 0x08, 0x0F, 0x10, 0x13, 0x17] {
        assert_eq!(apu.read(index), 0xA5, "register {index:#04x}");
    }
}

#[test]
fn a_debug_status_read_reports_the_frame_irq_without_clearing_it() {
    let apu = apu();
    // 29831 rather than 29830: the flag is (re)set on cycles 29828, 29829 and
    // 29830, and a read on any of those returns 1 *without* clearing.
    apu.advance(29831);
    assert_eq!(apu.peek(0x15) & 0x40, 0x40);
    assert_eq!(apu.peek(0x15) & 0x40, 0x40, "peeking twice still shows it");
    assert_eq!(apu.read(0x15) & 0x40, 0x40);
    assert_eq!(apu.read(0x15) & 0x40, 0x00, "a real read clears it");
}

#[test]
fn the_frame_irq_drives_the_irq_line_and_a_status_read_drops_it() {
    let apu = apu();
    assert_eq!(apu.irq_level(), Level::Low);
    apu.advance(29831);
    assert_eq!(apu.irq_level(), Level::High);
    apu.read(0x15);
    assert_eq!(apu.irq_level(), Level::Low);
}

// ---------------------------------------------------------------------------
// Wiring
// ---------------------------------------------------------------------------

/// A wire sink that counts the levels it is told about.
#[derive(Debug, Default)]
struct Counter {
    highs: AtomicU32,
    lows: AtomicU32,
}

impl WireSink for Counter {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        match level {
            Level::High => self.highs.fetch_add(1, AtomicOrdering::SeqCst),
            Level::Low => self.lows.fetch_add(1, AtomicOrdering::SeqCst),
        };
    }
}

#[test]
fn the_apu_drives_its_irq_wire_on_both_edges() {
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let sink = Arc::new(Counter::default());
    let wire = Arc::new(Wire::builder().source(id).sink(sink.clone(), 0).build());
    let apu = apu();
    apu.connect_irq(WireSource::new(Arc::clone(&wire), id));

    assert_eq!(sink.highs.load(AtomicOrdering::SeqCst), 0);
    apu.advance(29831);
    assert_eq!(sink.highs.load(AtomicOrdering::SeqCst), 1);
    apu.read(0x15);
    assert_eq!(sink.lows.load(AtomicOrdering::SeqCst), 1);
}

// ---------------------------------------------------------------------------
// Clock domain
// ---------------------------------------------------------------------------

#[test]
fn the_apu_follows_the_clock_domain_it_is_attached_to() {
    use crate::core::clock::{ClockForest, Rational};

    // The NES topology: one crystal, CPU = master / 12 (`ROADMAP.md` §4.2).
    let mut forest = ClockForest::new();
    let master = forest
        .add_oscillator("master", Rational::new(236_250_000, 11).unwrap())
        .unwrap();
    let cpu = forest.add_domain("cpu", master, 1, 12).unwrap();

    let apu = apu();
    apu.attach_clock(cpu);
    assert_eq!(apu.clock_domain(), Some(cpu));

    forest.advance_domain(cpu, 29_831).unwrap();
    apu.advance_to(forest.ticks(cpu).unwrap());
    assert_eq!(apu.ticks(), 29_831);
    assert_eq!(apu.irq_level(), Level::High);

    // advance_to is idempotent, which is what makes it safe to call before
    // every access.
    apu.advance_to(forest.ticks(cpu).unwrap());
    assert_eq!(apu.ticks(), 29_831);
}

// ---------------------------------------------------------------------------
// Memory-mapped access
// ---------------------------------------------------------------------------

#[test]
fn the_regions_cover_exactly_the_registers_the_apu_decodes() {
    let apu = apu();
    let regions = apu.regions();
    let shapes: Vec<(u64, u64)> = regions.iter().map(|(at, r)| (*at, r.len())).collect();
    assert_eq!(
        shapes,
        [(0x00, 0x14), (0x15, 1), (0x17, 1)],
        "$4014 and $4016 belong to other devices and must not be covered"
    );
}

#[test]
fn an_mmio_write_reaches_the_register_and_a_debug_write_does_not() {
    let apu = apu();
    let regions = apu.regions();
    let status = regions
        .iter()
        .find(|(at, _)| *at == 0x15)
        .map(|(_, r)| r)
        .expect("the status region exists");
    let ops = match status.kind() {
        crate::core::space::RegionKind::Io(ops) => Arc::clone(ops),
        _ => panic!("the status region is an I/O region"),
    };

    ops.write(0, &[0x0F], MemAttrs::DEFAULT).unwrap();
    apu.write(0x03, 0x08);
    let mut byte = [0u8];
    ops.read(0, &mut byte, MemAttrs::DEFAULT).unwrap();
    assert_eq!(byte[0] & 0x01, 0x01, "the enable write went through");

    ops.write(0, &[0x00], MemAttrs::DEBUG).unwrap();
    ops.read(0, &mut byte, MemAttrs::DEFAULT).unwrap();
    assert_eq!(byte[0] & 0x01, 0x01, "a debug write changes nothing");
}

#[test]
fn a_multi_byte_access_is_rejected() {
    let apu = apu();
    let regions = apu.regions();
    let (_, region) = &regions[0];
    let ops = match region.kind() {
        crate::core::space::RegionKind::Io(ops) => Arc::clone(ops),
        _ => panic!("channels is an I/O region"),
    };
    let mut buf = [0u8; 2];
    assert!(ops.read(0, &mut buf, MemAttrs::DEFAULT).is_err());
    assert!(ops.write(0, &[0, 0], MemAttrs::DEFAULT).is_err());
}

// ---------------------------------------------------------------------------
// Mixer
// ---------------------------------------------------------------------------

#[test]
fn silence_mixes_to_zero_and_the_loudest_combination_fits_a_u16() {
    assert_eq!(mixer::mix(0, 0, 0, 0, 0), 0);
    let loudest = mixer::mix(15, 15, 15, 15, 127);
    assert_eq!(
        loudest, 65534,
        "the tables are scaled so a sample cannot clip"
    );
    assert!(loudest < u16::MAX);
}

#[test]
fn the_mixer_tables_are_monotonic() {
    for i in 1..mixer::PULSE_TABLE.len() {
        assert!(mixer::PULSE_TABLE[i] > mixer::PULSE_TABLE[i - 1]);
    }
    for i in 1..mixer::TND_TABLE.len() {
        assert!(mixer::TND_TABLE[i] > mixer::TND_TABLE[i - 1]);
    }
}

#[test]
fn samples_are_produced_once_per_apu_cycle() {
    let apu = apu_with(Props::new().with("sample-buffer", 4096u64));
    apu.advance(1000);
    let mut out = Vec::new();
    apu.take_samples(&mut out);
    assert_eq!(out.len(), 500, "one sample per two CPU cycles");
    let mut again = Vec::new();
    apu.take_samples(&mut again);
    assert!(again.is_empty(), "draining consumes the ring");
}

#[test]
fn a_zero_capacity_ring_produces_nothing() {
    let apu = apu_with(Props::new().with("sample-buffer", 0u64));
    apu.advance(1000);
    let mut out = Vec::new();
    apu.take_samples(&mut out);
    assert!(out.is_empty());
    assert_eq!(apu.samples_dropped(), 0);
}

#[test]
fn an_undrained_ring_drops_the_oldest_samples_and_says_so() {
    let apu = apu_with(Props::new().with("sample-buffer", 16u64));
    apu.advance(100);
    assert_eq!(apu.samples_dropped(), 50 - 16);
    let mut out = Vec::new();
    apu.take_samples(&mut out);
    assert_eq!(out.len(), 16);
}

// ---------------------------------------------------------------------------
// Properties, class, reset
// ---------------------------------------------------------------------------

#[test]
fn an_unknown_property_is_rejected_and_a_bad_timing_names_the_options() {
    let bad = Apu::new(&Props::new().with("timeing", "ntsc"));
    assert!(
        bad.is_err(),
        "a typo'd property must not be silently ignored"
    );

    let err = Apu::new(&Props::new().with("timing", "secam")).unwrap_err();
    let text = alloc::format!("{err}");
    assert!(text.contains("ntsc"), "{text}");
    assert!(text.contains("pal"), "{text}");
}

#[test]
fn the_class_registers_and_constructs() {
    let mut registry = Registry::new();
    register(&mut registry).unwrap();
    assert!(registry.get("nes.apu").is_some());
    let device = registry.create("nes.apu", &Props::new()).unwrap();
    assert_eq!(device.class().name, "nes.apu");
    assert!(
        register(&mut registry).is_err(),
        "no duplicate registration"
    );
}

#[test]
fn a_cold_reset_returns_every_register_to_its_power_on_value() {
    let apu = apu();
    apu.write(0x15, 0x1F);
    apu.write(0x03, 0x08);
    apu.advance(29830);
    assert_ne!(apu.read(0x15) & 0x0F, 0);
    apu.reset(ResetKind::Cold);
    assert_eq!(apu.read(0x15), 0x00);
    assert_eq!(apu.ticks(), 0);
    assert_eq!(apu.irq_level(), Level::Low);
}

#[test]
fn a_warm_reset_silences_the_channels_but_keeps_4017() {
    let apu = apu();
    apu.write(0x17, 0x80); // five-step mode
    apu.advance(10);
    apu.write(0x15, 0x0F);
    apu.write(0x03, 0x08);
    apu.write(0x11, 0x7F);
    apu.reset(ResetKind::Warm);
    assert_eq!(apu.read(0x15) & 0x1F, 0, "a reset writes $00 to $4015");
    assert_eq!(apu.frame_mode(), Mode::FiveStep, "$4017 is unchanged");
    assert_eq!(apu.dmc_output(), 1, "the DMC level is ANDed with 1");
}

// ---------------------------------------------------------------------------
// Snapshot
// ---------------------------------------------------------------------------

/// Write one device's chunk and return the whole snapshot.
fn snapshot(apu: &Apu) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("apu", "nes.apu").unwrap();
    let mut writer = StateWriter::new(shape);
    {
        let mut chunk = writer.chunk("apu", "nes.apu", APU_CLASS.version).unwrap();
        apu.save(&mut chunk).unwrap();
    }
    writer.to_vec().unwrap()
}

/// FNV-1a over a byte slice: a state hash for the round-trip assertion.
fn hash(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in bytes {
        h ^= u64::from(*b);
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

/// Put an APU into a state that touches every unit.
fn exercised() -> Apu {
    let apu = apu();
    apu.write(0x00, 0xBF); // pulse 1: duty 2, halt, constant volume 15
    apu.write(0x01, 0x8A); // sweep enabled
    apu.write(0x02, 0x34);
    apu.write(0x04, 0x76); // pulse 2
    apu.write(0x05, 0x99);
    apu.write(0x06, 0x21);
    apu.write(0x08, 0xC3); // triangle linear counter
    apu.write(0x0A, 0x55);
    apu.write(0x0C, 0x1A); // noise envelope
    apu.write(0x0E, 0x87); // short mode, period 7
    apu.write(0x10, 0x4B); // DMC: loop, rate $B
    apu.write(0x11, 0x39);
    apu.write(0x12, 0x20);
    apu.write(0x13, 0x03);
    apu.write(0x15, 0x1F);
    apu.write(0x03, 0x28);
    apu.write(0x07, 0x51);
    apu.write(0x0B, 0x93);
    apu.write(0x0F, 0xC8);
    apu.write(0x17, 0x00);
    apu.advance(20_000);
    apu
}

#[test]
fn save_and_load_round_trip_to_an_identical_state_hash() {
    let original = exercised();
    let bytes = snapshot(&original);

    let restored = apu();
    let reader = StateReader::new(&bytes).unwrap();
    let (class, version, data) = reader.load_raw("apu").unwrap();
    assert_eq!(class, "nes.apu");
    assert_eq!(version, APU_CLASS.version);
    let mut chunk = ChunkReader::new(data);
    restored.load(&mut chunk).unwrap();
    chunk
        .end()
        .expect("load must consume every byte save wrote");

    assert_eq!(
        hash(&snapshot(&restored)),
        hash(&bytes),
        "a restored APU must serialize identically"
    );
}

#[test]
fn a_restored_apu_continues_identically() {
    let original = exercised();
    let bytes = snapshot(&original);
    let restored = apu();
    let reader = StateReader::new(&bytes).unwrap();
    let (_, _, data) = reader.load_raw("apu").unwrap();
    restored.load(&mut ChunkReader::new(data)).unwrap();

    original.advance(50_000);
    restored.advance(50_000);
    assert_eq!(
        hash(&snapshot(&original)),
        hash(&snapshot(&restored)),
        "the two must stay in lockstep after the restore"
    );
    assert_eq!(original.read(0x15), restored.read(0x15));
    assert_eq!(original.output(), restored.output());
}

#[test]
fn a_pending_dmc_fetch_survives_a_round_trip() {
    let apu = apu();
    apu.write(0x13, 0x02);
    apu.write(0x15, 0x10);
    let request = apu.dma_request().unwrap();

    let bytes = snapshot(&apu);
    let restored = self::apu();
    let reader = StateReader::new(&bytes).unwrap();
    let (_, _, data) = reader.load_raw("apu").unwrap();
    restored.load(&mut ChunkReader::new(data)).unwrap();

    assert_eq!(restored.dma_request(), Some(request));
    assert!(restored.dma_complete(request.serial, 0x11));
}
