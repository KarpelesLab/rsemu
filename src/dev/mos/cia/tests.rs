//! The 8520's own tests: what the data sheets say, asserted one rule at a time.
//!
//! Each name is the claim. The register numbers are the chip's own — `$0`
//! through `$F` — because a board's idea of where they land is a board's
//! business and none of these tests has a board.

use super::*;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::wire::{Wire, WireId};
use alloc::string::ToString;
use alloc::vec::Vec;

/// The register block, which is what a bus access reaches.
fn regs(cia: &Cia) -> Arc<CiaRegs> {
    Arc::new(CiaRegs {
        shared: Arc::clone(&cia.shared),
    })
}

fn peek(cia: &Cia, index: u64) -> u8 {
    let mut byte = [0u8; 1];
    regs(cia)
        .read(index, &mut byte, MemAttrs::DEFAULT)
        .expect("a byte read is legal");
    byte[0]
}

fn peek_debug(cia: &Cia, index: u64) -> u8 {
    let mut byte = [0u8; 1];
    regs(cia)
        .read(index, &mut byte, MemAttrs::DEBUG)
        .expect("a byte read is legal");
    byte[0]
}

fn poke(cia: &Cia, index: u64, value: u8) {
    regs(cia)
        .write(index, &[value], MemAttrs::DEFAULT)
        .expect("a byte write is legal");
}

/// A wire with one source, so a pin has something to drive.
fn dummy_source(id: u64) -> WireSource {
    let src = WireId::new(id);
    WireSource::new(Wire::builder().source(src).build_shared(), src)
}

/// Load a timer's latch and start it counting φ2, continuously.
fn start_timer_a(cia: &Cia, period: u16, cr: u8) {
    poke(cia, 0x4, period as u8);
    poke(cia, 0x5, (period >> 8) as u8);
    poke(cia, 0xe, CR_START | cr);
}

// ---------------------------------------------------------------------------
// the ports
// ---------------------------------------------------------------------------

#[test]
fn an_unwired_port_pin_reads_as_the_pull_up_that_holds_it() {
    // "The port pins are set as inputs and port registers to zero (although a
    // read of the ports will return all highs because of passive pullups)" —
    // 6526 data sheet, RESET. It is the rule an Amiga's `/CHNG` depends on: a
    // disk-change line nobody is pulling is not a disk change.
    let cia = Cia::bare();
    assert_eq!(peek(&cia, 0x0), 0xff, "PRA");
    assert_eq!(peek(&cia, 0x1), 0xff, "PRB");
    assert_eq!(peek(&cia, 0x2), 0x00, "DDRA: every pin an input");

    // An output pin reads its own register; an input pin reads the pin.
    poke(&cia, 0x2, 0x0f);
    poke(&cia, 0x0, 0xa5);
    assert_eq!(peek(&cia, 0x0), 0xf5, "0x5 from PRA, 0xF from the pull-ups");
    cia.set_port_a(0x00);
    assert_eq!(
        peek(&cia, 0x0),
        0x05,
        "and now something is pulling them low"
    );
    assert_eq!(cia.port_a(), 0x05);
}

#[test]
fn a_direction_bit_decides_whether_the_pin_drives_or_merely_pulls_up() {
    let cia = Cia::bare();
    let pa0 = dummy_source(1);
    let pa1 = dummy_source(2);
    cia.connect_pin("pa0", pa0.clone()).expect("pa0 exists");
    cia.connect_pin("pa1", pa1.clone()).expect("pa1 exists");
    assert_eq!(
        pa0.drive_state(),
        Drive::WeakHigh,
        "an input is the pull-up"
    );

    poke(&cia, 0x2, 0x01); // DDRA: pa0 out, pa1 still in
    poke(&cia, 0x0, 0x02); // PRA: a zero on pa0, a one on pa1
    assert_eq!(pa0.drive_state(), Drive::Low, "an output drives");
    assert_eq!(pa1.drive_state(), Drive::WeakHigh, "and an input does not");

    poke(&cia, 0x0, 0x01);
    assert_eq!(pa0.drive_state(), Drive::High);

    assert!(cia.connect_pin("pa8", dummy_source(3)).is_err());
    let e = cia
        .connect_pin("rdy", dummy_source(4))
        .expect_err("no such pin")
        .to_string();
    assert!(e.contains("rdy"), "{e}");
}

// ---------------------------------------------------------------------------
// the timers
// ---------------------------------------------------------------------------

#[test]
fn a_timer_underflows_one_cycle_after_it_reaches_zero() {
    // "The timer counts down from the latched value to zero, generates an
    // interrupt and reloads the latched value" — so a latch of N is a period of
    // N+1 counts.
    let cia = Cia::bare();
    start_timer_a(&cia, 9, 0);
    assert_eq!(cia.timer_a(), 9, "a stopped timer loads on the high byte");
    assert_eq!(
        cia.shared.next_event.load(Ordering::Relaxed),
        10,
        "a counter of 9 started at tick 0 underflows on tick 10"
    );

    cia.advance_to(9);
    assert_eq!(cia.timer_a(), 0);
    assert_eq!(cia.icr() & ICR_TA, 0, "not yet");

    cia.advance_to(10);
    assert_eq!(cia.icr() & ICR_TA, ICR_TA, "ICR0 on the count past zero");
    assert_eq!(cia.timer_a(), 9, "and it reloaded from the latch");

    // Many periods in one budget is still one flag and the right phase.
    let _ = peek(&cia, 0xd);
    cia.advance_to(10 + 10 * 1000 + 3);
    assert_eq!(cia.icr() & ICR_TA, ICR_TA);
    assert_eq!(cia.timer_a(), 6);
}

#[test]
fn a_one_shot_timer_stops_itself_and_clears_its_own_start_bit() {
    let cia = Cia::bare();
    start_timer_a(&cia, 4, CR_ONESHOT);
    cia.advance_to(4);
    assert_eq!(cia.icr() & ICR_TA, 0);
    cia.advance_to(5);
    assert_eq!(cia.icr() & ICR_TA, ICR_TA);
    assert_eq!(cia.timer_a(), 4, "reloaded");
    assert_eq!(peek(&cia, 0xe) & CR_START, 0, "and stopped by the chip");
    assert_eq!(
        cia.shared.next_event.load(Ordering::Relaxed),
        NO_EVENT,
        "a stopped timer has nothing on the calendar"
    );

    let _ = peek(&cia, 0xd);
    cia.advance_to(100_000);
    assert_eq!(cia.icr() & ICR_TA, 0, "one shot means one");
}

#[test]
fn the_high_byte_loads_the_counter_only_while_the_timer_is_stopped() {
    // "The timer latch is loaded into the timer on any timer underflow, on a
    // force load, or following a write to the high byte of the prescaler while
    // the timer is stopped."
    let cia = Cia::bare();
    start_timer_a(&cia, 0x2000, 0);
    cia.advance_to(0x100);
    let counter = cia.timer_a();
    assert_eq!(counter, 0x2000 - 0x100);

    poke(&cia, 0x4, 0x34);
    poke(&cia, 0x5, 0x12); // running: the latch moves, the counter does not
    assert_eq!(cia.timer_a(), counter, "a running timer keeps counting");

    // The force-load strobe does move it, and never reads back.
    poke(&cia, 0xe, CR_START | CR_LOAD);
    assert_eq!(cia.timer_a(), 0x1234);
    assert_eq!(peek(&cia, 0xe) & CR_LOAD, 0, "bit 4 is a strobe");
    assert_eq!(
        peek(&cia, 0x4),
        0x34,
        "and the counter reads low byte first"
    );
    assert_eq!(peek(&cia, 0x5), 0x12);
}

#[test]
fn the_start_a_high_byte_write_makes_raises_a_toggle_output_like_any_other() {
    // "The toggle output is set high whenever the timer is started, and set low
    // by RES" (6526 data sheet, TIMER A OUTPUT MODES). The high-byte write is
    // one of the ways an 8520's one-shot timer starts, so PB6 rises on it.
    let cia = Cia::bare();
    poke(&cia, 0xe, CR_ONESHOT | CR_OUTMODE | CR_PBON);
    assert_eq!(peek(&cia, 0x1) & 0x40, 0, "PB6 low until the timer starts");
    poke(&cia, 0x4, 0x10);
    poke(&cia, 0x5, 0x00);
    assert_eq!(peek(&cia, 0x1) & 0x40, 0x40, "and high once it has");
}

#[test]
fn a_high_byte_write_starts_a_one_shot_timer_whatever_the_start_bit_says() {
    // "In one-shot mode, a write to timer-high (register 5 for timer A,
    // register 7 for Timer B) will transfer the timer latch to the counter and
    // initiate counting regardless of the start bit" (Amiga Hardware Reference
    // Manual, Appendix F). Kickstart's timer.device leans on it: CRA = $08,
    // then the latch, and no start bit is ever written.
    let cia = Cia::bare();
    poke(&cia, 0xe, CR_ONESHOT);
    poke(&cia, 0x4, 4);
    assert_eq!(peek(&cia, 0xe) & CR_START, 0, "the low byte starts nothing");
    poke(&cia, 0x5, 0);
    assert_eq!(peek(&cia, 0xe) & CR_START, CR_START, "the high byte did");
    assert_eq!(cia.timer_a(), 4, "from the latch");
    cia.advance_to(5);
    assert_eq!(
        cia.icr() & ICR_TA,
        ICR_TA,
        "and it runs out like any one-shot"
    );
    assert_eq!(peek(&cia, 0xe) & CR_START, 0);

    poke(&cia, 0xf, CR_ONESHOT);
    poke(&cia, 0x6, 2);
    poke(&cia, 0x7, 0);
    assert_eq!(peek(&cia, 0xf) & CR_START, CR_START, "timer B too");

    // A continuous timer is still only loaded, and only while stopped.
    let cia = Cia::bare();
    poke(&cia, 0x4, 4);
    poke(&cia, 0x5, 0);
    assert_eq!(peek(&cia, 0xe) & CR_START, 0);
    assert_eq!(cia.timer_a(), 4);
}

#[test]
fn timer_b_counts_timer_a_underflows_which_is_how_a_long_delay_is_made() {
    // Timer B's input mode 10: the chained mode. At an Amiga's 709 kHz E clock
    // a single timer tops out at 92 ms and the pair at just under an hour.
    let cia = Cia::bare();
    start_timer_a(&cia, 3, 0); // a timer A period of four ticks
    poke(&cia, 0x6, 2);
    poke(&cia, 0x7, 0);
    poke(&cia, 0xf, CR_START | TB_IN_TA); // three timer A underflows

    assert_eq!(
        cia.shared.next_event.load(Ordering::Relaxed),
        4,
        "timer A is first"
    );

    cia.advance_to(4);
    assert_eq!(cia.icr() & ICR_TA, ICR_TA, "one A underflow");
    assert_eq!(cia.timer_b(), 1, "which counted B down by one");
    assert_eq!(cia.icr() & ICR_TB, 0);
    assert_eq!(
        cia.shared.next_event.load(Ordering::Relaxed),
        8,
        "timer A's next underflow comes first, and B's is four ticks after it"
    );

    cia.advance_to(11);
    assert_eq!(cia.icr() & ICR_TB, 0, "not yet");
    cia.advance_to(12);
    assert_eq!(cia.icr() & ICR_TB, ICR_TB, "and there it is");
    assert_eq!(cia.timer_b(), 2, "reloaded from its own latch");

    // φ2 does not reach timer B in this mode, so the counter only moves when
    // timer A does.
    let before = cia.timer_b();
    poke(&cia, 0xe, 0); // stop timer A
    cia.advance_to(10_000);
    assert_eq!(cia.timer_b(), before, "no A underflows, no B counting");
}

#[test]
fn pb6_toggles_on_each_underflow_and_pulses_for_one_cycle_otherwise() {
    // "PB ON … this overrides the DDRB bit", so the pin carries the timer
    // whether or not DDRB says output.
    let cia = Cia::bare();
    poke(&cia, 0x4, 3);
    poke(&cia, 0x5, 0);
    poke(&cia, 0xe, CR_START | CR_PBON | CR_OUTMODE);
    assert_eq!(
        peek(&cia, 0x1) & 0x40,
        0x40,
        "toggle starts the output high"
    );

    cia.advance_to(4);
    assert_eq!(peek(&cia, 0x1) & 0x40, 0x00, "and the underflow toggles it");
    cia.advance_to(8);
    assert_eq!(peek(&cia, 0x1) & 0x40, 0x40);

    // Pulse mode: high for the underflow cycle and low either side of it.
    let cia = Cia::bare();
    poke(&cia, 0x4, 3);
    poke(&cia, 0x5, 0);
    poke(&cia, 0xe, CR_START | CR_PBON);
    assert_eq!(peek(&cia, 0x1) & 0x40, 0x00);
    cia.advance_to(3);
    assert_eq!(peek(&cia, 0x1) & 0x40, 0x00);
    cia.advance_to(4);
    assert_eq!(peek(&cia, 0x1) & 0x40, 0x40, "one cycle, on the underflow");
    cia.advance_to(5);
    assert_eq!(peek(&cia, 0x1) & 0x40, 0x00, "and gone again");
}

#[test]
fn a_one_shot_timer_b_pulses_pb7_on_exactly_the_cycle_it_underflows() {
    // Timer B on φ2, one-shot, pulse mode: the tick the pulse is visible on is
    // computed rather than stepped to, so assert it lands where the count says.
    let cia = Cia::bare();
    cia.advance_to(1_000);
    poke(&cia, 0x6, 9);
    poke(&cia, 0x7, 0);
    poke(&cia, 0xf, CR_START | CR_ONESHOT | CR_PBON);
    cia.advance_to(1_009);
    assert_eq!(peek(&cia, 0x1) & 0x80, 0x00);
    cia.advance_to(1_010);
    assert_eq!(peek(&cia, 0x1) & 0x80, 0x80, "a latch of 9 from tick 1000");
    assert_eq!(peek(&cia, 0xf) & CR_START, 0, "and it stopped itself");
    cia.advance_to(1_011);
    assert_eq!(peek(&cia, 0x1) & 0x80, 0x00);

    // Jumping straight past the underflow leaves the pin low, because the pulse
    // was a cycle that has already gone by.
    let cia = Cia::bare();
    poke(&cia, 0x6, 9);
    poke(&cia, 0x7, 0);
    poke(&cia, 0xf, CR_START | CR_ONESHOT | CR_PBON);
    cia.advance_to(500);
    assert_eq!(cia.icr() & ICR_TB, ICR_TB);
    assert_eq!(peek(&cia, 0x1) & 0x80, 0x00);
}

#[test]
fn cra_bit_seven_is_stored_and_does_nothing_on_an_8520() {
    // The 6526's 50/60 Hz TOD divider select. The 8520 counts edges on a pin.
    let cia = Cia::bare();
    poke(&cia, 0xe, CRA_TODIN);
    assert_eq!(peek(&cia, 0xe), CRA_TODIN, "it reads back");
    assert!(cia.todin());
    for _ in 0..6 {
        cia.tod_pulse();
    }
    assert_eq!(cia.tod(), 6, "and every edge still counts one");
}

#[test]
fn a_cnt_edge_counts_a_timer_that_was_told_to_count_cnt() {
    let cia = Cia::bare();
    poke(&cia, 0x4, 2);
    poke(&cia, 0x5, 0);
    poke(&cia, 0xe, CR_START | CRA_INMODE);
    cia.advance_to(1000);
    assert_eq!(cia.timer_a(), 2, "φ2 does not reach a CNT-mode timer");

    // CNT idles high, like every pin with a pull-up on it, so a pulse is a
    // fall and then the rise that counts.
    for _ in 0..3 {
        cia.cnt_edge(false);
        cia.cnt_edge(true);
    }
    assert_eq!(cia.icr() & ICR_TA, ICR_TA, "three edges for a latch of two");
    assert_eq!(cia.timer_a(), 2);
}

// ---------------------------------------------------------------------------
// the TOD counter
// ---------------------------------------------------------------------------

/// Set the 24-bit counter through its registers, MSB first as the stop/start
/// rule intends.
fn set_tod(cia: &Cia, value: u32) {
    poke(cia, 0xa, (value >> 16) as u8);
    poke(cia, 0x9, (value >> 8) as u8);
    poke(cia, 0x8, value as u8);
}

#[test]
fn the_tod_counter_is_twenty_four_binary_bits_clocked_by_a_pin() {
    // The 8520's one visible difference from a 6526: no tenths, no seconds, no
    // minutes, no hours and no BCD — a plain counter, and register `$B` is not
    // a fourth byte of it.
    let cia = Cia::bare();
    assert_eq!(cia.tod(), 0);
    for _ in 0..5 {
        cia.tod_pulse();
    }
    assert_eq!(cia.tod(), 5);
    assert_eq!(peek(&cia, 0x8), 5);
    assert_eq!(peek(&cia, 0xb), 0, "the 6526's hours register is not here");

    set_tod(&cia, TOD_MASK);
    assert_eq!(cia.tod(), 0x00ff_ffff);
    cia.tod_pulse();
    assert_eq!(cia.tod(), 0, "and it wraps at 24 bits");
}

#[test]
fn reading_the_high_byte_latches_the_time_and_reading_the_low_byte_releases_it() {
    // The rule firmware depends on, and the one a register model usually gets
    // wrong: without it a read that straddles a carry returns a time that never
    // existed. On a 6526 the MSB is the hours register and the LSB the tenths;
    // on an 8520 they are bits 23-16 and bits 7-0, which is `$A` and `$8`.
    let cia = Cia::bare();
    set_tod(&cia, 0x0000ff);

    assert_eq!(peek(&cia, 0xa), 0x00, "the MSB latches all three bytes");
    cia.tod_pulse(); // the counter carries under the latch: 0x0000ff -> 0x000100
    assert_eq!(cia.tod(), 0x000100, "the counter never stops for a read");
    assert_eq!(peek(&cia, 0x9), 0x00, "but the latch still holds 0x0000ff");
    assert_eq!(peek(&cia, 0x8), 0xff, "and so does the LSB, which releases");
    assert_eq!(peek(&cia, 0x9), 0x01, "now it reads live again");
    assert_eq!(peek(&cia, 0x8), 0x00);

    // A read of the LSB alone latches nothing, so it is live from the start.
    set_tod(&cia, 0x0000ff);
    assert_eq!(peek(&cia, 0x8), 0xff);
    cia.tod_pulse();
    assert_eq!(peek(&cia, 0x8), 0x00, "no latch was ever taken");
}

#[test]
fn writing_the_high_byte_stops_the_counter_until_the_low_byte_is_written() {
    // The write-side half of the same problem: the counter must not carry
    // between the three stores that set it.
    let cia = Cia::bare();
    poke(&cia, 0xa, 0x00);
    for _ in 0..10 {
        cia.tod_pulse();
    }
    assert_eq!(cia.tod(), 0, "halted by the write of the MSB");
    poke(&cia, 0x9, 0x12);
    poke(&cia, 0x8, 0x34);
    assert_eq!(cia.tod(), 0x001234);
    cia.tod_pulse();
    assert_eq!(cia.tod(), 0x001235, "and the LSB started it again");
}

#[test]
fn crb_bit_seven_sends_a_tod_write_to_the_alarm_instead() {
    let cia = Cia::bare();
    set_tod(&cia, 0);
    poke(&cia, 0xf, CRB_ALARM);
    set_tod(&cia, 5);
    assert_eq!(cia.tod(), 0, "the clock did not move");
    poke(&cia, 0xf, 0);

    poke(&cia, 0xd, ICR_IR | ICR_ALARM); // arm the interrupt
    for _ in 0..4 {
        cia.tod_pulse();
    }
    assert_eq!(cia.icr() & ICR_ALARM, 0);
    assert_eq!(cia.irq_level(), Level::Low);
    cia.tod_pulse();
    assert_eq!(cia.icr() & ICR_ALARM, ICR_ALARM, "the fifth is the alarm");
    assert_eq!(cia.irq_level(), Level::High);

    // And a halted counter reaches no alarm.
    let _ = peek(&cia, 0xd);
    poke(&cia, 0xa, 0x00);
    for _ in 0..40 {
        cia.tod_pulse();
    }
    assert_eq!(cia.icr() & ICR_ALARM, 0);
}

// ---------------------------------------------------------------------------
// the shift register
// ---------------------------------------------------------------------------

#[test]
fn a_byte_shifts_in_on_cnt_most_significant_bit_first() {
    // How an Amiga keyboard arrives: the keyboard drives SP and clocks CNT, and
    // the eighth bit raises ICR3.
    let cia = Cia::bare();
    poke(&cia, 0xd, ICR_IR | ICR_SP);
    for bit in (0..8).rev() {
        cia.set_sp(0xa5 & (1 << bit) != 0);
        cia.cnt_edge(false);
        cia.cnt_edge(true);
    }
    assert_eq!(peek(&cia, 0xc), 0xa5, "SDR");
    assert_eq!(cia.icr() & ICR_SP, ICR_SP);
    assert_eq!(cia.irq_level(), Level::High);

    // And the register is ready for the next byte immediately.
    let _ = peek(&cia, 0xd);
    for bit in (0..8).rev() {
        cia.set_sp(0x3c & (1 << bit) != 0);
        cia.cnt_edge(false);
        cia.cnt_edge(true);
    }
    assert_eq!(peek(&cia, 0xc), 0x3c);
}

#[test]
fn a_byte_shifts_out_at_half_the_timer_a_underflow_rate() {
    // "TIMER A is used for the baud rate generator… data is shifted out at 1/2
    // the underflow rate of TIMER A", with CNT as the clock the receiver counts.
    let cia = Cia::bare();
    poke(&cia, 0x4, 0);
    poke(&cia, 0x5, 0); // a timer A period of one tick
    poke(&cia, 0xe, CR_START | CRA_SPMODE);
    let sp = dummy_source(1);
    let cnt = dummy_source(2);
    cia.connect_pin("sp", sp.clone()).expect("sp exists");
    cia.connect_pin("cnt", cnt.clone()).expect("cnt exists");
    assert_eq!(
        sp.drive_state(),
        Drive::Low,
        "output mode before any bit has left: the latch's reset level, low"
    );

    poke(&cia, 0xc, 0xa5);
    cia.advance_to(2);
    assert_eq!(
        sp.drive_state(),
        Drive::HiZ,
        "two underflows, and 0xa5's top bit is a one: the open-drain stage lets go"
    );
    cia.advance_to(4);
    assert_eq!(sp.drive_state(), Drive::Low, "then a zero");
    assert_eq!(cia.icr() & ICR_SP, 0, "and no interrupt until the eighth");

    cia.advance_to(16);
    assert_eq!(cia.icr() & ICR_SP, ICR_SP, "sixteen underflows is a byte");

    // In input mode the two pins are let go of, because they are inputs.
    poke(&cia, 0xe, CR_START);
    assert_eq!(sp.drive_state(), Drive::HiZ);
    assert_eq!(cnt.drive_state(), Drive::HiZ);
}

// ---------------------------------------------------------------------------
// interrupts and the handshake
// ---------------------------------------------------------------------------

#[test]
fn the_interrupt_register_clears_every_flag_when_it_is_read() {
    // "All flags remain set until the DATA register is read, whereupon the
    // register is cleared and the /IRQ line returns high."
    let cia = Cia::bare();
    let irq = dummy_source(1);
    cia.connect_pin("irq", irq.clone()).expect("irq exists");
    assert_eq!(cia.irq_level(), Level::Low);

    start_timer_a(&cia, 1, 0);
    cia.advance_to(2);
    assert_eq!(cia.icr() & ICR_TA, ICR_TA);
    assert_eq!(cia.icr() & ICR_IR, 0, "IR is the *enabled* wired-OR");
    assert_eq!(irq.drive_state(), Drive::Low, "and nothing is requesting");

    // Enabling it asserts immediately: IR is combinational over the latched
    // flags and the mask.
    poke(&cia, 0xd, ICR_IR | ICR_TA);
    assert_eq!(cia.icr() & ICR_IR, ICR_IR);
    assert_eq!(irq.drive_state(), Drive::High);

    let value = peek(&cia, 0xd);
    assert_eq!(value & (ICR_TA | ICR_IR), ICR_TA | ICR_IR);
    assert_eq!(cia.icr(), 0, "the read cleared every flag");
    assert_eq!(irq.drive_state(), Drive::Low, "and released the pin");
    assert_eq!(peek(&cia, 0xd), 0, "a second read has nothing to say");
}

#[test]
fn the_mask_register_sets_or_clears_by_its_top_bit() {
    // "Bit 7 of the data written determines whether the mask bits written are
    // set or cleared", which is why a write of zero changes nothing.
    let cia = Cia::bare();
    poke(&cia, 0xd, ICR_IR | ICR_TA | ICR_TB);
    assert_eq!(cia.shared.state.lock().icr_mask, ICR_TA | ICR_TB);
    poke(&cia, 0xd, ICR_TB); // bit 7 clear: clear timer B's
    assert_eq!(cia.shared.state.lock().icr_mask, ICR_TA);
    poke(&cia, 0xd, 0x00);
    assert_eq!(
        cia.shared.state.lock().icr_mask,
        ICR_TA,
        "a zero is a no-op"
    );
    poke(&cia, 0xd, ICR_IR | ICR_SOURCES);
    assert_eq!(cia.shared.state.lock().icr_mask, ICR_SOURCES);
}

#[test]
fn a_falling_edge_on_flag_sets_its_bit() {
    // `/FLAG` keeps its true polarity: the pin is active low and it is the
    // negative edge that counts, which is what lets it be wired straight to
    // another CIA's `/PC`.
    let cia = Cia::bare();
    cia.set_flag(true);
    assert_eq!(cia.icr() & ICR_FLAG, 0);
    cia.set_flag(false);
    assert_eq!(cia.icr() & ICR_FLAG, ICR_FLAG);

    let _ = peek(&cia, 0xd);
    cia.set_flag(false);
    assert_eq!(cia.icr() & ICR_FLAG, 0, "a level is not an edge");
    cia.set_flag(true);
    cia.set_flag(false);
    assert_eq!(cia.icr() & ICR_FLAG, ICR_FLAG);
}

#[test]
fn the_pc_strobe_goes_low_for_one_cycle_after_a_prb_access() {
    let cia = Cia::bare();
    let pc = dummy_source(1);
    cia.connect_pin("pc", pc.clone()).expect("pc exists");
    assert_eq!(pc.drive_state(), Drive::High, "it idles high");

    cia.advance_to(100);
    let _ = peek(&cia, 0x1);
    assert_eq!(pc.drive_state(), Drive::Low, "a read of PRB strobes it");
    assert_eq!(
        cia.shared.next_event.load(Ordering::Relaxed),
        101,
        "and the strobe has to end on its own cycle"
    );
    cia.advance_to(101);
    assert_eq!(pc.drive_state(), Drive::High);

    poke(&cia, 0x1, 0x00);
    assert_eq!(pc.drive_state(), Drive::Low, "a write strobes it too");

    // Reading any other register does not.
    cia.advance_to(102);
    let _ = peek(&cia, 0x0);
    assert_eq!(pc.drive_state(), Drive::High);
}

#[test]
fn one_chips_output_can_drive_another_chips_input_without_nesting_locks() {
    // The regression: `refresh` used to hold its `WIRE`-ranked output table
    // while driving, so the far chip's sink took its `DEVICE` state lock under
    // `WIRE`. The rank checker is live in this build (`cfg(test)`), which the
    // `--no-default-features` board test in `tests/` is not, so it lives here.
    let a = Cia::bare();
    let b = Cia::bare();
    let src = WireId::new(1);
    let flag = b.sink("flag", &[src]).expect("flag is a sink");
    let wire = Wire::builder()
        .source(src)
        .sink(flag.sink, flag.line)
        .build_shared();
    a.connect_pin("pc", WireSource::new(wire, src))
        .expect("pc is a source");

    a.advance_to(10);
    let _ = peek(&a, 0x1); // /PC low: a falling edge on B's /FLAG
    assert_eq!(b.icr() & ICR_FLAG, ICR_FLAG);
    a.advance_to(11); // and back up, from inside the catch-up path
    assert_eq!(peek(&b, 0xd) & ICR_FLAG, ICR_FLAG);
}

// ---------------------------------------------------------------------------
// the invariants every device has
// ---------------------------------------------------------------------------

#[test]
fn a_debug_access_advances_nothing_latches_nothing_and_clears_nothing() {
    let cia = Cia::bare();
    start_timer_a(&cia, 1, 0);
    cia.advance_to(2);
    set_tod(&cia, 0x0000ff);

    assert_eq!(peek_debug(&cia, 0xd) & ICR_TA, ICR_TA);
    assert_eq!(cia.icr() & ICR_TA, ICR_TA, "a debug read cleared no flag");

    assert_eq!(peek_debug(&cia, 0xa), 0x00);
    cia.tod_pulse();
    assert_eq!(
        peek_debug(&cia, 0x8),
        0x00,
        "no latch was taken, so this is live"
    );
    assert!(!cia.shared.state.lock().tod_latched);

    let pc = dummy_source(1);
    cia.connect_pin("pc", pc.clone()).expect("pc exists");
    let _ = peek_debug(&cia, 0x1);
    assert_eq!(pc.drive_state(), Drive::High, "and PRB strobed nothing");

    // A debug *write* is refused outright: there is no way to start a timer
    // harmlessly.
    assert_eq!(
        regs(&cia).write(0xe, &[CR_START], MemAttrs::DEBUG),
        Err(BusError::BadAccess)
    );
}

#[test]
fn only_byte_accesses_are_accepted() {
    let cia = Cia::bare();
    let r = regs(&cia);
    assert_eq!(
        r.read(0, &mut [0u8; 2], MemAttrs::DEFAULT),
        Err(BusError::BadAccess)
    );
    assert_eq!(
        r.write(0, &[0, 0], MemAttrs::DEFAULT),
        Err(BusError::BadAccess)
    );
    assert_eq!(r.constraints().min, Width::U8);
}

#[test]
fn a_reset_leaves_the_timer_latches_at_all_ones_and_everything_else_at_zero() {
    // 6526 data sheet, RESET.
    let cia = Cia::bare();
    poke(&cia, 0x2, 0xff);
    poke(&cia, 0x0, 0xa5);
    start_timer_a(&cia, 0x1234, 0);
    poke(&cia, 0xd, ICR_IR | ICR_TA);
    set_tod(&cia, 0x123456);
    cia.set_port_b(0x00);

    cia.reset(ResetKind::Cold);
    assert_eq!(peek(&cia, 0x2), 0x00, "DDRA");
    assert_eq!(peek(&cia, 0x0), 0xff, "PRA, back to its pull-ups");
    assert_eq!(peek(&cia, 0x4), 0xff, "and the timer holds the latch");
    assert_eq!(peek(&cia, 0x5), 0xff);
    assert_eq!(peek(&cia, 0xe), 0x00, "CRA");
    assert_eq!(cia.tod(), 0);
    assert_eq!(cia.icr(), 0);
    assert_eq!(
        peek(&cia, 0x1),
        0x00,
        "what a *neighbour* drives onto a pin is the neighbour's state, not ours"
    );
}

#[test]
fn the_whole_register_block_is_the_region() {
    let cia = Cia::bare();
    assert_eq!(cia.region("").expect("mapped").len(), REGISTER_COUNT);
    assert!(cia.region("regs").is_some());
    assert!(cia.region("porta").is_none());
}

#[test]
fn a_snapshot_round_trips_to_identical_state() {
    let saved = Cia::bare();
    poke(&saved, 0x3, 0xff);
    poke(&saved, 0x1, 0x5a);
    start_timer_a(&saved, 0x00ff, CR_PBON | CR_OUTMODE);
    poke(&saved, 0x6, 0x03);
    poke(&saved, 0x7, 0x00);
    poke(&saved, 0xf, CR_START | TB_IN_TA);
    poke(&saved, 0xd, ICR_IR | ICR_TA | ICR_ALARM);
    set_tod(&saved, 0x00abcd);
    for _ in 0..7 {
        saved.tod_pulse();
    }
    let _ = peek(&saved, 0xa); // and with the TOD latch held
    saved.advance_to(2_000);

    let mut shape = MachineShape::new();
    shape.add_device("cia", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("cia", CLASS_NAME, STATE_VERSION).unwrap();
        saved.save(&mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();

    let restored = Cia::bare();
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("cia", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    restored.load(&mut chunk.reader()).unwrap();

    let before: Vec<u8> = (0..16).map(|i| peek_debug(&saved, i)).collect();
    let after: Vec<u8> = (0..16).map(|i| peek_debug(&restored, i)).collect();
    assert_eq!(before, after);
    assert_eq!(restored.ticks(), 2_000, "and it resumes from the same tick");
    assert_eq!(restored.tod(), saved.tod());

    // Both keep running identically from there, timer B's chain included.
    saved.advance_to(20_000);
    restored.advance_to(20_000);
    assert_eq!(saved.timer_a(), restored.timer_a());
    assert_eq!(saved.timer_b(), restored.timer_b());
    assert_eq!(saved.icr(), restored.icr());
    assert_eq!(saved.port_b(), restored.port_b());
}

#[test]
fn the_class_is_registrable_and_takes_no_properties() {
    let mut registry = crate::core::Registry::new();
    register(&mut registry).expect("a fresh registry");
    let class = registry.get(CLASS_NAME).expect("registered");
    assert_eq!(class.version, STATE_VERSION);
    assert!(class.properties.is_empty());
    let device = (class.construct)(&Props::new()).expect("nothing to give it");
    assert_eq!(device.class().name, CLASS_NAME);
    assert!(device.is_lazy(), "the timers are sampled");

    let e = Cia::new(&Props::new().with("tod", "vsync"))
        .expect_err("a property it does not have")
        .to_string();
    assert!(e.contains("tod"), "{e}");
}

#[test]
fn every_pin_the_schema_names_is_a_pin_the_device_has() {
    // The failure this exists for: a schema that promises a pin the device
    // answers `None` for builds, and the board that wires it fails at realize
    // with a message about a pin the validator said was fine.
    let schema = schema();
    for name in [
        "irq", "pc", "sp", "cnt", "flag", "tod", "pa0", "pa7", "pb0", "pb7",
    ] {
        let port = schema
            .port_named(name)
            .unwrap_or_else(|| panic!("`{name}` is in the schema"));
        let cia = Cia::bare();
        assert_eq!(
            cia.sink(name, &[]).is_some(),
            port.dir.can_receive(),
            "`{name}` as a sink"
        );
        assert_eq!(
            cia.connect_pin(name, dummy_source(1)).is_ok(),
            port.dir.can_drive(),
            "`{name}` as a source"
        );
    }
    // And a pin neither half has.
    assert!(schema.port_named("pa8").is_none());
    assert!(Cia::bare().sink("pa8", &[]).is_none());
}
