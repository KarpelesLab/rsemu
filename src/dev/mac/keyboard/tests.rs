//! `mac.keyboard` against a real `mac.via`, which is the only end that can say
//! whether the protocol works: the two halves have to agree about which edge
//! carries which bit, and a keyboard tested against a model of the VIA would
//! agree with the model rather than with the chip.

use alloc::sync::Arc;
use alloc::vec::Vec;

use super::*;
use crate::core::device::Device;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::wire::{Pull, Wire, WireId};
use crate::dev::mac::via::{self, Via};

/// ACR for mode `111`: shift out under the external CB1 clock. What the ROM
/// writes to send a command.
const ACR_OUT_EXT: u8 = 0x1c;
/// ACR for mode `011`: shift in under the same clock. What it writes to take
/// the response.
const ACR_IN_EXT: u8 = 0x0c;
/// ACR for mode `110`: shift out under φ2. How the computer pulls the data line
/// low with no clock to do it with.
const ACR_OUT_PHI2: u8 = 0x18;

/// The VIA's register numbers, which the board puts 512 bytes apart.
const R_SR: u8 = 10;
const R_ACR: u8 = 11;
const R_IFR: u8 = 13;

/// A keyboard on the end of a cable, with a 6522 at the other end of it.
struct Rig {
    kbd: MacKeyboard,
    via: Via,
    /// Microseconds of the keyboard's clock.
    now: u64,
}

impl Rig {
    fn new() -> Rig {
        let kbd = MacKeyboard::with(Arc::new(Keyboard::new()));
        let via = Via::build();

        // The data line: both ends drive it and both ends sense it, pulled up,
        // which is what makes a shared open-collector line work in both
        // directions.
        let (kbd_id, via_id) = (WireId::new(1), WireId::new(2));
        let ids = [kbd_id, via_id];
        let kbd_sink = kbd.sink(DATA_PIN, &ids).expect("the keyboard senses data");
        let via_sink = via.sink("cb2", &ids).expect("the VIA senses CB2");
        let data = Wire::builder()
            .source(kbd_id)
            .source(via_id)
            .resolved(Pull::Up)
            .sink(kbd_sink.sink, kbd_sink.line)
            .sink(via_sink.sink, via_sink.line)
            .build_shared();

        // The clock: "driven only by the keyboard".
        let clk_id = WireId::new(3);
        let cb1 = via.sink("cb1", &[clk_id]).expect("the VIA senses CB1");
        let clk = Wire::builder()
            .source(clk_id)
            .resolved(Pull::Up)
            .sink(cb1.sink, cb1.line)
            .build_shared();

        kbd.connect(DATA_PIN, WireSource::new(Arc::clone(&data), kbd_id))
            .expect("data");
        kbd.connect(CLK_PIN, WireSource::new(clk, clk_id))
            .expect("clk");
        via.connect_pin("cb2", WireSource::new(data, via_id))
            .expect("cb2");

        Rig { kbd, via, now: 0 }
    }

    fn keyboard(&self) -> &Arc<Keyboard> {
        self.kbd.keyboard()
    }

    fn run(&mut self, ticks: u64) {
        self.now += ticks;
        self.keyboard().advance_to(self.now);
    }

    /// What the ROM's trace shows it doing to start a transaction: walk a zero
    /// onto the data line under φ2, disable the register, then load the command
    /// into the external-clock mode.
    fn send(&mut self, command: u8) {
        self.via.poke(R_ACR, ACR_OUT_PHI2);
        self.via.poke(R_SR, 0x00);
        // One φ2 tick is all the zero needs to reach the pin.
        self.via.advance_to(self.via.ticks() + 1);
        self.via.poke(R_ACR, 0x00);
        self.via.poke(R_ACR, ACR_OUT_EXT);
        self.via.poke(R_SR, command);
    }

    /// The other half: once the command is out, take the response.
    fn receive(&mut self) {
        let _ = self.via.peek(R_SR);
        self.via.poke(R_ACR, ACR_IN_EXT);
    }

    /// Whether the shift register has finished a transfer.
    fn sr_flag(&self) -> bool {
        self.via.peek(R_IFR) & via::IRQ_SR != 0
    }

    /// Run a whole exchange the way the ROM does, and give back what came in.
    fn exchange(&mut self, command: u8) -> u8 {
        self.send(command);
        // Eight 400 µs cycles, plus the keyboard's start delay.
        for _ in 0..(START_TICKS + 8 * TX_CYCLE_TICKS + 100) / 10 {
            self.run(10);
            if self.sr_flag() {
                break;
            }
        }
        assert!(self.sr_flag(), "the command did not shift out");
        self.receive();
        // The response: a quarter-second Inquiry timeout at worst, then eight
        // 330 µs cycles.
        for _ in 0..(INQUIRY_TIMEOUT_TICKS + 8 * RX_CYCLE_TICKS + 2_000) / 10 {
            self.run(10);
            if self.sr_flag() {
                break;
            }
        }
        assert!(self.sr_flag(), "the response did not shift in");
        self.via.peek(R_SR)
    }
}

/// Table 7-4's Model Number format, built out of its four fields.
#[test]
fn the_model_number_response_is_table_7_4s_four_fields() {
    // "Bit 0: 1. Bits 1-3: keyboard model number, 1-8."
    assert_eq!(code::model(1, None), 0x03);
    assert_eq!(code::MODEL_M0110, 0x03);
    assert_eq!(code::model(2, None), 0x05);
    // "Bits 4-6: next device number, 1-8. Bit 7: 1 if another device
    // connected."
    assert_eq!(code::model(1, Some(2)), 0xa3);
    // Bit 0 is always high, whatever else is.
    for m in 1..=8 {
        assert_eq!(code::model(m, None) & 1, 1, "model {m}");
    }
}

/// The whole first transaction a Macintosh makes: Model Number out, the model
/// number back. This is the one that has to work before the ROM will go on.
#[test]
fn model_number_goes_out_and_the_model_comes_back() {
    let mut rig = Rig::new();
    let answer = rig.exchange(code::MODEL_NUMBER);
    assert_eq!(
        rig.keyboard().last_exchange().0,
        code::MODEL_NUMBER,
        "the keyboard read the command off the line"
    );
    assert_eq!(answer, code::MODEL_M0110, "and answered with its model");
    assert_eq!(rig.keyboard().commands(), 1);
}

/// An Inquiry with nothing typed comes back Null — after the Guide's quarter
/// second, which is long enough to be worth asserting is really waited out.
#[test]
fn an_inquiry_with_no_key_waits_a_quarter_second_and_answers_null() {
    let mut rig = Rig::new();
    assert_eq!(rig.exchange(code::MODEL_NUMBER), code::MODEL_M0110);
    let before = rig.now;
    assert_eq!(rig.exchange(code::INQUIRY), code::NULL);
    assert!(
        rig.now - before >= INQUIRY_TIMEOUT_TICKS,
        "the quarter second was not waited out: {} µs",
        rig.now - before
    );
}

/// A key pressed between transactions comes back on the next Inquiry, with the
/// Guide's encoding: bit 7 low for a down, bit 0 high always.
#[test]
fn a_key_transition_comes_back_on_the_next_inquiry() {
    let mut rig = Rig::new();
    assert_eq!(rig.exchange(code::MODEL_NUMBER), code::MODEL_M0110);
    // `$19` is a key-down transition code — odd, bit 7 clear.
    rig.keyboard().press(0x19, true);
    assert_eq!(rig.keyboard().queued(), 1);
    assert_eq!(rig.exchange(code::INQUIRY), 0x19);
    assert!(rig.keyboard().held(0x19));
    // And the release, with bit 7 set.
    rig.keyboard().press(0x19, false);
    assert_eq!(rig.exchange(code::INQUIRY), 0x19 | code::KEY_UP);
    assert!(!rig.keyboard().held(0x19));
}

/// Instant does not wait, which is the whole difference between it and
/// Inquiry.
#[test]
fn instant_answers_without_the_quarter_second() {
    let mut rig = Rig::new();
    assert_eq!(rig.exchange(code::MODEL_NUMBER), code::MODEL_M0110);
    let before = rig.now;
    assert_eq!(rig.exchange(code::INSTANT), code::NULL);
    assert!(
        rig.now - before < INQUIRY_TIMEOUT_TICKS,
        "Instant waited: {} µs",
        rig.now - before
    );
}

/// Test self-tests and passes.
#[test]
fn test_is_acknowledged() {
    let mut rig = Rig::new();
    assert_eq!(rig.exchange(code::TEST), code::ACK);
}

/// A key held down does not send a second down: key repeat is the operating
/// system's on a Macintosh.
#[test]
fn a_held_key_does_not_repeat_itself() {
    let kbd = Keyboard::new();
    kbd.press(0x19, true);
    kbd.press(0x19, true);
    kbd.press(0x19, true);
    assert_eq!(kbd.queued(), 1);
    kbd.press(0x19, false);
    assert_eq!(kbd.queued(), 2);
    kbd.press(0x19, false);
    assert_eq!(kbd.queued(), 2);
}

/// A computer that never says it is ready to receive is abandoned, so that its
/// own half-second timeout can start the link again rather than deadlock
/// against a keyboard still holding the cable.
#[test]
fn a_computer_that_never_raises_the_line_is_given_up_on() {
    let mut rig = Rig::new();
    rig.send(code::MODEL_NUMBER);
    for _ in 0..((START_TICKS + 8 * TX_CYCLE_TICKS + 100) / 10) {
        rig.run(10);
    }
    assert!(rig.sr_flag(), "the command went out");
    // The ROM would now switch to shift-in, which lets CB2 go. This one does
    // not, so the line stays low and the keyboard waits — and then gives up,
    // rather than holding the cable until something else moves.
    rig.run(TURNAROUND_TIMEOUT_TICKS + 1_000);
    assert_eq!(rig.keyboard().commands(), 0, "it answered nothing");
    assert_eq!(rig.keyboard().last_exchange(), (0, 0));
    // A line that is *still* low is a computer still saying it is ready, so the
    // keyboard starts over rather than sulking. That is what makes the
    // computer's own half-second retry work.
    rig.run(START_TICKS + TX_CYCLE_TICKS);
    assert!(
        matches!(
            rig.keyboard().state.lock().phase,
            Phase::Starting | Phase::Command { .. }
        ),
        "it went back to listening"
    );
}

/// Save and load put back the same keyboard, mid-transaction.
#[test]
fn the_state_round_trips() {
    let mut rig = Rig::new();
    assert_eq!(rig.exchange(code::MODEL_NUMBER), code::MODEL_M0110);
    rig.keyboard().press(0x19, true);
    rig.send(code::INQUIRY);
    // Stop halfway through clocking the command in.
    rig.run(START_TICKS + 3 * TX_CYCLE_TICKS + 50);

    let image = |kbd: &MacKeyboard| -> Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("kbd", CLASS_NAME).expect("a fresh shape");
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("kbd", CLASS_NAME, STATE_VERSION).expect("a chunk");
            Device::save(kbd, &mut chunk).expect("a keyboard saves");
        }
        w.to_vec().expect("a complete image")
    };
    let first = image(&rig.kbd);

    let fresh = MacKeyboard::with(Arc::new(Keyboard::new()));
    let reader = StateReader::new(&first).expect("a well-formed image");
    let chunk = reader
        .load("kbd", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .expect("the chunk is there");
    Device::load(&fresh, &mut chunk.reader()).expect("a keyboard loads");

    assert_eq!(image(&fresh), first, "the same keyboard, bit for bit");
    // One at a time: two of these locks at once is two `DEVICE` ranks held
    // together, which `core::sync` refuses and is right to.
    let restored_phase = fresh.keyboard().state.lock().phase;
    let saved_phase = rig.keyboard().state.lock().phase;
    assert_eq!(restored_phase, saved_phase);
    assert!(
        matches!(saved_phase, Phase::Command { .. }),
        "the snapshot was taken mid-command: {saved_phase:?}"
    );
    assert_eq!(fresh.keyboard().queued(), rig.keyboard().queued());
}

/// The bits go out most significant first, which is the one thing about the
/// encoding the Guide states outright, and the recirculating shift register
/// leaves the command in SR afterwards.
#[test]
fn the_command_shifts_out_most_significant_bit_first() {
    let mut rig = Rig::new();
    rig.send(code::MODEL_NUMBER);
    for _ in 0..((START_TICKS + 8 * TX_CYCLE_TICKS + 100) / 10) {
        rig.run(10);
        if rig.sr_flag() {
            break;
        }
    }
    assert_eq!(rig.keyboard().state.lock().last_command, 0, "not yet asked");
    // A 6522 recirculates on a shift out, so eight shifts leave the byte where
    // it was — which is what the ROM reads back on its next retry.
    assert_eq!(rig.via.peek(R_SR), code::MODEL_NUMBER);
}
