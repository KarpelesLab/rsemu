//! The transceiver on its own: the link's byte handshake, the bus commands it
//! answers, and a snapshot round trip.

use alloc::sync::Arc;
use alloc::vec::Vec;

use super::*;
use crate::core::device::Device;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

/// Drive the link by hand, the way the VIA's external shift modes do.
///
/// The transceiver owns the clock, so a test cannot clock it; it moves the
/// state lines and then runs time forward, sampling the data line while the
/// clock is low (which is what a shift-in mode latches on the rising edge) and
/// presenting a bit there for a shift-out.
struct Link {
    adb: Arc<Adb>,
    t: u64,
}

impl Link {
    fn new() -> Link {
        Link {
            adb: Arc::new(Adb::new()),
            t: 0,
        }
    }

    fn run(&mut self, ticks: u64) {
        self.t += ticks;
        self.adb.advance_to(self.t);
    }

    /// Put `lines` on `ST1:ST0`.
    fn state(&mut self, lines: u8) {
        self.adb.update(|st| st.state_changed(lines));
    }

    /// Send `byte` to the transceiver: state 0 selects the command slot, and
    /// each bit is placed on the data line while the clock is low.
    fn send(&mut self, lines: u8, byte: u8) {
        self.state(lines);
        self.run(START_TICKS);
        let mut bits = byte;
        for _ in 0..8 {
            // The clock has just fallen; the computer's shift register presents
            // its bit. `sense` takes the *line* level, so a one is high.
            self.adb.update(|st| st.sense(bits & 0x80 == 0));
            bits <<= 1;
            self.run(BIT_TICKS);
        }
    }

    /// Take a byte from the transceiver, sampling the data line while the clock
    /// is low.
    fn receive(&mut self, lines: u8) -> u8 {
        self.state(lines);
        self.run(START_TICKS);
        let mut got = 0u8;
        for _ in 0..8 {
            let (_, data, _) = self.adb.lines();
            got = (got << 1) | u8::from(!data);
            self.run(BIT_TICKS);
        }
        got
    }
}

/// A command byte crosses the link and the transceiver decodes it.
#[test]
fn a_command_byte_crosses_the_link() {
    let mut link = Link::new();
    // Talk address 2, register 3 — the identity register of the keyboard.
    let command = (cmd::ADDR_KEYBOARD << 4) | (cmd::TALK << 2) | 3;
    link.send(0, command);
    let (last, _, received) = link.adb.last_exchange();
    assert_eq!(last, command, "the command byte arrived whole");
    assert_eq!(received, command);
}

/// And a Talk of register 3 answers with the address and the handler.
#[test]
fn talk_register_three_answers_with_the_address() {
    let mut link = Link::new();
    let command = (cmd::ADDR_KEYBOARD << 4) | (cmd::TALK << 2) | 3;
    link.send(0, command);
    let high = link.receive(1);
    let low = link.receive(2);
    assert_eq!(
        high & 0x0f,
        cmd::ADDR_KEYBOARD,
        "register 3's address field, got {high:#04x}"
    );
    assert_eq!(low, 1, "the Apple Standard Keyboard's handler identifier");
}

/// A Talk of a register nothing answers still clocks the link, and what the
/// computer latches is `$FF` — the pulled-up bus with nothing driving it.
///
/// **This is the one that was measured.** A transceiver that stayed quiet for
/// an absent device left a real Macintosh Classic ROM waiting on a
/// shift-register interrupt that never came, at address 0 — the first address
/// of its own bus scan, which it walks all the way to 15.
#[test]
fn an_unanswered_talk_reads_as_all_ones() {
    let mut link = Link::new();
    // Address 5 holds no device in this model.
    let command = (5u8 << 4) | (cmd::TALK << 2) | 3;
    link.send(0, command);
    assert_eq!(link.receive(1), 0xff, "register 3's high byte");
    assert_eq!(link.receive(2), 0xff, "and its low one");
}

/// A key movement makes the transceiver ask for attention, and the next Talk 0
/// of the keyboard's address hands the transition over.
#[test]
fn a_key_is_reported_through_talk_zero() {
    let mut link = Link::new();
    link.adb.press(0x24, true);
    let (_, _, int) = link.adb.lines();
    assert!(int, "the attention line went low");
    let command = (cmd::ADDR_KEYBOARD << 4) | (cmd::TALK << 2);
    link.send(0, command);
    let first = link.receive(1);
    let second = link.receive(2);
    assert_eq!(first, 0x24, "the key transition");
    assert_eq!(second, 0xff, "and no second one");
}

/// `SendReset` puts every device back at its power-on address.
#[test]
fn send_reset_puts_the_devices_back() {
    let mut link = Link::new();
    // Move the keyboard to address 8 with a Listen of register 3.
    let listen = (cmd::ADDR_KEYBOARD << 4) | (cmd::LISTEN << 2) | 3;
    link.send(0, listen);
    link.send(1, 0x08);
    link.send(2, 0x01);
    link.state(3);
    link.run(BIT_TICKS);
    assert_eq!(link.adb.addresses()[0], 8, "the keyboard moved");

    link.send(0, cmd::RESET);
    link.state(3);
    link.run(BIT_TICKS);
    assert_eq!(
        link.adb.addresses(),
        alloc::vec![cmd::ADDR_KEYBOARD, cmd::ADDR_MOUSE],
        "and a bus reset put it back"
    );
}

/// The mouse's Talk 0 carries the button and the two deltas.
#[test]
fn the_mouse_reports_a_movement() {
    let mut link = Link::new();
    link.adb.mouse(3, -2, true);
    let command = (cmd::ADDR_MOUSE << 4) | (cmd::TALK << 2);
    link.send(0, command);
    let first = link.receive(1);
    let second = link.receive(2);
    assert_eq!(first & 0x80, 0, "the button is down, so bit 7 is clear");
    assert_eq!(first & 0x7f, (-2i8 as u8) & 0x7f, "dy");
    assert_eq!(second & 0x7f, 3, "dx");
}

/// Save, load, and the image is identical — mid-transaction, which is the only
/// interesting place to stop.
#[test]
fn a_snapshot_round_trip_is_identical() {
    let a = MacAdb::with(Arc::new(Adb::new()));
    let mut link = Link {
        adb: Arc::clone(a.bus()),
        t: 0,
    };
    link.adb.press(0x31, true);
    let command = (cmd::ADDR_KEYBOARD << 4) | (cmd::TALK << 2) | 3;
    link.send(0, command);
    // Stop halfway through clocking the answer out.
    link.state(1);
    link.run(START_TICKS + 3 * BIT_TICKS + 10);

    let image = |adb: &MacAdb| -> Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("adb", CLASS_NAME).expect("a fresh shape");
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("adb", CLASS_NAME, STATE_VERSION).expect("a chunk");
            Device::save(adb, &mut chunk).expect("a transceiver saves");
        }
        w.to_vec().expect("a complete image")
    };
    let first = image(&a);

    let fresh = MacAdb::with(Arc::new(Adb::new()));
    let reader = StateReader::new(&first).expect("a well-formed image");
    let chunk = reader
        .load("adb", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .expect("the chunk is there");
    Device::load(&fresh, &mut chunk.reader()).expect("a transceiver loads");
    assert_eq!(image(&fresh), first, "the same transceiver, bit for bit");
}

/// The class registers and its schema names every pin the board wires.
#[test]
fn the_class_declares_its_pins() {
    let s = schema();
    let ports: Vec<&str> = s.ports.iter().map(|p| p.name.as_str()).collect();
    for want in [CLK_PIN, DATA_PIN, INT_PIN, ST0_PIN, ST1_PIN] {
        assert!(ports.contains(&want), "the schema is missing `{want}`");
    }
}

/// And a command byte's name, for a monitor.
#[test]
fn a_command_describes_itself() {
    assert_eq!(describe(cmd::RESET), "SendReset");
    assert_eq!(describe(0x2c), "Talk 2 r0");
    assert_eq!(describe(0x3f), "Talk 3 r3");
    assert_eq!(describe(0x2a), "Listen 2 r2");
    assert_eq!(describe(0x21), "Flush 2");
}
