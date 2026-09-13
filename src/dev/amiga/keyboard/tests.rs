//! The keyboard's own tests: Appendix G, one rule at a time.
//!
//! Most of them put a real 8520 on the other end of the cable, because the
//! protocol only means anything against the part that receives it: the codes
//! are read out of the chip's serial data register, and the handshake is the
//! chip's serial port turned to output and back, as chapter 8 describes.

use super::*;

#[test]
fn a_code_goes_out_rotated_and_arrives_inverted() {
    // "The keycode for B is $35 = 00110101; due to the rotation of the byte,
    // the bits transmitted are 01101010" — and released, $B5, "01101011".
    assert_eq!(wire_order(0x35), 0b0110_1010);
    assert_eq!(wire_order(0xb5), 0b0110_1011);
    // KDAT is active low, so the 8520 shifts in the complement.
    assert_eq!(sdr_of(0x35), !0b0110_1010);
    for c in 0..=255u8 {
        assert_eq!(code_of(sdr_of(c)), c);
    }
}

#[test]
fn a_keyboard_on_no_cable_still_keeps_time_and_state() {
    let kb = Keyboard::new();
    assert!(kb.synchronising());
    assert!(kb.caps_lock_led(), "lit until the start-up sequence ends");
    kb.press(0x20, true);
    assert!(kb.held(0x20));
    assert_eq!(
        kb.queued(),
        0,
        "a key held while synchronising is not queued"
    );
    kb.advance_to(1_000_000);
    assert_eq!(kb.ticks(), 1_000_000);
    assert!(kb.synchronising(), "nobody answered");
}

#[cfg(feature = "dev-mos8520")]
mod against_an_8520 {
    use super::*;
    use crate::core::space::{AddressSpace, MemAttrs};
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::value::Width;
    use crate::core::wire::{Pull, Wire};
    use crate::dev::mos::Cia;

    /// `CRA` bit 6: the serial port shifts out.
    const SPMODE: u8 = 0x40;
    /// `ICR` bit 3: a byte has arrived.
    const ICR_SP: u8 = 0x08;

    /// A keyboard and an 8520 on one cable.
    struct Rig {
        device: AmigaKeyboard,
        cia: Cia,
        space: AddressSpace,
        now: u64,
    }

    impl Rig {
        fn new() -> Rig {
            Rig::with(Keyboard::new())
        }

        fn with(keyboard: Keyboard) -> Rig {
            let device = AmigaKeyboard::with(Arc::new(keyboard));
            let cia = Cia::bare();

            // KDAT: both ends drive it and both ends sense it.
            let (kb_id, cia_id) = (WireId::new(1), WireId::new(2));
            let ids = [kb_id, cia_id];
            let kb_sink = device.sink(KDAT_PIN, &ids).expect("kdat");
            let cia_sink = cia.sink("sp", &ids).expect("sp");
            let kdat = Wire::builder()
                .source(kb_id)
                .source(cia_id)
                .resolved(Pull::Up)
                .sink(kb_sink.sink, kb_sink.line)
                .sink(cia_sink.sink, cia_sink.line)
                .build_shared();
            // KCLK: the keyboard's alone.
            let clk_id = WireId::new(3);
            let cnt = cia.sink("cnt", &[clk_id]).expect("cnt");
            let kclk = Wire::builder()
                .source(clk_id)
                .resolved(Pull::Up)
                .sink(cnt.sink, cnt.line)
                .build_shared();

            device
                .connect(KDAT_PIN, WireSource::new(Arc::clone(&kdat), kb_id))
                .expect("kdat");
            device
                .connect(KCLK_PIN, WireSource::new(kclk, clk_id))
                .expect("kclk");
            cia.connect_pin("sp", WireSource::new(kdat, cia_id))
                .expect("sp");

            let space = AddressSpace::new("cia-a", 16);
            space
                .topology()
                .map(cia.region("").expect("registers"), 0)
                .expect("maps");
            Rig {
                device,
                cia,
                space,
                now: 0,
            }
        }

        fn keyboard(&self) -> &Arc<Keyboard> {
            self.device.keyboard()
        }

        fn poke(&self, reg: u64, v: u8) {
            self.space
                .write(reg, Width::U8, u64::from(v), MemAttrs::DEFAULT)
                .expect("a CIA register");
        }

        fn peek(&self, reg: u64) -> u8 {
            self.space
                .read(reg, Width::U8, MemAttrs::DEFAULT)
                .expect("a CIA register") as u8
        }

        fn run(&mut self, ticks: u64) {
            self.now += ticks;
            self.keyboard().advance_to(self.now);
        }

        /// "Pulsing the SP line low then high": the port to output for 85 µs
        /// — the manual's figure for every keyboard model — and back.
        fn handshake(&mut self) {
            self.poke(0xe, SPMODE);
            self.run(85);
            self.poke(0xe, 0);
        }

        /// Wait up to `limit` ticks for a byte, the way an interrupt handler
        /// would get one: `ICR` says so, `SDR` has it.
        fn wait_for_byte(&mut self, limit: u64) -> Option<u8> {
            let until = self.now + limit;
            while self.now < until {
                self.run(10);
                if self.cia.icr() & ICR_SP != 0 {
                    let _ = self.peek(0xd);
                    return Some(code_of(self.peek(0xc)));
                }
            }
            None
        }

        /// A byte, acknowledged.
        fn receive(&mut self) -> Option<u8> {
            let got = self.wait_for_byte(2_000)?;
            self.handshake();
            Some(got)
        }

        /// Answer the first sync bit and take the start-up stream.
        fn synchronise(&mut self) -> Vec<u8> {
            self.run(100);
            // Turning the port round also empties the shift register of the
            // sync bit it caught, which is what a driver's first handshake
            // after reset gets it.
            self.handshake();
            let mut got = Vec::new();
            while let Some(b) = self.receive() {
                got.push(b);
                if b == code::END_POWER_UP_STREAM {
                    break;
                }
            }
            got
        }
    }

    #[test]
    fn power_up_clocks_out_ones_every_143_ms_until_it_is_answered() {
        let mut rig = Rig::new();
        // Four sync bits in half a second: at power-on, then every 143 ms.
        // Each is a one — KDAT low — so the chip shifts in zeroes.
        rig.run(500_000);
        assert!(rig.keyboard().synchronising());
        assert_eq!(rig.cia.icr() & ICR_SP, 0, "four bits is not a byte");
        rig.run(4 * HANDSHAKE_TIMEOUT_TICKS);
        assert_eq!(
            rig.cia.icr() & ICR_SP,
            ICR_SP,
            "eight sync bits fill the shift register: 'no more than eight clocks'"
        );
        assert_eq!(rig.peek(0xc), 0x00, "eight ones, active low");
        let _ = rig.peek(0xd);

        // A handshake: the stream, and the end of the LED.
        rig.handshake();
        assert!(!rig.keyboard().synchronising());
        assert_eq!(rig.receive(), Some(code::POWER_UP_STREAM));
        assert!(rig.keyboard().caps_lock_led());
        assert_eq!(rig.receive(), Some(code::END_POWER_UP_STREAM));
        assert!(!rig.keyboard().caps_lock_led(), "finally … shut off");
        assert_eq!(rig.wait_for_byte(HANDSHAKE_TIMEOUT_TICKS), None);
    }

    #[test]
    fn the_start_up_stream_reports_the_keys_held_down() {
        let mut rig = Rig::new();
        rig.keyboard().press(0x63, true); // Ctrl
        rig.keyboard().press(0x20, true); // A
        assert_eq!(
            rig.synchronise(),
            [code::POWER_UP_STREAM, 0x20, 0x63, code::END_POWER_UP_STREAM]
        );
    }

    #[test]
    fn a_key_goes_down_and_up_each_code_waiting_for_its_handshake() {
        let mut rig = Rig::new();
        rig.synchronise();
        rig.keyboard().press(0x35, true);
        rig.keyboard().press(0x35, false);
        assert_eq!(rig.keyboard().queued(), 1, "the down is on the line");
        assert_eq!(rig.receive(), Some(0x35));
        assert_eq!(rig.receive(), Some(0xb5));
        assert_eq!(rig.keyboard().acknowledged(), 4, "$FD, $FE, down, up");

        // A repeat from the host is not a transition.
        rig.keyboard().press(0x10, true);
        rig.keyboard().press(0x10, true);
        assert_eq!(rig.receive(), Some(0x10));
        assert_eq!(rig.wait_for_byte(1_000), None);
    }

    #[test]
    fn a_byte_takes_sixty_microseconds_a_bit() {
        let mut rig = Rig::new();
        rig.synchronise();
        rig.keyboard().press(0x40, true);
        // Eight bits of 20 µs setup, 20 µs low, 20 µs hold: the eighth clock
        // rises at 7 × 60 + 40.
        rig.run(459);
        assert_eq!(rig.cia.icr() & ICR_SP, 0);
        rig.run(1);
        assert_eq!(rig.cia.icr() & ICR_SP, ICR_SP);
        assert_eq!(code_of(rig.peek(0xc)), 0x40);
    }

    #[test]
    fn with_no_handshake_it_resyncs_and_says_so_before_sending_again() {
        let mut rig = Rig::new();
        rig.synchronise();
        rig.keyboard().press(0x21, true);
        assert_eq!(rig.wait_for_byte(2_000), Some(0x21));
        // Say nothing for 143 ms. The keyboard clocks out a one, the chip
        // takes it as the first bit of a new byte, and a driver that answers
        // it gets the lost-sync code and the key again.
        rig.run(HANDSHAKE_TIMEOUT_TICKS + 100);
        rig.handshake();
        assert_eq!(rig.receive(), Some(code::LOST_SYNC));
        assert_eq!(rig.receive(), Some(0x21));
        assert_eq!(rig.wait_for_byte(HANDSHAKE_TIMEOUT_TICKS), None);
    }

    #[test]
    fn a_resync_that_takes_several_tries_keeps_clocking_ones() {
        let mut rig = Rig::new();
        rig.synchronise();
        rig.keyboard().press(0x22, true);
        assert_eq!(rig.wait_for_byte(2_000), Some(0x22));
        // Seven more ones fill the register again; the garbage arrives as a
        // key *release*, which is why the flag goes last.
        let garbage = rig.wait_for_byte(8 * HANDSHAKE_TIMEOUT_TICKS + 2_000);
        assert_eq!(garbage, Some(0xff), "all ones: $FF, with the up bit set");
        rig.handshake();
        assert_eq!(rig.receive(), Some(code::LOST_SYNC));
        assert_eq!(rig.receive(), Some(0x22));
    }

    #[test]
    fn caps_lock_sends_only_when_pushed_and_says_what_the_led_did() {
        let mut rig = Rig::new();
        rig.synchronise();
        let caps = code::CAPS_LOCK;
        rig.keyboard().press(caps, true);
        rig.keyboard().press(caps, false);
        rig.keyboard().press(caps, true);
        rig.keyboard().press(caps, false);
        assert_eq!(rig.receive(), Some(caps), "LED on: the up/down bit is 0");
        assert_eq!(rig.receive(), Some(caps | code::KEY_UP), "LED off: 1");
        assert_eq!(rig.wait_for_byte(1_000), None, "releases send nothing");
        assert!(!rig.keyboard().caps_lock_led());
    }

    #[test]
    fn ten_codes_wait_and_the_eleventh_is_an_overflow() {
        let mut rig = Rig::new();
        rig.synchronise();
        // One on the line, ten waiting, and one too many.
        for key in 0x10..=0x1b {
            rig.keyboard().press(key, true);
        }
        assert_eq!(rig.keyboard().queued(), TYPE_AHEAD);
        let mut got = Vec::new();
        while let Some(b) = rig.receive() {
            got.push(b);
        }
        let mut want: Vec<u8> = (0x10..=0x1a).collect();
        want.push(code::BUFFER_OVERFLOW);
        assert_eq!(got, want, "$1B was lost, and the computer is told");
    }

    fn snapshot(device: &AmigaKeyboard) -> Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("kbd", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("kbd", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(device, &mut chunk).unwrap();
        }
        w.to_vec().unwrap()
    }

    #[test]
    fn a_snapshot_mid_byte_round_trips_and_carries_on_identically() {
        let mut rig = Rig::new();
        rig.synchronise();
        rig.keyboard().press(0x35, true);
        rig.keyboard().press(0x36, true);
        rig.run(250); // part of the way through the first byte
        let bytes = snapshot(&rig.device);

        let mut other = Rig::new();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("kbd", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        Device::load(&other.device, &mut chunk.reader()).unwrap();
        assert_eq!(snapshot(&other.device), bytes);

        // Both carry on to the same place. The fresh chip missed the first
        // bits, so compare the keyboards rather than what either chip caught.
        other.now = rig.now;
        rig.run(HANDSHAKE_TIMEOUT_TICKS * 2);
        other.run(HANDSHAKE_TIMEOUT_TICKS * 2);
        assert_eq!(snapshot(&other.device), snapshot(&rig.device));
    }

    #[test]
    fn a_warm_reset_leaves_the_keyboard_alone_and_a_cold_one_restarts_it() {
        let mut rig = Rig::new();
        rig.synchronise();
        rig.device.reset(ResetKind::Warm);
        assert!(!rig.keyboard().synchronising());
        rig.device.reset(ResetKind::Cold);
        assert!(rig.keyboard().synchronising());
        assert_eq!(rig.synchronise().first(), Some(&code::POWER_UP_STREAM));
    }
}

#[test]
fn the_class_names_its_pins() {
    let schema = schema();
    let debug = alloc::format!("{schema:?}");
    assert!(debug.contains(KCLK_PIN) && debug.contains(KDAT_PIN));
    assert_eq!(describe(0xb5), "$35 up");
    assert_eq!(describe(code::POWER_UP_STREAM), "power-up key stream");
}
