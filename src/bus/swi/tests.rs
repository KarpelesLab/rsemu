//! Tests for the single-wire fabric.
//!
//! Every assertion names the data sheet paragraph it is checking —
//! ATECC608B-TFLXTLS DS40002249B chapter 8 and 9.3.2, and ATSHA204A
//! DS40002025A chapter 5 — so a disagreement is either a bug here or a
//! misreading of that paragraph and nothing else.

use super::*;

use alloc::collections::VecDeque;
use alloc::vec;

use crate::core::hosts::HostObjects;
use crate::core::sync::Mutex;

/// A part that records what it was handed and drives back whatever it was
/// given to say.
#[derive(Debug)]
struct Recorder {
    seen: Mutex<Vec<(Token, u64)>>,
    out: Mutex<VecDeque<Token>>,
}

impl Recorder {
    fn new(out: &[Token]) -> Arc<Recorder> {
        Arc::new(Recorder {
            seen: Mutex::with_rank(LockRank::LEAF, Vec::new()),
            out: Mutex::with_rank(LockRank::LEAF, out.iter().copied().collect()),
        })
    }
}

impl SwiSlave for Recorder {
    fn token(&self, token: Token, low_ns: u64) {
        self.seen.lock().push((token, low_ns));
    }

    fn next_token(&self) -> Option<Token> {
        self.out.lock().pop_front()
    }

    fn peek_token(&self) -> Option<Token> {
        self.out.lock().front().copied()
    }
}

#[test]
fn the_token_values_are_the_data_sheet_s_table() {
    // DS40002025A Table 5-1.
    assert_eq!(Token::ONE, Token(0x7f));
    assert_eq!(Token::ZERO, Token(0x7d));
    assert_eq!(Token::WAKE, Token(0x00));
    assert_eq!(Token::ONE.bit(), Some(true));
    assert_eq!(Token::ZERO.bit(), Some(false));
    assert_eq!(Token::WAKE.bit(), None, "a wake carries no data bit");
    assert_eq!(Token::of(true), Token::ONE);
    assert_eq!(Token::of(false), Token::ZERO);
    // Bit 7 is not on the wire at 7N1 (DS40002249B §9.3.2 note 1).
    assert_eq!(Token(0xff).bit(), Some(true));
    // Everything else is reserved, and reaching this model is not an error —
    // the device has an answer to it (DS40002249B §8.3.1).
    assert_eq!(Token(0x55).bit(), None);
}

#[test]
fn the_flag_values_are_table_8_1() {
    // DS40002249B Table 8-1, which is also DS40002025A Table 5-2. Written out
    // one by one because a transposed pair here would be a protocol that looks
    // right and does the wrong thing.
    assert_eq!(Flag::COMMAND, Flag(0x77));
    assert_eq!(Flag::TRANSMIT, Flag(0x88));
    assert_eq!(Flag::IDLE, Flag(0xbb));
    assert_eq!(Flag::SLEEP, Flag(0xcc));
    assert_eq!(Flag::COMMAND.name(), "command");
    assert_eq!(Flag::TRANSMIT.name(), "transmit");
    assert_eq!(Flag::IDLE.name(), "idle");
    assert_eq!(Flag::SLEEP.name(), "sleep");
    assert_eq!(Flag(0x00).name(), "reserved");
}

#[test]
fn a_byte_goes_out_least_significant_bit_first() {
    // DS40002025A §5: "Flags are always transmitted LSb first."
    let tokens = tokens_of(0b1000_0001);
    assert_eq!(tokens[0], Token::ONE, "bit 0 leads");
    for token in &tokens[1..7] {
        assert_eq!(*token, Token::ZERO);
    }
    assert_eq!(tokens[7], Token::ONE, "bit 7 trails");
    assert_eq!(tokens_of(0x00), [Token::ZERO; 8]);
    assert_eq!(tokens_of(0xff), [Token::ONE; 8]);
}

#[test]
fn a_data_token_is_low_for_one_bit_time_and_a_wake_for_eight() {
    // The frame is start + seven data bits + stop (DS40002249B §9.3.2 note 1),
    // so the only long low pulse a 7N1 UART can make is 0x00's.
    assert_eq!(Token::ONE.low_bits(), 1, "the start pulse, and nothing else");
    assert_eq!(Token::ZERO.low_bits(), 1, "start, one high, then tZLO");
    assert_eq!(Token::WAKE.low_bits(), 8, "start plus seven zero data bits");

    // And the arithmetic the wake turns on: at the token rate a 0x00 is 34.7 µs
    // of low, which is under the 60 µs an ATECC wants, so a host drops its baud
    // rate to wake (DS40002249B §7.1.1).
    assert_eq!(Token::WAKE.low_ns(BAUD), 34_722);
    assert_eq!(Token::WAKE.low_ns(BAUD / 2), 69_444);
    assert_eq!(Token::ONE.low_ns(BAUD), 4_340);
    assert_eq!(Token::WAKE.low_ns(0), 0, "no rate, no frame");
}

#[test]
fn the_link_hands_the_device_each_frame_with_its_low_time() {
    let link = SwiLink::new();
    let part = Recorder::new(&[]);
    link.attach(Arc::clone(&part) as Arc<dyn SwiSlave>)
        .expect("an empty wire");
    assert!(link.is_attached());
    assert_eq!(link.baud(), BAUD, "the data sheet's token rate");

    link.send(Token::ONE);
    link.set_baud(BAUD / 2).expect("a real rate");
    link.send(Token::WAKE);

    let seen = part.seen.lock().clone();
    assert_eq!(
        seen,
        vec![(Token::ONE, 4_340), (Token::WAKE, 69_444)],
        "the wire carries the frame and how long it held the line low, and \
         nothing here knows what tWLO is"
    );
    assert_eq!(link.sent(), 2);
}

#[test]
fn a_second_part_on_one_wire_is_refused() {
    // There is no address on this wire (DS40002249B §8), so two parts could
    // not be told apart.
    let link = SwiLink::new();
    link.attach(Recorder::new(&[]) as Arc<dyn SwiSlave>)
        .expect("an empty wire");
    let err = link
        .attach(Recorder::new(&[]) as Arc<dyn SwiSlave>)
        .expect_err("one wire, one part");
    assert!(alloc::format!("{err}").contains("one device"), "{err}");
}

#[test]
fn a_baud_rate_of_zero_is_refused() {
    let link = SwiLink::new();
    link.set_baud(0).expect_err("a self-clocked wire needs one");
    assert_eq!(link.baud(), BAUD, "and the old rate stands");
}

#[test]
fn a_group_goes_out_and_comes_back_a_byte_at_a_time() {
    // The group is the I²C face's packet, byte for byte (DS40002249B §4.1).
    let group = [0x04u8, 0x11, 0x33, 0x43];
    let mut out = Vec::new();
    for byte in group {
        out.extend_from_slice(&tokens_of(byte));
    }
    let link = SwiLink::new();
    let part = Recorder::new(&out);
    link.attach(Arc::clone(&part) as Arc<dyn SwiSlave>)
        .expect("an empty wire");

    link.flag(Flag::TRANSMIT);
    assert!(link.pending(), "the part is driving");
    assert_eq!(link.read_group(), Some(group.to_vec()));
    assert!(!link.pending(), "and has stopped");
    assert_eq!(link.recv(), None);

    // The flag went out as eight tokens, LSb first, like any other byte.
    let seen = part.seen.lock().clone();
    assert_eq!(seen.len(), 8, "a flag is eight tokens (DS40002249B §8)");
    let flag: Vec<Token> = seen.iter().map(|(token, _)| *token).collect();
    assert_eq!(flag, tokens_of(Flag::TRANSMIT.0).to_vec());

    link.write_group(&group);
    let sent: Vec<Token> = part.seen.lock()[8..].iter().map(|(t, _)| *t).collect();
    assert_eq!(sent, out, "and so does every byte of a group");
}

#[test]
fn a_part_that_stops_driving_part_way_through_a_group_reads_as_nothing() {
    // §8.3.2: a device that is busy or out of synchronisation simply does not
    // answer, and a host's UART receives nothing.
    let mut out = tokens_of(0x04).to_vec();
    out.extend_from_slice(&tokens_of(0x11));
    let link = SwiLink::new();
    link.attach(Recorder::new(&out) as Arc<dyn SwiSlave>)
        .expect("an empty wire");
    assert_eq!(link.read_group(), None, "the count promised four bytes");
}

#[test]
fn an_unattached_wire_swallows_frames_and_answers_nothing() {
    let link = SwiLink::new();
    link.send(Token::ONE);
    assert_eq!(link.recv(), None);
    assert!(!link.pending());
    assert_eq!(link.sent(), 1, "the host still clocked a frame out");
}

#[test]
fn two_ends_meet_through_the_named_table() {
    let hosts = HostObjects::new();
    let a = links::open(&hosts, "swi0").expect("a fresh name");
    let b = links::open(&hosts, "swi0").expect("the same name");
    assert!(Arc::ptr_eq(&a, &b), "both ends land on one wire");
    let c = links::open(&hosts, "swi1").expect("another name");
    assert!(!Arc::ptr_eq(&a, &c));
    assert_eq!(links::names(&hosts), vec!["swi0", "swi1"]);
    assert!(links::get(&hosts, "swi0").expect("no clash").is_some());
    assert!(links::close(&hosts, "swi0"));
    assert!(links::get(&hosts, "swi0").expect("no clash").is_none());
}
