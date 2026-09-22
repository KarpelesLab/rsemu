//! The SCC's own tests: the decode, the one register pointer, and what a
//! chip with nothing plugged into it reports.

use super::*;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use alloc::string::ToString;

/// Channel A is 0 and channel B is 1 throughout, which is the chip's own
/// numbering; the *addresses* are the other way round and that is what the
/// decode test is about.
const A: usize = 0;
const B: usize = 1;

fn read_at(scc: &Scc, region: &str, offset: u64) -> u8 {
    let mut byte = [0u8; 1];
    let region = Device::region(scc, region).expect("the window");
    match region.kind() {
        crate::core::space::RegionKind::Io(ops) => {
            ops.read(offset, &mut byte, MemAttrs::DEFAULT)
                .expect("a byte read is legal");
        }
        _ => unreachable!("the windows are I/O regions"),
    }
    byte[0]
}

/// `A1` picks the channel and `A2` picks data over control, in both windows.
/// `$9FFFF8` is channel B's control address and `$BFFFF9` is where it is
/// written; both land on the same select.
#[test]
fn the_decode_is_a1_and_a2_in_both_windows() {
    let scc = Scc::build();
    // Set the pointer to 4 through the write window's channel B control
    // address, which is offset 1 of a copy.
    scc.poke(B, false, 4);
    assert_eq!(scc.pointer(), 4);
    // And read it back through the read window's channel B control address,
    // which is offset 0. RR4 mirrors WR4, which is still zero.
    assert_eq!(read_at(&scc, "read", 0), 0);
    assert_eq!(scc.pointer(), 0, "a control access consumes the pointer");

    // The published addresses land where they should once the window repeats
    // every eight bytes.
    assert_eq!(0x9f_fff8 % WINDOW_SPAN, 0, "B control, read");
    assert_eq!(0x9f_fffa % WINDOW_SPAN, 2, "A control, read");
    assert_eq!(0x9f_fffc % WINDOW_SPAN, 4, "B data, read");
    assert_eq!(0x9f_fffe % WINDOW_SPAN, 6, "A data, read");
    assert_eq!(0xbf_fff9 % WINDOW_SPAN, 1, "B control, write");
    assert_eq!(0xbf_ffff % WINDOW_SPAN, 7, "A data, write");
}

/// "The pointer resets to zero after the read or write": a control write with
/// the pointer at zero sets it, and the next control access uses it.
#[test]
fn the_register_pointer_is_set_then_consumed() {
    let scc = Scc::build();
    scc.poke(A, false, 12); // point at WR12
    assert_eq!(scc.pointer(), 12);
    scc.poke(A, false, 0x5a); // write it
    assert_eq!(scc.pointer(), 0, "and the pointer is back to zero");
    scc.poke(A, false, 12);
    assert_eq!(scc.peek(A, false), 0x5a, "RR12 mirrors WR12");
}

/// The read and the write register files are **different files**, and
/// [`Scc::write_register`] is the only way to see the second one.
///
/// `RR1` is a computed status byte with nothing to do with `WR1`, so a test
/// that read it back through [`Scc::peek`] would be reading a constant and
/// calling it configuration — which is exactly the mistake that made this
/// accessor necessary.
#[test]
fn the_write_registers_are_not_what_the_read_side_answers() {
    let scc = Scc::build();
    scc.poke(A, false, 1); // point at WR1
    scc.poke(A, false, 0x01); // external/status interrupts on
    assert_eq!(scc.write_register(A, 1), 0x01);
    scc.poke(A, false, 1);
    assert_eq!(scc.peek(A, false), 0x06, "RR1 is `All Sent`, not WR1");
    // And the two channels keep their own copies of everything but `WR9`.
    assert_eq!(scc.write_register(B, 1), 0x00);
}

/// `Point High` — command 1 — adds eight to the register number, which is how
/// the top half of the register file is reached at all.
#[test]
fn point_high_reaches_the_upper_registers() {
    let scc = Scc::build();
    scc.poke(A, false, 0x08 | 5); // point high, register 5 -> 13
    assert_eq!(scc.pointer(), 13);
    scc.poke(A, false, 0x33);
    scc.poke(A, false, 0x08 | 5);
    assert_eq!(scc.peek(A, false), 0x33, "RR13 mirrors WR13");
}

/// `RR0` on a chip with nothing plugged in: the transmitter is always empty
/// because nothing here ever holds a character, and nothing ever arrives.
#[test]
fn the_status_register_reports_an_idle_transmitter() {
    let scc = Scc::build();
    let rr0 = scc.peek(A, false);
    assert_eq!(rr0 & RR0_TX_EMPTY, RR0_TX_EMPTY);
    assert_eq!(rr0 & RR0_CTS, RR0_CTS);
    assert_eq!(rr0 & 1, 0, "no character available");
    assert_eq!(rr0 & RR0_DCD, 0, "no carrier: the pin is at its pull-up");
    // And a character handed to it leaves immediately.
    scc.poke(A, true, b'x');
    assert_eq!(scc.peek(A, false) & RR0_TX_EMPTY, RR0_TX_EMPTY);
    assert_eq!(scc.peek(A, true), 0, "and nothing ever arrives");
}

/// Arm a channel the way a Macintosh Plus ROM leaves the chip: `WR15 = $08`
/// so a `DCD` transition counts as an external status change, `WR1 = $01` so
/// the channel asks for one, and `WR9 = $0A` so the chip is allowed to pull
/// `/INT` at all.
fn arm(scc: &Scc, c: usize) {
    scc.poke(c, false, 0x08 | 7); // point high, 7 -> WR15
    scc.poke(c, false, WR15_DCD_IE);
    scc.poke(c, false, 1);
    scc.poke(c, false, WR1_EXT_IE);
    scc.poke(c, false, 0x08 | 1); // point high, 1 -> WR9
    scc.poke(c, false, WR9_MIE | WR9_VIS);
}

/// A carrier-detect transition is an external status change: it latches `RR0`
/// and raises the interrupt while `WR15`, `WR1` and `WR9` enable it, and
/// `Reset Ext/Status Interrupts` releases both.
#[test]
fn a_carrier_transition_raises_an_external_status_interrupt() {
    let scc = Scc::build();
    arm(&scc, A);
    assert!(!scc.irq());

    scc.set_dcd(A, false); // the far end asserts /DCD
    assert!(scc.irq(), "the mouse's phase reaches the processor");
    assert_eq!(scc.peek(A, false) & RR0_DCD, RR0_DCD);
    // RR3 on channel A names which channel is asking.
    scc.poke(A, false, 3);
    assert_eq!(scc.peek(A, false), 1 << 3);

    scc.poke(A, false, 2 << 3); // Reset Ext/Status Interrupts
    assert!(!scc.irq());
}

/// **The `/INT` pin follows the reset command, not only the state behind it.**
///
/// This is the regression for the defect that locked a Macintosh Plus up the
/// moment anything drove a carrier detect: `Reset Ext/Status Interrupts` is a
/// control write with the register pointer at zero, and that branch of the
/// write path returned before the chip re-announced its output. Every
/// register the ROM could read said the interrupt was gone; the wire into
/// `IPL1` still said it was there, so the handler ran, did exactly what the
/// manual asks, returned, and was entered again for ever.
///
/// So this test watches the **net**, which is the only place the fault was
/// visible. `Scc::irq` reads the same thing now, but a device that publishes
/// its output through a wire is only correct if the wire moves.
#[test]
fn the_interrupt_pin_drops_when_the_reset_command_is_written() {
    let scc = Scc::build();
    let ids = crate::core::wire::WireIdAllocator::new();
    let id = ids.alloc();
    let wire = crate::core::wire::Wire::builder().source(id).build_shared();
    Device::connect(
        &scc,
        IRQ_PIN,
        WireSource::new(alloc::sync::Arc::clone(&wire), id),
    )
    .expect("the chip has an `irq` output");
    assert_eq!(
        wire.level_of(id).expect("the source is on the net"),
        Level::Low,
        "nothing is asking yet"
    );

    arm(&scc, B);
    scc.set_dcd(B, false);
    assert_eq!(
        wire.level_of(id).expect("the source is on the net"),
        Level::High,
        "a carrier change pulls /INT"
    );

    // Exactly what the ROM's handler does: read RR0, then issue command 2.
    let rr0 = scc.peek(B, false);
    assert_eq!(rr0 & RR0_DCD, RR0_DCD, "RR0 latched the new level");
    scc.poke(B, false, 2 << 3);
    assert_eq!(
        wire.level_of(id).expect("the source is on the net"),
        Level::Low,
        "and /INT lets go, or the processor never leaves the handler"
    );
}

/// **`RR0` reports what the latches caught, not what the pin is doing now**,
/// and a transition that arrives while they are shut is counted *late* rather
/// than never.
///
/// *Am8530H/Am85C30* technical manual §3.8.6 ("Data Carrier Detect") — AMD's
/// edition of this part's manual, which prints whole the sentence Zilog's own
/// printing breaks off in the middle of:
///
/// > "The DCD Status bit reports the state of the DCD input pin the last time
/// > any of the enabled External/Status bits changed. Any transition on the DCD
/// > pin, while no other interrupts are pending, latches the state of the DCD
/// > pin and generates an External/Status interrupt if the DCD IE bit in WR15
/// > is set to '1'. However, only an odd number of transitions on the DCD pin
/// > while another External/Status is pending will cause an External/Status
/// > interrupt after the Reset External/Status Interrupt command is issued."
///
/// > "Note that after the Reset External/Status Interrupt command is issued, if
/// > the latches were closed, they will close again if there was an odd number
/// > of transitions on the DCD pin; they will remain open if there was an even
/// > number of transitions on the input pin."
///
/// So one extra transition inside a handler is one more interrupt after the
/// acknowledgement, which is why a fast mouse does not lose counts on this
/// chip. `tests/mac_plus.rs` counts it on the assembled board.
#[test]
fn a_transition_inside_the_handler_is_counted_after_the_reset() {
    let scc = Scc::build();
    arm(&scc, A);
    scc.set_dcd(A, false);
    assert!(scc.irq());
    assert_eq!(scc.peek(A, false) & RR0_DCD, RR0_DCD, "the latched level");

    // A second transition, with the first still unacknowledged. The pin is
    // high again; `RR0` must still read what the latch caught, or the handler
    // cannot tell which condition changed.
    scc.set_dcd(A, true);
    assert_eq!(
        scc.peek(A, false) & RR0_DCD,
        RR0_DCD,
        "the latches are shut, so `RR0` still reports the level they caught"
    );
    assert!(scc.irq(), "and the chip is still asking");

    // The handler acknowledges. One transition since the latches closed is an
    // odd number, so they close again on the new level and the chip asks once
    // more: the edge arrived late, not never.
    scc.poke(A, false, 2 << 3);
    assert!(
        scc.irq(),
        "an odd number of transitions re-closes the latches"
    );
    assert_eq!(
        scc.peek(A, false) & RR0_DCD,
        0,
        "and `RR0` now reports the level the second transition left"
    );

    // The second acknowledgement finds pin and latch agreeing, so they stay
    // open and the chip lets go.
    scc.poke(A, false, 2 << 3);
    assert!(!scc.irq());
    let counters = scc.counters();
    assert_eq!(counters.dcd_edges[A], 2, "two transitions arrived");
    assert_eq!(
        counters.ext_latches[A], 2,
        "and each one closed the latches once — the second at the reset"
    );
    assert_eq!(counters.ext_resets[A], 2, "two acknowledgements");
    assert_eq!(counters.int_assertions, 1, "`/INT` never let go in between");
}

/// An **even** number of transitions inside the handler is no further
/// interrupt, because the pin is back where the latch caught it and the chip
/// has nothing to report. *Am8530H/Am85C30* §3.8.6, quoted above: "they will
/// remain open if there was an even number of transitions on the input pin."
///
/// This is a real loss of two counts, and it is the chip's — but it needs two
/// transitions inside one service, which on this board takes a mouse moving
/// some sixteen thousand counts a second.
#[test]
fn an_even_number_of_transitions_inside_the_handler_raises_nothing_more() {
    let scc = Scc::build();
    arm(&scc, B);
    scc.set_dcd(B, false);
    assert!(scc.irq());
    scc.set_dcd(B, true);
    scc.set_dcd(B, false);
    assert_eq!(
        scc.peek(B, false) & RR0_DCD,
        RR0_DCD,
        "the latched level, which is also where the pin ended up"
    );
    scc.poke(B, false, 2 << 3);
    assert!(!scc.irq(), "an even number leaves the latches open");
    let counters = scc.counters();
    assert_eq!(counters.dcd_edges[1], 3);
    assert_eq!(counters.ext_latches[1], 1, "one interrupt for three edges");
}

/// With `WR15`'s `DCD IE` clear the latch is out of the signal path, so `RR0`
/// is the live pin and a transition raises nothing.
///
/// Zilog's *SCC/ESCC User Manual*: "If the individual enable is set to 0, then
/// RR0 reflects the current unlatched status, and if the individual enable is
/// set to 1, then RR0 reflects the latched status." And *Am8530H/Am85C30*
/// §3.8: "An interrupt source whose individual enable bit in WR15 is set to
/// '0' is not a source of External/Status interrupts even though the
/// External/Status Master Interrupt Enable bit is set to '1' in WR1 (D0)."
#[test]
fn with_the_individual_enable_clear_rr0_is_the_live_pin() {
    let scc = Scc::build();
    arm(&scc, A);
    scc.poke(A, false, 0x08 | 7); // point high, 7 -> WR15
    scc.poke(A, false, 0); // no DCD IE
    scc.set_dcd(A, false);
    assert!(
        !scc.irq(),
        "not a source of interrupts with its enable clear"
    );
    assert_eq!(scc.peek(A, false) & RR0_DCD, RR0_DCD, "the current status");
    scc.set_dcd(A, true);
    assert_eq!(scc.peek(A, false) & RR0_DCD, 0, "which follows the pin");
    assert_eq!(scc.counters().ext_latches[A], 0);
}

/// `WR9`'s Master Interrupt Enable gates the pin and nothing else: with it
/// clear the pending bit still sets and `RR3` still shows it, but the chip
/// does not ask.
#[test]
fn the_master_interrupt_enable_gates_the_pin() {
    let scc = Scc::build();
    // Everything but `WR9`.
    scc.poke(A, false, 0x08 | 7);
    scc.poke(A, false, WR15_DCD_IE);
    scc.poke(A, false, 1);
    scc.poke(A, false, WR1_EXT_IE);

    scc.set_dcd(A, false);
    assert!(!scc.irq(), "MIE is clear, so /INT stays put");
    scc.poke(A, false, 3);
    assert_eq!(scc.peek(A, false), 1 << 3, "but the pending bit is set");

    scc.poke(A, false, 0x08 | 1); // point high, 1 -> WR9
    scc.poke(A, false, WR9_MIE);
    assert!(scc.irq(), "and enabling it asks straight away");
}

/// `RR2` read on channel B carries the status code; on channel A it is the
/// vector as written.
#[test]
fn the_vector_is_modified_only_on_channel_b() {
    let scc = Scc::build();
    scc.poke(A, false, 2);
    scc.poke(A, false, 0x40); // WR2 = $40
    scc.poke(B, false, 2);
    scc.poke(B, false, 0x40);
    // WR9 bit 1: include the status.
    scc.poke(A, false, 0x08 | 1); // point high, 1 -> WR9
    scc.poke(A, false, WR9_VIS);

    scc.poke(A, false, 2);
    assert_eq!(scc.peek(A, false), 0x40, "channel A reads it raw");
    scc.poke(B, false, 2);
    // No interrupt pending is status 011, in bits 3-1.
    assert_eq!(scc.peek(B, false), 0x40 | (0b011 << 1));
}

/// `WR9`'s top two bits reset a channel or the whole chip, and what the far
/// end is driving survives it.
#[test]
fn the_reset_command_clears_the_registers_and_keeps_the_pins() {
    let scc = Scc::build();
    scc.set_dcd(B, false);
    scc.poke(A, false, 4);
    scc.poke(A, false, 0x44);
    scc.poke(A, false, 0x08 | 1); // WR9
    scc.poke(A, false, WR9_RESET); // hardware reset
    scc.poke(A, false, 4);
    assert_eq!(scc.peek(A, false), 0, "WR4 is gone");
    assert_eq!(scc.pointer(), 0);
    scc.poke(B, false, 0);
    assert_eq!(
        scc.peek(B, false) & RR0_DCD,
        RR0_DCD,
        "what the mouse is driving is the mouse's"
    );
}

/// Invariant 5: a debug read leaves the pointer alone, and a debug write is
/// refused because every control address moves it.
#[test]
fn a_debug_access_changes_nothing() {
    let scc = Scc::build();
    scc.poke(A, false, 12);
    let region = Device::region(&scc, "read").expect("the read window");
    let crate::core::space::RegionKind::Io(ops) = region.kind() else {
        unreachable!()
    };
    let mut byte = [0u8; 1];
    ops.read(2, &mut byte, MemAttrs::DEBUG)
        .expect("a byte read");
    assert_eq!(scc.pointer(), 12, "the pointer did not move");
    assert_eq!(
        ops.write(2, &[0], MemAttrs::DEBUG),
        Err(BusError::BadAccess)
    );
}

/// An eight-bit part on one byte lane: a word access is refused rather than
/// given an invented value for the other half.
#[test]
fn only_byte_accesses_are_accepted() {
    let scc = Scc::build();
    let region = Device::region(&scc, "read").expect("the read window");
    let crate::core::space::RegionKind::Io(ops) = region.kind() else {
        unreachable!()
    };
    assert_eq!(
        ops.read(0, &mut [0u8; 2], MemAttrs::DEFAULT),
        Err(BusError::BadAccess)
    );
    assert_eq!(ops.constraints().max, Width::U8);
}

/// Invariant 6.
#[test]
fn a_snapshot_round_trips_to_an_identical_state_hash() {
    let saved = Scc::build();
    saved.poke(A, false, 4);
    saved.poke(A, false, 0x44);
    saved.poke(B, false, 3);
    saved.poke(B, false, 0xc1);
    saved.poke(A, false, 12); // leave the pointer somewhere

    let image = |scc: &Scc| -> alloc::vec::Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("scc", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("scc", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(scc, &mut chunk).unwrap();
        }
        w.to_vec().unwrap()
    };
    let first = image(&saved);

    let restored = Scc::build();
    let reader = StateReader::new(&first).unwrap();
    let chunk = reader
        .load("scc", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(&restored, &mut chunk.reader()).unwrap();
    assert_eq!(image(&restored), first, "the same chip, bit for bit");
    assert_eq!(restored.pointer(), 12);
}

/// The class registers and names its two windows and three pins.
#[test]
fn the_class_is_registrable_and_its_schema_matches() {
    let mut registry = crate::core::Registry::new();
    register(&mut registry).expect("a fresh registry");
    assert_eq!(registry.get(CLASS_NAME).unwrap().version, STATE_VERSION);
    let scc = Scc::build();
    assert!(scc.region("").is_some() && scc.region("read").is_some());
    assert!(scc.region("write").is_some());
    assert!(scc.region("both").is_none());
    assert!(scc.sink(DCDA_PIN, &[]).is_some() && scc.sink(DCDB_PIN, &[]).is_some());
    assert!(scc.sink("rxa", &[]).is_none());
    let schema = schema();
    for pin in [IRQ_PIN, DCDA_PIN, DCDB_PIN] {
        assert!(schema.port_named(pin).is_some(), "{pin}");
    }
    let err = Scc::new(&Props::new().with("baud", 9600u64))
        .expect_err("a property this class does not know")
        .to_string();
    assert!(err.contains("baud"), "{err}");
}
