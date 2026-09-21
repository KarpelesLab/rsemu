//! The SCC's own tests: the decode, the one register pointer, and what a
//! chip with nothing plugged into it reports.

use super::*;
use alloc::string::ToString;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

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

/// A carrier-detect transition is an external status change: it latches `RR0`
/// and raises the interrupt while `WR15` and `WR1` enable it, and `Reset
/// Ext/Status Interrupts` releases both.
#[test]
fn a_carrier_transition_raises_an_external_status_interrupt() {
    let scc = Scc::build();
    // WR15 bit 3, then WR1 bit 0.
    scc.poke(A, false, 0x08 | 7); // point high, 7 -> WR15
    scc.poke(A, false, WR15_DCD_IE);
    scc.poke(A, false, 1);
    scc.poke(A, false, WR1_EXT_IE);
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
