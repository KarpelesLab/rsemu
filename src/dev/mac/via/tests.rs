//! The 6522's own tests: what the data sheet says, asserted one rule at a
//! time, plus the decode the Macintosh board wraps it in.
//!
//! The register numbers are the chip's own, `$0` to `$F`; where a test uses a
//! byte *offset* instead it is saying something about the board's A9-A12
//! wiring rather than about the chip.

use super::*;
use alloc::string::ToString;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use alloc::vec::Vec;

/// Read a register through the address space, at the offset the board's decode
/// puts it at.
fn peek(via: &Via, index: u8) -> u8 {
    let mut byte = [0u8; 1];
    via.shared
        .read(
            u64::from(index) * REGISTER_STRIDE,
            &mut byte,
            MemAttrs::DEFAULT,
        )
        .expect("a byte read is legal");
    byte[0]
}

fn peek_debug(via: &Via, index: u8) -> u8 {
    let mut byte = [0u8; 1];
    via.shared
        .read(
            u64::from(index) * REGISTER_STRIDE,
            &mut byte,
            MemAttrs::DEBUG,
        )
        .expect("a byte read is legal");
    byte[0]
}

fn poke(via: &Via, index: u8, value: u8) {
    via.shared
        .write(
            u64::from(index) * REGISTER_STRIDE,
            &[value],
            MemAttrs::DEFAULT,
        )
        .expect("a byte write is legal");
}

// ---------------------------------------------------------------------------
// the decode
// ---------------------------------------------------------------------------

/// The board puts the register selects on A9-A12, so the sixteen registers are
/// 512 bytes apart and `$EFE1FE`'s low bits land on register 0.
#[test]
fn the_registers_are_five_hundred_and_twelve_bytes_apart() {
    assert_eq!(register_of(0), 0);
    assert_eq!(register_of(0x1fe), 0, "the odd byte of the same word");
    assert_eq!(register_of(0x200), 1);
    assert_eq!(register_of(0x1e00), 15, "vBufA, port A with no handshake");
    // And it repeats: the published base is $EFE1FE, which is offset $1FE of
    // the copy at $EFE000, and vBufA is $EFFFFE, offset $1FFE of the same one.
    assert_eq!(register_of(0xefe1fe % REGISTER_SPAN), 0);
    assert_eq!(register_of(0xeffffe % REGISTER_SPAN), 15);
    assert_eq!(REGISTER_SPAN, 16 * REGISTER_STRIDE);
}

/// A 6522 is an eight-bit part on one byte lane. A word access is not a thing
/// that can happen, and accepting one would invent a value for the other half.
#[test]
fn only_byte_accesses_are_accepted() {
    let via = Via::build();
    assert_eq!(
        via.shared.read(0, &mut [0u8; 2], MemAttrs::DEFAULT),
        Err(BusError::BadAccess)
    );
    assert_eq!(
        via.shared.write(0, &[0, 0], MemAttrs::DEFAULT),
        Err(BusError::BadAccess)
    );
    assert_eq!(via.shared.constraints().min, Width::U8);
    assert_eq!(via.shared.constraints().max, Width::U8);
}

// ---------------------------------------------------------------------------
// the ports
// ---------------------------------------------------------------------------

/// Out of reset every pin is an input, and an input with nothing driving it
/// reads as the pull-up that holds it — which is what puts the ROM at zero on
/// a Macintosh before any code has run.
#[test]
fn an_undriven_pin_reads_as_its_pull_up() {
    let via = Via::build();
    assert_eq!(peek(&via, R_DDRA), 0, "port A is all inputs");
    assert_eq!(peek(&via, R_ORA_NH), 0xff);
    assert!(via.overlay_bit(), "PA4 high is the ROM at zero");
}

/// An output bit reads back from the output register; an input bit reads the
/// pin. The Macintosh's port A is `$7F`: seven outputs and `SCCWREQ` in.
#[test]
fn the_direction_register_decides_which_half_of_a_port_a_read_sees() {
    let via = Via::build();
    poke(&via, R_DDRA, 0x7f);
    poke(&via, R_ORA_NH, 0x2b);
    // Bits 0-6 from ORA, bit 7 from the pin, which is pulled up.
    assert_eq!(peek(&via, R_ORA_NH), 0xab);
    assert!(!via.overlay_bit(), "PA4 is an output driving zero now");

    // And the other device's level on an input pin reaches the read.
    via.set_pa(7, false);
    assert_eq!(peek(&via, R_ORA_NH), 0x2b);
}

/// Reading `$1` clears the port A flags and reading `$F` does not — which is
/// exactly why a Macintosh uses `$F`, `vBufA`, for everything.
#[test]
fn the_no_handshake_address_leaves_the_flags_alone() {
    let via = Via::build();
    via.set_ca1(false); // a falling edge with PCR at 0
    assert_eq!(peek(&via, R_IFR) & IRQ_CA1, IRQ_CA1);
    assert_eq!(peek_debug(&via, R_ORA_NH), 0xff);
    assert_eq!(
        peek(&via, R_IFR) & IRQ_CA1,
        IRQ_CA1,
        "vBufA cleared nothing"
    );
    let _ = peek(&via, R_ORA);
    assert_eq!(peek(&via, R_IFR) & IRQ_CA1, 0, "vBufA-with-handshake did");
}

// ---------------------------------------------------------------------------
// the timers
// ---------------------------------------------------------------------------

/// "Writing the high order byte ... causes the counter to be loaded from the
/// latch and starts the timer." A one-shot flags once, at `N + 2` cycles, and
/// not again.
#[test]
fn timer_one_flags_once_in_one_shot_mode() {
    let via = Via::build();
    poke(&via, R_T1CL, 0x10);
    poke(&via, R_T1CH, 0x00); // N = 16
    assert_eq!(peek(&via, R_IFR) & IRQ_T1, 0, "not yet");
    assert_eq!(via.shared.state.lock().next_event(), 18);

    via.advance_to(17);
    assert_eq!(peek(&via, R_IFR) & IRQ_T1, 0, "one cycle short");
    via.advance_to(18);
    assert_eq!(peek(&via, R_IFR) & IRQ_T1, IRQ_T1);

    // Reading the low counter byte clears the flag, and a one-shot does not
    // set it again however long the machine runs.
    let _ = peek(&via, R_T1CL);
    assert_eq!(peek(&via, R_IFR) & IRQ_T1, 0);
    via.advance_to(1_000_000);
    assert_eq!(peek(&via, R_IFR) & IRQ_T1, 0, "a one-shot fires once");
}

/// Free-run mode reloads from the latch and keeps going, and a machine that
/// spends a whole second between two scheduler visits must not spend a second
/// in the catch-up: the flag is one bit, so the arithmetic jumps.
#[test]
fn timer_one_free_running_reloads_and_catches_up_in_one_step() {
    let via = Via::build();
    poke(&via, R_ACR, 0x40); // free-run
    poke(&via, R_T1CL, 0x63);
    poke(&via, R_T1CH, 0x00); // N = 99, so a period of 101
    via.advance_to(1_000_000);
    assert_eq!(peek(&via, R_IFR) & IRQ_T1, IRQ_T1);
    // The next underflow is still on the calendar, at the right tick.
    let next = via.shared.state.lock().next_event();
    assert!(next > 1_000_000 && next <= 1_000_000 + 101, "{next}");
}

/// Timer 2 in one-shot mode flags once and stays quiet until its high byte is
/// written again — the data sheet's "one-shot mode ... will not generate
/// another interrupt".
#[test]
fn timer_two_arms_only_on_a_write_of_its_high_byte() {
    let via = Via::build();
    poke(&via, R_T2CL, 0x20);
    poke(&via, R_T2CH, 0x00);
    via.advance_to(100);
    assert_eq!(peek(&via, R_IFR) & IRQ_T2, IRQ_T2);
    let _ = peek(&via, R_T2CL);
    assert_eq!(peek(&via, R_IFR) & IRQ_T2, 0);
    via.advance_to(1_000_000);
    assert_eq!(peek(&via, R_IFR) & IRQ_T2, 0);
    poke(&via, R_T2CH, 0x00);
    via.advance_to(1_000_100);
    assert_eq!(peek(&via, R_IFR) & IRQ_T2, IRQ_T2, "re-armed");
}

// ---------------------------------------------------------------------------
// interrupts
// ---------------------------------------------------------------------------

/// "Bit 7 of the data written determines whether the mask bits are set or
/// cleared", and IER always reads back with bit 7 set.
#[test]
fn the_enable_register_is_set_or_cleared_by_bit_seven() {
    let via = Via::build();
    poke(&via, R_IER, 0x80 | IRQ_CA1 | IRQ_T1);
    assert_eq!(peek(&via, R_IER), 0x80 | IRQ_CA1 | IRQ_T1);
    poke(&via, R_IER, IRQ_T1); // bit 7 clear: clear these
    assert_eq!(peek(&via, R_IER), 0x80 | IRQ_CA1);
}

/// IFR's bit 7 is not stored: it is the OR of every enabled flag, and the
/// interrupt output follows it.
#[test]
fn the_flag_register_ors_its_enabled_sources_into_bit_seven() {
    let via = Via::build();
    via.set_ca1(false);
    assert_eq!(peek(&via, R_IFR), IRQ_CA1, "flagged but not enabled");
    assert!(!via.irq());
    poke(&via, R_IER, 0x80 | IRQ_CA1);
    assert_eq!(peek(&via, R_IFR), IRQ_ANY | IRQ_CA1);
    assert!(via.irq());
    // A one in a write to IFR clears that flag.
    poke(&via, R_IFR, IRQ_CA1);
    assert_eq!(peek(&via, R_IFR), 0);
    assert!(!via.irq());
}

/// PCR picks the edge each handshake pin latches on. With PCR at zero — which
/// is what a Macintosh's ROM leaves it at — it is the falling edge.
#[test]
fn the_peripheral_control_register_picks_the_edge() {
    let via = Via::build();
    via.set_ca1(true);
    assert_eq!(peek(&via, R_IFR) & IRQ_CA1, 0, "a rise, with PCR 0");
    via.set_ca1(false);
    assert_eq!(peek(&via, R_IFR) & IRQ_CA1, IRQ_CA1);

    poke(&via, R_IFR, IRQ_CA1);
    poke(&via, R_PCR, 0x01); // positive edge
    via.set_ca1(true);
    assert_eq!(peek(&via, R_IFR) & IRQ_CA1, IRQ_CA1);
}

// ---------------------------------------------------------------------------
// the invariants every device has
// ---------------------------------------------------------------------------

/// Invariant 5: a debugger read must not clear a flag or advance a counter,
/// and a debug write is refused rather than guessed at.
#[test]
fn a_debug_access_changes_nothing() {
    let via = Via::build();
    via.set_ca1(false);
    poke(&via, R_T1CL, 0xff);
    poke(&via, R_T1CH, 0xff);

    assert_eq!(peek_debug(&via, R_IFR) & IRQ_CA1, IRQ_CA1);
    assert_eq!(peek_debug(&via, R_ORA), 0xff);
    assert_eq!(
        peek(&via, R_IFR) & IRQ_CA1,
        IRQ_CA1,
        "the debug read of vBufA left the flag"
    );
    let ticks = via.ticks();
    let _ = peek_debug(&via, R_T1CL);
    assert_eq!(via.ticks(), ticks, "a debug read advanced nothing");

    assert_eq!(
        via.shared.write(0, &[0x00], MemAttrs::DEBUG),
        Err(BusError::BadAccess)
    );
}

/// Reset clears every register but leaves the levels other devices are
/// driving, which are theirs.
#[test]
fn reset_clears_the_registers_and_keeps_the_pins() {
    let via = Via::build();
    via.set_pa(7, false);
    poke(&via, R_DDRA, 0x7f);
    poke(&via, R_ORA_NH, 0x00);
    assert!(!via.overlay_bit());

    via.reset(ResetKind::Cold);
    assert_eq!(peek(&via, R_DDRA), 0);
    assert_eq!(peek(&via, R_ACR), 0);
    assert_eq!(peek(&via, R_IER), 0x80);
    assert!(via.overlay_bit(), "port A is inputs again, so PA4 pulls up");
    // PA7 is still what the other device is driving.
    assert_eq!(peek(&via, R_ORA_NH) & 0x80, 0);
}

/// Invariant 6: `save` and `load` agree, and the restored chip is the same one
/// bit for bit.
#[test]
fn a_snapshot_round_trips_to_an_identical_state_hash() {
    let saved = Via::build();
    saved.poke(R_DDRB, 0x87);
    saved.poke(R_ORB, 0x3f);
    saved.poke(R_ACR, 0x40);
    saved.poke(R_T1CL, 0x34);
    saved.poke(R_T1CH, 0x12);
    saved.poke(R_IER, 0x80 | IRQ_CA1 | IRQ_T1);
    saved.set_ca1(false);
    saved.advance_to(5_000);

    let image = |via: &Via| -> Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("via", CLASS_NAME).expect("a fresh shape");
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("via", CLASS_NAME, STATE_VERSION).expect("a chunk");
            Device::save(via, &mut chunk).expect("a 6522 saves");
        }
        w.to_vec().expect("a complete image")
    };
    let first = image(&saved);

    let restored = Via::build();
    let reader = StateReader::new(&first).expect("a well-formed image");
    let chunk = reader
        .load("via", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .expect("the chunk is there");
    Device::load(&restored, &mut chunk.reader()).expect("a 6522 loads");

    assert_eq!(image(&restored), first, "the same chip, bit for bit");
    // And every guest-visible register agrees, read the way the guest would.
    let before: Vec<u8> = (0..16).map(|r| peek_debug(&saved, r)).collect();
    let after: Vec<u8> = (0..16).map(|r| peek_debug(&restored, r)).collect();
    assert_eq!(before, after);

    // Run both on and they stay identical: the timers were restored running.
    saved.advance_to(20_000);
    restored.advance_to(20_000);
    assert_eq!(image(&restored), image(&saved));
}

/// The class registers, describes itself, and the schema matches the pins the
/// device really answers on.
#[test]
fn the_class_is_registrable_and_its_schema_matches() {
    let mut registry = crate::core::Registry::new();
    register(&mut registry).expect("a fresh registry");
    assert!(register(&mut registry).is_err(), "twice is an error");
    let class = registry.get(CLASS_NAME).expect("registered");
    assert_eq!(class.version, STATE_VERSION);
    let device = (class.construct)(&Props::new()).expect("no properties needed");
    assert_eq!(device.class().name, CLASS_NAME);

    let schema = schema();
    for pin in ["irq", "ca1", "ca2", "cb1", "cb2", "pa4", "pb7"] {
        assert!(schema.port_named(pin).is_some(), "{pin} is in the schema");
    }
    let via = Via::build();
    for pin in ["ca1", "cb1", "pa0", "pb7"] {
        assert!(via.sink(pin, &[]).is_some(), "{pin} is a real sink");
    }
    assert!(via.sink("pa8", &[]).is_none(), "there is no PA8");
    assert!(via.region("").is_some() && via.region("regs").is_some());
    assert!(via.region("timers").is_none());

    // A property nothing here accepts is refused rather than ignored.
    let err = Via::new(&Props::new().with("clockk", 1u64))
        .expect_err("a typo")
        .to_string();
    assert!(err.contains("clockk"), "{err}");
}
