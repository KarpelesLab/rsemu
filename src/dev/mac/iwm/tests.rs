//! The IWM's own tests: the soft switches, the status register, and the
//! drive's own register file.

use super::*;
use alloc::string::ToString;
use crate::core::props::Value;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

/// Set the switch `index` names and take whatever the read returns.
fn touch(iwm: &Iwm, index: u8) -> u8 {
    iwm.peek(index)
}

/// Put the four `CA`/`SEL` lines where a drive register address needs them.
fn address(iwm: &Iwm, addr: u8) {
    touch(iwm, if addr & 2 != 0 { 1 } else { 0 }); // CA0
    touch(iwm, if addr & 4 != 0 { 3 } else { 2 }); // CA1
    touch(iwm, if addr & 8 != 0 { 5 } else { 4 }); // CA2
    iwm.set_sel(addr & 1 != 0);
}

/// Read the drive status line `addr` selects, through the status register.
fn sense(iwm: &Iwm, addr: u8) -> bool {
    address(iwm, addr);
    touch(iwm, 13); // Q6 on
    touch(iwm, 14); // Q7 off — the status register
    touch(iwm, 14) & STATUS_SENSE != 0
}

/// Pulse `LSTRB` over the write register `addr` with `data` on `CA2`.
fn control(iwm: &Iwm, addr: u8, data: bool) {
    touch(iwm, if addr & 1 != 0 { 1 } else { 0 }); // CA0
    touch(iwm, if addr & 2 != 0 { 3 } else { 2 }); // CA1
    touch(iwm, if data { 5 } else { 4 }); // CA2 carries the data
    touch(iwm, 6); // LSTRB low
    touch(iwm, 7); // and the rising edge latches it
    touch(iwm, 6);
}

/// The board puts the switches 512 bytes apart on A9-A12, and the published
/// base `$DFE1FF` is switch 0 of a copy.
#[test]
fn the_switches_are_five_hundred_and_twelve_bytes_apart() {
    assert_eq!(REGISTER_SPAN, 16 * REGISTER_STRIDE);
    assert_eq!((0xdf_e1ff % REGISTER_SPAN) / REGISTER_STRIDE, 0);
    assert_eq!((0xdf_e3ff % REGISTER_SPAN) / REGISTER_STRIDE, 1);
    assert_eq!((0xdf_ffff % REGISTER_SPAN) / REGISTER_STRIDE, 15);
}

/// **Every access is a switch**, read or write — which is why a ROM drives
/// this chip almost entirely with reads.
#[test]
fn a_read_moves_the_switch_it_names() {
    let iwm = Iwm::with_drives([true, false]);
    assert_eq!(iwm.switches(), 0);
    touch(&iwm, 1); // CA0 on
    assert_eq!(iwm.switches() & SW_CA0, SW_CA0);
    touch(&iwm, 0); // CA0 off
    assert_eq!(iwm.switches() & SW_CA0, 0);
    touch(&iwm, 9); // the motor
    assert_eq!(iwm.switches() & SW_ENABLE, SW_ENABLE);
    touch(&iwm, 11); // the second drive
    assert_eq!(iwm.switches() & SW_SELECT, SW_SELECT);
    // And a *write* moves one too.
    iwm.poke(10, 0);
    assert_eq!(iwm.switches() & SW_SELECT, 0);
}

/// `Q7:Q6` choose what a read of any of the sixteen addresses returns.
#[test]
fn q6_and_q7_choose_which_register_a_read_answers_from() {
    let iwm = Iwm::with_drives([true, false]);
    // 1 1: a write here loads the mode register.
    touch(&iwm, 13);
    touch(&iwm, 15);
    iwm.poke(15, 0x1f);
    assert_eq!(iwm.mode(), 0x1f);

    // 0 1: the status register, whose low five bits mirror the mode.
    touch(&iwm, 14);
    assert_eq!(touch(&iwm, 14) & 0x1f, 0x1f);

    // 1 0: the write handshake, always ready and never underrun.
    touch(&iwm, 12);
    touch(&iwm, 15);
    assert_eq!(
        touch(&iwm, 15) & (HANDSHAKE_READY | HANDSHAKE_NO_UNDERRUN),
        HANDSHAKE_READY | HANDSHAKE_NO_UNDERRUN
    );
}

/// The enable switch shows up as bit 5 of the status register.
#[test]
fn the_status_register_reports_the_drive_enable() {
    let iwm = Iwm::with_drives([true, false]);
    touch(&iwm, 13); // Q6 on
    touch(&iwm, 14); // Q7 off
    assert_eq!(touch(&iwm, 14) & STATUS_ENABLE, 0);
    touch(&iwm, 9); // the motor
    assert_eq!(touch(&iwm, 14) & STATUS_ENABLE, STATUS_ENABLE);
}

/// An empty drive says so on `/CSTIN`, and a disk put in says so too — which
/// is the one line that decides whether a ROM looks for a boot block at all.
#[test]
fn the_drive_reports_whether_a_disk_is_in_it() {
    let iwm = Iwm::with_drives([true, false]);
    assert!(sense(&iwm, 1), "/CSTIN high: nothing in the drive");
    iwm.set_disk(0, true, false);
    assert!(!sense(&iwm, 1), "/CSTIN low: a disk is in place");
    assert!(sense(&iwm, 3), "/WRTPRT high: it is writable");
    iwm.set_disk(0, true, true);
    assert!(!sense(&iwm, 3), "/WRTPRT low: it is protected");
}

/// A cable position with nothing on it lets go, and every line reads as the
/// pull-up that holds it.
#[test]
fn an_empty_cable_position_reads_as_its_pull_ups() {
    let iwm = Iwm::with_drives([true, false]);
    touch(&iwm, 11); // select drive 2, which is not there
    for addr in 0..16u8 {
        assert!(
            sense(&iwm, addr),
            "line {addr} of a drive that is not there"
        );
    }
}

/// Stepping: register 0 sets the direction and register 1 issues one step,
/// and `/TK0` says when the head is home.
#[test]
fn the_head_steps_in_the_direction_the_drive_was_told() {
    let iwm = Iwm::with_drives([true, false]);
    assert_eq!(iwm.track(0), 0);
    assert!(!sense(&iwm, 5), "/TK0 low: the head is at track 0");

    control(&iwm, 0, false); // toward track 79
    for _ in 0..3 {
        control(&iwm, 1, false);
    }
    assert_eq!(iwm.track(0), 3);
    assert!(sense(&iwm, 5), "/TK0 high: it has left track 0");

    control(&iwm, 0, true); // toward track 0
    control(&iwm, 1, false);
    assert_eq!(iwm.track(0), 2);

    // And it stops at either end rather than walking off the mechanism.
    control(&iwm, 0, false);
    for _ in 0..200 {
        control(&iwm, 1, false);
    }
    assert_eq!(iwm.track(0), MAX_TRACK);
}

/// The motor and the eject are the other two write registers.
#[test]
fn the_motor_and_the_eject_are_write_registers() {
    let iwm = Iwm::with_drives([true, false]);
    iwm.set_disk(0, true, false);
    assert!(!iwm.motor(0));
    control(&iwm, 2, false); // motor on
    assert!(iwm.motor(0));
    assert!(!sense(&iwm, 4), "MOTORON is asserted low while it runs");
    assert!(!sense(&iwm, 11), "/READY: a spinning disk is ready");
    control(&iwm, 2, true); // motor off
    assert!(!iwm.motor(0));

    control(&iwm, 3, false); // eject
    assert!(!iwm.has_disk(0));
    assert!(sense(&iwm, 6), "SWITCHED: a disk has been changed");
}

/// Invariant 5: a debug read must not move a switch, and there is no harmless
/// debug write because every address is one.
#[test]
fn a_debug_access_changes_nothing() {
    let iwm = Iwm::with_drives([true, false]);
    let mut byte = [0u8; 1];
    iwm.shared
        .read(REGISTER_STRIDE, &mut byte, MemAttrs::DEBUG)
        .expect("a byte read");
    assert_eq!(iwm.switches(), 0, "CA0 did not move");
    assert_eq!(
        iwm.shared.write(REGISTER_STRIDE, &[0], MemAttrs::DEBUG),
        Err(BusError::BadAccess)
    );
}

/// An eight-bit part on one byte lane.
#[test]
fn only_byte_accesses_are_accepted() {
    let iwm = Iwm::with_drives([true, false]);
    assert_eq!(
        iwm.shared.read(0, &mut [0u8; 2], MemAttrs::DEFAULT),
        Err(BusError::BadAccess)
    );
    assert_eq!(iwm.shared.constraints().max, Width::U8);
}

/// Reset brings the switches and the mechanism back, and leaves the disk where
/// it is: a thing in a slot is not a register.
#[test]
fn reset_clears_the_chip_and_leaves_the_disk_in_the_drive() {
    let iwm = Iwm::with_drives([true, false]);
    iwm.set_disk(0, true, true);
    control(&iwm, 0, false);
    control(&iwm, 1, false);
    control(&iwm, 2, false);
    assert!(iwm.motor(0) && iwm.track(0) == 1);

    iwm.reset(ResetKind::Warm);
    assert_eq!(iwm.switches(), 0);
    assert!(!iwm.motor(0));
    assert_eq!(iwm.track(0), 0);
    assert!(iwm.has_disk(0), "the disk is still in the slot");
    assert!(!sense(&iwm, 3), "and it is still write protected");
}

/// Invariant 6.
#[test]
fn a_snapshot_round_trips_to_an_identical_state_hash() {
    let saved = Iwm::with_drives([true, true]);
    saved.set_disk(0, true, false);
    control(&saved, 0, false);
    for _ in 0..17 {
        control(&saved, 1, false);
    }
    control(&saved, 2, false);
    touch(&saved, 13);
    saved.poke(15, 0x17);

    let image = |iwm: &Iwm| -> alloc::vec::Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("iwm", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("iwm", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(iwm, &mut chunk).unwrap();
        }
        w.to_vec().unwrap()
    };
    let first = image(&saved);

    let restored = Iwm::with_drives([true, true]);
    let reader = StateReader::new(&first).unwrap();
    let chunk = reader
        .load("iwm", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(&restored, &mut chunk.reader()).unwrap();
    assert_eq!(image(&restored), first, "the same chip, bit for bit");
    assert_eq!(restored.track(0), 17);
    assert!(restored.motor(0) && restored.has_disk(0));
    assert_eq!(restored.mode(), 0x17);
}

/// The class registers, takes one property, and names its region and pin.
#[test]
fn the_class_is_registrable_and_its_schema_matches() {
    let mut registry = crate::core::Registry::new();
    register(&mut registry).expect("a fresh registry");
    assert_eq!(registry.get(CLASS_NAME).unwrap().version, STATE_VERSION);

    assert!(Iwm::new(&Props::new()).is_ok(), "one drive by default");
    let two = Iwm::new(&Props::new().with("drives", Value::Uint(2))).expect("two");
    touch(&two, 11);
    assert!(!sense(&two, 12), "/DRVIN low: the external drive is there");
    let err = Iwm::new(&Props::new().with("drives", Value::Uint(3)))
        .expect_err("a cable takes two")
        .to_string();
    assert!(err.contains("drives"), "{err}");

    let iwm = Iwm::with_drives([true, false]);
    assert!(iwm.region("").is_some() && iwm.region("regs").is_some());
    assert!(iwm.region("drive").is_none());
    assert!(iwm.sink(SEL_PIN, &[]).is_some());
    assert!(iwm.sink("ca0", &[]).is_none(), "CA0 is a switch, not a pin");
    assert!(schema().port_named(SEL_PIN).is_some());
}
