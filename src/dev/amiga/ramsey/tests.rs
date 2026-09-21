//! Ramsey's two registers, at the addresses the A3000 decodes them at.

use super::*;
use crate::core::props::{Props, Value};
use crate::core::space::{AddressSpace, UnassignedPolicy};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::value::Width;
use alloc::vec::Vec;

/// What an undriven byte reads: the floating-bus byte the access carries.
const FLOAT: u8 = 0xA5;

const BASE: u64 = 0x00DE_0000;

struct Rig {
    ramsey: Ramsey,
    space: Arc<AddressSpace>,
}

fn rig_with(props: Props) -> Rig {
    let ramsey = Ramsey::new(&props).expect("a memory controller");
    let space = Arc::new(AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::OPEN_BUS));
    space
        .topology()
        .map(
            Device::region(&ramsey, REGS_REGION).expect("a region"),
            BASE,
        )
        .expect("it maps");
    Rig { ramsey, space }
}

fn rig() -> Rig {
    rig_with(Props::new())
}

impl Rig {
    fn rb(&self, at: u64) -> u8 {
        self.space
            .read(at, Width::U8, MemAttrs::DEFAULT.with_bus(FLOAT))
            .expect("mapped") as u8
    }

    fn wb(&self, at: u64, value: u8) {
        self.space
            .write(at, Width::U8, u64::from(value), MemAttrs::DEFAULT)
            .expect("mapped");
    }
}

#[test]
fn the_two_registers_are_where_table_2_2_puts_them() {
    let r = rig();
    // §2.2, Table 2-2: control at `$00DE0003`, version at `$00DE0043`.
    assert_eq!(r.rb(BASE + CONTROL), CONTROL_RESET);
    assert_eq!(r.rb(BASE + VERSION), VERSION_A3000);
    // And nothing else in the window is a Ramsey register, so it floats.
    for at in [0x00, 0x01, 0x02, 0x04, 0x40, 0x42, 0x44, 0x7f] {
        assert_eq!(r.rb(BASE + at), FLOAT, "offset {at:#04x}");
    }
}

#[test]
fn the_control_register_reads_back_what_kickstart_spins_on() {
    let r = rig();
    // The three mode bits, which is what Kickstart's memory sizing sets and
    // then waits to see — the whole reason this device exists.
    r.wb(BASE + CONTROL, CONTROL_RESET | WRAP | BURST | PAGE_DETECT);
    assert_eq!(
        r.rb(BASE + CONTROL) & (WRAP | BURST | PAGE_DETECT),
        WRAP | BURST | PAGE_DETECT
    );
    r.wb(BASE + CONTROL, CONTROL_RESET);
    assert_eq!(r.rb(BASE + CONTROL) & (WRAP | BURST | PAGE_DETECT), 0);
    // Every bit of it is writable, including the refresh rate and `TEST`.
    r.wb(BASE + CONTROL, 0xff);
    assert_eq!(r.rb(BASE + CONTROL), 0xff);
    assert_eq!(r.rb(BASE + CONTROL) & REFRESH_RATE, REFRESH_RATE);
    assert_eq!(r.rb(BASE + CONTROL) & TEST, TEST);
}

#[test]
fn the_version_register_is_read_only_and_says_which_part_this_is() {
    let r = rig();
    r.wb(BASE + VERSION, 0x5a);
    assert_eq!(r.rb(BASE + VERSION), VERSION_A3000);
    // A board may fit the later part instead.
    let enhanced = rig_with(Props::new().with("version", Value::Uint(u64::from(VERSION_ENHANCED))));
    assert_eq!(enhanced.rb(BASE + VERSION), VERSION_ENHANCED);
}

#[test]
fn a_reset_puts_the_straps_back_and_the_mode_bits_down() {
    let r = rig();
    r.wb(BASE + CONTROL, 0xff);
    Device::reset(&r.ramsey, ResetKind::Cold);
    assert_eq!(r.rb(BASE + CONTROL), CONTROL_RESET);
    r.wb(BASE + CONTROL, 0xff);
    Device::reset(&r.ramsey, ResetKind::Warm);
    assert_eq!(r.rb(BASE + CONTROL), CONTROL_RESET);
}

#[test]
fn a_debug_read_is_an_ordinary_one_and_a_debug_write_is_refused() {
    let r = rig();
    r.wb(BASE + CONTROL, CONTROL_RESET | BURST);
    // Neither register has a read side effect, so a monitor may read both.
    assert_eq!(
        r.space
            .read(BASE + CONTROL, Width::U8, MemAttrs::DEBUG.with_bus(FLOAT))
            .expect("mapped") as u8,
        CONTROL_RESET | BURST
    );
    assert_eq!(
        r.space
            .read(BASE + VERSION, Width::U8, MemAttrs::DEBUG.with_bus(FLOAT))
            .expect("mapped") as u8,
        VERSION_A3000
    );
    // A write changes how the guest's own memory is refreshed.
    assert!(
        r.space
            .write(BASE + CONTROL, Width::U8, 0, MemAttrs::DEBUG)
            .is_err()
    );
    assert_eq!(r.rb(BASE + CONTROL), CONTROL_RESET | BURST);
}

#[test]
fn a_wider_access_reaches_the_byte_that_is_the_register() {
    let r = rig();
    // A longword read at `$00DE0000` covers `$…03`, which is the control
    // register, and three bytes that are not.
    let word = r
        .space
        .read(BASE, Width::U32, MemAttrs::DEFAULT.with_bus(FLOAT))
        .expect("mapped");
    assert_eq!(word as u8, CONTROL_RESET);
    assert_eq!((word >> 24) as u8, FLOAT);
    // And a longword write puts its low byte there.
    r.space
        .write(BASE, Width::U32, 0x1122_3344, MemAttrs::DEFAULT)
        .expect("mapped");
    assert_eq!(r.rb(BASE + CONTROL), 0x44);
}

// ---------------------------------------------------------------------------
// snapshot
// ---------------------------------------------------------------------------

fn snapshot(r: &Ramsey) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("ramsey", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("ramsey", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(r, &mut chunk).unwrap();
    }
    w.to_vec().unwrap()
}

#[test]
fn a_snapshot_round_trips_to_identical_state() {
    let saved = rig();
    saved.wb(BASE + CONTROL, CONTROL_RESET | WRAP | PAGE_DETECT);
    let bytes = snapshot(&saved.ramsey);

    let restored = rig();
    assert_ne!(snapshot(&restored.ramsey), bytes);
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("ramsey", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(&restored.ramsey, &mut chunk.reader()).unwrap();
    assert_eq!(snapshot(&restored.ramsey), bytes, "identical state");
    assert_eq!(
        restored.rb(BASE + CONTROL),
        CONTROL_RESET | WRAP | PAGE_DETECT
    );
}

#[test]
fn a_snapshot_of_a_different_part_is_refused() {
    let bytes = snapshot(&rig().ramsey);
    let other = rig_with(Props::new().with("version", Value::Uint(u64::from(VERSION_ENHANCED))));
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("ramsey", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    let e = Device::load(&other.ramsey, &mut chunk.reader())
        .expect_err("a `$0D` part is not a `$0F` one");
    assert!(alloc::format!("{e}").contains("version"), "{e}");
}
