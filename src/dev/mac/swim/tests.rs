//! The SWIM on its own: that it is the IWM while it is in IWM mode, that its
//! SuperDrive says so, and that the two cell rates come out at the two spindle
//! speeds.

use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;

use super::*;
use crate::core::device::Device;
use crate::core::props::Value;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::dev::mac::{gcr, mfm};

/// The soft switches are the IWM's, at the IWM's stride, and the mode register
/// is loaded the IWM's way — which is the whole shape of the older part and
/// the reason this device forwards rather than reimplements.
#[test]
fn the_register_file_is_the_iwms() {
    let swim = Swim::with_drives([true, false]);
    assert_eq!(REGISTER_SPAN, iwm::REGISTER_SPAN);
    // The mode register is loaded by a write while `Q7:Q6` is `11`.
    let _ = swim.peek(13); // Q6 on
    swim.poke(15, 0x17); // Q7 on, and the write lands in the mode register
    assert_eq!(swim.iwm().mode(), 0x17, "the write reached the IWM inside");
    // With `Q7` off again the status register mirrors the mode's low bits.
    let status = swim.peek(14);
    assert_eq!(status & 0x1f, 0x17, "{status:#04x}");
}

/// **The mechanism says it is a SuperDrive**, at the drive register address an
/// 800K one leaves to the cable's pull-up.
#[test]
fn the_drive_reports_itself_as_a_superdrive() {
    // `CA2:CA1:CA0 = 101` with `SEL` low is drive register 10: set CA0 and
    // CA2, clear CA1, then read the status register and look at `SENSE`.
    let sense = |read: &dyn Fn(u8) -> u8| -> bool {
        let _ = read(1); // CA0 on
        let _ = read(2); // CA1 off
        let _ = read(5); // CA2 on
        let _ = read(13); // Q6 on: the status register
        read(14) & 0x80 != 0
    };
    let swim = Swim::with_drives([true, false]);
    swim.set_sel(false);
    assert!(
        !sense(&|i| swim.peek(i)),
        "a SuperDrive pulls the line low, like every other line on the cable"
    );

    let plus = Iwm::with_drives([true, false]);
    plus.set_sel(false);
    assert!(
        sense(&|i| plus.peek(i)),
        "an 800K mechanism leaves it to the pull-up, which is what a Plus sees"
    );
}

/// A 1.44 MB image goes in, and a GCR one still does.
#[test]
fn both_densities_go_into_the_drive() {
    let swim = Swim::with_drives([true, false]);
    assert!(!swim.has_disk(0));
    swim.insert(0, Disk::blank_mfm());
    assert_eq!(swim.density(0), Some(Density::Mfm));
    swim.insert(0, Disk::blank(2));
    assert_eq!(swim.density(0), Some(Density::Gcr));
    swim.eject(0);
    assert_eq!(swim.density(0), None);
}

/// The two cell rates, and the two spindle speeds they come out at.
///
/// One tick of this chip is one MFM cell. A GCR disk's cells are half that
/// rate, so it gets every second tick, and the arithmetic below is the whole
/// of why the two media turn at the speeds they do.
#[test]
fn a_gcr_disk_gets_every_second_tick_and_an_mfm_disk_every_one() {
    let swim = Swim::with_drives([true, false]);
    swim.insert(0, Disk::blank(2));
    swim.advance_to(1000);
    assert_eq!(swim.ticks(), 1000);
    assert_eq!(swim.iwm().ticks(), 500, "a GCR cell is two of this chip's");

    let swim = Swim::with_drives([true, false]);
    swim.insert(0, Disk::blank_mfm());
    swim.advance_to(1000);
    assert_eq!(swim.iwm().ticks(), 1000, "an MFM cell is one");

    // And the speeds that follow, with nothing else to get wrong: a cylinder
    // is its track's length and the spindle turns once per pass.
    let gcr_rpm = 500_000 * 60 / (gcr::SECTOR_CELLS * 12);
    assert_eq!(gcr_rpm, 394, "zone 0 of an 800K disk");
    let mfm_rpm = 1_000_000 * 60 / mfm::CELLS_PER_REVOLUTION;
    assert_eq!(mfm_rpm, usize::try_from(mfm::RPM).unwrap());
}

/// A 1.44 MB disk turns under the head and its cells go past it.
#[test]
fn an_mfm_disk_turns_under_the_head() {
    let swim = Swim::with_drives([true, false]);
    let mut image = alloc::vec![0u8; mfm::BYTES];
    for (block, chunk) in image.chunks_mut(512).enumerate() {
        chunk[..4].copy_from_slice(&(block as u32).to_be_bytes());
    }
    swim.insert(
        0,
        Disk::from_image_for(&image, Reader::Swim).expect("a SWIM reads one"),
    );
    // Motor on: the drive's control register `CA1:CA0:SEL = 100` with `CA2`
    // low, latched when `LSTRB` goes high.
    swim.set_sel(false);
    let _ = swim.peek(0); // CA0 off
    let _ = swim.peek(3); // CA1 on
    let _ = swim.peek(4); // CA2 off — motor on
    let _ = swim.peek(7); // LSTRB on: latch it
    let _ = swim.peek(6); // LSTRB off
    assert!(swim.motor(0), "the motor is running");

    // One revolution of cells. The chip's shifter is the IWM's GCR one, so
    // what it latches out of MFM is not sectors — see the module docs — but
    // the medium does move, which is what this asserts.
    let one = u64::try_from(mfm::CELLS_PER_REVOLUTION).unwrap();
    swim.advance_to(one);
    assert_eq!(swim.iwm().ticks(), one);
}

/// Save, load, and the image is identical.
#[test]
fn a_snapshot_round_trip_is_identical() {
    let a = Swim::with_drives([true, false]);
    a.insert(0, Disk::blank_mfm());
    let _ = a.peek(13);
    a.poke(15, 0x1f);
    a.advance_to(12_345);

    let image = |swim: &Swim| -> Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("swim", CLASS_NAME).expect("a fresh shape");
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("swim", CLASS_NAME, STATE_VERSION).expect("a chunk");
            Device::save(swim, &mut chunk).expect("a SWIM saves");
        }
        w.to_vec().expect("a complete image")
    };
    let first = image(&a);

    let b = Swim::with_drives([true, false]);
    b.insert(0, Disk::blank_mfm());
    let reader = StateReader::new(&first).expect("a well-formed image");
    let chunk = reader
        .load("swim", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .expect("the chunk is there");
    Device::load(&b, &mut chunk.reader()).expect("a SWIM loads");
    assert_eq!(image(&b), first, "the same chip, bit for bit");
    assert_eq!(b.ticks(), 12_345);
}

/// The class registers, its schema names the pin and the regions a machine
/// file may write, and `drives` is checked.
#[test]
fn the_class_is_registrable_and_its_schema_matches() {
    let mut registry = crate::core::Registry::new();
    register(&mut registry).expect("a fresh registry");
    let s = schema();
    assert_eq!(s.class, CLASS_NAME);
    let ports: Vec<&str> = s.ports.iter().map(|p| p.name.as_str()).collect();
    assert_eq!(ports, alloc::vec![SEL_PIN]);
    assert!(s.regions.contains(&String::from("regs")));

    let err = Swim::new(&Props::new().with("drives", Value::Uint(3)))
        .expect_err("three mechanisms")
        .to_string();
    assert!(err.contains("one or two"), "{err}");
}

/// And the region it hands the address space is the one a `map` places.
#[test]
fn the_region_is_the_span_the_board_maps() {
    let swim = Arc::new(Swim::with_drives([true, false]));
    let region = Device::region(&*swim, "").expect("a register file");
    assert_eq!(region.len(), REGISTER_SPAN);
    assert!(Device::region(&*swim, "nope").is_none());
}

/// The cached density is what the tick path reads, and it follows the medium
/// through every door the medium can change by.
///
/// It is derived state and is never serialized: `advance_to` runs once a
/// scheduler round — fifty thousand times a virtual second while a disk is
/// turning — and asking the drive would mean cloning a megabyte and a half of
/// disk every round.
#[test]
fn the_cached_density_follows_the_medium() {
    let swim = Swim::with_drives([true, false]);
    assert!(!swim.is_mfm(), "an empty drive is not MFM");
    swim.insert(0, Disk::blank_mfm());
    assert!(swim.is_mfm());
    swim.insert(0, Disk::blank(2));
    assert!(!swim.is_mfm(), "a GCR disk went in over it");
    swim.insert(0, Disk::blank_mfm());
    swim.eject(0);
    assert!(!swim.is_mfm(), "and the drive is empty again");

    // A reset leaves the disk in the drive, so the cache has to come back with
    // it rather than be assumed.
    swim.insert(0, Disk::blank_mfm());
    Device::reset(&swim, crate::core::device::ResetKind::Warm);
    assert!(
        swim.has_disk(0),
        "a disk is a thing in a slot, not a register"
    );
    assert!(swim.is_mfm(), "and the cache came back with it");
}
