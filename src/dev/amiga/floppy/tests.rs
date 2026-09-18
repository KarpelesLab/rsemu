//! The drive's own tests: Table 8-5 and Appendix E's connector, pin by pin.

use super::*;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::wire::{Pull, Wire};
use crate::dev::amiga::custom::{Custom, Origin};
use crate::dev::amiga::paula::Paula;
use crate::host::chardev::{CharDevice, CharPort};

struct Rig {
    drive: Floppy,
    paula: Paula,
    custom: Custom,
    outs: [WireSource; 5],
}

impl Rig {
    fn new() -> Rig {
        Rig::with(Floppy::bare(String::from("paula")))
    }

    /// A rig around a drive built some other way.
    fn with(drive: Floppy) -> Rig {
        let paula = Paula::with_port(
            String::from("custom"),
            Arc::new(CharPort::new()) as Arc<dyn CharDevice>,
            String::from("serial"),
        );
        let custom = Custom::new(&Props::new()).expect("no properties");
        custom.bus().attach(paula.chip()).expect("attaches");
        drive.attach(paula.port());
        let outs = core::array::from_fn(|i| {
            let src = WireId::new(50 + i as u64);
            let wire = Wire::builder()
                .source(src)
                .resolved(Pull::Up)
                .build_shared();
            let source = WireSource::new(wire, src);
            drive
                .connect(OUTPUT_PINS[i], source.clone())
                .expect("an output");
            source
        });
        Rig {
            drive,
            paula,
            custom,
            outs,
        }
    }

    /// Set an input pin's level, the way a CIA port bit reaches it.
    fn set(&self, pin: &str, high: bool) {
        let src = WireId::new(1);
        let sink = self.drive.sink(pin, &[src]).expect("an input");
        sink.sink.set_level(src, sink.line, Level::from_bool(high));
    }

    /// Whether the drive is pulling output `pin` low.
    fn low(&self, pin: &str) -> bool {
        let i = OUTPUT_PINS
            .iter()
            .position(|p| *p == pin)
            .expect("an output");
        self.outs[i].drive_state() == Drive::Low
    }

    fn select(&self, motor_on: bool) {
        self.set("sel", true);
        self.set("mtr", !motor_on);
        self.set("sel", false);
    }

    fn step(&self, inward: bool) {
        self.set("dir", !inward);
        self.set("step", false);
        self.set("step", true);
    }
}

/// A disk whose track `t` starts with the byte `t`, then `$4489`.
fn numbered() -> MfmDisk {
    let mut disk = MfmDisk::blank();
    for t in 0..TRACKS {
        disk.set_track(t, &[t as u8, 0x44, 0x89]);
    }
    disk
}

#[test]
fn a_deselected_drive_lets_go_of_every_line() {
    let rig = Rig::new();
    rig.drive.insert(numbered());
    for pin in OUTPUT_PINS {
        assert!(!rig.low(pin), "{pin}");
    }
    rig.select(false);
    assert!(rig.low("tk0"), "the head is over track 0");
    assert!(rig.low("chng"), "the change flop is set at power up");
    rig.set("sel", true);
    for pin in OUTPUT_PINS {
        assert!(!rig.low(pin), "{pin}, deselected again");
    }
}

#[test]
fn the_motor_flop_is_clocked_by_select_and_holds_while_deselected() {
    let rig = Rig::new();
    rig.select(true);
    assert!(rig.drive.motor());
    rig.set("mtr", true);
    assert!(
        rig.drive.motor(),
        "MTR* moving while selected changes nothing"
    );
    rig.set("sel", true);
    assert!(rig.drive.motor(), "the drive remembers its motor");
    rig.set("sel", false);
    assert!(!rig.drive.motor(), "the next select clocks MTR* high: off");
}

#[test]
fn stepping_moves_the_head_refuses_track_minus_one_and_clears_the_change_flop() {
    let rig = Rig::new();
    rig.select(false);
    rig.step(false);
    assert_eq!(rig.drive.cylinder(), 0, "refused outward at track 0");
    assert!(rig.low("chng"), "no disk, so the flop stays set");

    rig.drive.insert(numbered());
    rig.step(true);
    assert_eq!(rig.drive.cylinder(), 1);
    assert!(!rig.low("tk0"));
    assert!(!rig.low("chng"), "selected, stepped, with a disk in");
    for _ in 0..100 {
        rig.step(true);
    }
    assert_eq!(rig.drive.cylinder(), CYLINDERS - 1);
    rig.step(false);
    assert_eq!(rig.drive.cylinder(), CYLINDERS - 2);

    // A deselected drive ignores the step line.
    rig.set("sel", true);
    rig.step(false);
    assert_eq!(rig.drive.cylinder(), CYLINDERS - 2);

    rig.set("sel", false);
    let _ = rig.drive.eject();
    assert!(rig.low("chng"), "removing the disk sets it again");
}

/// The head moves on the **trailing** edge of `STEP*`, which is what makes a
/// Kickstart 1.x step land at all.
///
/// A 1.3 `trackdisk` deselects the drive between pulses and then asserts
/// `SEL0*`, `DIR` and `STEP*` in a single `PRB` write, so the drive sees the
/// leading edge of `STEP*` on the very instant it is being selected — and the
/// CIA hands its port-B pins over one at a time, `PB0` (`STEP*`) before `PB3`
/// (`SEL0*`). A leading-edge model therefore drops the step, the head never
/// leaves cylinder 0, the change flop is never reset, and `trackdisk` decides
/// the drive is empty. Only the trailing edge is unambiguously inside the
/// selected window, which is also what a Shugart-compatible mechanism does:
/// "the access motion is initiated on the trailing edge of the step pulse".
///
/// The pins are set here in the order the CIA delivers them, which is the
/// order that exposes it.
#[test]
fn a_step_asserted_in_the_same_write_as_select_still_moves_the_head() {
    let rig = Rig::new();
    rig.drive.insert(numbered());
    rig.select(false);
    rig.set("sel", true); // deselected, as 1.3 leaves it between pulses

    // One PRB write: STEP* low, DIR low (inward), SEL0* low.
    rig.set("step", false);
    rig.set("dir", false);
    rig.set("sel", false);
    assert_eq!(rig.drive.cylinder(), 0, "the leading edge moves nothing");

    // The next write raises STEP* alone.
    rig.set("step", true);
    assert_eq!(rig.drive.cylinder(), 1, "the trailing edge steps inward");
    assert!(
        !rig.low("chng"),
        "the change flop is reset by a step with a disk in, which is how \
         trackdisk knows there is one"
    );

    // And the same shape stepping back out: DIR high with STEP* low, then up.
    rig.set("sel", true);
    rig.set("step", false);
    rig.set("dir", true);
    rig.set("sel", false);
    rig.set("step", true);
    assert_eq!(rig.drive.cylinder(), 0, "and outward again");
}

#[test]
fn ready_needs_the_motor_and_a_disk_and_protect_needs_the_tab() {
    let rig = Rig::new();
    rig.select(true);
    assert!(!rig.low("rdy"), "no disk");
    let mut disk = numbered();
    disk.write_protected = true;
    rig.drive.insert(disk);
    assert!(rig.low("rdy"));
    assert!(rig.low("wpro"));
}

#[test]
fn with_the_motor_off_rdy_carries_the_identification_word() {
    let rig = Rig::new();
    let mut word = 0u32;
    for _ in 0..32 {
        rig.set("sel", true);
        rig.set("mtr", true);
        rig.set("sel", false);
        word = (word << 1) | u32::from(rig.low("rdy"));
    }
    assert_eq!(word, DRIVE_ID, "Amiga standard, most significant bit first");
}

#[test]
fn the_index_pulse_is_on_the_calendar_and_falls_once_a_revolution() {
    let rig = Rig::new();
    rig.drive.insert(numbered());
    assert_eq!(rig.drive.next_event_tick(), None, "not spinning");
    rig.select(true);
    assert!(rig.low("index"), "tick zero is the index");
    let width = INDEX_CELLS * FAST_CELL_TICKS;
    assert_eq!(rig.drive.next_event_tick(), Some(width));
    rig.drive.advance_to(width);
    assert!(!rig.low("index"));
    assert_eq!(rig.drive.next_event_tick(), Some(REVOLUTION_TICKS));
    rig.drive.advance_to(REVOLUTION_TICKS);
    assert!(rig.low("index"), "and again");
}

#[test]
fn paula_reads_the_track_under_the_head_on_the_side_selected() {
    let rig = Rig::new();
    rig.drive.insert(numbered());
    rig.select(true);
    rig.step(true);
    rig.step(true);
    rig.set("side", false); // SIDE* active: side 1
    // FAST, and the sync mark is two bytes into the track.
    rig.custom.bus().write(0x09e, 0x8100, Origin::cpu());
    rig.custom.bus().write(0x07e, 0x4489, Origin::cpu());
    // Cylinder 2, side 1 is track 5, whose first byte is its number.
    rig.paula.advance_to(8 * FAST_CELL_TICKS);
    let dskbytr = rig.custom.bus().read(0x01a, Origin::cpu());
    assert_eq!(dskbytr & 0x80ff, 0x8005);
    rig.paula.advance_to(24 * FAST_CELL_TICKS);
    let intreq = rig.custom.bus().read(0x01e, Origin::cpu());
    assert_eq!(intreq & 0x1000, 0x1000, "DSKSYN");

    // Side 0 of the same cylinder is track 4, a revolution later.
    rig.set("side", true);
    rig.paula.advance_to(REVOLUTION_TICKS + 8 * FAST_CELL_TICKS);
    let dskbytr = rig.custom.bus().read(0x01a, Origin::cpu());
    assert_eq!(dskbytr & 0x80ff, 0x8004);
}

#[test]
fn writes_land_under_the_head_unless_the_tab_is_open() {
    let rig = Rig::new();
    let mut disk = MfmDisk::blank();
    disk.write_protected = true;
    rig.drive.insert(disk);
    rig.select(true);
    let shared = Arc::clone(&rig.drive.shared);
    shared.write_cells(0, FAST_CELL_TICKS, &[1, 0, 1, 0, 1, 0, 1, 0]);
    assert_eq!(rig.drive.disk().unwrap().track(0)[0], 0, "protected");

    let mut disk = rig.drive.eject().unwrap();
    disk.write_protected = false;
    rig.drive.insert(disk);
    // One revolution and eight cells later is the same place on the disk.
    shared.write_cells(REVOLUTION_TICKS, FAST_CELL_TICKS, &[1, 0, 1, 0, 1, 0, 1, 1]);
    assert_eq!(rig.drive.disk().unwrap().track(0)[0], 0xab);
    let mut back = [0u8; 8];
    shared.read_cells(0, FAST_CELL_TICKS, &mut back);
    assert_eq!(back, [1, 0, 1, 0, 1, 0, 1, 1]);
}

#[cfg(feature = "dev-mos8520")]
#[test]
fn cia_port_b_drives_the_drive_and_the_drive_drives_cia_port_a_without_nesting_locks() {
    use crate::core::space::{AddressSpace, MemAttrs};
    use crate::core::value::Width;
    use crate::dev::mos::Cia;

    // The rank checker is live here. CIA-B's refresh drives MTR* and SEL0*
    // into the drive, whose update drives RDY* into CIA-A and tells Paula,
    // which asks the drive whether it is reading: three devices and four locks
    // on one call chain.
    let rig = Rig::new();
    let cia_a = Cia::bare();
    let cia_b = Cia::bare();
    for (cia_pin, drive_pin, id) in [("pb7", "mtr", 1u64), ("pb3", "sel", 2)] {
        let src = WireId::new(id);
        let sink = rig.drive.sink(drive_pin, &[src]).expect("an input");
        let wire = Wire::builder()
            .source(src)
            .resolved(Pull::Up)
            .sink(sink.sink, sink.line)
            .build_shared();
        cia_b
            .connect_pin(cia_pin, WireSource::new(wire, src))
            .expect("a port pin");
    }
    let src = WireId::new(3);
    let pa5 = cia_a.sink("pa5", &[src]).expect("pa5");
    let wire = Wire::builder()
        .source(src)
        .resolved(Pull::Up)
        .sink(pa5.sink, pa5.line)
        .build_shared();
    rig.drive
        .connect("rdy", WireSource::new(wire, src))
        .expect("rdy");
    rig.drive.insert(numbered());

    let space_a = AddressSpace::new("cia-a", 16);
    space_a
        .topology()
        .map(cia_a.region("").expect("registers"), 0)
        .expect("maps");
    let space_b = AddressSpace::new("cia-b", 16);
    space_b
        .topology()
        .map(cia_b.region("").expect("registers"), 0)
        .expect("maps");
    let poke = |space: &AddressSpace, reg: u64, v: u8| {
        space
            .write(reg, Width::U8, u64::from(v), MemAttrs::DEFAULT)
            .expect("a CIA register");
    };
    let peek = |space: &AddressSpace, reg: u64| {
        space
            .read(reg, Width::U8, MemAttrs::DEFAULT)
            .expect("a CIA register")
    };

    poke(&space_b, 0x1, 0xff); // PRB: everything inactive
    poke(&space_b, 0x3, 0xff); // DDRB: all outputs, as Appendix F says
    // "All software that selects drives must set up the motor signal before
    // selecting any drives" (Table 8-5): MTR* first, then SEL0*.
    poke(&space_b, 0x1, 0x7f);
    poke(&space_b, 0x1, 0x77);
    assert!(rig.drive.selected() && rig.drive.motor());
    assert_eq!(peek(&space_a, 0x0) & 0x20, 0, "RDY* low at CIA-A");
    poke(&space_b, 0x1, 0xff);
    assert_eq!(peek(&space_a, 0x0) & 0x20, 0x20, "let go when deselected");
}

fn snapshot(f: &Floppy) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("df0", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("df0", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(f, &mut chunk).unwrap();
    }
    w.to_vec().unwrap()
}

#[test]
fn a_snapshot_round_trips_to_identical_state_disk_included() {
    let rig = Rig::new();
    rig.drive.insert(numbered());
    rig.select(true);
    rig.step(true);
    rig.drive.advance_to(12_345);
    let bytes = snapshot(&rig.drive);

    let other = Rig::new();
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("df0", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(&other.drive, &mut chunk.reader()).unwrap();
    assert_eq!(snapshot(&other.drive), bytes);
    assert_eq!(other.drive.cylinder(), 1);
    assert_eq!(other.drive.disk(), rig.drive.disk());
    assert!(other.low("rdy"), "the outputs were driven again");
}

#[test]
fn a_reset_turns_the_motor_off_and_leaves_the_mechanics_alone() {
    let rig = Rig::new();
    rig.drive.insert(numbered());
    rig.select(true);
    rig.step(true);
    Device::reset(&rig.drive, ResetKind::Warm);
    assert!(!rig.drive.motor());
    assert_eq!(rig.drive.cylinder(), 1);
    assert!(rig.drive.disk().is_some());
}

#[test]
fn the_class_names_its_controller_and_its_pins() {
    use crate::core::props::{Link, Value};
    let props = Props::new().with("paula", Value::Link(Link::new("paula").unwrap()));
    assert!(Floppy::new(&props).is_ok());
    assert!(Floppy::new(&Props::new()).is_err());
    let f = Floppy::new(&props).unwrap();
    assert!(f.disk().is_none(), "no image, no disk");

    // A raw image is exactly 160 tracks; anything else is refused by name.
    let raw = alloc::vec![0x55u8; TRACKS * TRACK_BYTES];
    let with = |bytes: Vec<u8>| {
        props
            .clone()
            .with("image", crate::core::props::Media::new("df0", bytes))
            .with("write-protected", Value::Bool(true))
    };
    let loaded = Floppy::new(&with(raw)).unwrap().disk().expect("a disk");
    assert!(loaded.write_protected);
    assert_eq!(loaded.track(159)[TRACK_BYTES - 1], 0x55);
    let e = Floppy::new(&with(alloc::vec![0; 901_121]))
        .unwrap_err()
        .to_string();
    assert!(
        e.contains("df0") && e.contains("901121") && e.contains("901120"),
        "{e}"
    );
    let e = Floppy::new(&with(alloc::vec![0; 2 * 901_120]))
        .unwrap_err()
        .to_string();
    assert!(e.contains("high-density"), "{e}");
    assert!(Floppy::new(&with(Vec::new())).unwrap().disk().is_none());

    let schema = schema();
    for pin in INPUT_PINS.iter().chain(OUTPUT_PINS.iter()) {
        assert!(schema.port_named(pin).is_some(), "{pin}");
    }
    assert!(f.sink("mtr", &[WireId::new(1)]).is_some());
    assert!(f.sink("rdy", &[WireId::new(1)]).is_none());
}

// ---------------------------------------------------------------------------
// ADF images, and writing them back
// ---------------------------------------------------------------------------

/// An ADF whose every sector is distinct.
fn adf_image() -> Vec<u8> {
    (0..adf::ADF_BYTES)
        .map(|i| (i as u8) ^ ((i / adf::SECTOR_BYTES) as u8).wrapping_mul(29))
        .collect()
}

/// Props for a drive whose `df0` slot holds `bytes`, in a build whose host
/// objects are `hosts`.
fn props_with(bytes: Vec<u8>, hosts: Option<Arc<crate::core::HostObjects>>) -> Props {
    use crate::core::props::{Link, Media, Value};
    let props = Props::new()
        .with("paula", Value::Link(Link::new("paula").unwrap()))
        .with("image", Media::new("df0", bytes));
    match hosts {
        Some(hosts) => props.with_hosts(hosts),
        None => props,
    }
}

/// A drive whose disk is an ADF medium, and the medium.
fn backed_drive(image: &[u8]) -> (Floppy, Arc<crate::core::space::RamStore>) {
    let hosts = Arc::new(crate::core::HostObjects::new());
    let store = Arc::new(crate::core::space::RamStore::new(adf::ADF_BYTES as u64));
    Medium::write_at(&*store, 0, image).unwrap();
    assert!(medium::install(&hosts, "df0", Arc::clone(&store) as Arc<dyn Medium>).unwrap());
    let drive = Floppy::new(&props_with(Vec::new(), Some(hosts))).expect("a drive");
    (drive, store)
}

/// Lay `mfm` under the head of the track the drive is on, as Paula would.
fn write_track(rig: &Rig, mfm: &[u8]) {
    let cells: Vec<u8> = (0..TRACK_CELLS)
        .map(|i| mfm[(i / 8) as usize] >> (7 - i % 8) & 1)
        .collect();
    let shared = Arc::clone(&rig.drive.shared);
    shared.write_cells(0, FAST_CELL_TICKS, &cells);
}

#[test]
fn an_adf_in_the_slot_is_encoded_into_tracks_that_decode_back_to_it() {
    let image = adf_image();
    let drive = Floppy::new(&props_with(image.clone(), None)).expect("an ADF is a disk");
    let disk = drive.disk().expect("a disk");
    assert_eq!(
        disk.track(3),
        &adf::encode_track(3, &image[3 * 5632..4 * 5632])[..]
    );
    assert_eq!(disk.to_adf(), (image, 0));
}

#[test]
fn a_medium_gets_a_written_track_when_the_head_leaves_it_and_not_before() {
    let image = adf_image();
    let (drive, store) = backed_drive(&image);
    let rig = Rig::with(drive);
    rig.select(true);
    rig.step(true); // cylinder 1, side 0: track 2

    let new: Vec<u8> = image[2 * 5632..3 * 5632].iter().map(|b| !b).collect();
    write_track(&rig, &adf::encode_track(2, &new));
    let mut back = vec![0u8; adf::ADF_BYTES];
    Medium::read_at(&*store, 0, &mut back).unwrap();
    assert_eq!(back, image, "the head is still over the track: nothing yet");

    rig.step(true);
    Medium::read_at(&*store, 0, &mut back).unwrap();
    assert_eq!(
        &back[2 * 5632..3 * 5632],
        &new[..],
        "stepping off handed it back"
    );
    assert_eq!(&back[..2 * 5632], &image[..2 * 5632]);
    assert_eq!(&back[3 * 5632..], &image[3 * 5632..]);
    assert!(Device::flush(&rig.drive).is_ok());
}

#[test]
fn a_flush_hands_back_the_track_under_the_head_and_stopping_the_motor_does_too() {
    let image = adf_image();
    let (drive, store) = backed_drive(&image);
    let rig = Rig::with(drive);
    rig.select(true);
    let new = vec![0x11; 5632];
    write_track(&rig, &adf::encode_track(0, &new));
    Device::flush(&rig.drive).expect("a clean flush");
    let mut back = vec![0u8; 5632];
    Medium::read_at(&*store, 0, &mut back).unwrap();
    assert_eq!(back, new, "flush");

    let newer = vec![0x22; 5632];
    write_track(&rig, &adf::encode_track(0, &newer));
    rig.set("sel", true);
    rig.set("mtr", true);
    rig.set("sel", false); // the motor flop clocked off
    Medium::read_at(&*store, 0, &mut back).unwrap();
    assert_eq!(back, newer, "motor off");
}

#[test]
fn a_track_an_adf_cannot_hold_keeps_the_files_sectors_and_fails_the_flush() {
    let image = adf_image();
    let (drive, store) = backed_drive(&image);
    let rig = Rig::with(drive);
    rig.select(true);
    // Cells, but no AmigaDOS sectors: a disk formatted some other way.
    write_track(&rig, &vec![0x92; TRACK_BYTES]);
    let e = Device::flush(&rig.drive).expect_err("an ADF cannot say this");
    assert!(e.to_string().contains("11 sector(s)"), "{e}");
    let mut back = vec![0u8; adf::ADF_BYTES];
    Medium::read_at(&*store, 0, &mut back).unwrap();
    assert_eq!(back, image, "the file keeps what it had");
    assert!(
        Device::flush(&rig.drive).is_ok(),
        "a fault is reported once, not forever"
    );
}

#[test]
fn bytes_in_a_slot_are_a_copy_and_a_flush_has_nothing_to_say() {
    let image = adf_image();
    let rig = Rig::with(Floppy::new(&props_with(image, None)).unwrap());
    rig.select(true);
    write_track(&rig, &vec![0x92; TRACK_BYTES]);
    rig.step(true);
    assert!(Device::flush(&rig.drive).is_ok());
    assert_eq!(rig.drive.disk().unwrap().track(0), &[0x92; TRACK_BYTES][..]);
}

#[test]
fn a_medium_that_is_not_an_adf_is_refused_by_name() {
    let hosts = Arc::new(crate::core::HostObjects::new());
    let store: Arc<dyn Medium> = Arc::new(crate::core::space::RamStore::new(1024));
    assert!(medium::install(&hosts, "df0", store).unwrap());
    let e = Floppy::new(&props_with(Vec::new(), Some(hosts)))
        .unwrap_err()
        .to_string();
    assert!(e.contains("df0") && e.contains("901120"), "{e}");
}

#[test]
fn a_snapshot_keeps_the_written_tracks_owed_and_a_backed_load_owes_them_all() {
    let rig = Rig::new();
    rig.drive.insert(MfmDisk::from_adf(&adf_image()).unwrap());
    rig.select(true);
    write_track(&rig, &vec![0x55; TRACK_BYTES]);
    let bytes = snapshot(&rig.drive);

    let other = Rig::new();
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("df0", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(&other.drive, &mut chunk.reader()).unwrap();
    assert_eq!(snapshot(&other.drive), bytes, "dirty tracks included");

    // Loaded into a drive whose disk is a medium, the snapshot's disk is what
    // the medium must come to hold.
    let image = adf_image();
    let (drive, store) = backed_drive(&vec![0u8; adf::ADF_BYTES]);
    let backed = Rig::with(drive);
    let chunk = reader
        .load("df0", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(&backed.drive, &mut chunk.reader()).unwrap();
    let e = Device::flush(&backed.drive).expect_err("track 0 is not an AmigaDOS track");
    assert!(e.to_string().contains("track 0"), "{e}");
    let mut back = vec![0u8; adf::ADF_BYTES];
    Medium::read_at(&*store, 0, &mut back).unwrap();
    assert_eq!(
        &back[5632..],
        &image[5632..],
        "every other track was written"
    );
}
