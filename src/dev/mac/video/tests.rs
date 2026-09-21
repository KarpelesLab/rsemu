//! The video circuit's own tests: the raster, the two blanking signals, and a
//! known bitmap in memory turning into known pixels.

use super::*;
use crate::core::props::{Link, Value};
use crate::core::space::{RamStore, Region};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::value::Endian;

const RAM_LEN: u64 = 0x10_0000;

fn props() -> Props {
    Props::new().with("ram", Value::Link(Link::new("dram").unwrap()))
}

/// A video circuit pointed at a megabyte of memory the test can write.
fn wired() -> (Video, Arc<RamStore>) {
    let video = Video::new(&props()).expect("`ram` is enough");
    let store = Arc::new(RamStore::new(RAM_LEN));
    let ram: RegionRef = Arc::new(Region::ram("dram", Arc::clone(&store)).with_endian(Endian::Big));
    video.attach(&ram, 24).expect("it maps");
    (video, store)
}

/// The raster the Guide gives: 704 dot clocks a line of which 512 are visible,
/// 370 lines of which 342 are, and 60.15 Hz falls out of it.
#[test]
fn the_raster_is_the_one_the_guide_documents() {
    assert_eq!(DOTS_PER_LINE, 704);
    assert_eq!(LINES_PER_FRAME, 370);
    assert_eq!(FRAME_TICKS, 260_480);
    assert_eq!(FRAME_BYTES, 21_888);
    assert_eq!(ROW_BYTES, 64);
    // 15 667 200 / 260 480 = 60.1472 Hz, so a frame is 16.6258 ms.
    assert_eq!(FRAME_TICKS * 1_000_000_000 / 15_667_200, 16_625_816);
}

/// Blanking is a level, so whichever edge `PCR` selects, software gets one a
/// line and one a frame.
#[test]
fn the_beam_raises_blanking_where_the_visible_picture_ends() {
    let (video, _ram) = wired();
    assert!(!video.in_hblank(), "dot 0 is visible");
    assert!(!video.in_vblank());

    video.advance_to(511);
    assert!(!video.in_hblank(), "the last visible dot");
    video.advance_to(512);
    assert!(video.in_hblank());
    video.advance_to(DOTS_PER_LINE);
    assert!(!video.in_hblank(), "the next line has started");

    video.advance_to(DOTS_PER_LINE * 341);
    assert!(!video.in_vblank(), "the last visible line");
    video.advance_to(DOTS_PER_LINE * 342);
    assert!(video.in_vblank());
    video.advance_to(FRAME_TICKS);
    assert!(!video.in_vblank(), "the next frame has started");
}

/// The frame counter is the number of vertical blanking intervals that have
/// begun, in closed form — a machine may run a virtual minute between two
/// captures and must not loop once per frame to say so.
#[test]
fn the_frame_counter_counts_blanking_intervals_without_looping() {
    let (video, _ram) = wired();
    assert_eq!(video.screen().frames(), 0);
    video.advance_to(DOTS_PER_LINE * 342);
    assert_eq!(video.screen().frames(), 1);
    video.advance_to(FRAME_TICKS + DOTS_PER_LINE * 342);
    assert_eq!(video.screen().frames(), 2);
    video.advance_to(FRAME_TICKS * 3600);
    assert_eq!(video.screen().frames(), 3600, "a virtual minute");
}

/// `hblank` is only scheduled when a machine file wires it: a level that
/// changes 44,509 times a second is 44,509 scheduler visits a second, and a
/// board that does not read the pin should not pay for them.
#[test]
fn the_line_rate_is_only_scheduled_when_something_is_wired_to_it() {
    let (video, _ram) = wired();
    let state = *video.shared.state.lock();
    assert_eq!(
        state.next_event(false),
        DOTS_PER_LINE * 342,
        "with nothing on `hblank`, the next event is the frame's"
    );
    assert_eq!(
        state.next_event(true),
        u64::from(WIDTH),
        "with it wired, the next event is this line's"
    );
    assert_eq!(
        Device::next_event_tick(&video),
        Some(DOTS_PER_LINE * 342),
        "and the device publishes the cheaper one until the pin is claimed"
    );
}

/// The screen buffer hangs a fixed distance below the top of memory, and
/// `PAGE2` picks which of the two. Low is the alternate one.
#[test]
fn the_buffer_hangs_below_the_top_of_memory_and_page_two_picks_which() {
    let (video, _ram) = wired();
    assert_eq!(video.screen().base(), RAM_LEN - MAIN_OFFSET);
    video.set_page2(false);
    assert_eq!(video.screen().base(), RAM_LEN - ALT_OFFSET);
    video.set_page2(true);
    assert_eq!(video.screen().base(), RAM_LEN - MAIN_OFFSET);
    // The 896 bytes between the main buffer's end and the top of memory are
    // the sound and disk-speed buffer, which is why the two are this far
    // apart.
    assert_eq!(MAIN_OFFSET - FRAME_BYTES, 896);
}

/// A known bitmap in memory comes back as known bits: the most significant bit
/// of each byte is the leftmost pixel, and a row is 64 bytes.
#[test]
fn a_known_bitmap_in_memory_reads_back_as_known_bits() {
    let (video, ram) = wired();
    let base = RAM_LEN - MAIN_OFFSET;
    // A single pixel at (0, 0), one at (7, 0) and one at (511, 341).
    ram.write_at(base, &[0x81]).unwrap();
    ram.write_at(base + ROW_BYTES * 341 + 63, &[0x01]).unwrap();

    let mut bits = Vec::new();
    let (w, h, serial) = video.screen().copy_frame(&mut bits);
    assert_eq!((w, h), (WIDTH, HEIGHT));
    assert_eq!(serial, 0, "no frame has finished yet");
    assert_eq!(bits.len(), FRAME_BYTES as usize);
    assert_eq!(bits[0], 0x81);
    assert_eq!(bits[(ROW_BYTES * 341 + 63) as usize], 0x01);
    assert_eq!(bits.iter().map(|b| b.count_ones()).sum::<u32>(), 3);

    // And the alternate buffer is a different 21,888 bytes.
    video.set_page2(false);
    let (_, _, _) = video.screen().copy_frame(&mut bits);
    assert_eq!(bits.iter().map(|b| b.count_ones()).sum::<u32>(), 0);
}

/// The framebuffer is ordinary guest memory: the `ram` object saves it, not
/// this device. What this device has is where the beam is.
#[test]
fn a_snapshot_round_trips_to_an_identical_state_hash() {
    let (saved, _ram) = wired();
    saved.advance_to(FRAME_TICKS * 7 + 1234);

    let image = |video: &Video| -> Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("video", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("video", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(video, &mut chunk).unwrap();
        }
        w.to_vec().unwrap()
    };
    let first = image(&saved);

    let (restored, _other) = wired();
    let reader = StateReader::new(&first).unwrap();
    let chunk = reader
        .load("video", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(&restored, &mut chunk.reader()).unwrap();
    assert_eq!(image(&restored), first);
    assert_eq!(restored.screen().frames(), saved.screen().frames());
    assert_eq!(restored.in_vblank(), saved.in_vblank());
}

/// A memory too small to hold a screen buffer where a Macintosh puts one is
/// refused, rather than reading off the end of it every frame.
#[test]
fn properties_are_checked() {
    assert!(Video::new(&Props::new()).is_err(), "`ram` is required");
    let err = Video::new(&props().with("size", Value::Size(1)))
        .expect_err("a property this class does not know")
        .to_string();
    assert!(err.contains("size"), "{err}");

    let video = Video::new(&props()).unwrap();
    let small: RegionRef = Arc::new(Region::ram("tiny", Arc::new(RamStore::new(0x1000))));
    let err = video.attach(&small, 24).expect_err("too small").to_string();
    assert!(err.contains("alternate screen buffer"), "{err}");
}

/// The class registers, has no registers of its own, and names its pins.
#[test]
fn the_class_is_registrable_and_its_schema_matches() {
    let mut registry = crate::core::Registry::new();
    register(&mut registry).expect("a fresh registry");
    assert_eq!(registry.get(CLASS_NAME).unwrap().version, STATE_VERSION);
    let (video, _ram) = wired();
    assert!(
        video.region("").is_none(),
        "the video circuit has no registers at all"
    );
    assert!(video.sink(PAGE2_PIN, &[]).is_some());
    assert!(video.sink("vblank", &[]).is_none(), "that one is an output");
    let schema = schema();
    for pin in [VBLANK_PIN, HBLANK_PIN, PAGE2_PIN] {
        assert!(schema.port_named(pin).is_some(), "{pin}");
    }
    assert!(
        Device::connect(&video, "nosuch", dummy_source()).is_err(),
        "an unknown pin is refused"
    );
}

/// A wire source that drives nothing, for the pin-name check above.
fn dummy_source() -> WireSource {
    let wire = crate::core::wire::Wire::builder()
        .source(WireId::new(7))
        .build_shared();
    WireSource::new(wire, WireId::new(7))
}
