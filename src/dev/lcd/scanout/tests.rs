//! Tests for the generic scanout engine.

use super::*;

use alloc::string::ToString;

use crate::core::props::Value;
use crate::core::space::{RamStore, RegionKind, UnassignedPolicy};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

/// A 16 KiB address space with RAM at 0, and the engine bound to it.
///
/// Returns the engine, the space, and the RAM store so a test can paint into
/// the framebuffer the way a guest would.
fn fixture(props: &[(&str, Value)]) -> (Scanout, Arc<AddressSpace>, Arc<RamStore>) {
    let mut p = Props::new();
    for (name, value) in props {
        p.insert(*name, value.clone());
    }
    let engine = Scanout::new(&p).expect("a scanout engine");

    let space = Arc::new(AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::FAULT));
    let ram = Arc::new(RamStore::new(64 * 1024));
    space
        .topology()
        .map(Arc::new(Region::ram("ram", Arc::clone(&ram))), 0)
        .expect("ram maps at 0");
    *engine.shared.bus.lock() = Some(Arc::clone(&space));
    (engine, space, ram)
}

fn ops(region: &RegionRef) -> Arc<dyn MemOps> {
    match region.kind() {
        RegionKind::Io(o) => Arc::clone(o),
        other => panic!("expected an io region, got {other:?}"),
    }
}

fn poke(engine: &Scanout, offset: u64, value: u32) {
    let region = engine.region("").unwrap();
    ops(&region)
        .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
        .expect("a register write");
}

fn peek(engine: &Scanout, offset: u64) -> u32 {
    let region = engine.region("").unwrap();
    let mut buf = [0u8; 4];
    ops(&region)
        .read(offset, &mut buf, MemAttrs::DEFAULT)
        .expect("a register read");
    u32::from_le_bytes(buf)
}

/// A small engine with the framebuffer at 0x100 and nothing else in the way.
fn small() -> (Scanout, Arc<AddressSpace>, Arc<RamStore>) {
    fixture(&[
        ("width", Value::Uint(4)),
        ("height", Value::Uint(3)),
        ("base", Value::Uint(0x100)),
    ])
}

// ---------------------------------------------------------------------------
// Pixel formats
// ---------------------------------------------------------------------------

#[test]
fn every_format_names_itself_and_decodes_the_bytes_it_claims() {
    for name in FbFormat::NAMES {
        let format = FbFormat::from_name(name).expect("a listed format parses");
        assert_eq!(format.name(), *name);
    }
    assert_eq!(FbFormat::from_name("yuv"), None);

    assert_eq!(FbFormat::RGB888.decode(&[1, 2, 3]), [1, 2, 3]);
    assert_eq!(FbFormat::BGR888.decode(&[1, 2, 3]), [3, 2, 1]);
    // 0xXXRRGGBB little-endian is bytes B, G, R, X.
    assert_eq!(
        FbFormat::XRGB8888.decode(&[0xbb, 0x99, 0x77, 0xff]),
        [0x77, 0x99, 0xbb]
    );
    assert_eq!(FbFormat::RGB888.bytes_per_pixel(), 3);
    assert_eq!(FbFormat::RGB565.bytes_per_pixel(), 2);
    assert_eq!(FbFormat::XRGB8888.bytes_per_pixel(), 4);
}

#[test]
fn rgb565_expands_so_that_white_stays_white() {
    // Replicating the high bits into the low ones, rather than shifting and
    // leaving zeroes, is what keeps a saturated value saturated.
    assert_eq!(FbFormat::RGB565.decode(&[0xff, 0xff]), [0xff, 0xff, 0xff]);
    assert_eq!(FbFormat::RGB565.decode(&[0x00, 0x00]), [0x00, 0x00, 0x00]);
    // Pure red is 0xF800.
    assert_eq!(FbFormat::RGB565.decode(&[0x00, 0xf8]), [0xff, 0x00, 0x00]);
    // Pure green is 0x07E0, pure blue 0x001F.
    assert_eq!(FbFormat::RGB565.decode(&[0xe0, 0x07]), [0x00, 0xff, 0x00]);
    assert_eq!(FbFormat::RGB565.decode(&[0x1f, 0x00]), [0x00, 0x00, 0xff]);
}

// ---------------------------------------------------------------------------
// Scanning out
// ---------------------------------------------------------------------------

#[test]
fn a_disabled_engine_shows_nothing() {
    let (engine, _space, ram) = small();
    ram.write_at(0x100, &[0xff; 36]).unwrap();
    let mut row = [[9u8; 3]; 4];
    assert!(!engine.read_row(0, &mut row), "nothing was scanned");
    assert_eq!(row, [[0, 0, 0]; 4], "and the destination is black");
}

#[test]
fn an_enabled_engine_reads_the_framebuffer_out_of_the_address_space() {
    let (engine, _space, ram) = small();
    // Three rows of four RGB888 pixels: 0x100..0x124.
    let mut fb = Vec::new();
    for y in 0..3u8 {
        for x in 0..4u8 {
            fb.extend_from_slice(&[x, y, x + y]);
        }
    }
    ram.write_at(0x100, &fb).unwrap();
    poke(&engine, 0x00, 1); // CTRL.EN

    for y in 0..3u32 {
        let mut row = [[0u8; 3]; 4];
        assert!(engine.read_row(y, &mut row));
        for x in 0..4u32 {
            assert_eq!(
                row[x as usize],
                [x as u8, y as u8, (x + y) as u8],
                "pixel ({x}, {y})"
            );
        }
    }
}

#[test]
fn a_row_past_the_bottom_is_black_rather_than_a_fault() {
    let (engine, _space, _ram) = small();
    poke(&engine, 0x00, 1);
    let mut row = [[9u8; 3]; 4];
    assert!(!engine.read_row(3, &mut row));
    assert_eq!(row, [[0, 0, 0]; 4]);
}

#[test]
fn a_base_pointing_at_a_hole_shows_black_rather_than_faulting() {
    // A guest that programs a wrong address gets a black screen, which is what
    // the hardware does, rather than taking the emulator down.
    let (engine, _space, _ram) = small();
    poke(&engine, 0x00, 1);
    poke(&engine, 0x04, 0xf000_0000); // nothing is mapped there
    let mut row = [[9u8; 3]; 4];
    assert!(!engine.read_row(0, &mut row));
    assert_eq!(row, [[0, 0, 0]; 4]);
}

#[test]
fn stride_puts_the_rows_where_the_guest_says() {
    let (engine, _space, ram) = small();
    // A 16-byte stride with only 12 bytes of pixels: four bytes of padding a
    // row, which is exactly why a stride register exists.
    poke(&engine, 0x00, 1);
    poke(&engine, 0x0c, 16);
    let mut fb = vec![0u8; 16 * 3];
    for y in 0..3usize {
        fb[y * 16] = 0x10 + y as u8;
    }
    ram.write_at(0x100, &fb).unwrap();
    for y in 0..3u32 {
        let mut row = [[0u8; 3]; 4];
        engine.read_row(y, &mut row);
        assert_eq!(row[0][0], 0x10 + y as u8, "row {y} starts at its stride");
    }
    assert_eq!(engine.stride(), 16);
    poke(&engine, 0x0c, 0);
    assert_eq!(engine.stride(), 12, "0 means width x bytes-per-pixel");
}

#[test]
fn the_frame_is_read_at_capture_time_so_a_flip_can_tear() {
    // The behaviour the module docs promise: nothing is buffered, so the base
    // register that is live *now* is the one that is read. A model that
    // stabilised this would hide a guest's missing VSYNC wait.
    let (engine, _space, ram) = small();
    poke(&engine, 0x00, 1);
    ram.write_at(0x100, &[0x11; 36]).unwrap();
    ram.write_at(0x200, &[0x22; 36]).unwrap();

    let mut row = [[0u8; 3]; 4];
    engine.read_row(0, &mut row);
    assert_eq!(row[0], [0x11, 0x11, 0x11]);

    // The guest flips, mid-capture as far as anything here knows.
    poke(&engine, 0x04, 0x200);
    engine.read_row(0, &mut row);
    assert_eq!(row[0], [0x22, 0x22, 0x22], "the new buffer, immediately");
}

#[test]
fn read_frame_returns_the_whole_picture() {
    let (engine, _space, ram) = small();
    poke(&engine, 0x00, 1);
    ram.write_at(0x100, &[0x40; 36]).unwrap();
    let frame = read_frame(&engine);
    assert_eq!(frame.len(), 3);
    assert_eq!(frame[0].len(), 4);
    assert!(frame.iter().all(|row| row.iter().all(|p| *p == [0x40; 3])));
}

#[test]
fn a_scanout_read_is_a_debug_access() {
    // A host redrawing a window is not the guest making an access, and a
    // framebuffer that happened to overlap a device must not have its FIFO
    // popped by a screenshot (`ROADMAP.md` §15, invariant 5).
    let (engine, _space, _ram) = small();
    assert!(engine.attrs(RequesterId::ANONYMOUS).debug);
}

// ---------------------------------------------------------------------------
// Time
// ---------------------------------------------------------------------------

#[test]
fn the_frame_period_comes_from_the_clock_domain_not_from_sixty_hertz() {
    let (engine, _space, _ram) = fixture(&[
        ("width", Value::Uint(320)),
        ("height", Value::Uint(240)),
        ("htotal", Value::Uint(371)),
        ("vtotal", Value::Uint(260)),
    ]);
    assert_eq!(engine.frame_ticks(), 371 * 260);
    assert_eq!(engine.frame_period_nanos(), 0, "not known before bind");

    // 6 MHz, the ST7272A's typical DCLK (datasheet §7.3.4).
    set_frame_rate(&engine, 6_000_000, 1);
    // 96,460 ticks at 6 MHz is 16.0767 ms, or about 62.2 frames a second —
    // which is what the datasheet's typicals actually give, not 60.
    assert_eq!(
        engine.frame_period_nanos(),
        96_460 * 1_000_000_000 / 6_000_000
    );
    assert_eq!(engine.frame_period_nanos(), 16_076_666);

    // A rational rate stays exact: 236250000/11 Hz is the NES master clock and
    // the kind of number this arithmetic exists for.
    set_frame_rate(&engine, 236_250_000, 11);
    assert_eq!(
        engine.frame_period_nanos(),
        96_460u64 * 11 * 1_000_000_000 / 236_250_000
    );
    set_frame_rate(&engine, 0, 1);
    assert_eq!(
        engine.frame_period_nanos(),
        0,
        "an impossible rate is no rate"
    );
}

#[test]
fn frames_are_counted_only_while_the_engine_is_enabled() {
    let (engine, _space, _ram) = small();
    assert_eq!(engine.frame_ticks(), 4 * 3, "no blanking by default");
    engine.advance_to(100);
    assert_eq!(engine.frame(), 0, "a disabled controller scans nothing");
    poke(&engine, 0x00, 1);
    engine.advance_to(112);
    assert_eq!(engine.frame(), 1);
    engine.advance_to(160);
    assert_eq!(engine.frame(), 5);
    assert_eq!(peek(&engine, 0x1c), 5, "and FRAMES reports it");
}

#[test]
fn the_next_frame_boundary_is_always_ahead_of_the_present() {
    let (engine, _space, _ram) = small();
    for tick in [0u64, 1, 11, 12, 13, 1000] {
        engine.advance_to(tick);
        let next = Device::next_event_tick(&engine).expect("always a next frame");
        assert!(
            next > Device::current_tick(&engine),
            "at {tick}: {next} must be past the present"
        );
    }
}

// ---------------------------------------------------------------------------
// Registers
// ---------------------------------------------------------------------------

#[test]
fn the_registers_read_back_what_was_written() {
    let (engine, _space, _ram) = small();
    poke(&engine, 0x00, 0xffff_ffff);
    assert_eq!(peek(&engine, 0x00), 1, "only EN is defined");
    poke(&engine, 0x04, 0x1234_5678);
    poke(&engine, 0x08, 0x9abc_def0);
    assert_eq!(engine.base(), 0x9abc_def0_1234_5678);
    assert_eq!(peek(&engine, 0x04), 0x1234_5678);
    assert_eq!(peek(&engine, 0x08), 0x9abc_def0);
    poke(&engine, 0x10, 640);
    poke(&engine, 0x14, 480);
    assert_eq!(engine.geometry(), (640, 480));
    poke(&engine, 0x18, u32::from(FbFormat::RGB565.0));
    assert_eq!(engine.format(), FbFormat::RGB565);
}

#[test]
fn a_zero_geometry_is_clamped_rather_than_faulted() {
    let (engine, _space, _ram) = small();
    poke(&engine, 0x10, 0);
    poke(&engine, 0x14, 0);
    assert_eq!(engine.geometry(), (1, 1));
}

#[test]
fn a_debug_write_is_refused_rather_than_moving_the_framebuffer() {
    let (engine, _space, _ram) = small();
    let region = engine.region("").unwrap();
    let err = ops(&region)
        .write(0x04, &0u32.to_le_bytes(), MemAttrs::DEBUG)
        .unwrap_err();
    assert!(matches!(err, BusError::BadAccess));
}

#[test]
fn the_register_block_is_word_wide_only() {
    let (engine, _space, _ram) = small();
    let region = engine.region("").unwrap();
    let block = ops(&region);
    let mut byte = [0u8; 1];
    assert!(block.read(0x00, &mut byte, MemAttrs::DEFAULT).is_err());
    let mut word = [0u8; 4];
    assert!(block.read(0x02, &mut word, MemAttrs::DEFAULT).is_err());
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

#[test]
fn a_geometry_is_required() {
    let err = Scanout::new(&Props::new()).unwrap_err().to_string();
    assert!(err.contains("width"), "{err}");
}

#[test]
fn an_unknown_format_names_the_ones_that_exist() {
    let err = Scanout::new(
        &Props::new()
            .with("width", Value::Uint(4))
            .with("height", Value::Uint(4))
            .with("format", Value::Str("yuv".to_string())),
    )
    .unwrap_err()
    .to_string();
    assert!(err.contains("rgb888"), "{err}");
}

#[test]
fn totals_smaller_than_the_visible_area_are_refused() {
    let err = Scanout::new(
        &Props::new()
            .with("width", Value::Uint(320))
            .with("height", Value::Uint(240))
            .with("htotal", Value::Uint(100)),
    )
    .unwrap_err()
    .to_string();
    assert!(err.contains("blanking"), "{err}");
}

#[test]
fn an_engine_with_no_address_space_shows_nothing() {
    // A bus master with nothing to master is a machine-file bug, and
    // `Instance::bind` refuses it — the realizer hands a device its space
    // there, *after* every region is mapped, rather than at realize, so
    // realize itself must not be the place that checks. Reaching a `BindCtx`
    // needs a whole realized machine, so what is checked here is the other half
    // of the promise: an engine with no bus is black rather than a panic on the
    // first capture.
    let engine = Scanout::new(
        &Props::new()
            .with("width", Value::Uint(4))
            .with("height", Value::Uint(4)),
    )
    .unwrap();
    let mut deferred = crate::core::device::Deferred::new();
    let ctx_hosts = crate::core::HostObjects::new();
    let mut ctx = RealizeCtx::new("lcdc", RequesterId::ANONYMOUS, &mut deferred, &ctx_hosts);
    engine
        .realize(&mut ctx)
        .expect("realize does not need the space");
    let mut row = [[9u8; 3]; 4];
    assert!(!engine.read_row(0, &mut row));
    assert_eq!(row, [[0, 0, 0]; 4]);
}

// ---------------------------------------------------------------------------
// Snapshots
// ---------------------------------------------------------------------------

#[test]
fn the_engine_round_trips_through_a_snapshot() {
    let (engine, space, ram) = small();
    poke(&engine, 0x00, 1);
    poke(&engine, 0x04, 0x140);
    poke(&engine, 0x0c, 24);
    poke(&engine, 0x10, 6);
    poke(&engine, 0x14, 2);
    poke(&engine, 0x18, u32::from(FbFormat::BGR888.0));
    engine.advance_to(500);
    ram.write_at(0x140, &[7u8; 48]).unwrap();

    let mut shape = MachineShape::new();
    shape.add_device("lcdc", SCANOUT_CLASS.name).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w
            .chunk("lcdc", SCANOUT_CLASS.name, SCANOUT_CLASS.version)
            .unwrap();
        engine.save(&mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();

    let (other, _space2, _ram2) = small();
    *other.shared.bus.lock() = Some(Arc::clone(&space));
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load(
            "lcdc",
            SCANOUT_CLASS.name,
            SCANOUT_CLASS.version,
            &Migrations::new(),
        )
        .unwrap();
    other.load(&mut chunk.reader()).unwrap();

    assert_eq!(other.base(), engine.base());
    assert_eq!(other.stride(), engine.stride());
    assert_eq!(other.geometry(), engine.geometry());
    assert_eq!(other.format(), engine.format());
    assert_eq!(other.frame(), engine.frame());
    assert!(other.enabled());
    assert_eq!(Device::current_tick(&other), Device::current_tick(&engine));
    // The picture is identical, which is the property a frame hash compares.
    assert_eq!(read_frame(&other), read_frame(&engine));

    // The chunk is small: the framebuffer is ordinary guest memory and the RAM
    // device saves it once, so a display controller must not save it again
    // (`ROADMAP.md` §4.5).
    assert!(
        bytes.len() < 256,
        "the chunk is {} bytes; it must not contain a framebuffer",
        bytes.len()
    );
}

#[test]
fn a_cold_reset_returns_the_registers_to_what_the_machine_file_said() {
    let (engine, _space, _ram) = small();
    poke(&engine, 0x00, 1);
    poke(&engine, 0x04, 0x999);
    poke(&engine, 0x10, 99);
    engine.advance_to(1000);
    engine.reset(ResetKind::Cold);
    assert!(!engine.enabled());
    assert_eq!(engine.base(), 0x100, "the `base` property, not zero");
    assert_eq!(engine.geometry(), (4, 3));
    assert_eq!(engine.frame(), 0);
}
