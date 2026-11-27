//! Tests for the S-PPU.
//!
//! Register semantics are tested through the [`MemOps`] a guest reaches, so
//! what is tested is the bus surface; scenes are built by poking memory
//! directly and asserting framebuffer pixels.

use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::{Device, ExportId, ResetKind};
use crate::core::props::Props;
use crate::core::space::{MemAttrs, MemOps, RegionKind};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::sync::Mutex;
use crate::core::wire::{Level, Wire, WireId, WireIdAllocator, WireSink, WireSource};

use super::timing::{MASTER_PER_LINE, dot_start};
use super::*;

// ---------------------------------------------------------------------------
// Harness
// ---------------------------------------------------------------------------

fn ppu() -> SnesPpu {
    SnesPpu::default()
}

fn port(ppu: &SnesPpu) -> Arc<dyn MemOps> {
    let region = Device::region(ppu, REGISTER_REGION).expect("the register block");
    match region.kind() {
        RegionKind::Io(ops) => Arc::clone(ops),
        other => panic!("expected an I/O region, found {other:?}"),
    }
}

fn rd(ops: &Arc<dyn MemOps>, reg: u8) -> u8 {
    let mut b = [0];
    ops.read(u64::from(reg), &mut b, MemAttrs::DEFAULT.with_bus(0xA5))
        .expect("a mapped byte");
    b[0]
}

fn peek(ops: &Arc<dyn MemOps>, reg: u8) -> u8 {
    let mut b = [0];
    ops.read(u64::from(reg), &mut b, MemAttrs::DEBUG.with_bus(0xA5))
        .expect("a mapped byte");
    b[0]
}

fn wr(ops: &Arc<dyn MemOps>, reg: u8, value: u8) {
    ops.write(u64::from(reg), &[value], MemAttrs::DEFAULT)
        .expect("an accepted write");
}

/// Advance to the start of dot `h` of line `v`.
fn seek(ppu: &SnesPpu, v: u16, h: u16) {
    let d = ppu
        .with_engine(|e| e.master_until(v, h))
        .expect("a position that exists");
    ppu.advance_by(d);
    let p = ppu.position();
    assert_eq!((p.v, p.h), (v, h));
}

/// Draw line `v` with whatever is set up, and return its framebuffer row as
/// 256 low-resolution (main-screen) pixels.
fn draw(ppu: &SnesPpu, v: u16) -> Vec<u16> {
    seek(ppu, v, 23);
    let row = usize::from(v - 1);
    ppu.with_framebuffer(|fb| (0..256).map(|x| fb[fb_index(x, row, 256, 224)]).collect())
}

/// A full 512-entry row, for hires.
fn draw_wide(ppu: &SnesPpu, v: u16) -> Vec<u16> {
    seek(ppu, v, 23);
    let row = 2 * usize::from(v - 1);
    ppu.with_framebuffer(|fb| fb[row * FB_WIDTH..(row + 1) * FB_WIDTH].to_vec())
}

const RED: u16 = 0x001F;
const GREEN: u16 = 0x03E0;
const BLUE: u16 = 0x7C00;
const WHITE: u16 = 0x7FFF;

/// Display on, full brightness.
fn display_on(ppu: &SnesPpu) {
    ppu.write_register(0x00, 0x0F);
}

/// Fill 2bpp/4bpp/8bpp tile `tile` at `char_base` so every pixel is `value`.
fn solid_tile(e: &mut Engine, char_base: usize, bpp: usize, tile: usize, value: u16) {
    let words = bpp * 4;
    for row in 0..8 {
        for pair in 0..bpp / 2 {
            let lo = if value >> (2 * pair) & 1 != 0 {
                0xFF
            } else {
                0
            };
            let hi = if value >> (2 * pair + 1) & 1 != 0 {
                0xFF
            } else {
                0
            };
            e.vram[char_base + tile * words + row + 8 * pair] = lo | (hi << 8);
        }
    }
}

/// Fill a whole 32×32 tilemap at `base` with `entry`.
fn fill_map(e: &mut Engine, base: usize, entry: u16) {
    for i in 0..0x400 {
        e.vram[base + i] = entry;
    }
}

// ---------------------------------------------------------------------------
// Registers and latches
// ---------------------------------------------------------------------------

#[test]
fn bg_scroll_writes_share_one_latch() {
    let p = ppu();
    let ops = port(&p);
    wr(&ops, 0x0D, 0x12);
    wr(&ops, 0x0D, 0x03);
    // HOFS = (value << 8) | (latch & !7) | ((HOFS >> 8) & 7)
    let r = p.with_engine(|e| e.r);
    assert_eq!(r.hofs[0], 0x312);
    // The mode-7 half of $210D has its own latch: plain (value << 8) | prev.
    assert_eq!(r.m7hofs, 0x0312);
    // The BG latch now holds $03, and BG2's VOFS sees it.
    wr(&ops, 0x10, 0x34);
    assert_eq!(p.with_engine(|e| e.r.vofs[1]), 0x003);
    wr(&ops, 0x10, 0x01);
    assert_eq!(p.with_engine(|e| e.r.vofs[1]), 0x134);
}

#[test]
fn mode7_registers_feed_the_multiplier() {
    let p = ppu();
    let ops = port(&p);
    wr(&ops, 0x1B, 0x00);
    wr(&ops, 0x1B, 0x01); // M7A = $0100
    wr(&ops, 0x1C, 0x05); // the last M7B byte is 5
    assert_eq!(
        [rd(&ops, 0x34), rd(&ops, 0x35), rd(&ops, 0x36)],
        [0x00, 0x05, 0x00]
    );
    wr(&ops, 0x1B, 0xFF);
    wr(&ops, 0x1B, 0xFF); // M7A = -1
    wr(&ops, 0x1C, 0x02);
    assert_eq!(
        [rd(&ops, 0x34), rd(&ops, 0x35), rd(&ops, 0x36)],
        [0xFE, 0xFF, 0xFF],
        "-1 x 2, 24-bit signed"
    );
    // The two-write latch is shared with M7X: the previous byte was $02.
    wr(&ops, 0x1F, 0x01);
    assert_eq!(p.with_engine(|e| e.r.m7x), 0x0102);
}

#[test]
fn vram_writes_step_on_the_byte_vmain_names() {
    let p = ppu();
    let ops = port(&p);
    wr(&ops, 0x15, 0x80);
    wr(&ops, 0x16, 0x00);
    wr(&ops, 0x17, 0x10);
    wr(&ops, 0x18, 0x34);
    assert_eq!(p.with_engine(|e| e.r.vmaddr), 0x1000);
    wr(&ops, 0x19, 0x12);
    assert_eq!(
        p.with_engine(|e| (e.vram[0x1000], e.r.vmaddr)),
        (0x1234, 0x1001)
    );

    wr(&ops, 0x15, 0x01); // step 32 on the low byte
    wr(&ops, 0x18, 0x56);
    assert_eq!(
        p.with_engine(|e| (e.vram[0x1001], e.r.vmaddr)),
        (0x0056, 0x1021)
    );
    wr(&ops, 0x15, 0x02);
    wr(&ops, 0x18, 0);
    assert_eq!(p.with_engine(|e| e.r.vmaddr), 0x10A1);
}

#[test]
fn vmain_remaps_the_low_address_bits() {
    let p = ppu();
    p.write_register(0x15, 0x04);
    assert_eq!(p.with_engine(|e| e.vram_remap(0x0021)), 0x0009);
    p.write_register(0x15, 0x08);
    assert_eq!(p.with_engine(|e| e.vram_remap(0x0041)), 0x0009);
    p.write_register(0x15, 0x0C);
    assert_eq!(p.with_engine(|e| e.vram_remap(0x0081)), 0x0009);
    assert_eq!(p.with_engine(|e| e.vram_remap(0xFF81)), 0x7C0F);
}

#[test]
fn vram_reads_answer_from_the_prefetch() {
    let p = ppu();
    let ops = port(&p);
    p.with_engine(|e| {
        e.vram[0x10] = 0xAAAA;
        e.vram[0x11] = 0xBBBB;
    });
    wr(&ops, 0x15, 0x80);
    wr(&ops, 0x16, 0x10);
    wr(&ops, 0x17, 0x00);
    assert_eq!(rd(&ops, 0x39), 0xAA);
    assert_eq!(rd(&ops, 0x3A), 0xAA);
    // The incrementing read refilled from $10 and then stepped.
    assert_eq!(p.with_engine(|e| e.r.vmaddr), 0x11);
    assert_eq!(rd(&ops, 0x3A), 0xAA);
    assert_eq!(rd(&ops, 0x39), 0xBB);
}

#[test]
fn vram_is_read_only_during_the_active_display() {
    let p = ppu();
    let ops = port(&p);
    display_on(&p);
    seek(&p, 50, 100);
    wr(&ops, 0x15, 0x80);
    wr(&ops, 0x16, 0x00);
    wr(&ops, 0x17, 0x00);
    wr(&ops, 0x18, 0x11);
    wr(&ops, 0x19, 0x22);
    assert_eq!(p.with_engine(|e| (e.vram[0], e.r.vmaddr)), (0, 1));
    seek(&p, 230, 0);
    wr(&ops, 0x18, 0x11);
    wr(&ops, 0x19, 0x22);
    assert_eq!(p.with_engine(|e| e.vram[1]), 0x2211, "vblank accepts it");
}

#[test]
fn cgram_takes_words_through_a_flip_flop() {
    let p = ppu();
    let ops = port(&p);
    wr(&ops, 0x21, 5);
    wr(&ops, 0x22, 0x34);
    assert_eq!(p.with_engine(|e| e.cgram[5]), 0);
    wr(&ops, 0x22, 0xFF);
    assert_eq!(p.with_engine(|e| e.cgram[5]), 0x7F34);
    wr(&ops, 0x21, 5);
    assert_eq!(rd(&ops, 0x3B), 0x34);
    // Bit 7 of the high byte is PPU2 open bus: the $34 just read.
    assert_eq!(rd(&ops, 0x3B), 0x7F);
    assert_eq!(p.with_engine(|e| e.r.cgadd), 6);
}

#[test]
fn oam_low_table_writes_commit_in_pairs_and_the_high_table_at_once() {
    let p = ppu();
    let ops = port(&p);
    wr(&ops, 0x02, 0x00);
    wr(&ops, 0x03, 0x00);
    wr(&ops, 0x04, 0x11);
    assert_eq!(p.with_engine(|e| e.oam[0]), 0, "held in the latch");
    wr(&ops, 0x04, 0x22);
    assert_eq!(p.with_engine(|e| [e.oam[0], e.oam[1]]), [0x11, 0x22]);

    wr(&ops, 0x02, 0x00);
    wr(&ops, 0x03, 0x01); // word $100 = byte $200, the high table
    wr(&ops, 0x04, 0x55);
    assert_eq!(p.with_engine(|e| e.oam[0x200]), 0x55);
    // $220-$3FF mirror the 32 high-table bytes.
    p.with_engine(|e| e.r.oam_addr = 0x221);
    wr(&ops, 0x04, 0x66);
    assert_eq!(p.with_engine(|e| e.oam[0x201]), 0x66);

    wr(&ops, 0x02, 0x00);
    wr(&ops, 0x03, 0x00);
    assert_eq!(rd(&ops, 0x38), 0x11);
    assert_eq!(rd(&ops, 0x38), 0x22);
    assert_eq!(p.with_engine(|e| e.r.oam_addr), 2);
}

#[test]
fn the_oam_address_reloads_at_vblank_unless_forced_blank() {
    let p = ppu();
    let ops = port(&p);
    wr(&ops, 0x02, 0x10);
    wr(&ops, 0x04, 0x00);
    assert_eq!(p.with_engine(|e| e.r.oam_addr), 0x21);
    seek(&p, 226, 0);
    assert_eq!(p.with_engine(|e| e.r.oam_addr), 0x21, "forced blank: kept");
    display_on(&p);
    wr(&ops, 0x04, 0x00);
    seek(&p, 225, 1);
    assert_eq!(p.with_engine(|e| e.r.oam_addr), 0x20);
}

#[test]
fn write_only_registers_read_as_the_right_open_bus() {
    let p = ppu();
    let ops = port(&p);
    // A PPU1 read loads PPU1's latch.
    wr(&ops, 0x1B, 0x00);
    wr(&ops, 0x1B, 0x01);
    wr(&ops, 0x1C, 0x42);
    assert_eq!(rd(&ops, 0x35), 0x42);
    for reg in PPU1_OPEN_BUS {
        assert_eq!(rd(&ops, reg), 0x42, "${:04X}", 0x2100 + u16::from(reg));
    }
    // Everything else write-only floats the S-CPU's own bus.
    for reg in [0x00, 0x01, 0x07, 0x0D, 0x21, 0x33, SLHV] {
        assert_eq!(rd(&ops, reg), 0xA5, "${:04X}", 0x2100 + u16::from(reg));
    }
}

#[test]
fn status_registers_carry_versions_region_and_open_bus() {
    let p = SnesPpu::new(Region::Pal, 1, 2);
    let ops = port(&p);
    assert_eq!(rd(&ops, STAT77) & 0x0F, 1);
    let s78 = rd(&ops, STAT78);
    assert_eq!(s78 & 0x0F, 2);
    assert_eq!(s78 & 0x10, 0x10, "PAL");
    assert_eq!(rd(&ops, STAT78) & 0x10, 0x10);
    let ntsc = ppu();
    assert_eq!(rd(&port(&ntsc), STAT78) & 0x1F, DEFAULT_PPU2_VERSION);
}

#[test]
fn props_pick_region_and_versions() {
    let p = SnesPpu::from_props(
        &Props::new()
            .with("region", "pal")
            .with("ppu1-version", 1u64)
            .with("ppu2-version", 1u64),
    )
    .unwrap();
    assert_eq!(p.tv_region(), Region::Pal);
    assert_eq!(p.peek_register(STAT78, 0) & 0x0F, 1);
    assert!(SnesPpu::from_props(&Props::new().with("ppu2-version", 16u64)).is_err());
    assert!(SnesPpu::from_props(&Props::new().with("region", "secam")).is_err());
}

#[test]
fn the_counters_latch_through_slhv_only_while_extlatch_is_high() {
    let p = ppu();
    let ops = port(&p);
    seek(&p, 100, 200);
    rd(&ops, SLHV);
    assert_eq!(rd(&ops, STAT78) & 0x40, 0, "EXTLATCH low: no latch");
    p.beam().set_extlatch(true);
    rd(&ops, SLHV);
    assert_eq!(rd(&ops, OPHCT), 200);
    assert_eq!(rd(&ops, OPHCT) & 1, 0);
    assert_eq!(rd(&ops, OPVCT), 100);
    assert_eq!(rd(&ops, STAT78) & 0x40, 0x40);
    assert_eq!(rd(&ops, STAT78) & 0x40, 0, "the STAT78 read cleared it");
}

#[test]
fn a_falling_extlatch_latches_and_stat78_resets_the_flip_flops() {
    let p = ppu();
    let ops = port(&p);
    seek(&p, 300 - 39, 300);
    let beam = p.beam();
    beam.set_extlatch(true);
    beam.set_extlatch(false);
    // 300 = $12C: low byte, then bit 8.
    assert_eq!(rd(&ops, OPHCT), 0x2C);
    rd(&ops, STAT78);
    assert_eq!(rd(&ops, OPHCT), 0x2C, "the flip-flop was reset");
    assert_eq!(rd(&ops, OPHCT) & 1, 1);
    assert_eq!(rd(&ops, OPVCT), 0x05, "261 = $105");
}

#[test]
fn the_extlatch_pin_is_a_wire_sink() {
    let p = ppu();
    let sink = p.sink(EXTLATCH_PIN, &[]).expect("an input");
    assert!(p.sink("nope", &[]).is_none());
    seek(&p, 10, 20);
    let id = WireIdAllocator::new().alloc();
    sink.sink.set_level(id, 0, Level::High);
    sink.sink.set_level(id, 0, Level::Low);
    assert_eq!(
        p.with_engine(|e| (e.r.hlatch, e.r.vlatch, e.r.latched)),
        (20, 10, true)
    );
}

#[test]
fn debug_reads_have_no_side_effects() {
    let p = ppu();
    let ops = port(&p);
    p.with_engine(|e| {
        e.vram[0] = 0x1111;
        e.vram[1] = 0x2222;
        e.oam[0] = 0x33;
        e.cgram[0] = 0x4444;
    });
    wr(&ops, 0x16, 0);
    wr(&ops, 0x17, 0);
    p.beam().set_extlatch(true);
    rd(&ops, SLHV);
    let before = p.with_engine(|e| e.r);
    for reg in 0x00..0x40 {
        peek(&ops, reg);
        peek(&ops, reg);
    }
    assert_eq!(p.with_engine(|e| e.r), before, "a debugger moved a latch");
    // And they still say what a guest read would.
    assert_eq!(peek(&ops, RDVRAML), 0x11);
    assert_eq!(peek(&ops, RDOAM), 0x33);
    assert_eq!(peek(&ops, STAT78) & 0x40, 0x40);
    // A debug write is refused silently.
    ops.write(0x00, &[0x0F], MemAttrs::DEBUG).unwrap();
    assert_eq!(p.with_engine(|e| e.r.inidisp), 0x80);
}

#[test]
fn only_byte_accesses_reach_the_block() {
    let p = ppu();
    let ops = port(&p);
    let mut two = [0u8; 2];
    assert!(ops.read(0, &mut two, MemAttrs::DEFAULT).is_err());
    assert!(ops.write(0, &[1, 2], MemAttrs::DEFAULT).is_err());
}

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------

/// Master clocks from the start of one frame to the next.
fn frame_length(p: &SnesPpu) -> u64 {
    seek(p, 0, 0);
    let start = p.master();
    p.advance_by(1);
    seek(p, 0, 0);
    p.master() - start
}

#[test]
fn ntsc_frames_alternate_with_the_short_line() {
    let p = ppu();
    // The first frame starts at master 0 with field 0.
    let start_field = p.position().field;
    p.advance_by(1);
    seek(&p, 0, 0);
    let first = p.master();
    assert!(!start_field);
    assert_eq!(first, 262 * 1364);
    let second = frame_length(&p);
    let third = frame_length(&p);
    // Field 1 frames carry the 1360-clock line 240.
    assert_eq!(second + third, 2 * 262 * 1364 - 4);
}

#[test]
fn pal_frames_are_312_lines() {
    let p = SnesPpu::new(Region::Pal, 1, 3);
    assert_eq!(frame_length(&p), 312 * 1364);
    assert_eq!(frame_length(&p), 312 * 1364);
}

#[test]
fn interlace_adds_a_line_and_pal_gets_its_long_line() {
    let p = SnesPpu::new(Region::Pal, 1, 3);
    p.write_register(0x33, 0x01);
    // The frame in progress latched interlace off at its start.
    p.advance_by(1);
    seek(&p, 0, 0);
    let a = frame_length(&p);
    let b = frame_length(&p);
    // One frame 313 lines, the other 312 with the 1368-clock line 311.
    assert_eq!(a + b, 313 * 1364 + 312 * 1364 + 4);
    let n = ppu();
    n.write_register(0x33, 0x01);
    n.advance_by(1);
    seek(&n, 0, 0);
    let a = frame_length(&n);
    let b = frame_length(&n);
    assert_eq!(a + b, (263 + 262) * 1364, "no short line in interlace");
}

#[test]
fn a_line_is_340_dots_with_two_long_ones() {
    let p = ppu();
    seek(&p, 10, 0);
    let start = p.master();
    seek(&p, 10, 323);
    assert_eq!(p.master() - start, 323 * 4);
    p.advance_by(4);
    assert_eq!(p.position().h, 323, "still in the six-clock dot");
    p.advance_by(2);
    assert_eq!(p.position().h, 324);
    seek(&p, 11, 0);
    assert_eq!(p.master() - start, u64::from(MASTER_PER_LINE));
}

#[test]
fn blank_levels_move_on_their_documented_dots() {
    let p = ppu();
    seek(&p, 100, 0);
    assert!(p.with_engine(|e| e.hblank));
    seek(&p, 100, 1);
    assert!(!p.with_engine(|e| e.hblank));
    seek(&p, 100, 273);
    assert!(!p.with_engine(|e| e.hblank));
    seek(&p, 100, 274);
    assert!(p.with_engine(|e| e.hblank));
    seek(&p, 224, 339);
    assert!(!p.with_engine(|e| e.vblank));
    seek(&p, 225, 0);
    assert!(p.with_engine(|e| e.vblank));
    seek(&p, 261, 339);
    assert!(p.with_engine(|e| e.vblank));
    seek(&p, 0, 0);
    assert!(!p.with_engine(|e| e.vblank));
}

#[test]
fn overscan_moves_vblank_to_line_240() {
    let p = ppu();
    p.write_register(0x33, 0x04);
    seek(&p, 225, 0);
    assert!(!p.with_engine(|e| e.vblank));
    assert_eq!(p.beam().vblank_line(), 240);
    seek(&p, 240, 0);
    assert!(p.with_engine(|e| e.vblank));
}

#[derive(Default)]
struct Recorder {
    levels: Mutex<Vec<bool>>,
}

impl WireSink for Recorder {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        self.levels.lock().push(level.is_high());
    }
}

#[test]
fn the_pins_drive_every_edge() {
    let p = ppu();
    let ids = WireIdAllocator::new();
    let mut sinks = Vec::new();
    for pin in [VBLANK_PIN, HBLANK_PIN] {
        let id = ids.alloc();
        let sink = Arc::new(Recorder::default());
        let wire = Wire::builder()
            .source(id)
            .sink(Arc::clone(&sink) as Arc<dyn WireSink>, 0)
            .build_shared();
        p.connect(pin, WireSource::new(wire, id)).unwrap();
        sinks.push(sink);
    }
    assert!(
        p.connect(
            "nmi",
            WireSource::new(Wire::builder().build_shared(), ids.alloc())
        )
        .is_err()
    );
    p.announce(HBLANK_PIN);
    assert_eq!(sinks[1].levels.lock().last(), Some(&true));
    // Run one whole frame in one call.
    p.advance_by(262 * 1364);
    let v: Vec<bool> = sinks[0].levels.lock().clone();
    assert!(v.contains(&true), "vblank rose: {v:?}");
    assert_eq!(v.last(), Some(&false), "and fell at V=0");
    let h = sinks[1].levels.lock().len();
    assert!(h >= 2 * 262, "every hblank edge was delivered, got {h}");
}

#[test]
fn master_until_counts_long_dots_and_frame_wrap() {
    let p = ppu();
    let e = |v, h| p.with_engine(|e| e.master_until(v, h));
    assert_eq!(e(0, 0), Some(0));
    assert_eq!(e(0, 330), Some(u64::from(dot_start(330, true))));
    assert_eq!(e(1, 0), Some(1364));
    assert_eq!(e(262, 0), None, "no line 262 without interlace");
    assert_eq!(e(0, 340), None, "no dot 340 on NTSC");
    p.advance_by(5);
    assert_eq!(e(0, 0), Some(262 * 1364 - 5));
}

#[test]
fn the_beam_is_published_on_its_export_id() {
    let p = ppu();
    let export = p.export(ExportId::SNES_PPU_BEAM).expect("published");
    let beam = Arc::clone(export.opaque().expect("opaque"))
        .downcast::<SnesPpuBeam>()
        .expect("the beam type");
    p.advance_by(1364 * 3 + 40);
    assert_eq!(
        beam.position(),
        BeamPosition {
            h: 10,
            v: 3,
            field: false,
            master_in_line: 40
        }
    );
    assert_eq!(beam.region(), Region::Ntsc);
    assert_eq!(beam.lines_in_frame(), 262);
    assert!(!beam.vblank());
    assert!(!beam.hblank());
    assert!(p.export(ExportId::TIMEBASE).is_none());
    beam.latch_counters();
    assert_eq!(p.peek_register(OPHCT, 0), 10);
}

#[test]
fn the_frame_counter_ticks_at_vblank() {
    let p = ppu();
    seek(&p, 224, 0);
    assert_eq!(p.frame(), 0);
    seek(&p, 225, 0);
    assert_eq!(p.frame(), 1);
    assert_eq!(p.beam().frame(), 1);
}

#[test]
fn reset_keeps_the_clock_and_a_warm_one_keeps_memory() {
    let p = ppu();
    p.advance_by(12_345);
    p.with_engine(|e| {
        e.vram[7] = 0x77;
        e.r.inidisp = 0x0F;
    });
    p.reset(ResetKind::Warm);
    assert_eq!(p.master(), 12_345);
    assert_eq!(p.with_engine(|e| (e.vram[7], e.r.inidisp)), (0x77, 0x80));
    p.reset(ResetKind::Cold);
    assert_eq!(p.with_engine(|e| e.vram[7]), 0);
    assert_eq!(p.master(), 12_345);
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------

#[test]
fn forced_blank_is_black_and_brightness_scales() {
    let p = ppu();
    p.with_engine(|e| e.cgram[0] = WHITE);
    assert!(draw(&p, 10).iter().all(|&c| c == 0), "forced blank");
    p.write_register(0x00, 0x0F);
    assert!(draw(&p, 11).iter().all(|&c| c == WHITE));
    p.write_register(0x00, 0x07);
    // 31 * 7 / 15 = 14 per channel.
    assert!(
        draw(&p, 12)
            .iter()
            .all(|&c| c == 14 | (14 << 5) | (14 << 10))
    );
    p.write_register(0x00, 0x00);
    assert!(draw(&p, 13).iter().all(|&c| c == 0));
}

/// Mode 0, BG `bg` showing solid 2bpp pixel `pixel` from palette `pal`.
fn mode0_layer(p: &SnesPpu, bg: usize, pixel: u16, pal: u16, prio: bool) {
    p.with_engine(|e| {
        let map = 0x400 * (bg + 1);
        e.r.bgsc[bg] = ((map >> 10) << 2) as u8;
        // All four BGs share character base $2000 words, tiles distinct.
        e.r.bgnba = [0x22, 0x22];
        solid_tile(e, 0x2000, 2, 1 + bg, pixel);
        fill_map(
            e,
            map,
            (1 + bg as u16) | (pal << 10) | (u16::from(prio) << 13),
        );
    });
}

#[test]
fn mode0_gives_each_bg_its_own_palette_section() {
    let p = ppu();
    display_on(&p);
    mode0_layer(&p, 1, 3, 2, false);
    p.with_engine(|e| {
        // BG2's colours start at 32: palette 2, pixel 3 is entry 32 + 8 + 3.
        e.cgram[43] = GREEN;
        e.cgram[11] = RED;
        e.r.tm = 0x02;
    });
    assert!(draw(&p, 5).iter().all(|&c| c == GREEN));
}

#[test]
fn mode0_priorities_put_bg1_high_over_bg3_high() {
    let p = ppu();
    display_on(&p);
    mode0_layer(&p, 0, 1, 0, true);
    mode0_layer(&p, 2, 1, 0, true);
    p.with_engine(|e| {
        e.cgram[1] = RED;
        e.cgram[64 + 1] = BLUE;
        e.r.tm = 0x05;
    });
    assert!(draw(&p, 5).iter().all(|&c| c == RED), "BG1 over BG3");
    p.with_engine(|e| e.r.tm = 0x04);
    assert!(draw(&p, 6).iter().all(|&c| c == BLUE));
}

/// Mode 1's BG1 (4bpp) and BG3 (2bpp), both with priority set.
fn mode1_scene(p: &SnesPpu) {
    p.with_engine(|e| {
        e.r.bgmode = 0x01;
        e.r.bgsc[0] = 0x04; // map at $0400
        e.r.bgsc[2] = 0x08; // map at $0800
        e.r.bgnba = [0x01, 0x02]; // BG1 chars $1000, BG3 chars $2000
        solid_tile(e, 0x1000, 4, 1, 0x5); // pixel 5
        solid_tile(e, 0x2000, 2, 1, 0x2);
        fill_map(e, 0x400, 1 | (3 << 10) | 0x2000); // palette 3
        fill_map(e, 0x800, 1 | 0x2000);
        e.cgram[3 * 16 + 5] = RED;
        e.cgram[2] = BLUE;
        e.r.tm = 0x05;
    });
}

#[test]
fn mode1_bg3_priority_bit_lifts_bg3_to_the_front() {
    let p = ppu();
    display_on(&p);
    mode1_scene(&p);
    assert!(draw(&p, 5).iter().all(|&c| c == RED));
    p.write_register(0x05, 0x09);
    assert!(draw(&p, 6).iter().all(|&c| c == BLUE));
}

#[test]
fn scroll_moves_the_tile_grid() {
    let p = ppu();
    display_on(&p);
    mode1_scene(&p);
    p.with_engine(|e| {
        e.r.tm = 0x01;
        // Column 1 of the map is tile 0 (transparent).
        for row in 0..32 {
            e.vram[0x400 + row * 32 + 1] = 0;
        }
        e.cgram[0] = GREEN;
    });
    let line = draw(&p, 5);
    assert_eq!(line[7], RED);
    assert_eq!(line[8], GREEN);
    assert_eq!(line[16], RED);
    p.with_engine(|e| e.r.hofs[0] = 4);
    let line = draw(&p, 6);
    assert_eq!(line[3], RED);
    assert_eq!(line[4], GREEN);
    assert_eq!(line[12], RED);
}

#[test]
fn vertical_scroll_and_16x16_tiles() {
    let p = ppu();
    display_on(&p);
    p.with_engine(|e| {
        e.r.bgmode = 0x11; // mode 1, BG1 16x16
        e.r.bgsc[0] = 0x04;
        e.r.bgnba = [0x01, 0];
        // A 16x16 tile 2 covers 2, 3, 18, 19; only 19 (bottom right) is solid.
        solid_tile(e, 0x1000, 4, 19, 1);
        fill_map(e, 0x400, 2);
        e.cgram[1] = RED;
        e.cgram[0] = BLUE;
        e.r.tm = 0x01;
        e.r.vofs[0] = 0x3FF; // -1, so V = 1 shows BG row 0
    });
    let top = draw(&p, 1);
    assert_eq!((top[0], top[8]), (BLUE, BLUE));
    let bottom = draw(&p, 9);
    assert_eq!(
        (bottom[7], bottom[8], bottom[15], bottom[16]),
        (BLUE, RED, RED, BLUE)
    );
}

#[test]
fn mode3_direct_colour_ignores_cgram() {
    let p = ppu();
    display_on(&p);
    p.with_engine(|e| {
        e.r.bgmode = 0x03;
        e.r.bgsc[0] = 0x04;
        e.r.bgnba = [0x01, 0];
        // Pixel %11_000_111: blue 3, green 0, red 7.
        solid_tile(e, 0x1000, 8, 1, 0xC7);
        fill_map(e, 0x400, 1 | (5 << 10)); // palette bits 101
        e.cgram[0xC7] = GREEN;
        e.r.tm = 0x01;
    });
    assert_eq!(draw(&p, 5)[0], GREEN, "indexed");
    p.write_register(0x30, 0x01);
    // R = 111 << 2 | 1 << 1, G = 0, B = 11 << 3 | 1 << 2.
    assert_eq!(draw(&p, 6)[0], 30 | (28 << 10));
}

#[test]
fn mode2_offset_per_tile_replaces_a_columns_scroll() {
    let p = ppu();
    display_on(&p);
    p.with_engine(|e| {
        e.r.bgmode = 0x02;
        e.r.bgsc[0] = 0x04;
        e.r.bgsc[2] = 0x08;
        e.r.bgnba = [0x01, 0];
        solid_tile(e, 0x1000, 4, 1, 1);
        // BG1's map: only map column 4 holds the solid tile.
        fill_map(e, 0x400, 0);
        for row in 0..32 {
            e.vram[0x400 + row * 32 + 4] = 1;
        }
        // BG3's row 0, column 0 is the H offset for screen column 1: 24 px,
        // applied to BG1 (bit 13). So screen column 1 shows map column 4.
        e.vram[0x800] = 0x2000 | 24;
        e.cgram[1] = RED;
        e.cgram[0] = BLUE;
        e.r.tm = 0x01;
    });
    let line = draw(&p, 5);
    assert_eq!(line[0], BLUE, "column 0 is never affected");
    assert_eq!(line[8], RED, "column 1 scrolled by 24");
    assert_eq!(line[16], BLUE);
    assert_eq!(line[32], RED, "column 4 unscrolled");
}

#[test]
fn mode5_is_hires_with_sub_on_even_and_main_on_odd_columns() {
    let p = ppu();
    display_on(&p);
    p.with_engine(|e| {
        e.r.bgmode = 0x05;
        e.r.bgsc[0] = 0x04;
        e.r.bgnba = [0x01, 0];
        // Tile 1: the left 8 pixels solid, tile 2 (its right half) empty.
        solid_tile(e, 0x1000, 4, 1, 1);
        fill_map(e, 0x400, 1);
        e.cgram[1] = RED;
        e.cgram[0] = BLUE;
        e.r.tm = 0x01;
        e.r.ts = 0x01;
    });
    let wide = draw_wide(&p, 5);
    // 16-pixel tiles at hires resolution: 8 lit hires pixels, 8 not.
    assert_eq!(&wide[..8], &[RED; 8]);
    assert_eq!(&wide[8..16], &[BLUE; 8]);
    // With BG1 only on the sub screen, the odd (main) columns are backdrop.
    p.with_engine(|e| e.r.tm = 0);
    let wide = draw_wide(&p, 6);
    assert_eq!(&wide[..4], &[RED, BLUE, RED, BLUE]);
    // A frame with a hires line reports 512 wide.
    seek(&p, 226, 0);
    assert_eq!(p.output_size(), (512, 224));
}

/// Mode 7 with an identity matrix, map tile `t` at `(tx, ty)` being solid
/// pixel `t`.
fn mode7_scene(p: &SnesPpu) {
    p.with_engine(|e| {
        e.r.bgmode = 0x07;
        e.r.m7 = [0x0100, 0, 0, 0x0100];
        for tile in 1..4u16 {
            for i in 0..64 {
                let w = &mut e.vram[usize::from(tile) * 64 + i];
                *w = (*w & 0x00FF) | (tile << 8);
            }
        }
        // Map row 0: tiles 1, 2, 3 at columns 0, 1, 2.
        for (col, tile) in [(0usize, 1u16), (1, 2), (2, 3)] {
            let w = &mut e.vram[col];
            *w = (*w & 0xFF00) | tile;
        }
        e.cgram[0] = BLUE;
        e.cgram[1] = RED;
        e.cgram[2] = GREEN;
        e.cgram[3] = WHITE;
        e.r.tm = 0x01;
        // Line V = 1 at map row y = 1, inside tile row 0.
    });
}

#[test]
fn mode7_identity_matrix_maps_the_screen_onto_the_map() {
    let p = ppu();
    display_on(&p);
    mode7_scene(&p);
    let line = draw(&p, 1);
    assert_eq!(
        (line[0], line[8], line[16], line[24]),
        (RED, GREEN, WHITE, BLUE)
    );
}

#[test]
fn mode7_truncates_the_line_products_as_well_as_the_origin_ones() {
    // nocash, *fullsnes*, "SNES PPU Rotation/Scaling":
    //   VRAM.X = VRAM.X + ((M7B*SCREEN.Y) AND NOT 3Fh) + (M7A*SCREEN.X)
    //   VRAM.Y = VRAM.Y + ((M7D*SCREEN.Y) AND NOT 3Fh) + (M7C*SCREEN.X)
    // "before summing up the multiply results, it DOES strip the lower 6bit".
    // Only the per-pixel M7A/M7C terms keep all their bits.
    let p = ppu();
    display_on(&p);
    mode7_scene(&p);
    // On line 1, M7B * 1 = $20, which the strip removes. M7A * 8 = $7E0 is
    // then map pixel 7, the last of tile 1; with the $20 left in it is $800,
    // pixel 8, the first of tile 2.
    p.with_engine(|e| {
        e.r.m7[0] = 0x00FC;
        e.r.m7[1] = 0x0020;
    });
    let line = draw(&p, 1);
    assert_eq!(line[8], RED);
    assert_eq!(line[9], GREEN, "and the next pixel is where tile 2 starts");
}

#[test]
fn mode7_scaling_and_flip() {
    let p = ppu();
    display_on(&p);
    mode7_scene(&p);
    // A = 2.0: two map pixels per screen pixel.
    p.with_engine(|e| e.r.m7[0] = 0x0200);
    let line = draw(&p, 1);
    assert_eq!(
        (line[0], line[4], line[8], line[12]),
        (RED, GREEN, WHITE, BLUE)
    );
    // Horizontal flip: screen x reads map x = 255 - x.
    p.with_engine(|e| {
        e.r.m7[0] = 0x0100;
        e.r.m7sel = 0x01;
    });
    let line = draw(&p, 2);
    assert_eq!((line[255], line[247], line[239]), (RED, GREEN, WHITE));
}

#[test]
fn mode7_screen_over_transparent_and_tile_zero() {
    let p = ppu();
    display_on(&p);
    mode7_scene(&p);
    p.with_engine(|e| {
        // Centre at 0 and scroll left by 8 so x 0..7 read map x -8..-1.
        e.r.m7hofs = 0x1FF8;
        e.r.m7sel = 0x80;
    });
    let line = draw(&p, 1);
    assert_eq!((line[0], line[8]), (BLUE, RED), "outside is transparent");
    p.with_engine(|e| {
        e.r.m7sel = 0xC0;
        // Tile 0's pixels are colour 3.
        for w in &mut e.vram[..64] {
            *w = (*w & 0x00FF) | (3 << 8);
        }
    });
    let line = draw(&p, 2);
    assert_eq!((line[0], line[8]), (WHITE, RED), "outside repeats tile 0");
    p.with_engine(|e| e.r.m7sel = 0x00);
    let line = draw(&p, 3);
    // Wrapping: map x -8 is x 1016, column 127 — tile 0 again.
    assert_eq!(line[0], WHITE);
}

#[test]
fn mode7_extbg_uses_bit_7_as_bg2_priority() {
    let p = ppu();
    display_on(&p);
    mode7_scene(&p);
    p.with_engine(|e| {
        // Tile 1's pixels become $81: BG1 colour $81, BG2 colour 1 at prio 1.
        for i in 0..64 {
            e.vram[64 + i] = (e.vram[64 + i] & 0x00FF) | (0x81 << 8);
        }
        e.cgram[0x81] = GREEN;
        e.r.setini = 0x40;
        e.r.tm = 0x02;
    });
    assert_eq!(draw(&p, 1)[0], RED, "BG2 shows colour 1");
    p.with_engine(|e| e.r.tm = 0x03);
    assert_eq!(draw(&p, 2)[0], RED, "BG2 priority 1 is in front of BG1");
}

/// Sprite `n` at `(x, y)`, tile `tile`, attribute `attr`, small size.
fn sprite(e: &mut Engine, n: usize, x: i32, y: u8, tile: u8, attr: u8, large: bool) {
    e.oam[n * 4] = x as u8;
    e.oam[n * 4 + 1] = y;
    e.oam[n * 4 + 2] = tile;
    e.oam[n * 4 + 3] = attr;
    let shift = 2 * (n % 4);
    let hi = &mut e.oam[0x200 + n / 4];
    *hi &= !(3 << shift);
    *hi |= (u8::from(x & 0x100 != 0) | (u8::from(large) << 1)) << shift;
}

/// Every sprite parked off the bottom of the screen.
fn park_sprites(e: &mut Engine) {
    for n in 0..128 {
        sprite(e, n, 0, 0xF0, 0, 0, false);
    }
}

#[test]
fn a_sprite_draws_on_the_line_after_its_y() {
    let p = ppu();
    display_on(&p);
    p.with_engine(|e| {
        park_sprites(e);
        e.r.obsel = 0x02; // name base $4000 words
        solid_tile(e, 0x4000, 4, 1, 3);
        sprite(e, 0, 10, 20, 1, 0x34 | (2 << 1), false); // palette 2, prio 3
        e.cgram[128 + 2 * 16 + 3] = RED;
        e.cgram[0] = BLUE;
        e.r.tm = 0x10;
    });
    assert_eq!(draw(&p, 20)[10], BLUE, "Y is the line before");
    let line = draw(&p, 21);
    assert_eq!(
        (line[9], line[10], line[17], line[18]),
        (BLUE, RED, RED, BLUE)
    );
    assert_eq!(draw(&p, 28)[10], RED);
    assert_eq!(draw(&p, 29)[10], BLUE);
}

#[test]
fn sprite_priority_goes_to_the_lower_index_and_rotation_moves_it() {
    let p = ppu();
    display_on(&p);
    p.with_engine(|e| {
        park_sprites(e);
        solid_tile(e, 0, 4, 1, 1);
        solid_tile(e, 0, 4, 2, 2);
        sprite(e, 0, 0, 0, 1, 0, false);
        sprite(e, 1, 0, 0, 2, 0, false);
        e.cgram[129] = RED;
        e.cgram[130] = GREEN;
        e.r.tm = 0x10;
    });
    assert_eq!(draw(&p, 1)[0], RED);
    // Rotation: OAMADD = word 2 makes sprite 1 first.
    p.write_register(0x02, 0x02);
    p.write_register(0x03, 0x80);
    assert_eq!(draw(&p, 2)[0], GREEN);
}

#[test]
fn sprites_interleave_with_bgs_by_priority() {
    let p = ppu();
    display_on(&p);
    mode1_scene(&p);
    p.with_engine(|e| {
        park_sprites(e);
        e.r.obsel = 0x02;
        solid_tile(e, 0x4000, 4, 1, 1);
        sprite(e, 0, 0, 0, 1, 0x10, false); // priority 1
        e.cgram[129] = GREEN;
        e.r.tm = 0x11;
    });
    // Mode 1: BG1 with its priority bit is in front of an OBJ at priority 1.
    assert_eq!(draw(&p, 1)[0], RED);
    p.with_engine(|e| e.oam[3] = 0x20); // priority 2: still behind BG1 high
    assert_eq!(draw(&p, 2)[0], RED);
    p.with_engine(|e| e.oam[3] = 0x30); // priority 3: in front of everything
    assert_eq!(draw(&p, 3)[0], GREEN);
}

#[test]
fn a_33rd_sprite_on_a_line_sets_range_over() {
    let p = ppu();
    display_on(&p);
    p.with_engine(|e| {
        park_sprites(e);
        for n in 0..32 {
            sprite(e, n, 8 * n as i32, 50, 0, 0, false);
        }
        e.r.tm = 0x10;
    });
    draw(&p, 51);
    assert!(!p.with_engine(|e| e.r.range_over));
    p.with_engine(|e| sprite(e, 40, 0, 50, 0, 0, false));
    draw(&p, 52);
    assert_eq!(p.peek_register(STAT77, 0) & 0xC0, 0x40);
    // Cleared at the end of vblank.
    seek(&p, 0, 1);
    assert_eq!(p.peek_register(STAT77, 0) & 0xC0, 0);
}

#[test]
fn a_sprite_at_minus_256_counts_toward_the_range_limit() {
    let p = ppu();
    display_on(&p);
    p.with_engine(|e| {
        park_sprites(e);
        for n in 0..32 {
            sprite(e, n, 0x100, 50, 0, 0, false);
        }
        sprite(e, 32, 0x1F8, 50, 0, 0, false); // X = -8: wholly off screen
        sprite(e, 33, 0x100, 50, 0, 0, false);
    });
    draw(&p, 51);
    assert!(p.with_engine(|e| e.r.range_over));
}

#[test]
fn a_35th_tile_sets_time_over_and_drops_the_front_sprites() {
    let p = ppu();
    display_on(&p);
    p.with_engine(|e| {
        park_sprites(e);
        for tile in 0..4 {
            solid_tile(e, 0, 4, tile, 1);
        }
        // Nine 32-pixel sprites: 36 tiles.
        e.r.obsel = 0x20; // 8x8 / 32x32
        for n in 0..9 {
            sprite(e, n, 28 * n as i32, 50, 0, 0, true);
        }
        e.cgram[129] = RED;
        e.cgram[0] = BLUE;
        e.r.tm = 0x10;
    });
    let line = draw(&p, 51);
    assert_eq!(p.peek_register(STAT77, 0) & 0xC0, 0x80);
    // Fetching runs from sprite 8 down, so sprites 8-1 take 32 tiles and
    // sprite 0 — the highest priority — gets two of its four.
    assert_eq!(
        (line[0], line[15]),
        (RED, RED),
        "sprite 0's first two tiles"
    );
    assert_eq!(line[16], BLUE, "its third was never fetched");
    assert_eq!(line[255], RED);
    let p2 = ppu();
    display_on(&p2);
    p2.with_engine(|e| {
        park_sprites(e);
        e.r.obsel = 0x20;
        for n in 0..8 {
            sprite(e, n, 28 * n as i32, 50, 0, 0, true);
        }
        sprite(e, 8, 28 * 8, 50, 0, 0, false);
    });
    draw(&p2, 51);
    assert_eq!(p2.peek_register(STAT77, 0) & 0xC0, 0, "33 tiles fit");
}

#[test]
fn large_sprites_walk_the_name_grid_and_flip() {
    let p = ppu();
    display_on(&p);
    p.with_engine(|e| {
        park_sprites(e);
        // 16x16: tiles $0E, $0F, $1E, $1F. Make $0F solid (top right).
        solid_tile(e, 0, 4, 0x0F, 1);
        sprite(e, 0, 0, 0, 0x0E, 0, true);
        e.cgram[129] = RED;
        e.cgram[0] = BLUE;
        e.r.tm = 0x10;
    });
    let line = draw(&p, 1);
    assert_eq!((line[7], line[8], line[15]), (BLUE, RED, RED));
    p.with_engine(|e| e.oam[3] = 0x40); // hflip
    let line = draw(&p, 2);
    assert_eq!((line[0], line[7], line[8]), (RED, RED, BLUE));
    assert_eq!(draw(&p, 9)[0], BLUE, "the bottom row of tiles is empty");
}

#[test]
fn windows_mask_a_layer_on_the_main_screen() {
    let p = ppu();
    display_on(&p);
    mode1_scene(&p);
    p.with_engine(|e| {
        e.r.tm = 0x01;
        e.cgram[0] = BLUE;
        e.r.wh = [10, 20, 15, 30];
        e.r.wsel[0] = 0x02; // BG1: W1 on
    });
    assert!(draw(&p, 1).iter().all(|&c| c == RED), "TMW off: no effect");
    p.write_register(0x2E, 0x01);
    let line = draw(&p, 2);
    assert_eq!(
        (line[9], line[10], line[20], line[21]),
        (RED, BLUE, BLUE, RED)
    );
    // W1 inverted.
    p.with_engine(|e| e.r.wsel[0] = 0x03);
    let line = draw(&p, 3);
    assert_eq!((line[9], line[10], line[21]), (BLUE, RED, BLUE));
    // Both windows, AND: only 15..=20.
    p.with_engine(|e| {
        e.r.wsel[0] = 0x0A;
        e.r.wbglog = 0x01;
    });
    let line = draw(&p, 4);
    assert_eq!(
        (line[14], line[15], line[20], line[21]),
        (RED, BLUE, BLUE, RED)
    );
    // XOR: 10..=14 and 21..=30.
    p.with_engine(|e| e.r.wbglog = 0x02);
    let line = draw(&p, 5);
    assert_eq!(
        (line[10], line[15], line[21], line[31]),
        (BLUE, RED, BLUE, RED)
    );
}

#[test]
fn colour_math_adds_subtracts_and_halves() {
    let p = ppu();
    display_on(&p);
    p.with_engine(|e| {
        e.cgram[0] = 10 | (20 << 5) | (30 << 10);
        e.r.fixed = [4, 4, 4];
        e.r.cgadsub = 0x20; // backdrop, add
    });
    assert_eq!(draw(&p, 1)[0], 14 | (24 << 5) | (31 << 10), "add saturates");
    p.write_register(0x31, 0xA0);
    assert_eq!(draw(&p, 2)[0], 6 | (16 << 5) | (26 << 10), "subtract");
    p.write_register(0x31, 0x60);
    assert_eq!(draw(&p, 3)[0], 7 | (12 << 5) | (17 << 10), "add, halved");
    // COLDATA sets the channels its bits name.
    p.write_register(0x32, 0xE0 | 1);
    assert_eq!(p.with_engine(|e| e.r.fixed), [1, 1, 1]);
}

#[test]
fn colour_math_with_the_sub_screen() {
    let p = ppu();
    display_on(&p);
    mode1_scene(&p);
    p.with_engine(|e| {
        e.cgram[3 * 16 + 5] = 8 | (8 << 5);
        e.cgram[2] = 4 | (4 << 10);
        e.r.tm = 0x01;
        e.r.ts = 0x04;
        e.r.cgwsel = 0x02; // operand is the sub screen
        e.r.cgadsub = 0x41; // BG1, add, half
        e.r.fixed = [31, 31, 31];
    });
    // (8+4)/2, (8+0)/2, (0+4)/2.
    assert_eq!(draw(&p, 1)[0], 6 | (4 << 5) | (2 << 10));
    // With nothing on the sub screen the operand is the fixed colour and
    // halving is skipped.
    p.with_engine(|e| e.r.ts = 0);
    assert_eq!(draw(&p, 2)[0], 31 | (31 << 5) | (31 << 10));
    // OBJ palettes 0-3 never take part.
    p.with_engine(|e| {
        park_sprites(e);
        e.r.obsel = 0x02;
        solid_tile(e, 0x4000, 4, 1, 1);
        sprite(e, 0, 0, 0, 1, 0x30, false);
        e.cgram[129] = RED;
        e.r.tm = 0x11;
        e.r.cgadsub = 0x11;
    });
    assert_eq!(draw(&p, 1 + 3)[0], RED);
}

#[test]
fn the_colour_window_clips_to_black_and_prevents_math() {
    let p = ppu();
    display_on(&p);
    p.with_engine(|e| {
        e.cgram[0] = 10;
        e.r.fixed = [1, 0, 0];
        e.r.cgadsub = 0x20;
        e.r.wh = [100, 199, 0, 0];
        e.r.wsel[2] = 0x20; // colour window: W1
        e.r.cgwsel = 0x80; // clip inside
    });
    let line = draw(&p, 1);
    assert_eq!((line[99], line[100]), (11, 1), "clipped then added");
    p.with_engine(|e| e.r.cgwsel = 0x20); // prevent inside... i.e. math only outside
    let line = draw(&p, 2);
    assert_eq!((line[99], line[100], line[200]), (11, 10, 11));
}

#[test]
fn mosaic_repeats_the_top_left_of_each_block() {
    let p = ppu();
    display_on(&p);
    p.with_engine(|e| {
        e.r.bgmode = 0x01;
        e.r.bgsc[0] = 0x04;
        e.r.bgnba = [0x01, 0];
        // Tile 1: column 0 of each row lit only.
        for row in 0..8 {
            e.vram[0x1000 + 16 + row] = 0x0080;
        }
        fill_map(e, 0x400, 1);
        e.cgram[1] = RED;
        e.cgram[0] = BLUE;
        e.r.tm = 0x01;
    });
    let line = draw(&p, 1);
    assert_eq!((line[0], line[1], line[8]), (RED, BLUE, RED));
    p.write_register(0x06, 0x31); // 4x4, BG1
    let line = draw(&p, 2);
    assert_eq!(&line[..5], &[RED, RED, RED, RED, BLUE]);
}

#[test]
fn interlace_fills_alternate_rows_and_reports_448_lines() {
    let p = ppu();
    display_on(&p);
    p.with_engine(|e| e.cgram[0] = RED);
    p.write_register(0x33, 0x01);
    // The frame in progress latched interlace off; the next one is on.
    p.advance_by(1);
    seek(&p, 0, 0);
    let field = p.position().field;
    p.with_engine(|e| e.fb.fill(0));
    seek(&p, 226, 0);
    assert_eq!(p.output_size(), (256, 448));
    let parity = usize::from(field);
    p.with_framebuffer(|fb| {
        assert_eq!(fb[(2 * 10 + parity) * FB_WIDTH], RED);
        assert_eq!(fb[(2 * 10 + 1 - parity) * FB_WIDTH], 0);
    });
    assert_eq!(p.pixel(0, 2 * 10 + parity), Some(RED));
}

// ---------------------------------------------------------------------------
// Snapshots
// ---------------------------------------------------------------------------

fn encode(d: &SnesPpu) -> Vec<u8> {
    let mut writer = StateWriter::new(MachineShape::new());
    {
        let mut chunk = writer
            .chunk("ppu", CLASS.name, CLASS.version)
            .expect("a chunk");
        d.save(&mut chunk).expect("saves");
    }
    writer.to_vec().expect("serialises")
}

fn fnv(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf2_9ce4_8422_2325u64, |h, &b| {
        (h ^ u64::from(b)).wrapping_mul(0x0000_0100_0000_01b3)
    })
}

#[test]
fn a_ppu_round_trips_with_an_identical_state_hash() {
    let p = ppu();
    display_on(&p);
    mode1_scene(&p);
    let ops = port(&p);
    wr(&ops, 0x0D, 0x12); // a half-written scroll latch
    wr(&ops, 0x22, 0x34); // a half-written CGRAM word
    p.beam().set_extlatch(true);
    seek(&p, 120, 77);
    let bytes = encode(&p);

    let restored = SnesPpu::new(Region::Ntsc, 1, 3);
    let reader = StateReader::new(&bytes).expect("well formed");
    let chunk = reader
        .load("ppu", CLASS.name, CLASS.version, &Migrations::new())
        .expect("the chunk is there");
    restored.load(&mut chunk.reader()).expect("loads");
    assert_eq!(fnv(&encode(&restored)), fnv(&bytes));
    assert_eq!(restored.master(), p.master());
    assert_eq!(restored.position(), p.position());
    // And both run on identically.
    p.advance_by(500_000);
    restored.advance_by(500_000);
    assert_eq!(fnv(&encode(&restored)), fnv(&encode(&p)));
}

#[test]
fn a_truncated_snapshot_is_refused() {
    let p = ppu();
    let mut writer = StateWriter::new(MachineShape::new());
    {
        let mut chunk = writer.chunk("ppu", CLASS.name, CLASS.version).unwrap();
        crate::core::state::Sink::write_u32(&mut chunk, 1).unwrap();
        crate::core::state::Sink::write_bytes(&mut chunk, &[0; 10]).unwrap();
    }
    let bytes = writer.to_vec().unwrap();
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("ppu", CLASS.name, CLASS.version, &Migrations::new())
        .unwrap();
    assert!(p.load(&mut chunk.reader()).is_err());
}

#[test]
fn the_schema_names_every_pin_and_the_region() {
    let s = schema();
    let text = alloc::format!("{s:?}");
    for name in [
        VBLANK_PIN,
        HBLANK_PIN,
        EXTLATCH_PIN,
        REGISTER_REGION,
        "ppu2-version",
    ] {
        assert!(text.contains(name), "{name} missing from {text}");
    }
}
