//! Timing facts, asserted directly.
//!
//! Every test here names the NESdev page it comes from. A PPU test that only
//! says "the output looks right" is not a test of the thing that is hard.

use alloc::string::ToString;
use alloc::sync::Arc;
use alloc::vec::Vec;

use super::*;
use crate::core::clock::{ClockForest, Rational};
use crate::core::device::Deferred;
use crate::core::space::{RamStore, Region as MmioRegion, RequesterId, UnassignedPolicy};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::wire::{Wire, WireId, WireIdAllocator, WireSink};

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------

/// A PPU with 8 KiB of CHR RAM and 4 KiB of nametable RAM, no write lockout.
fn new_ppu() -> (NesPpu, Arc<RamStore>, Arc<RamStore>) {
    new_ppu_in(Region::Ntsc)
}

/// The same, for a named region.
fn new_ppu_in(region: Region) -> (NesPpu, Arc<RamStore>, Arc<RamStore>) {
    let mut props = Props::new();
    props.insert("region", region.name());
    props.insert("warmup", false);
    let ppu = NesPpu::new(&props).expect("properties are valid");

    let chr = Arc::new(RamStore::new(0x2000));
    let nt = Arc::new(RamStore::new(0x1000));
    let space = AddressSpace::new("ppu", 14).with_unassigned(UnassignedPolicy::ZEROS);
    space
        .topology()
        .map(Arc::new(MmioRegion::ram("chr", Arc::clone(&chr))), 0x0000)
        .expect("chr fits");
    space
        .topology()
        .map(
            Arc::new(
                MmioRegion::mirror(
                    "nametables",
                    Arc::new(MmioRegion::ram("nt", Arc::clone(&nt))),
                    0x1000,
                )
                .expect("mirror fits"),
            ),
            0x2000,
        )
        .expect("nametables fit");
    ppu.attach_bus(Arc::new(space));
    (ppu, chr, nt)
}

/// Let a register access finish the way a real one does.
///
/// A `$2007` read and a `$2006` write are both two-dot-cadence events now — the
/// read buffer fills four dots after the access and `v` arrives two dots after
/// it — and the shortest instruction that touches a PPU register takes four CPU
/// cycles, twelve dots. Tests that assert the *settled* result therefore have
/// to let those twelve dots run, exactly as the guest does.
fn settle(ppu: &NesPpu) {
    ppu.advance_by(12);
}

/// Put the PPU at `(scanline, dot)` — the position of the dot about to run.
fn seek(ppu: &NesPpu, scanline: u16, dot: u16) {
    ppu.with_engine(|e| {
        e.scanline = scanline;
        e.dot = dot;
    });
}

/// Run two full frames and stop at the start of post-render.
///
/// The frame is fully drawn, but the pre-render line — which clears vblank,
/// sprite 0 hit and overflow at its dot 1 — has not run yet, so the status flags
/// are still there to be asserted on. Two frames because the first one starts on
/// scanline 0 with nothing primed.
fn draw_two_frames(ppu: &NesPpu) {
    ppu.advance_by(DOTS_PER_FRAME * 2 - u64::from(DOTS_PER_SCANLINE) * 22);
    assert_eq!(ppu.position(), (SCREEN_HEIGHT as u16, 0));
}

fn enable_rendering(ppu: &NesPpu) {
    set_mask(ppu, MASK_BG | MASK_SPRITE | MASK_BG_LEFT | MASK_SPRITE_LEFT);
}

/// Write `$2001` and let its travel time elapse at once.
///
/// A `$2001` write reaches the pipeline three dots later on hardware. Nearly
/// every test here is *setting the chip up* rather than measuring that, and
/// three dots of setup would put each of them on a different dot from the one
/// it means to assert about. The write still goes through the register path, so
/// the I/O latch is charged the way a real one charges it.
fn set_mask(ppu: &NesPpu, value: u8) {
    ppu.write_register(PPUMASK, value);
    ppu.with_engine(|e| {
        e.mask = value;
        e.mask_delay = 0;
    });
}

// ---------------------------------------------------------------------------
// Clock
// ---------------------------------------------------------------------------

#[test]
fn three_dots_per_cpu_cycle_exactly() {
    // ROADMAP.md §4.2: both counters descend from one crystal, so the ratio is
    // exact regardless of what the master frequency is believed to be.
    let mut forest = ClockForest::new();
    let master = forest
        .add_oscillator("master", Rational::new(236_250_000, 11).unwrap())
        .unwrap();
    let cpu = forest.add_domain("cpu", master, 1, 12).unwrap();
    let dots = add_clock_domain(&mut forest, master, Region::Ntsc).unwrap();
    assert_eq!(forest.convert_ticks(cpu, dots, 1).unwrap(), 3);
    assert_eq!(forest.convert_ticks(cpu, dots, 29_780).unwrap(), 89_340);
}

// ---------------------------------------------------------------------------
// Frame geometry
// ---------------------------------------------------------------------------

#[test]
fn an_even_frame_is_341_by_262_dots() {
    // NESdev PPU rendering: 262 scanlines of 341 dots.
    let (ppu, _, _) = new_ppu();
    ppu.advance_by(DOTS_PER_FRAME);
    assert_eq!(ppu.frame(), 1);
    assert_eq!(ppu.position(), (0, 0));
    assert_eq!(DOTS_PER_FRAME, 89_342);
}

#[test]
fn an_odd_frame_skips_a_dot_when_rendering() {
    // NESdev PPU frame timing: with rendering enabled, an odd frame jumps
    // straight from (339, 261) to (0, 0).
    let (ppu, _, _) = new_ppu();
    enable_rendering(&ppu);
    ppu.advance_by(DOTS_PER_FRAME); // frame 0, even, full length
    assert_eq!(ppu.frame(), 1);
    assert_eq!(ppu.position(), (0, 0));
    ppu.advance_by(DOTS_PER_FRAME - 1); // frame 1, odd, one dot shorter
    assert_eq!(ppu.frame(), 2);
    assert_eq!(ppu.position(), (0, 0));
}

#[test]
fn the_odd_frame_skip_needs_rendering_enabled() {
    // "This is done internally by jumping directly from (339,261) to (0,0)" —
    // but only when rendering is on at that dot.
    let (ppu, _, _) = new_ppu();
    ppu.advance_by(DOTS_PER_FRAME);
    ppu.advance_by(DOTS_PER_FRAME);
    assert_eq!(ppu.frame(), 2);
    assert_eq!(ppu.position(), (0, 0));
}

// ---------------------------------------------------------------------------
// VBlank, NMI, and the $2002 race
// ---------------------------------------------------------------------------

/// Records every level change on a wire, for the NMI tests.
#[derive(Debug, Default)]
struct Recorder {
    levels: crate::core::sync::Mutex<Vec<bool>>,
}

impl WireSink for Recorder {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        self.levels.lock().push(level.is_high());
    }
}

fn with_nmi(ppu: &NesPpu) -> Arc<Recorder> {
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let sink = Arc::new(Recorder::default());
    let wire = Wire::builder()
        .source(id)
        .sink(Arc::clone(&sink) as Arc<dyn WireSink>, 0)
        .build_shared();
    ppu.attach_nmi(WireSource::new(wire, id));
    sink
}

#[test]
fn the_vblank_flag_is_set_at_dot_1_of_scanline_241() {
    // NESdev PPU frame timing.
    let (ppu, _, _) = new_ppu();
    seek(&ppu, VBLANK_SCANLINE, 0);
    ppu.advance_by(1); // runs dot 0; reading here would be the race, so peek
    assert_eq!(ppu.with_engine(|e| e.status) & STATUS_VBLANK, 0);
    ppu.advance_by(1); // runs dot 1
    assert_eq!(ppu.with_engine(|e| e.status) & STATUS_VBLANK, STATUS_VBLANK);
    // And two dots later it is a plain read.
    ppu.advance_by(3);
    assert_eq!(ppu.read_register(PPUSTATUS) & STATUS_VBLANK, STATUS_VBLANK);
    assert_eq!(
        ppu.with_engine(|e| e.status) & STATUS_VBLANK,
        0,
        "cleared by the read"
    );
}

#[test]
fn the_vblank_flag_is_cleared_at_dot_1_of_the_pre_render_line() {
    let (ppu, _, _) = new_ppu();
    seek(&ppu, VBLANK_SCANLINE, 1);
    ppu.advance_by(1);
    ppu.with_engine(|e| e.status |= STATUS_SPRITE0 | STATUS_OVERFLOW);
    seek(&ppu, PRE_RENDER_SCANLINE, 0);
    ppu.advance_by(1);
    assert_ne!(ppu.with_engine(|e| e.status) & STATUS_VBLANK, 0);
    ppu.advance_by(1); // dot 1
    assert_eq!(
        ppu.with_engine(|e| e.status) & (STATUS_VBLANK | STATUS_SPRITE0 | STATUS_OVERFLOW),
        0
    );
}

#[test]
fn reading_2002_one_dot_before_the_set_suppresses_the_flag_entirely() {
    // NESdev PPU frame timing: "Reading one PPU clock before reads it as clear
    // and never sets the flag or generates NMI for that frame."
    let (ppu, _, _) = new_ppu();
    let nmi = with_nmi(&ppu);
    ppu.write_register(PPUCTRL, CTRL_NMI);
    seek(&ppu, VBLANK_SCANLINE, 1);
    assert_eq!(ppu.read_register(PPUSTATUS) & STATUS_VBLANK, 0);
    ppu.advance_by(3);
    assert_eq!(ppu.with_engine(|e| e.status) & STATUS_VBLANK, 0);
    assert!(
        !nmi.levels.lock().iter().any(|high| *high),
        "no NMI may be requested this frame"
    );
}

#[test]
fn reading_2002_on_the_set_dot_returns_the_flag_and_suppresses_the_nmi() {
    // "Reading on the same PPU clock or one later reads it as set, clears it,
    // and suppresses the NMI for that frame."
    for dot in [2u16, 3] {
        let (ppu, _, _) = new_ppu();
        let nmi = with_nmi(&ppu);
        ppu.write_register(PPUCTRL, CTRL_NMI);
        seek(&ppu, VBLANK_SCANLINE, 1);
        ppu.advance_by(u64::from(dot - 1)); // run up to and including dot 1
        assert_eq!(ppu.position(), (VBLANK_SCANLINE, dot));
        let status = ppu.read_register(PPUSTATUS);
        assert_eq!(status & STATUS_VBLANK, STATUS_VBLANK, "dot {dot}");
        assert!(ppu.with_engine(|e| e.suppress_nmi), "dot {dot}");
        // Through to the end of vblank: the request never comes back.
        let before = nmi.levels.lock().len();
        ppu.advance_by(u64::from(DOTS_PER_SCANLINE) * 19);
        assert!(
            !nmi.levels.lock()[before..].iter().any(|high| *high),
            "dot {dot}: the NMI must stay suppressed for the frame"
        );
    }
    // At dot 2 the read is early enough that the output — which lags the
    // request by a dot, because the CPU samples `/NMI` at φ2 — never went high
    // at all. At dot 3 it had, and the read only pulls it back down; whether
    // the CPU acted on it is exactly the case AccuracyCoin leaves as "either".
    let (ppu, _, _) = new_ppu();
    let nmi = with_nmi(&ppu);
    ppu.write_register(PPUCTRL, CTRL_NMI);
    seek(&ppu, VBLANK_SCANLINE, 1);
    ppu.advance_by(1);
    ppu.read_register(PPUSTATUS);
    ppu.advance_by(20);
    assert!(
        !nmi.levels.lock().iter().any(|high| *high),
        "a read on the set dot withdraws the request before the CPU sees it"
    );
}

#[test]
fn a_read_four_dots_after_the_set_is_too_late_to_suppress() {
    // By then the output has been high for a dot, so the CPU has had it under
    // its φ2 sample and clearing the flag only lowers the line again.
    let (ppu, _, _) = new_ppu();
    let nmi = with_nmi(&ppu);
    ppu.write_register(PPUCTRL, CTRL_NMI);
    seek(&ppu, VBLANK_SCANLINE, 1);
    ppu.advance_by(3);
    assert_eq!(ppu.position(), (VBLANK_SCANLINE, 4));
    assert_eq!(nmi.levels.lock().as_slice(), &[true]);
    ppu.read_register(PPUSTATUS);
    // The output follows a dot later, which is the whole point of the delay.
    ppu.advance_by(1);
    assert_eq!(nmi.levels.lock().as_slice(), &[true, false]);
}

#[test]
fn enabling_nmi_mid_vblank_requests_one_immediately() {
    // NESdev NMI: /NMI is pulled low iff vblank_flag and nmi_output are both
    // true, so toggling bit 7 during vblank can request several NMIs.
    let (ppu, _, _) = new_ppu();
    let nmi = with_nmi(&ppu);
    seek(&ppu, VBLANK_SCANLINE, 1);
    ppu.advance_by(10);
    assert!(nmi.levels.lock().is_empty(), "NMI output is still off");
    ppu.write_register(PPUCTRL, CTRL_NMI);
    // The output lags the request by one dot — the CPU samples `/NMI` at φ2 and
    // latches the data bus at the end of it — so the write is on the wire by
    // the time the core next looks, which is the next cycle.
    ppu.advance_by(1);
    assert_eq!(nmi.levels.lock().as_slice(), &[true]);
    ppu.write_register(PPUCTRL, 0);
    ppu.advance_by(1);
    ppu.write_register(PPUCTRL, CTRL_NMI);
    ppu.advance_by(1);
    assert_eq!(nmi.levels.lock().as_slice(), &[true, false, true]);
}

#[test]
fn the_nmi_falls_when_vblank_ends() {
    let (ppu, _, _) = new_ppu();
    let nmi = with_nmi(&ppu);
    ppu.write_register(PPUCTRL, CTRL_NMI);
    seek(&ppu, VBLANK_SCANLINE, 1);
    ppu.advance_by(3);
    assert_eq!(nmi.levels.lock().as_slice(), &[true]);
    // Straight through to the pre-render line's dot 1, where the flag clears —
    // and one dot further, for the output to follow it.
    seek(&ppu, PRE_RENDER_SCANLINE, 0);
    ppu.advance_by(3);
    assert_eq!(nmi.levels.lock().as_slice(), &[true, false]);
}

// ---------------------------------------------------------------------------
// Registers
// ---------------------------------------------------------------------------

#[test]
fn the_2007_read_buffer_delays_by_one_access() {
    // NESdev PPU registers: a read returns the buffer, then refills it.
    let (ppu, _, nt) = new_ppu();
    nt.write_u8(0x000, 0x11).unwrap();
    nt.write_u8(0x001, 0x22).unwrap();
    ppu.write_register(PPUADDR, 0x20);
    settle(&ppu);
    ppu.write_register(PPUADDR, 0x00);
    settle(&ppu);
    let dummy = ppu.read_register(PPUDATA);
    settle(&ppu);
    assert_eq!(dummy, 0x00, "the first read returns the stale buffer");
    assert_eq!(ppu.read_register(PPUDATA), 0x11);
    settle(&ppu);
    assert_eq!(ppu.read_register(PPUDATA), 0x22);
}

#[test]
fn a_2007_read_fills_the_buffer_four_dots_after_the_access() {
    // The read does not fetch during the CPU's cycle: it starts a latch chain
    // clocked off the PPU, which raises ALE two PPU cycles after M2 falls and
    // Read two after that (AccuracyCoin.asm's "PPU DATA State Machine").
    let (ppu, _, nt) = new_ppu();
    nt.write_u8(0x000, 0x11).unwrap();
    ppu.write_register(PPUADDR, 0x20);
    settle(&ppu);
    ppu.write_register(PPUADDR, 0x00);
    settle(&ppu);
    ppu.read_register(PPUDATA);
    ppu.advance_by(4);
    assert_eq!(
        ppu.with_engine(|e| e.read_buffer),
        0x00,
        "the dot ALE landed on and the one after it have run; Read has not"
    );
    ppu.advance_by(1);
    assert_eq!(ppu.with_engine(|e| e.read_buffer), 0x11);
}

#[test]
fn a_2006_write_reaches_v_two_dots_after_the_access() {
    // Same two PPU cycles, on the write path: `t` reaches `v` at *t2*, not at
    // the end of the CPU's cycle.
    let (ppu, _, _) = new_ppu();
    ppu.write_register(PPUADDR, 0x2c);
    settle(&ppu);
    ppu.write_register(PPUADDR, 0x19);
    ppu.advance_by(2);
    assert_eq!(
        ppu.with_engine(|e| e.v),
        0x0000,
        "not during the CPU's cycle, and not on the dot after it"
    );
    ppu.advance_by(1);
    assert_eq!(ppu.with_engine(|e| e.v), 0x2c19);
}

#[test]
fn a_2006_write_between_a_fetch_s_two_dots_makes_a_hybrid_address() {
    // The 2C02 multiplexes the low eight address bits onto its data pins, so a
    // fetch is ALE then Read and the low eight live in an octal latch on the
    // board in between. Move `v` across that gap and the read goes to an
    // address the chip never emitted: top six from the new `v`, low eight from
    // the latch the old one strobed. AccuracyCoin's "Hybrid Addresses".
    let (ppu, _, nt) = new_ppu();
    nt.write_u8(0xc19, 0xa5).unwrap();
    ppu.with_engine(|e| {
        e.mask = MASK_BG;
        // Coarse X $18; the dot-8 increment carries it to $19, which is what
        // dot 9 latches — out of nametable $2800, not $2C00.
        e.v = 0x0818;
    });
    seek(&ppu, 0, 8);
    ppu.write_register(PPUADDR, 0x2c);
    ppu.with_engine(|e| e.w = true);
    ppu.write_register(PPUADDR, 0x00);
    // Dot 8 finishes the previous fetch, dot 9 strobes ALE with the *old* `v`,
    // and `v` arrives in time for dot 10's read.
    ppu.advance_by(3);
    assert_eq!(ppu.with_engine(|e| e.v), 0x2c00);
    assert_eq!(
        ppu.with_engine(|e| e.nt_latch),
        0xa5,
        "read $2C19: $2C from the new v, $19 from the octal latch"
    );
}

#[test]
fn the_oam_read_line_answers_for_the_dot_before() {
    // `$2004` is driven off a latch and the CPU takes the bus at the end of its
    // cycle, so the phase boundaries the guest sees sit one dot after the
    // sprite unit's own: dot 1 still reads secondary OAM entry 0, and the
    // forced `$FF` of the clear runs dots 2-65 (AccuracyCoin "$2004 Stress
    // Test").
    let (ppu, _, _) = new_ppu();
    ppu.with_engine(|e| {
        e.mask = MASK_SPRITE;
        e.secondary_oam[0] = 0x5a;
    });
    seek(&ppu, 0, 1);
    assert_eq!(
        ppu.read_register(OAMDATA),
        0x5a,
        "dot 1 is not yet the clear"
    );
    ppu.advance_by(1);
    assert_eq!(ppu.read_register(OAMDATA), 0xff);
    seek(&ppu, 0, 65);
    assert_eq!(ppu.read_register(OAMDATA), 0xff, "dot 65 still reads $FF");
}

#[test]
fn the_sprite_x_counters_run_through_forced_blank() {
    // "Disabling rendering does not stop the sprite counters" — only the
    // shifters pause (AccuracyCoin "Stale Sprite Shift Regs", subtests 2 and 3).
    let (ppu, _, _) = new_ppu();
    ppu.with_engine(|e| {
        e.mask = MASK_SPRITE;
        e.sprite_x[0] = 40;
        e.sprite_pat_lo[0] = 0xff;
    });
    seek(&ppu, 0, 1);
    ppu.advance_by(10);
    assert_eq!(ppu.with_engine(|e| e.sprite_x[0]), 30);
    // Ten dots of forced blank still count.
    ppu.with_engine(|e| e.mask = 0);
    ppu.advance_by(10);
    assert_eq!(ppu.with_engine(|e| e.sprite_x[0]), 20);
    // But a unit that has started drawing does not shift while blanked.
    ppu.with_engine(|e| {
        e.sprite_x[0] = 0;
        e.sprite_halted = 1;
    });
    ppu.advance_by(4);
    assert_eq!(ppu.with_engine(|e| e.sprite_pat_lo[0]), 0xff);
}

#[test]
fn a_palette_read_is_not_buffered_but_still_fills_the_buffer() {
    // The palette answers immediately; the buffer gets the nametable byte
    // hiding under the mirror at $2F00 (NESdev PPU registers, PPUDATA).
    let (ppu, _, nt) = new_ppu();
    nt.write_u8(0xf01, 0x5a).unwrap();
    ppu.poke_palette(0x3f01, 0x21);
    ppu.write_register(PPUADDR, 0x3f);
    settle(&ppu);
    ppu.write_register(PPUADDR, 0x01);
    settle(&ppu);
    assert_eq!(ppu.read_register(PPUDATA) & 0x3f, 0x21);
    settle(&ppu);
    // The next read comes out of the buffer the palette read loaded.
    ppu.write_register(PPUADDR, 0x20);
    settle(&ppu);
    ppu.write_register(PPUADDR, 0x00);
    settle(&ppu);
    assert_eq!(ppu.read_register(PPUDATA), 0x5a);
}

#[test]
fn the_registers_mirror_every_eight_bytes_to_3fff() {
    let (ppu, _, _) = new_ppu();
    let port = ppu.port();
    let mut byte = [0u8; 1];
    // $2002 and $3FFA are the same register.
    ppu.with_engine(|e| e.status |= STATUS_VBLANK);
    port.read(0x3ffa - REGISTER_BASE, &mut byte, MemAttrs::DEFAULT)
        .unwrap();
    assert_eq!(byte[0] & STATUS_VBLANK, STATUS_VBLANK);
    // And it cleared the flag, because it really is the same register.
    assert_eq!(ppu.with_engine(|e| e.status) & STATUS_VBLANK, 0);
}

#[test]
fn the_write_toggle_is_shared_and_2002_clears_it() {
    // NESdev PPU scrolling: $2005 and $2006 share t and w.
    let (ppu, _, _) = new_ppu();
    ppu.write_register(PPUADDR, 0x21); // first write: t high byte
    assert!(ppu.with_engine(|e| e.w));
    ppu.read_register(PPUSTATUS);
    assert!(!ppu.with_engine(|e| e.w));
    // So this is treated as another *first* write, not the low byte.
    ppu.write_register(PPUADDR, 0x2c);
    settle(&ppu);
    ppu.write_register(PPUADDR, 0x00);
    settle(&ppu);
    assert_eq!(ppu.with_engine(|e| e.v), 0x2c00);
}

#[test]
fn scroll_writes_land_in_t_and_x_exactly() {
    // NESdev PPU scrolling, "$2005 first/second write".
    let (ppu, _, _) = new_ppu();
    ppu.write_register(PPUSCROLL, 0b1010_1011); // coarse X 10101, fine X 011
    assert_eq!(ppu.with_engine(|e| e.t) & 0x001f, 0b10101);
    assert_eq!(ppu.with_engine(|e| e.x), 0b011);
    ppu.write_register(PPUSCROLL, 0b0110_1101); // coarse Y 01101, fine Y 101
    let t = ppu.with_engine(|e| e.t);
    assert_eq!((t >> 5) & 0x1f, 0b01101);
    assert_eq!((t >> 12) & 0x07, 0b101);
}

#[test]
fn the_2006_high_write_clears_bit_14() {
    let (ppu, _, _) = new_ppu();
    ppu.write_register(PPUSCROLL, 0);
    ppu.write_register(PPUSCROLL, 0xff); // sets fine Y, i.e. t bits 12-14
    ppu.write_register(PPUADDR, 0xff); // ..AAAAAA: only 6 bits survive
    assert_eq!(ppu.with_engine(|e| e.t) & 0x7f00, 0x3f00);
}

#[test]
fn open_bus_fills_the_unused_bits_of_2002_and_decays() {
    // NESdev PPU registers, "PPU I/O latch".
    let mut props = Props::new();
    props.insert("warmup", false);
    props.insert("open-bus-decay-dots", 100u64);
    let ppu = NesPpu::new(&props).unwrap();
    ppu.attach_bus(Arc::new(AddressSpace::new("ppu", 14)));
    // A write to a write-only port charges the whole latch.
    set_mask(&ppu, 0x1f);
    assert_eq!(ppu.read_register(PPUSTATUS) & 0x1f, 0x1f);
    // Reading a write-only port returns the latch, unchanged.
    assert_eq!(ppu.read_register(PPUCTRL), 0x1f);
    ppu.advance_by(200);
    assert_eq!(
        ppu.read_register(PPUSTATUS) & 0x1f,
        0x00,
        "the charge decayed"
    );
}

#[test]
fn a_debug_read_of_2002_has_no_side_effects() {
    // ROADMAP.md §15, invariant 5.
    let (ppu, _, _) = new_ppu();
    ppu.with_engine(|e| e.status |= STATUS_VBLANK);
    ppu.write_register(PPUADDR, 0x21); // sets w
    let port = ppu.port();
    let mut byte = [0u8; 1];
    port.read(2, &mut byte, MemAttrs::DEBUG).unwrap();
    assert_eq!(byte[0] & STATUS_VBLANK, STATUS_VBLANK);
    assert_eq!(ppu.with_engine(|e| e.status) & STATUS_VBLANK, STATUS_VBLANK);
    assert!(ppu.with_engine(|e| e.w), "the toggle must not have moved");
}

#[test]
fn a_debug_read_of_2007_does_not_advance_the_address() {
    let (ppu, _, nt) = new_ppu();
    nt.write_u8(0, 0x77).unwrap();
    ppu.write_register(PPUADDR, 0x20);
    settle(&ppu);
    ppu.write_register(PPUADDR, 0x00);
    settle(&ppu);
    let port = ppu.port();
    let mut byte = [0u8; 1];
    port.read(7, &mut byte, MemAttrs::DEBUG).unwrap();
    assert_eq!(ppu.with_engine(|e| e.v), 0x2000);
    // The real read still sees the stale buffer, i.e. the debug read did not
    // fill it either.
    assert_eq!(ppu.read_register(PPUDATA), 0x00);
}

#[test]
fn the_warmup_lockout_ignores_early_writes() {
    // NESdev PPU registers: $2000/$2001/$2005/$2006 are ignored for the first
    // ~29658 CPU cycles after reset.
    let ppu = NesPpu::new(&Props::new()).unwrap();
    ppu.attach_bus(Arc::new(AddressSpace::new("ppu", 14)));
    ppu.write_register(PPUCTRL, CTRL_NMI);
    assert_eq!(ppu.with_engine(|e| e.ctrl), 0, "too early");
    ppu.with_engine(|e| e.dots = WARMUP_DOTS);
    ppu.write_register(PPUCTRL, CTRL_NMI);
    assert_eq!(ppu.with_engine(|e| e.ctrl), CTRL_NMI);
    // $2003 and $2004 were never locked out.
    ppu.with_engine(|e| e.dots = 0);
    ppu.write_register(OAMADDR, 0x40);
    assert_eq!(ppu.oam_addr(), 0x40);
}

// ---------------------------------------------------------------------------
// Palette
// ---------------------------------------------------------------------------

#[test]
fn the_sprite_palette_zeros_alias_the_background_ones() {
    // NESdev PPU palettes: $3F10/$3F14/$3F18/$3F1C are $3F00/$3F04/$3F08/$3F0C.
    let (ppu, _, _) = new_ppu();
    for (sprite, background) in [
        (0x3f10, 0x3f00),
        (0x3f14, 0x3f04),
        (0x3f18, 0x3f08),
        (0x3f1c, 0x3f0c),
    ] {
        ppu.poke_palette(sprite, 0x2a);
        assert_eq!(ppu.peek_palette(background), 0x2a, "{sprite:#x}");
        ppu.poke_palette(background, 0x15);
        assert_eq!(ppu.peek_palette(sprite), 0x15, "{sprite:#x}");
    }
    // $3F11 is its own entry.
    ppu.poke_palette(0x3f01, 0x01);
    ppu.poke_palette(0x3f11, 0x02);
    assert_eq!(ppu.peek_palette(0x3f01), 0x01);
}

#[test]
fn palette_ram_repeats_through_3fff() {
    let (ppu, _, _) = new_ppu();
    ppu.poke_palette(0x3f03, 0x30);
    assert_eq!(ppu.peek_palette(0x3fe3), 0x30);
    assert_eq!(ppu.peek_palette(0x3f23), 0x30);
}

#[test]
fn palette_entries_are_six_bits() {
    let (ppu, _, _) = new_ppu();
    ppu.poke_palette(0x3f00, 0xff);
    assert_eq!(ppu.peek_palette(0x3f00), 0x3f);
}

#[test]
fn a_palette_read_reports_the_top_two_bits_as_open_bus() {
    let (ppu, _, _) = new_ppu();
    ppu.poke_palette(0x3f00, 0x0f);
    set_mask(&ppu, 0xc0); // charges the latch with $C0
    ppu.write_register(PPUADDR, 0x3f);
    settle(&ppu);
    ppu.write_register(PPUADDR, 0x00);
    settle(&ppu);
    // $2006 writes recharged the latch with the last written byte, $00.
    ppu.write_register(PPUSTATUS, 0xc0); // read-only port: latch only
    settle(&ppu);
    assert_eq!(ppu.read_register(PPUDATA), 0xc0 | 0x0f);
}

// ---------------------------------------------------------------------------
// Background rendering
// ---------------------------------------------------------------------------

/// Fill CHR with a tile whose every pixel is colour 1, and one nametable entry.
fn plain_background(chr: &RamStore, nt: &RamStore, tile: u8) {
    for row in 0..8u64 {
        chr.write_u8(u64::from(tile) * 16 + row, 0xff).unwrap(); // low plane
        chr.write_u8(u64::from(tile) * 16 + 8 + row, 0x00).unwrap(); // high plane
    }
    for index in 0..0x3c0u64 {
        nt.write_u8(index, tile).unwrap();
    }
}

#[test]
fn the_background_pipeline_draws_a_uniform_screen() {
    let (ppu, chr, nt) = new_ppu();
    plain_background(&chr, &nt, 1);
    ppu.poke_palette(0x3f00, 0x0f); // backdrop
    ppu.poke_palette(0x3f01, 0x21); // colour 1 of palette 0
    set_mask(&ppu, MASK_BG | MASK_BG_LEFT);
    draw_two_frames(&ppu);
    for x in [0usize, 1, 7, 8, 128, 255] {
        assert_eq!(ppu.pixel(x, 100).unwrap().index(), 0x21, "x = {x}");
    }
}

#[test]
fn the_left_column_mask_hides_the_first_eight_pixels() {
    let (ppu, chr, nt) = new_ppu();
    plain_background(&chr, &nt, 1);
    ppu.poke_palette(0x3f00, 0x0f);
    ppu.poke_palette(0x3f01, 0x21);
    set_mask(&ppu, MASK_BG); // no MASK_BG_LEFT
    draw_two_frames(&ppu);
    assert_eq!(ppu.pixel(0, 100).unwrap().index(), 0x0f);
    assert_eq!(ppu.pixel(7, 100).unwrap().index(), 0x0f);
    assert_eq!(ppu.pixel(8, 100).unwrap().index(), 0x21);
}

#[test]
fn fine_x_shifts_the_background_left() {
    // A tile with only its leftmost pixel set, repeated: fine X moves the
    // pattern by that many pixels (NESdev PPU scrolling).
    let (ppu, chr, nt) = new_ppu();
    for row in 0..8u64 {
        chr.write_u8(16 + row, 0x80).unwrap();
        chr.write_u8(16 + 8 + row, 0x00).unwrap();
    }
    for index in 0..0x3c0u64 {
        nt.write_u8(index, 1).unwrap();
    }
    ppu.poke_palette(0x3f00, 0x0f);
    ppu.poke_palette(0x3f01, 0x21);
    set_mask(&ppu, MASK_BG | MASK_BG_LEFT);
    ppu.write_register(PPUSCROLL, 3); // fine X = 3, coarse X = 0
    ppu.write_register(PPUSCROLL, 0);
    draw_two_frames(&ppu);
    assert_eq!(ppu.pixel(5, 100).unwrap().index(), 0x21);
    assert_eq!(ppu.pixel(4, 100).unwrap().index(), 0x0f);
}

#[test]
fn greyscale_masks_the_palette_index() {
    let (ppu, chr, nt) = new_ppu();
    plain_background(&chr, &nt, 1);
    ppu.poke_palette(0x3f01, 0x21);
    set_mask(&ppu, MASK_BG | MASK_BG_LEFT | MASK_GREYSCALE);
    draw_two_frames(&ppu);
    assert_eq!(ppu.pixel(100, 100).unwrap().index(), 0x20);
}

#[test]
fn emphasis_travels_with_the_pixel() {
    let (ppu, _, _) = new_ppu();
    ppu.poke_palette(0x3f00, 0x0f);
    set_mask(&ppu, MASK_EMPHASIS_R | MASK_EMPHASIS_B);
    draw_two_frames(&ppu);
    assert_eq!(ppu.pixel(10, 10).unwrap().emphasis(), 0b101);
}

#[test]
fn rendering_disabled_with_v_in_the_palette_shows_that_entry() {
    // NESdev PPU palettes: the backdrop override.
    let (ppu, _, _) = new_ppu();
    ppu.poke_palette(0x3f00, 0x0f);
    ppu.poke_palette(0x3f05, 0x16);
    ppu.write_register(PPUADDR, 0x3f);
    ppu.write_register(PPUADDR, 0x05);
    draw_two_frames(&ppu);
    assert_eq!(ppu.pixel(10, 10).unwrap().index(), 0x16);
}

#[test]
fn coarse_x_wraps_into_the_next_nametable() {
    // NESdev PPU scrolling, "coarse X increment".
    let (ppu, _, _) = new_ppu();
    ppu.with_engine(|e| e.v = 0x001f);
    enable_rendering(&ppu);
    seek(&ppu, 0, 8);
    ppu.advance_by(1);
    assert_eq!(ppu.with_engine(|e| e.v), 0x0400);
}

#[test]
fn y_increment_wraps_at_row_29_and_at_row_31() {
    // Row 29 is the last tile row; 30 and 31 hold attribute data, so hardware
    // treats them differently (NESdev PPU scrolling).
    let (ppu, _, _) = new_ppu();
    enable_rendering(&ppu);
    ppu.with_engine(|e| e.v = 0x7000 | (29 << 5));
    seek(&ppu, 0, 256);
    ppu.advance_by(1);
    assert_eq!(ppu.with_engine(|e| e.v) & 0x0be0, 0x0800, "toggles bit 11");

    ppu.with_engine(|e| e.v = 0x7000 | (31 << 5));
    seek(&ppu, 0, 256);
    ppu.advance_by(1);
    assert_eq!(ppu.with_engine(|e| e.v) & 0x0be0, 0x0000, "no toggle");
}

#[test]
fn dot_257_copies_the_horizontal_bits_and_280_to_304_the_vertical_ones() {
    let (ppu, _, _) = new_ppu();
    enable_rendering(&ppu);
    ppu.with_engine(|e| {
        e.v = 0;
        e.t = 0x7fff;
    });
    seek(&ppu, 0, 257);
    ppu.advance_by(1);
    assert_eq!(ppu.with_engine(|e| e.v), 0x041f);

    ppu.with_engine(|e| e.v = 0);
    seek(&ppu, PRE_RENDER_SCANLINE, 280);
    ppu.advance_by(1);
    assert_eq!(ppu.with_engine(|e| e.v), 0x7be0);
}

#[test]
fn writing_2007_while_rendering_moves_the_scroll_counters() {
    // NESdev PPU registers, PPUDATA: during rendering the address increment is
    // a coarse-X and a Y increment, not +1 or +32.
    let (ppu, _, _) = new_ppu();
    enable_rendering(&ppu);
    seek(&ppu, 0, 100);
    ppu.with_engine(|e| e.v = 0x0000);
    ppu.write_register(PPUDATA, 0x00);
    assert_eq!(ppu.with_engine(|e| e.v), 0x1001);
}

// ---------------------------------------------------------------------------
// Sprites
// ---------------------------------------------------------------------------

/// A sprite whose whole 8x8 tile is colour 1.
fn opaque_sprite(chr: &RamStore, tile: u8) {
    for row in 0..8u64 {
        chr.write_u8(u64::from(tile) * 16 + row, 0xff).unwrap();
        chr.write_u8(u64::from(tile) * 16 + 8 + row, 0x00).unwrap();
    }
}

#[test]
fn a_sprite_is_drawn_one_line_below_its_y_byte() {
    // NESdev PPU OAM: byte 0 is the Y coordinate minus one.
    let (ppu, chr, _) = new_ppu();
    opaque_sprite(&chr, 1);
    ppu.poke_palette(0x3f00, 0x0f);
    ppu.poke_palette(0x3f11, 0x27);
    ppu.poke_oam(0, 50); // Y
    ppu.poke_oam(1, 1); // tile
    ppu.poke_oam(2, 0); // attributes: palette 0, in front
    ppu.poke_oam(3, 40); // X
    set_mask(&ppu, MASK_SPRITE | MASK_SPRITE_LEFT);
    draw_two_frames(&ppu);
    assert_eq!(ppu.pixel(40, 50).unwrap().index(), 0x0f, "not on line 50");
    assert_eq!(ppu.pixel(40, 51).unwrap().index(), 0x27);
    assert_eq!(ppu.pixel(47, 58).unwrap().index(), 0x27);
    assert_eq!(ppu.pixel(48, 58).unwrap().index(), 0x0f);
}

#[test]
fn sprites_never_appear_on_scanline_zero() {
    // Evaluation does not run on the pre-render line, so nothing is ever in
    // secondary OAM for line 0 (NESdev PPU sprite evaluation).
    let (ppu, chr, _) = new_ppu();
    opaque_sprite(&chr, 1);
    ppu.poke_palette(0x3f00, 0x0f);
    ppu.poke_palette(0x3f11, 0x27);
    ppu.poke_oam(0, 0xff); // Y = 255 wraps into range for line 0 if evaluated
    ppu.poke_oam(1, 1);
    ppu.poke_oam(3, 0);
    set_mask(&ppu, MASK_SPRITE | MASK_SPRITE_LEFT);
    draw_two_frames(&ppu);
    assert_eq!(ppu.pixel(0, 0).unwrap().index(), 0x0f);
}

#[test]
fn an_8x16_sprite_takes_its_bank_from_the_tile_byte() {
    // NESdev PPU OAM: bit 0 of the tile byte selects the pattern table and the
    // bottom half is the next tile up.
    let (ppu, chr, _) = new_ppu();
    // Top half in bank $1000 tile $02, bottom half tile $03.
    for row in 0..8u64 {
        chr.write_u8(0x1000 + 0x02 * 16 + row, 0xff).unwrap();
        chr.write_u8(0x1000 + 0x03 * 16 + row, 0x00).unwrap();
        chr.write_u8(0x1000 + 0x03 * 16 + 8 + row, 0xff).unwrap();
    }
    ppu.poke_palette(0x3f00, 0x0f);
    ppu.poke_palette(0x3f11, 0x27); // colour 1
    ppu.poke_palette(0x3f12, 0x28); // colour 2
    ppu.poke_oam(0, 50);
    ppu.poke_oam(1, 0x03); // tile $02, bank 1
    ppu.poke_oam(2, 0);
    ppu.poke_oam(3, 40);
    ppu.write_register(PPUCTRL, CTRL_SPRITE_16);
    set_mask(&ppu, MASK_SPRITE | MASK_SPRITE_LEFT);
    draw_two_frames(&ppu);
    assert_eq!(ppu.pixel(40, 51).unwrap().index(), 0x27, "top half");
    assert_eq!(ppu.pixel(40, 59).unwrap().index(), 0x28, "bottom half");
    assert_eq!(ppu.pixel(40, 67).unwrap().index(), 0x0f, "16 rows only");
}

#[test]
fn sprite_flipping_mirrors_the_pattern() {
    let (ppu, chr, _) = new_ppu();
    // A tile with only the leftmost column and the top row set.
    chr.write_u8(16, 0xff).unwrap();
    for row in 1..8u64 {
        chr.write_u8(16 + row, 0x80).unwrap();
    }
    ppu.poke_palette(0x3f00, 0x0f);
    ppu.poke_palette(0x3f11, 0x27);
    ppu.poke_oam(0, 50);
    ppu.poke_oam(1, 1);
    ppu.poke_oam(2, SPRITE_FLIP_X | SPRITE_FLIP_Y);
    ppu.poke_oam(3, 40);
    set_mask(&ppu, MASK_SPRITE | MASK_SPRITE_LEFT);
    draw_two_frames(&ppu);
    // Flipped both ways, the solid row is at the bottom and the column right.
    assert_eq!(ppu.pixel(47, 58).unwrap().index(), 0x27);
    assert_eq!(ppu.pixel(40, 58).unwrap().index(), 0x27);
    assert_eq!(ppu.pixel(40, 51).unwrap().index(), 0x0f);
}

#[test]
fn only_eight_sprites_are_drawn_and_the_ninth_sets_overflow() {
    // NESdev PPU sprite evaluation: secondary OAM holds eight.
    let (ppu, chr, _) = new_ppu();
    opaque_sprite(&chr, 1);
    ppu.poke_palette(0x3f00, 0x0f);
    ppu.poke_palette(0x3f11, 0x27);
    for index in 0..9u8 {
        ppu.poke_oam(index * 4, 50);
        ppu.poke_oam(index * 4 + 1, 1);
        ppu.poke_oam(index * 4 + 2, 0);
        ppu.poke_oam(index * 4 + 3, index * 8);
    }
    set_mask(&ppu, MASK_SPRITE | MASK_SPRITE_LEFT);
    draw_two_frames(&ppu);
    assert_eq!(
        ppu.pixel(56, 51).unwrap().index(),
        0x27,
        "the eighth is drawn"
    );
    assert_eq!(ppu.pixel(64, 51).unwrap().index(), 0x0f, "the ninth is not");
    assert_ne!(ppu.with_engine(|e| e.status) & STATUS_OVERFLOW, 0);
}

#[test]
fn the_overflow_bug_reads_a_tile_byte_as_a_y_coordinate() {
    // Step 3 of NESdev PPU sprite evaluation: on a miss the hardware increments
    // n *and* m without carry, so it walks OAM diagonally and range-checks bytes
    // that are not Y coordinates at all. Here exactly eight sprites are on the
    // line — no real overflow — but sprite 8's *tile* byte lands in the diagonal
    // walk with a value that is in range, and the flag comes up anyway.
    let (ppu, chr, _) = new_ppu();
    opaque_sprite(&chr, 1);
    for index in 0..8u8 {
        ppu.poke_oam(index * 4, 50);
        ppu.poke_oam(index * 4 + 1, 1);
        ppu.poke_oam(index * 4 + 2, 0);
        ppu.poke_oam(index * 4 + 3, index * 8);
    }
    // Sprites 8..63 are off-screen, so a correct chip would report nothing.
    for index in 8..64u8 {
        ppu.poke_oam(index * 4, 0xf0);
        ppu.poke_oam(index * 4 + 1, 0xf0);
        ppu.poke_oam(index * 4 + 2, 0xf0);
        ppu.poke_oam(index * 4 + 3, 0xf0);
    }
    // The walk examines (n=8, m=0), then (9, 1), then (10, 2): the second byte
    // it looks at is sprite 9's *tile* number.
    ppu.poke_oam(9 * 4 + 1, 50);
    set_mask(&ppu, MASK_SPRITE | MASK_SPRITE_LEFT);
    draw_two_frames(&ppu);
    assert_ne!(
        ppu.with_engine(|e| e.status) & STATUS_OVERFLOW,
        0,
        "a tile byte in range must trip the buggy overflow search"
    );
}

#[test]
fn the_overflow_flag_stays_clear_with_eight_sprites_and_nothing_in_range() {
    let (ppu, chr, _) = new_ppu();
    opaque_sprite(&chr, 1);
    for index in 0..8u8 {
        ppu.poke_oam(index * 4, 50);
        ppu.poke_oam(index * 4 + 1, 1);
        ppu.poke_oam(index * 4 + 2, 0);
        ppu.poke_oam(index * 4 + 3, index * 8);
    }
    for index in 8..64u8 {
        for byte in 0..4u8 {
            ppu.poke_oam(index * 4 + byte, 0xf0);
        }
    }
    set_mask(&ppu, MASK_SPRITE | MASK_SPRITE_LEFT);
    draw_two_frames(&ppu);
    assert_eq!(ppu.with_engine(|e| e.status) & STATUS_OVERFLOW, 0);
}

#[test]
fn sprite_priority_is_by_oam_index_then_by_the_priority_bit() {
    let (ppu, chr, nt) = new_ppu();
    opaque_sprite(&chr, 1);
    opaque_sprite(&chr, 2);
    plain_background(&chr, &nt, 3);
    // Tile 3 is transparent so the background does not interfere.
    for row in 0..8u64 {
        chr.write_u8(3 * 16 + row, 0x00).unwrap();
        chr.write_u8(3 * 16 + 8 + row, 0x00).unwrap();
    }
    ppu.poke_palette(0x3f00, 0x0f);
    ppu.poke_palette(0x3f11, 0x27); // sprite palette 0
    ppu.poke_palette(0x3f15, 0x28); // sprite palette 1
    ppu.poke_oam(0, 50);
    ppu.poke_oam(1, 1);
    ppu.poke_oam(2, 0);
    ppu.poke_oam(3, 40);
    ppu.poke_oam(4, 50);
    ppu.poke_oam(5, 2);
    ppu.poke_oam(6, 1); // palette 1
    ppu.poke_oam(7, 40);
    ppu.write_register(
        PPUMASK,
        MASK_BG | MASK_SPRITE | MASK_BG_LEFT | MASK_SPRITE_LEFT,
    );
    draw_two_frames(&ppu);
    assert_eq!(ppu.pixel(40, 51).unwrap().index(), 0x27, "lower index wins");
}

#[test]
fn a_behind_sprite_loses_to_opaque_background() {
    let (ppu, chr, nt) = new_ppu();
    opaque_sprite(&chr, 1);
    plain_background(&chr, &nt, 2);
    opaque_sprite(&chr, 2);
    ppu.poke_palette(0x3f00, 0x0f);
    ppu.poke_palette(0x3f01, 0x11); // background colour 1
    ppu.poke_palette(0x3f11, 0x27);
    ppu.poke_oam(0, 50);
    ppu.poke_oam(1, 1);
    ppu.poke_oam(2, SPRITE_BEHIND);
    ppu.poke_oam(3, 40);
    ppu.write_register(
        PPUMASK,
        MASK_BG | MASK_SPRITE | MASK_BG_LEFT | MASK_SPRITE_LEFT,
    );
    draw_two_frames(&ppu);
    assert_eq!(ppu.pixel(40, 51).unwrap().index(), 0x11);
}

// ---------------------------------------------------------------------------
// Sprite 0 hit
// ---------------------------------------------------------------------------

/// Put an opaque sprite 0 at `(x, y)` over an opaque background.
fn sprite_zero_scene(ppu: &NesPpu, chr: &RamStore, nt: &RamStore, x: u8, y: u8) {
    opaque_sprite(chr, 1);
    plain_background(chr, nt, 1);
    ppu.poke_palette(0x3f00, 0x0f);
    ppu.poke_palette(0x3f01, 0x11);
    ppu.poke_palette(0x3f11, 0x27);
    ppu.poke_oam(0, y);
    ppu.poke_oam(1, 1);
    ppu.poke_oam(2, 0);
    ppu.poke_oam(3, x);
}

#[test]
fn sprite_zero_hits_where_both_layers_are_opaque() {
    let (ppu, chr, nt) = new_ppu();
    sprite_zero_scene(&ppu, &chr, &nt, 40, 50);
    ppu.write_register(
        PPUMASK,
        MASK_BG | MASK_SPRITE | MASK_BG_LEFT | MASK_SPRITE_LEFT,
    );
    draw_two_frames(&ppu);
    assert_ne!(ppu.with_engine(|e| e.status) & STATUS_SPRITE0, 0);
}

#[test]
fn sprite_zero_never_hits_at_x_255() {
    // NESdev PPU registers, PPUSTATUS: no hit at x = 255.
    let (ppu, chr, nt) = new_ppu();
    sprite_zero_scene(&ppu, &chr, &nt, 255, 50);
    ppu.write_register(
        PPUMASK,
        MASK_BG | MASK_SPRITE | MASK_BG_LEFT | MASK_SPRITE_LEFT,
    );
    draw_two_frames(&ppu);
    assert_eq!(ppu.with_engine(|e| e.status) & STATUS_SPRITE0, 0);
}

#[test]
fn sprite_zero_never_hits_in_a_clipped_left_column() {
    let (ppu, chr, nt) = new_ppu();
    sprite_zero_scene(&ppu, &chr, &nt, 0, 50);
    // Background left column shown, sprites clipped: no hit in x 0..7.
    set_mask(&ppu, MASK_BG | MASK_SPRITE | MASK_BG_LEFT);
    draw_two_frames(&ppu);
    assert_eq!(ppu.with_engine(|e| e.status) & STATUS_SPRITE0, 0);
}

#[test]
fn sprite_zero_never_hits_with_a_layer_disabled() {
    let (ppu, chr, nt) = new_ppu();
    sprite_zero_scene(&ppu, &chr, &nt, 40, 50);
    set_mask(&ppu, MASK_SPRITE | MASK_SPRITE_LEFT); // no background
    draw_two_frames(&ppu);
    assert_eq!(ppu.with_engine(|e| e.status) & STATUS_SPRITE0, 0);
}

#[test]
fn sprite_zero_never_hits_on_a_transparent_background_pixel() {
    let (ppu, chr, nt) = new_ppu();
    sprite_zero_scene(&ppu, &chr, &nt, 40, 50);
    // Make the background tile transparent everywhere.
    for row in 0..8u64 {
        chr.write_u8(16 + row, 0x00).unwrap();
    }
    ppu.write_register(
        PPUMASK,
        MASK_BG | MASK_SPRITE | MASK_BG_LEFT | MASK_SPRITE_LEFT,
    );
    draw_two_frames(&ppu);
    assert_eq!(ppu.with_engine(|e| e.status) & STATUS_SPRITE0, 0);
}

#[test]
fn the_sprite_zero_flag_lands_one_dot_after_the_pixel() {
    // NESdev PPU rendering: "sprite 0 hit acts as if the image starts at cycle
    // 2", so a hit on pixel x is visible at dot x + 2.
    let (ppu, chr, nt) = new_ppu();
    sprite_zero_scene(&ppu, &chr, &nt, 0, 50);
    ppu.write_register(
        PPUMASK,
        MASK_BG | MASK_SPRITE | MASK_BG_LEFT | MASK_SPRITE_LEFT,
    );
    // Run to the line the sprite is on, then step dot by dot.
    ppu.advance_by(DOTS_PER_FRAME); // finish the first frame
    while ppu.position() != (51, 0) {
        ppu.advance_by(1);
    }
    ppu.advance_by(1); // dot 0: idle
    assert_eq!(ppu.with_engine(|e| e.status) & STATUS_SPRITE0, 0);
    ppu.advance_by(1); // dot 1 draws pixel 0 and arms the flag
    assert_eq!(
        ppu.with_engine(|e| e.status) & STATUS_SPRITE0,
        0,
        "not yet visible"
    );
    ppu.advance_by(1); // dot 2 publishes it
    assert_ne!(ppu.with_engine(|e| e.status) & STATUS_SPRITE0, 0);
}

// ---------------------------------------------------------------------------
// OAM
// ---------------------------------------------------------------------------

#[test]
fn the_unimplemented_attribute_bits_read_back_as_zero() {
    // NESdev PPU OAM: bits 2-4 of byte 2 do not exist.
    let (ppu, _, _) = new_ppu();
    ppu.write_register(OAMADDR, 2);
    ppu.write_register(OAMDATA, 0xff);
    ppu.write_register(OAMADDR, 2);
    assert_eq!(ppu.read_register(OAMDATA), 0xe3);
}

#[test]
fn writing_2004_while_rendering_bumps_oamaddr_without_storing() {
    // NESdev PPU registers, OAMDATA.
    let (ppu, _, _) = new_ppu();
    enable_rendering(&ppu);
    seek(&ppu, 10, 100);
    ppu.write_register(OAMADDR, 0x10);
    ppu.write_register(OAMDATA, 0x5a);
    assert_eq!(ppu.peek_oam(0x10), 0x00, "OAM is busy being evaluated");
    assert_eq!(ppu.oam_addr(), 0x14, "only the high six bits moved");
}

#[test]
fn reading_2004_listens_in_on_whatever_the_sprite_unit_is_reading() {
    // `OAMADDR` is not the answer while the sprite unit owns OAM: a `$2004`
    // read picks up the OAM read line, and that carries something different on
    // each phase of the scanline (NESdev, *PPU sprite evaluation*).
    let (ppu, _, _) = new_ppu();
    enable_rendering(&ppu);
    ppu.poke_oam(0, 0x12);
    // Dots 1-64: the secondary-OAM clear forces the read line.
    seek(&ppu, 10, 30);
    assert_eq!(ppu.read_register(OAMDATA), 0xff);
    // Dots 65-256: the primary-OAM read latch. Run dots 64 and 65 for real —
    // 64 captures the base OAMADDR, 65 is the odd dot that fills the latch with
    // the first sprite's Y coordinate.
    seek(&ppu, 10, 64);
    ppu.advance_by(2);
    assert_eq!(ppu.position(), (10, 66));
    assert_eq!(ppu.read_register(OAMDATA), 0x12);
    // Dots 257-320: secondary OAM, which nothing was copied into.
    seek(&ppu, 10, 300);
    assert_eq!(ppu.read_register(OAMDATA), 0xff);
    // And with rendering off the unit is not driving the line at all, so the
    // read is the ordinary `OAMADDR` one.
    ppu.write_register(PPUMASK, 0);
    ppu.with_engine(|e| {
        e.mask = 0;
        e.mask_delay = 0;
    });
    ppu.write_register(OAMADDR, 0);
    assert_eq!(ppu.read_register(OAMDATA), 0x12);
}

#[test]
fn oam_dma_bytes_go_through_the_2004_path() {
    let (ppu, _, _) = new_ppu();
    ppu.write_register(OAMADDR, 0);
    for index in 0..4u8 {
        ppu.oam_dma_write(0x40 + index);
    }
    assert_eq!(ppu.peek_oam(0), 0x40);
    assert_eq!(ppu.peek_oam(2), 0x42 & SPRITE_ATTR_IMPLEMENTED);
    assert_eq!(ppu.oam_addr(), 4);
}

#[test]
fn switching_rendering_off_mid_line_corrupts_the_row_it_comes_back_on() {
    // The other half of the same handover. Rendering goes away while the sprite
    // unit is standing somewhere in secondary OAM; the address goes back to
    // `OAMADDR`, and when rendering returns it is handed over again — copying
    // `OAMADDR`'s row over the row the unit had reached (NESdev, *Errata*).
    let (ppu, _, _) = new_ppu();
    for index in 0..64u8 {
        ppu.poke_oam(index, 0xa0 | (index & 0x0f));
    }
    ppu.write_register(OAMADDR, 0);
    enable_rendering(&ppu);
    // Somewhere inside the secondary-OAM clear, so the pointer is not zero.
    seek(&ppu, 10, 40);
    ppu.advance_by(1);

    // The real write path, because the handover is armed where the delayed
    // `$2001` write finally commits.
    ppu.write_register(PPUMASK, 0);
    ppu.advance_by(4);
    let row = ppu
        .with_engine(|e| e.corrupt_row)
        .expect("armed on the way down");
    assert_ne!(row, 0, "the unit was standing somewhere real");
    ppu.write_register(
        PPUMASK,
        MASK_BG | MASK_SPRITE | MASK_BG_LEFT | MASK_SPRITE_LEFT,
    );
    ppu.advance_by(4);

    for i in 0..8u8 {
        assert_eq!(
            ppu.peek_oam(row * 8 + i),
            ppu.peek_oam(i),
            "row {row}, byte {i}: OAMADDR's row should have been copied here"
        );
    }
}

#[test]
fn a_high_oamaddr_corrupts_the_first_eight_oam_bytes_at_rendering_start() {
    // NESdev PPU registers, OAMADDR: "if OAMADDR is not less than eight when
    // rendering begins, the eight bytes starting at OAMADDR & $F8 are copied to
    // the first eight bytes of OAM".
    let (ppu, _, _) = new_ppu();
    for index in 0..8u8 {
        ppu.poke_oam(index, 0x00);
        ppu.poke_oam(0x20 + index, 0xa0 + index);
    }
    enable_rendering(&ppu);
    ppu.write_register(OAMADDR, 0x20);
    seek(&ppu, PRE_RENDER_SCANLINE, 0);
    ppu.advance_by(1);
    for index in 0..8u8 {
        let expected = if index & 3 == 2 {
            (0xa0 + index) & SPRITE_ATTR_IMPLEMENTED
        } else {
            0xa0 + index
        };
        assert_eq!(ppu.peek_oam(index), expected, "byte {index}");
    }
}

#[test]
fn a_misaligned_oamaddr_reinterprets_oam_bytes() {
    // NESdev PPU registers, OAMADDR: evaluation starts at OAMADDR, so a base
    // that is not a multiple of four makes tile, attribute and X bytes act as Y
    // coordinates and shifts every sprite's fields along by that much.
    let (ppu, chr, _) = new_ppu();
    opaque_sprite(&chr, 1);
    ppu.poke_oam(0, 50);
    ppu.poke_oam(1, 1);
    ppu.poke_oam(2, 0);
    ppu.poke_oam(3, 40);
    ppu.poke_oam(4, 50);
    ppu.poke_oam(5, 1);
    ppu.poke_oam(6, 0);
    ppu.poke_oam(7, 60);
    set_mask(&ppu, MASK_SPRITE | MASK_SPRITE_LEFT);
    // OAMADDR is forced back to zero at dots 257-320 of every rendering line,
    // so the write has to land after that window and before the next line's
    // evaluation starts at dot 65.
    ppu.advance_by(DOTS_PER_FRAME);
    while ppu.position() != (40, 330) {
        ppu.advance_by(1);
    }
    ppu.write_register(OAMADDR, 3);
    ppu.advance_by(u64::from(DOTS_PER_SCANLINE));
    assert_eq!(ppu.position(), (41, 330));
    ppu.with_engine(|e| {
        assert_eq!(e.eval_base, 3);
        // Byte 3 (=40) was read as a Y coordinate, and bytes 4, 5 and 6 as the
        // tile, attributes and X of a sprite that does not exist in OAM.
        assert_eq!(&e.secondary_oam[..4], &[40, 50, 1, 0]);
        assert_eq!(e.eval_found, 1);
    });
}

// ---------------------------------------------------------------------------
// Device lifecycle and snapshots
// ---------------------------------------------------------------------------

#[test]
fn realize_no_longer_demands_a_bus_of_its_own() {
    // The check used to live here, which is the right place for a hand-wired
    // machine and the wrong one for a described one: the realizer hands a
    // device its address space at *bind* time, after every region is mapped, so
    // a DSL-built PPU has no bus yet when `realize` runs. `Instance::bind` is
    // where a missing `space =` is now reported — see
    // `machine::tests::a_ppu_without_an_address_space_is_refused`.
    let ppu = NesPpu::new(&Props::new()).unwrap();
    let mut deferred = Deferred::new();
    let mut ctx = RealizeCtx::new("ppu", RequesterId(1), &mut deferred);
    ppu.realize(&mut ctx)
        .expect("realize is about the wire, not the bus");
}

#[test]
fn realize_announces_the_nmi_line() {
    // ROADMAP.md §4.3: a freshly realized machine must have every wire driving
    // what its state implies.
    let (ppu, _, _) = new_ppu();
    let nmi = with_nmi(&ppu);
    let mut deferred = Deferred::new();
    let mut ctx = RealizeCtx::new("ppu", RequesterId(1), &mut deferred);
    ppu.realize(&mut ctx).unwrap();
    // Idle low, and announced rather than assumed.
    assert_eq!(nmi.levels.lock().as_slice(), &[] as &[bool]);
    assert!(!ppu.with_engine(|e| e.nmi_active()));
}

#[test]
fn a_cold_reset_returns_every_register_to_its_documented_value() {
    let (ppu, _, _) = new_ppu();
    ppu.write_register(PPUCTRL, 0xff);
    set_mask(&ppu, 0xff);
    ppu.advance_by(1000);
    ppu.reset(ResetKind::Cold);
    ppu.with_engine(|e| {
        assert_eq!(e.ctrl, 0);
        assert_eq!(e.mask, 0);
        assert_eq!(e.status, 0);
        assert_eq!(e.v, 0);
        assert_eq!(e.t, 0);
        assert_eq!(e.dots, 0);
    });
}

/// A cheap order-sensitive hash, so a round trip is compared by value rather
/// than by a hand-written field-by-field assertion that can rot.
fn state_hash(ppu: &NesPpu) -> u64 {
    let mut w = StateWriter::new(MachineShape::new());
    {
        let mut chunk = w
            .chunk("/ppu", NES_PPU_CLASS.name, NES_PPU_CLASS.version)
            .unwrap();
        ppu.save(&mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();
    let mut hash = 0xcbf2_9ce4_8422_2325u64;
    for byte in bytes {
        hash ^= u64::from(byte);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    hash
}

#[test]
fn a_mid_frame_snapshot_round_trips_including_the_shift_registers() {
    // Invariant 6, and the reason the background shifters are saved: a snapshot
    // taken mid-scanline has to resume drawing the same pixels.
    let (ppu, chr, nt) = new_ppu();
    plain_background(&chr, &nt, 1);
    ppu.poke_palette(0x3f01, 0x21);
    enable_rendering(&ppu);
    // Stop somewhere with a fetch in flight and sprites half-evaluated.
    ppu.advance_by(DOTS_PER_FRAME + 100 * u64::from(DOTS_PER_SCANLINE) + 123);
    assert_ne!(ppu.with_engine(|e| e.bg_shift_lo), 0, "a tile is in flight");

    let mut w = StateWriter::new(MachineShape::new());
    {
        let mut chunk = w
            .chunk("/ppu", NES_PPU_CLASS.name, NES_PPU_CLASS.version)
            .unwrap();
        ppu.save(&mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();
    let before = state_hash(&ppu);

    let (restored, chr2, nt2) = new_ppu();
    plain_background(&chr2, &nt2, 1);
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load(
            "/ppu",
            NES_PPU_CLASS.name,
            NES_PPU_CLASS.version,
            &Migrations::new(),
        )
        .unwrap();
    restored.load(&mut chunk.reader()).unwrap();
    assert_eq!(state_hash(&restored), before);

    // And they stay identical when both are run on.
    ppu.advance_by(5000);
    restored.advance_by(5000);
    assert_eq!(state_hash(&restored), state_hash(&ppu));
}

#[test]
fn a_short_chunk_is_rejected() {
    let (ppu, _, _) = new_ppu();
    let mut reader = ChunkReader::new(&[0u8; 4]);
    assert!(ppu.load(&mut reader).is_err());
}

// ---------------------------------------------------------------------------
// Framebuffer
// ---------------------------------------------------------------------------

#[test]
fn the_framebuffer_is_256_by_240_indexed_pixels() {
    let (ppu, _, _) = new_ppu();
    ppu.with_framebuffer(|fb| assert_eq!(fb.len(), FRAMEBUFFER_LEN));
    assert!(ppu.pixel(255, 239).is_some());
    assert!(ppu.pixel(256, 0).is_none());
    assert!(ppu.pixel(0, 240).is_none());
}

#[test]
fn a_pixel_packs_an_index_and_emphasis() {
    let pixel = Pixel::new(0x3f, 0b101);
    assert_eq!(pixel.index(), 0x3f);
    assert_eq!(pixel.emphasis(), 0b101);
    assert_eq!(Pixel::new(0xff, 0xff), Pixel::new(0x3f, 0b111));
}

// ---------------------------------------------------------------------------
// Access constraints
// ---------------------------------------------------------------------------

#[test]
fn the_register_block_takes_byte_accesses_only() {
    let (ppu, _, _) = new_ppu();
    let port = ppu.port();
    let mut wide = [0u8; 2];
    assert!(port.read(0, &mut wide, MemAttrs::DEFAULT).is_err());
    assert!(port.write(0, &[0, 0], MemAttrs::DEFAULT).is_err());
    assert_eq!(port.constraints().max, Width::U8);
}

#[test]
fn a_debug_write_is_refused_rather_than_guessed_at() {
    let (ppu, _, _) = new_ppu();
    let port = ppu.port();
    assert!(port.write(0, &[0xff], MemAttrs::DEBUG).is_err());
    assert_eq!(ppu.with_engine(|e| e.ctrl), 0);
}

// ---------------------------------------------------------------------------
// Regions
//
// Every figure asserted here comes from the NESdev cycle reference chart
// (https://www.nesdev.org/wiki/Cycle_reference_chart) and the two sentences on
// PPU rendering: "Dendy PPUs render 51 post-render scanlines instead of 1" and
// "PAL NES PPUs render 70 vblank scanlines instead of 20".
// ---------------------------------------------------------------------------

/// The three regions, for tests that must cover all of them.
const REGIONS: [Region; 3] = [Region::Ntsc, Region::Pal, Region::Dendy];

#[test]
fn every_regions_geometry_adds_up_to_its_frame() {
    // The chart states picture, post-render, vblank and pre-render separately
    // and a total dots-per-frame elsewhere; the two only agree if all four
    // parts and 240 rendered scanlines are accounted for. That they do is the
    // reason "picture height 239" is read as "240 rendered, top one blacked
    // out by the border" rather than "239 rendered".
    for region in REGIONS {
        let g = region.geometry();
        assert_eq!(
            g.visible_scanlines + g.post_render_lines + g.vblank_lines + 1,
            g.scanlines_per_frame,
            "{region}"
        );
        assert_eq!(g.vblank_scanline, g.visible_scanlines + g.post_render_lines);
        assert_eq!(g.pre_render_scanline, g.scanlines_per_frame - 1);
        assert_eq!(
            g.dots_per_frame,
            u64::from(DOTS_PER_SCANLINE) * u64::from(g.scanlines_per_frame)
        );
        assert!(g.picture_height <= g.visible_scanlines);
    }
}

#[test]
fn the_frame_is_89342_106392_and_106392_dots() {
    assert_eq!(Region::Ntsc.geometry().dots_per_frame, 89_342);
    assert_eq!(Region::Pal.geometry().dots_per_frame, 106_392);
    assert_eq!(Region::Dendy.geometry().dots_per_frame, 106_392);

    // And the engine really walks that many dots per frame.
    for region in REGIONS {
        let (ppu, _, _) = new_ppu_in(region);
        ppu.advance_by(region.geometry().dots_per_frame);
        assert_eq!(ppu.frame(), 1, "{region}");
        assert_eq!(ppu.position(), (0, 0), "{region}");
    }
}

#[test]
fn the_odd_frame_skip_is_ntsc_only() {
    // The chart gives 341 x 261 + 340.5 for the 2C02 and a flat 341 x 312 for
    // the 2C07 and the UA6538.
    for region in REGIONS {
        let g = region.geometry();
        let (ppu, _, _) = new_ppu_in(region);
        enable_rendering(&ppu);
        ppu.advance_by(g.dots_per_frame); // frame 0 is even, always full length
        assert_eq!(ppu.frame(), 1, "{region}");
        assert_eq!(ppu.position(), (0, 0), "{region}");

        // Frame 1 is odd: one dot shorter on NTSC, the same length elsewhere.
        let odd = g.dots_per_frame - u64::from(g.odd_frame_skip);
        ppu.advance_by(odd);
        assert_eq!(ppu.frame(), 2, "{region}");
        assert_eq!(ppu.position(), (0, 0), "{region}");
        assert_eq!(g.odd_frame_skip, region == Region::Ntsc, "{region}");
    }
}

#[test]
fn an_odd_pal_frame_is_not_short_even_with_rendering_on() {
    // The negative half of the test above, stated as the bug it prevents: on a
    // 2C07 the dot after (339, 311) is (340, 311), not (0, 0).
    let g = Region::Pal.geometry();
    let (ppu, _, _) = new_ppu_in(Region::Pal);
    enable_rendering(&ppu);
    ppu.advance_by(g.dots_per_frame); // frame 0
    ppu.advance_by(g.dots_per_frame - 2); // two dots short of the end of frame 1
    assert_eq!(ppu.position(), (g.pre_render_scanline, 339));
    ppu.advance_by(1);
    assert_eq!(ppu.position(), (g.pre_render_scanline, 340));
    assert_eq!(ppu.frame(), 1, "still inside the odd frame");
    ppu.advance_by(1);
    assert_eq!(ppu.frame(), 2);
}

#[test]
fn vblank_is_set_and_cleared_on_each_regions_own_scanlines() {
    // NTSC and PAL raise the NMI one scanline after the picture; Dendy waits
    // 51, which is the whole trick that lets it keep a Famicom CPU rate.
    assert_eq!(Region::Ntsc.geometry().vblank_scanline, 241);
    assert_eq!(Region::Pal.geometry().vblank_scanline, 241);
    assert_eq!(Region::Dendy.geometry().vblank_scanline, 291);
    assert_eq!(Region::Ntsc.geometry().pre_render_scanline, 261);
    assert_eq!(Region::Pal.geometry().pre_render_scanline, 311);
    assert_eq!(Region::Dendy.geometry().pre_render_scanline, 311);

    for region in REGIONS {
        let g = region.geometry();
        let (ppu, _, _) = new_ppu_in(region);

        seek(&ppu, g.vblank_scanline, 0);
        ppu.advance_by(1); // dot 0: not yet
        assert_eq!(ppu.with_engine(|e| e.status) & STATUS_VBLANK, 0, "{region}");
        ppu.advance_by(1); // dot 1: set
        assert_eq!(
            ppu.with_engine(|e| e.status) & STATUS_VBLANK,
            STATUS_VBLANK,
            "{region}"
        );

        seek(&ppu, g.pre_render_scanline, 0);
        ppu.advance_by(1); // dot 0: still set
        assert_ne!(ppu.with_engine(|e| e.status) & STATUS_VBLANK, 0, "{region}");
        ppu.advance_by(1); // dot 1: cleared
        assert_eq!(ppu.with_engine(|e| e.status) & STATUS_VBLANK, 0, "{region}");
    }
}

#[test]
fn the_vblank_window_is_the_length_the_chart_gives() {
    // 20, 70 and 20 scanlines between the NMI and the pre-render line.
    assert_eq!(Region::Ntsc.geometry().vblank_lines, 20);
    assert_eq!(Region::Pal.geometry().vblank_lines, 70);
    assert_eq!(Region::Dendy.geometry().vblank_lines, 20);
    for region in REGIONS {
        let g = region.geometry();
        assert_eq!(
            g.pre_render_scanline - g.vblank_scanline,
            g.vblank_lines,
            "{region}"
        );
    }
}

#[test]
fn the_dividers_give_3_16_over_5_and_3_dots_per_cpu_cycle() {
    // ROADMAP.md §4.2: the ratio is exact because both counters descend from
    // one crystal, and PAL's 3.2 is exact for the same reason — the forest
    // counts master ticks, so nothing ever holds 3.2.
    for region in REGIONS {
        let (num, den) = region.master_clock();
        let mut forest = ClockForest::new();
        let master = forest
            .add_oscillator("master", Rational::new(num, den).unwrap())
            .unwrap();
        let cpu = forest
            .add_domain("cpu", master, 1, region.cpu_divider())
            .unwrap();
        let dots = add_clock_domain(&mut forest, master, region).unwrap();
        // Five CPU cycles is a whole number of dots in every region, which is
        // the smallest common statement of 3, 3.2 and 3.
        assert_eq!(
            forest.convert_ticks(cpu, dots, 5).unwrap(),
            5 * region.cpu_divider() / region.dot_divider(),
            "{region}"
        );
    }
    assert_eq!(Region::Pal.cpu_divider(), 16);
    assert_eq!(Region::Pal.dot_divider(), 5);
    assert_eq!(Region::Dendy.cpu_divider(), 15);
    assert_eq!(Region::Dendy.dot_divider(), 5);
    // The famous one, stated the way games depend on it.
    let mut forest = ClockForest::new();
    let master = forest
        .add_oscillator("master", Rational::new(236_250_000, 11).unwrap())
        .unwrap();
    let cpu = forest.add_domain("cpu", master, 1, 12).unwrap();
    let dots = add_clock_domain(&mut forest, master, Region::Ntsc).unwrap();
    assert_eq!(forest.convert_ticks(cpu, dots, 1).unwrap(), 3);
}

#[test]
fn a_frame_is_the_number_of_cpu_cycles_the_chart_gives() {
    // 29780.5, 33247.5 and 35464 CPU cycles. The first two are not whole
    // numbers, so the check is on master clocks, which is where the exactness
    // actually lives.
    for (region, cycles_x2) in [
        (Region::Ntsc, 59_561u64), // 2 x 29780.5, with the odd-frame skip
        (Region::Pal, 66_495),     // 2 x 33247.5
        (Region::Dendy, 70_928),   // 2 x 35464
    ] {
        let g = region.geometry();
        let dots_x2 = 2 * g.dots_per_frame - u64::from(g.odd_frame_skip);
        assert_eq!(
            dots_x2 * region.dot_divider(),
            cycles_x2 * region.cpu_divider(),
            "{region}"
        );
    }
}

#[test]
fn the_nmi_output_lags_the_request_by_exactly_one_dot_in_every_region() {
    // A 6502 samples `/NMI` during φ2 and latches its data bus at the end of
    // it, so what it acts on is the level from a dot earlier — see
    // `Engine::nmi_active`. Every region, because the ratio differs and the
    // rule does not.
    for region in REGIONS {
        let g = region.geometry();
        let (ppu, _, _) = new_ppu_in(region);
        let log = with_nmi(&ppu);
        ppu.write_register(PPUCTRL, CTRL_NMI);
        seek(&ppu, g.vblank_scanline, 1);
        assert!(
            !log.levels.lock().iter().any(|high| *high),
            "{region}: nothing before the flag is even set"
        );
        // Run the dot that sets the flag: the output still shows the level from
        // before it.
        ppu.advance_by(1);
        assert!(
            !log.levels.lock().iter().any(|high| *high),
            "{region}: the output lags the request"
        );
        ppu.advance_by(1);
        assert!(
            log.levels.lock().iter().any(|high| *high),
            "{region}: and follows it one dot later"
        );
    }
}

#[test]
fn the_write_lockout_is_29658_cpu_cycles_in_every_region() {
    // The measurement is in CPU cycles; only the conversion to dots differs,
    // and PAL's is not a whole number of dots (94905.6), so it is floored.
    for region in REGIONS {
        let dots = region.geometry().warmup_dots;
        assert_eq!(
            dots,
            RESET_LOCKOUT_CPU_CYCLES * region.cpu_divider() / region.dot_divider(),
            "{region}"
        );
    }
    assert_eq!(Region::Ntsc.geometry().warmup_dots, 88_974);
    assert_eq!(Region::Pal.geometry().warmup_dots, 94_905);
    assert_eq!(Region::Dendy.geometry().warmup_dots, 88_974);
}

#[test]
fn pal_and_dendy_black_out_the_top_scanline_of_the_picture() {
    // The chart: the 2C07 border is "always black ($0E), intruding on left and
    // right 2 pixels and top 1 pixel of picture", which is exactly why its
    // picture is 239 scanlines out of 240 rendered.
    assert_eq!(Region::Ntsc.geometry().picture_height, 240);
    assert_eq!(Region::Pal.geometry().picture_height, 239);
    assert_eq!(Region::Dendy.geometry().picture_height, 239);

    for region in REGIONS {
        let (ppu, _, _) = new_ppu_in(region);
        // A backdrop nothing could mistake for black.
        ppu.poke_palette(0x3f00, 0x21);
        ppu.advance_by(u64::from(DOTS_PER_SCANLINE) * 2);
        let top = ppu.pixel(0, 0).unwrap().index();
        let next = ppu.pixel(0, 1).unwrap().index();
        assert_eq!(next, 0x21, "{region}: line 1 is picture in every region");
        if region.geometry().top_border_lines() == 0 {
            assert_eq!(top, 0x21, "{region}");
        } else {
            assert_eq!(top, BORDER_BLACK, "{region}");
        }
    }
}

#[test]
fn a_region_is_a_property_and_a_bad_one_names_the_alternatives() {
    for region in REGIONS {
        let ppu = NesPpu::new(&Props::new().with("region", region.name())).unwrap();
        assert_eq!(ppu.tv_region(), region);
        assert_eq!(ppu.geometry(), region.geometry());
    }
    // The default is the 2C02, so an existing machine file keeps working.
    assert_eq!(
        NesPpu::new(&Props::new()).unwrap().tv_region(),
        Region::Ntsc
    );

    let err = NesPpu::new(&Props::new().with("region", "secam"))
        .unwrap_err()
        .to_string();
    assert!(err.contains("ntsc"), "{err}");
    assert!(err.contains("pal"), "{err}");
    assert!(err.contains("dendy"), "{err}");

    assert_eq!(Region::from_name("dendy"), Some(Region::Dendy));
    assert_eq!(Region::from_name("secam"), None);
    for region in REGIONS {
        assert_eq!(Region::from_name(region.name()), Some(region));
        assert!(Region::NAMES.contains(&region.name()));
        assert!(!region.part_number().is_empty());
    }
}

#[test]
fn the_master_clock_is_an_exact_rational_in_every_region() {
    // Neither is an integer number of hertz (ROADMAP.md §4.2).
    assert_eq!(Region::Ntsc.master_clock(), (236_250_000, 11));
    assert_eq!(Region::Pal.master_clock(), (53_203_425, 2));
    assert_eq!(Region::Dendy.master_clock(), Region::Pal.master_clock());
    for region in REGIONS {
        let (num, den) = region.master_clock();
        assert_ne!(num % den, 0, "{region} is not a whole number of hertz");
        assert!(Rational::new(num, den).is_ok());
    }
}

#[test]
fn the_region_is_configuration_and_survives_a_snapshot_untouched() {
    // Derived from the machine, never from the stream: a snapshot must not be
    // able to turn a PAL machine into an NTSC one.
    let (pal, _, _) = new_ppu_in(Region::Pal);
    pal.advance_by(1234);
    let mut w = StateWriter::new(MachineShape::new());
    {
        let mut chunk = w
            .chunk("/ppu", NES_PPU_CLASS.name, NES_PPU_CLASS.version)
            .unwrap();
        pal.save(&mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();

    let (restored, _, _) = new_ppu_in(Region::Pal);
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load(
            "/ppu",
            NES_PPU_CLASS.name,
            NES_PPU_CLASS.version,
            &Migrations::new(),
        )
        .unwrap();
    restored.load(&mut chunk.reader()).unwrap();
    assert_eq!(restored.tv_region(), Region::Pal);
    assert_eq!(restored.dots(), 1234);
    // Loading a PAL snapshot into an NTSC PPU leaves it NTSC: the region is
    // the machine's, not the stream's.
    let (ntsc, _, _) = new_ppu_in(Region::Ntsc);
    ntsc.load(&mut chunk.reader()).unwrap();
    assert_eq!(ntsc.tv_region(), Region::Ntsc);
}

// ---------------------------------------------------------------------------
// The connection surface (`ROADMAP.md` §4.4)
// ---------------------------------------------------------------------------

#[test]
fn the_register_block_is_the_region_a_map_statement_names() {
    let (ppu, _, _) = new_ppu();
    let unnamed = Device::region(&ppu, "").expect("the whole aperture");
    let named = Device::region(&ppu, REGISTER_REGION).expect("`regs`");
    assert_eq!(unnamed.len(), REGISTER_WINDOW_LEN);
    // One piece of hardware, one region: a fresh Arc per call would be a second
    // identity for the same window.
    assert!(Arc::ptr_eq(&unnamed, &named));
    assert!(Device::region(&ppu, "vram").is_none());

    // And it really is the register block: $2002 through the mapped region.
    let space = AddressSpace::new("cpu", 16);
    space.topology().map(named, REGISTER_BASE).unwrap();
    ppu.with_engine(|e| e.status |= STATUS_VBLANK);
    let status = space
        .read(REGISTER_BASE + 2, Width::U8, MemAttrs::DEFAULT)
        .unwrap();
    assert_ne!(status as u8 & STATUS_VBLANK, 0);
    // Mirrored every 8 bytes across the whole 8 KiB.
    ppu.with_engine(|e| e.status |= STATUS_VBLANK);
    let mirrored = space
        .read(REGISTER_BASE + 0x1ffa, Width::U8, MemAttrs::DEFAULT)
        .unwrap();
    assert_ne!(mirrored as u8 & STATUS_VBLANK, 0);
}

#[test]
fn the_nmi_pin_connects_announces_and_refuses_anything_else() {
    let (ppu, _, _) = new_ppu();
    // Put it in vblank with NMIs enabled *before* connecting, so nothing has
    // driven the net yet when the sweep reaches it — which is the situation a
    // freshly restored machine is in.
    ppu.with_engine(|e| {
        e.status |= STATUS_VBLANK;
        e.ctrl |= CTRL_NMI;
        e.dots = 100;
    });

    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let sink = Arc::new(Recorder::default());
    let wire = Wire::builder()
        .source(id)
        .sink(Arc::clone(&sink) as Arc<dyn WireSink>, 0)
        .build_shared();
    ppu.connect(NMI_PIN, WireSource::new(wire, id)).unwrap();
    assert!(
        sink.levels.lock().is_empty(),
        "connecting is not an outward action; announcing is"
    );
    ppu.announce(NMI_PIN);
    // Nothing has ticked, so the sampled output is still the idle level and the
    // net has nothing new to deliver.
    assert!(sink.levels.lock().iter().all(|high| !*high));
    ppu.advance_by(1);
    assert_eq!(sink.levels.lock().last().copied(), Some(true));

    // An unknown pin is an error naming the port, and an unknown announce is
    // silently nothing — the sweep asks every device about every pin.
    let other = Wire::builder().source(id).build_shared();
    let err = ppu
        .connect("irq", WireSource::new(other, id))
        .unwrap_err()
        .to_string();
    assert!(err.contains("irq"), "{err}");
    ppu.announce("irq");
}

#[test]
fn the_class_constructs_through_the_registry_with_a_region() {
    let mut registry = crate::core::Registry::new();
    register(&mut registry).unwrap();
    let device = registry
        .create("nes.ppu", &Props::new().with("region", "dendy"))
        .unwrap();
    assert_eq!(device.class().name, "nes.ppu");
    // The class advertises the property the machine file writes.
    assert!(
        NES_PPU_CLASS.properties.iter().any(|p| p.name == "region"),
        "`rsemu describe nes.ppu` must list it"
    );
    assert!(Device::region(device.as_ref(), "").is_some());
}

// ---------------------------------------------------------------------------
// Lazily advanced (`ROADMAP.md` §4.2)
// ---------------------------------------------------------------------------

#[test]
fn the_next_event_is_always_far_enough_ahead_to_be_reachable() {
    // Catch-up that returns the tick a device already stands on makes no
    // progress and stalls where it is, so every candidate must be strictly
    // ahead — and none may be dropped for being close, because the one that
    // matters is two dots away.
    for region in [Region::Ntsc, Region::Pal, Region::Dendy] {
        let (ppu, _, _) = new_ppu_in(region);
        let lead = 1;
        // One scanline is the ceiling: the line boundary is always a candidate.
        let ceiling = u64::from(DOTS_PER_SCANLINE);
        // A frame and a bit, one dot at a time.
        for _ in 0..(ppu.geometry().dots_per_frame + 500) {
            let next = ppu.next_event_dot();
            let ahead = next - ppu.dots();
            assert!(
                ahead >= lead,
                "{region}: next event {next} is only {ahead} dots past {}",
                ppu.dots()
            );
            assert!(
                ahead <= ceiling,
                "{region}: next event {next} is {ahead} dots away; a mid-quantum \
                 $2002 read would be that stale"
            );
            ppu.advance_by(1);
        }
    }
}

#[test]
fn stopping_at_every_next_event_still_reaches_the_nmi_on_its_own_dot() {
    // What a run loop does: advance only as far as the chip's own next event,
    // and never past it. The vblank request must still be raised on the dot it
    // is raised on when the same span is run in one go.
    let (ppu, _, _) = new_ppu();
    let nmi = with_nmi(&ppu);
    ppu.write_register(regs::PPUCTRL, CTRL_NMI);
    let geom = ppu.geometry();
    let target = geom.dots_per_frame;
    let mut first_high = None;
    while ppu.dots() < target {
        let next = ppu.next_event_dot().min(target);
        ppu.advance_to(next);
        if first_high.is_none() && nmi.levels.lock().iter().any(|l| *l) {
            first_high = Some(ppu.dots());
        }
    }
    let at = first_high.expect("the NMI never asserted in a frame");
    // The flag is set by the dot at (vblank_scanline, 1) and the output follows
    // one dot later.
    let flag_dot = u64::from(geom.vblank_scanline) * u64::from(DOTS_PER_SCANLINE) + 1;
    assert!(
        (flag_dot + 1..=flag_dot + 8).contains(&at),
        "the request reached the wire at dot {at}, not just after {flag_dot}"
    );
}

#[test]
fn the_lock_free_position_tracks_the_engine() {
    // `Device::current_tick` and `Device::next_event_tick` are asked with the
    // scheduler's slot held at `LockRank::LEAF`, which nothing nests under, so
    // they read atomics rather than the engine. Those atomics are derived state
    // and every path that moves the engine has to republish them.
    let (ppu, _, _) = new_ppu();
    let device: &dyn Device = &ppu;
    assert!(device.is_lazy());
    assert_eq!(device.current_tick(), 0);

    ppu.advance_by(1000);
    assert_eq!(device.current_tick(), ppu.dots());
    assert_eq!(device.next_event_tick(), Some(ppu.next_event_dot()));

    // A register write moves neither, but it can change what the next event
    // *is* — `$2000` decides whether vblank will raise the request at all.
    ppu.write_register(regs::PPUCTRL, CTRL_NMI);
    assert_eq!(device.current_tick(), ppu.dots());

    // And a reset puts both back where a cold machine's are.
    Device::reset(&ppu, ResetKind::Cold);
    assert_eq!(device.current_tick(), 0);
    assert_eq!(device.current_tick(), ppu.dots());
}

#[test]
fn a_snapshot_restores_the_lock_free_position_too() {
    let (ppu, _, _) = new_ppu();
    ppu.advance_by(12_345);
    let mut w = StateWriter::new(MachineShape::new());
    {
        let mut chunk = w
            .chunk("ppu", NES_PPU_CLASS.name, NES_PPU_CLASS.version)
            .expect("a chunk");
        Device::save(&ppu, &mut chunk).expect("saves");
    }
    let bytes = w.to_vec().expect("serializes");

    let (restored, _, _) = new_ppu();
    let reader = StateReader::new(&bytes).expect("well formed");
    let chunk = reader
        .load(
            "ppu",
            NES_PPU_CLASS.name,
            NES_PPU_CLASS.version,
            &Migrations::new(),
        )
        .expect("the chunk is there");
    Device::load(&restored, &mut chunk.reader()).expect("loads");

    let device: &dyn Device = &restored;
    assert_eq!(device.current_tick(), 12_345);
    assert_eq!(device.next_event_tick(), Some(restored.next_event_dot()));
}
