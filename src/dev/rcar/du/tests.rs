//! Tests for the R-Car Display Unit.

use super::*;

use crate::core::clock::GlobalTime;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::sync::AtomicU32;
use crate::core::wire::{Wire, WireId, WireIdAllocator, WireSink};
use crate::machine::Machine;

/// Where the traced head unit maps the DU.
const DU_BASE: u64 = 0xfff8_0000;
/// Where its driver puts the framebuffer.
const FB: u64 = 0x9e00_0000;

/// One step of the traced driver's programming sequence.
#[derive(Debug, Clone, Copy)]
enum Op {
    Write(u32, u32),
    Read(u32),
}

/// The register sequence Renesas' `rcarfb` driver was observed making, in
/// order, less the final `DSYSR` write that turns the display on.
const RCARFB: &[Op] = &[
    Op::Write(0x020, 0x7773_0001),
    Op::Write(0x034, 0x7775_0001),
    Op::Write(0x038, 0x7776_0001),
    // The first timing set.
    Op::Write(0x040, 0xe3),
    Op::Write(0x044, 0x40d),
    Op::Write(0x048, 0x2c),
    Op::Write(0x04c, 0x20d),
    Op::Write(0x054, 0x75),
    Op::Write(0x050, 0x43d),
    Op::Write(0x05c, 0x221),
    Op::Write(0x058, 0x230),
    // The final one.
    Op::Write(0x040, 0xc5),
    Op::Write(0x044, 0x3e5),
    Op::Write(0x048, 0x1f),
    Op::Write(0x04c, 0x1ff),
    Op::Write(0x054, 0x7f),
    Op::Write(0x050, 0x497),
    Op::Write(0x05c, 0x20a),
    Op::Write(0x058, 0x20c),
    Op::Write(0x004, 0x0100_0000),
    Op::Write(0x090, 0),
    Op::Write(0x098, 0),
    Op::Read(0x018),
    Op::Read(0x0c8),
    Op::Write(0x03c, 0x7777_0000),
    Op::Write(0x184, 0x7775_0000),
    Op::Write(0x018, 0x8000_0000),
    Op::Write(0x0c8, 0x7776_00a8),
    // Plane 1.
    Op::Write(0x100, 0x4001),
    Op::Write(0x104, 0x320),
    Op::Write(0x108, 0xff),
    Op::Write(0x110, 0x320),
    Op::Write(0x114, 0x1e0),
    Op::Write(0x118, 0x10),
    Op::Write(0x11c, 0x9),
    Op::Write(0x120, 0x9e00_0000),
    Op::Write(0x124, 0x9e3c_0000),
    Op::Write(0x130, 0),
    Op::Write(0x134, 0),
    Op::Write(0x138, 0),
    Op::Write(0x13c, 0xfff),
    Op::Write(0x140, 0),
    Op::Write(0x150, 0),
    Op::Read(0x000),
];

fn port(du: &Du) -> Port {
    Port {
        shared: Arc::clone(&du.shared),
    }
}

fn read(du: &Du, offset: u64, len: usize) -> u32 {
    let mut buf = [0u8; 4];
    port(du)
        .read(offset, &mut buf[..len], MemAttrs::DEFAULT)
        .expect("a register read is legal");
    u32::from_le_bytes(buf)
}

fn write(du: &Du, offset: u64, len: usize, value: u32) {
    port(du)
        .write(offset, &value.to_le_bytes()[..len], MemAttrs::DEFAULT)
        .expect("a register write is legal");
}

fn unit() -> Du {
    Du::new(&Props::new()).expect("defaults are valid")
}

/// The test pattern's pixel at `(x, y)`, as RGB565.
fn pattern(x: u32, y: u32) -> u16 {
    let r = (x & 0x1f) as u16;
    let g = (y & 0x3f) as u16;
    let b = ((x >> 5) ^ (y >> 3)) as u16 & 0x1f;
    (r << 11) | (g << 5) | b
}

fn expand(v: u16) -> [u8; 3] {
    PlaneFormat::RGB565.decode(&v.to_le_bytes()).0
}

// ---------------------------------------------------------------------------
// A board: RAM and the DU in one space, as the head unit has them
// ---------------------------------------------------------------------------

/// A machine with 4 MiB of RAM at the framebuffer and the DU at its traced
/// address, on a 37.044 MHz dot clock — which makes the traced 1176 × 525
/// frame exactly 1/60 s. Returns the machine and the DU itself.
fn board(extra: &str) -> (Machine, Arc<Du>) {
    let mut options = crate::machine::BuildOptions::new();
    options.classes.insert(schema());
    for s in crate::machine::builtin::schemas() {
        options.classes.insert(s);
    }
    crate::machine::builtin::bind(&mut options.bindings).expect("ram");
    let seen: Arc<Mutex<Option<Arc<Du>>>> = Arc::new(Mutex::new(None));
    let keep = Arc::clone(&seen);
    options.bindings.replace(CLASS_NAME, move |props| {
        let du = Arc::new(Du::new(props)?);
        *keep.lock() = Some(Arc::clone(&du));
        Ok(du)
    });
    let mut registry = crate::core::Registry::new();
    register(&mut registry).expect("nothing else claims rcar.du");
    crate::machine::builtin::register(&mut registry).expect("ram");

    let text = alloc::format!(
        concat!(
            "machine \"navi\" {{\n",
            "  osc dclk = 37044000 Hz\n",
            "  space mem {{ width = 32 }}\n",
            "  object sdram \"ram\" {{ size = 4M }}\n",
            "  object du \"rcar.du\" {{\n",
            "    clock = dclk\n",
            "    space = mem\n",
            "{}",
            "  }}\n",
            "  map mem 0x9e000000 size 4M = sdram\n",
            "  map mem 0xfff80000 size 0x40000 = du\n",
            "}}\n"
        ),
        extra
    );
    let machine = crate::machine::build("navi.machine", &text, &registry, &options)
        .expect("the board builds");
    let du = seen.lock().take().expect("the DU was constructed");
    (machine, du)
}

fn space(machine: &Machine) -> Arc<AddressSpace> {
    Arc::clone(machine.space("mem").expect("mem"))
}

fn paint(space: &AddressSpace) {
    let mut fb = Vec::with_capacity(800 * 480 * 2);
    for y in 0..480 {
        for x in 0..800 {
            fb.extend_from_slice(&pattern(x, y).to_le_bytes());
        }
    }
    space
        .write_bytes(FB, &fb, MemAttrs::DEFAULT)
        .expect("the framebuffer is RAM");
}

fn replay(space: &AddressSpace, ops: &[Op]) {
    for op in ops {
        match *op {
            Op::Write(offset, value) => space
                .write(
                    DU_BASE + u64::from(offset),
                    Width::U32,
                    u64::from(value),
                    MemAttrs::DEFAULT,
                )
                .expect("a write reaches the DU"),
            Op::Read(offset) => {
                space
                    .read(DU_BASE + u64::from(offset), Width::U32, MemAttrs::DEFAULT)
                    .expect("a read reaches the DU");
            }
        }
    }
}

fn enable(space: &AddressSpace) {
    replay(space, &[Op::Write(0x000, 0x100)]);
}

#[test]
fn the_traced_driver_puts_its_framebuffer_on_the_surface() {
    let (_machine, du) = board("");
    let mem = space(&_machine);
    paint(&mem);
    replay(&mem, RCARFB);
    assert_eq!(du.reg(DSYSR), DSYSR_DRES, "read back before the enable");
    enable(&mem);
    assert!(du.displaying());

    let (w, h, pixels) = du.read_frame();
    assert_eq!((w, h), (800, 480), "HDER - HDSR by VDER - VDSR");
    assert_eq!(du.frame_ticks(), 1176 * 525);
    let at = |x: u32, y: u32| pixels[(y * w + x) as usize];
    // PnDPXR/PnDPYR = (16, 9) place the plane that far into the window, and
    // the background (BPOR = 0) shows where it is not.
    assert_eq!(at(0, 0), [0, 0, 0]);
    assert_eq!(at(15, 100), [0, 0, 0]);
    assert_eq!(at(100, 8), [0, 0, 0]);
    for (x, y) in [(0u32, 0u32), (1, 0), (31, 7), (100, 200), (783, 470)] {
        assert_eq!(at(x + 16, y + 9), expand(pattern(x, y)), "fb ({x}, {y})");
    }
}

#[test]
fn a_plane_origin_property_moves_the_window_origin() {
    let (machine, du) = board("    plane-origin-x = 16\n    plane-origin-y = 9\n");
    let mem = space(&machine);
    paint(&mem);
    replay(&mem, RCARFB);
    enable(&mem);
    let (w, h, pixels) = du.read_frame();
    assert_eq!((w, h), (800, 480));
    for (x, y) in [(0u32, 0u32), (799, 0), (0, 479), (799, 479), (400, 240)] {
        assert_eq!(pixels[(y * w + x) as usize], expand(pattern(x, y)));
    }
}

#[test]
fn the_display_is_black_until_den_and_while_dres() {
    let (machine, du) = board("");
    let mem = space(&machine);
    paint(&mem);
    replay(&mem, RCARFB);
    replay(&mem, &[Op::Write(BPOR, 0x00ff_ffff)]);
    let (_, _, pixels) = du.read_frame();
    assert!(pixels.iter().all(|p| *p == [0, 0, 0]), "DEN clear");
    replay(&mem, &[Op::Write(DSYSR, DSYSR_DEN | DSYSR_DRES)]);
    let (_, _, pixels) = du.read_frame();
    assert!(pixels.iter().all(|p| *p == [0, 0, 0]), "DRES set");
    enable(&mem);
    let (w, _, pixels) = du.read_frame();
    assert_eq!(pixels[0], [0xff, 0xff, 0xff], "the background is BPOR");
    assert_eq!(pixels[(9 * w + 16) as usize], expand(pattern(0, 0)));
}

#[test]
fn frames_count_on_the_dot_clock_while_displaying() {
    let (mut machine, du) = board("");
    let mem = space(&machine);
    replay(&mem, RCARFB);
    machine
        .run_for(GlobalTime::from_nanos(50_000_000))
        .expect("runs");
    assert_eq!(du.frames(), 0, "nothing counts while the display is off");
    enable(&mem);
    // 100 ms at exactly 60 Hz; a guest read catches the unit up.
    machine
        .run_for(GlobalTime::from_nanos(100_000_000))
        .expect("runs");
    let dssr = mem
        .read(DU_BASE + u64::from(DSSR), Width::U32, MemAttrs::DEFAULT)
        .expect("DSSR") as u32;
    assert!((5..=6).contains(&du.frames()), "{} frames", du.frames());
    assert_eq!(dssr & DSSR_MODELLED, DSSR_MODELLED, "FRM and VBK latched");
}

#[test]
fn a_debug_read_of_the_window_has_no_side_effects() {
    let (_machine, du) = board("");
    let mem = space(&_machine);
    replay(&mem, RCARFB);
    let before = du.shared.regs.lock().clone();
    for offset in [
        DSYSR,
        DSSR,
        DSRCR,
        DPPR,
        HCR,
        plane_reg(1, PNDSA0R),
        0x3fffc,
    ] {
        mem.read(DU_BASE + u64::from(offset), Width::U32, MemAttrs::DEBUG)
            .expect("a debug read is answered");
    }
    assert_eq!(*du.shared.regs.lock(), before);
    assert!(
        mem.write(
            DU_BASE + u64::from(DSRCR),
            Width::U32,
            0xffff_ffff,
            MemAttrs::DEBUG
        )
        .is_err(),
        "a debugger cannot clear status"
    );
}

// ---------------------------------------------------------------------------
// The register file, directly
// ---------------------------------------------------------------------------

#[test]
fn every_register_reads_back_what_was_written_in_any_width() {
    let du = unit();
    assert_eq!(read(&du, u64::from(DSYSR), 4), DSYSR_DRES, "reset value");
    write(&du, 0x0c8, 4, 0x7776_00a8);
    assert_eq!(read(&du, 0x0c8, 4), 0x7776_00a8);
    write(&du, 0x3fffc, 4, 0x1234_5678);
    assert_eq!(read(&du, 0x3fffc, 4), 0x1234_5678);
    assert_eq!(read(&du, 0x3fffe, 2), 0x1234);
    assert_eq!(read(&du, 0x3fffd, 1), 0x56);
    write(&du, 0x3ffff, 1, 0xab);
    assert_eq!(read(&du, 0x3fffc, 4), 0xab34_5678);
    write(&du, u64::from(plane_reg(8, PNDSA0R)), 2, 0xbeef);
    assert_eq!(du.reg(0x820), 0xbeef, "plane 8 lives at 0x800");
    write(&du, u64::from(DSSR), 4, 0xffff_ffff);
    assert_eq!(read(&du, u64::from(DSSR), 4), 0, "DSSR is read-only");
    let mut b = [0u8; 4];
    assert!(port(&du).read(0x4_0000, &mut b, MemAttrs::DEFAULT).is_err());
}

fn with_irq(du: &Du) -> Arc<Probe> {
    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let probe = Arc::new(Probe::default());
    let wire = Wire::builder()
        .source(id)
        .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
        .build_shared();
    du.connect(IRQ_PIN, WireSource::new(wire, id)).unwrap();
    probe
}

#[derive(Debug, Default)]
struct Probe {
    level: AtomicU32,
}

impl WireSink for Probe {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        self.level
            .store(u32::from(level.is_high()), Ordering::Relaxed);
    }
}

/// A unit programmed for a 10 × 4-dot frame, displaying from tick 0.
fn tiny() -> Du {
    let du = unit();
    write(&du, u64::from(HCR), 4, 9);
    write(&du, u64::from(VCR), 4, 3);
    write(&du, u64::from(DSYSR), 4, DSYSR_DEN);
    du
}

#[test]
fn vblank_sets_dssr_raises_irq_when_enabled_and_dsrcr_clears_it() {
    let du = tiny();
    let probe = with_irq(&du);
    assert_eq!(du.next_event_tick(), None, "no interrupt enabled");
    write(&du, u64::from(DIER), 4, DSSR_VBK);
    assert_eq!(du.next_event_tick(), Some(40));
    du.advance_to(39);
    assert_eq!(read(&du, u64::from(DSSR), 4), 0);
    assert_eq!(probe.level.load(Ordering::Relaxed), 0);
    du.advance_to(40);
    assert_eq!(read(&du, u64::from(DSSR), 4), DSSR_MODELLED);
    assert_eq!(probe.level.load(Ordering::Relaxed), 1);
    assert_eq!(du.next_event_tick(), None, "already raised");
    // Clearing FRM alone leaves VBK, and the line, up.
    write(&du, u64::from(DSRCR), 4, DSSR_FRM);
    assert_eq!(probe.level.load(Ordering::Relaxed), 1);
    write(&du, u64::from(DSRCR), 4, DSSR_VBK);
    assert_eq!(read(&du, u64::from(DSSR), 4), 0);
    assert_eq!(probe.level.load(Ordering::Relaxed), 0);
    assert_eq!(du.next_event_tick(), Some(80));
    assert_eq!(read(&du, u64::from(DSRCR), 4), 0, "DSRCR reads zero");
    du.advance_to(1000);
    assert_eq!(du.frames(), 25);
}

#[test]
fn changing_the_totals_restarts_the_frame() {
    let du = tiny();
    du.advance_to(55);
    assert_eq!(du.frames(), 1);
    write(&du, u64::from(HCR), 4, 19); // 20 × 4 = 80 dots, from tick 55
    write(&du, u64::from(DIER), 4, DSSR_FRM);
    write(&du, u64::from(DSRCR), 4, DSSR_MODELLED);
    assert_eq!(du.next_event_tick(), Some(135));
    // Rewriting the same value does not.
    write(&du, u64::from(HCR), 4, 19);
    assert_eq!(du.next_event_tick(), Some(135));
}

#[test]
fn formats_decode_as_the_module_docs_say() {
    assert_eq!(PlaneFormat::from_regs(0x4001, 0), PlaneFormat::RGB565);
    assert_eq!(PlaneFormat::from_regs(0x1002, 0), PlaneFormat::ARGB1555);
    assert_eq!(
        PlaneFormat::from_regs(0x1001, 0x7766_0001),
        PlaneFormat::ARGB8888
    );
    assert_eq!(
        PlaneFormat::from_regs(0x4001, 0x7766_0002),
        PlaneFormat::XRGB8888
    );
    assert_eq!(PlaneFormat::from_regs(0, 0), PlaneFormat::INDEX8);
    assert_eq!(PlaneFormat::from_regs(3, 0), PlaneFormat::RGB565, "YCbCr");

    assert_eq!(PlaneFormat::RGB565.decode(&[0xff, 0xff]), ([0xff; 3], 0xff));
    assert_eq!(
        PlaneFormat::RGB565.decode(&[0x00, 0xf8]),
        ([0xff, 0, 0], 0xff)
    );
    assert_eq!(
        PlaneFormat::RGB565.decode(&[0xe0, 0x07]),
        ([0, 0xff, 0], 0xff)
    );
    assert_eq!(
        PlaneFormat::ARGB1555.decode(&[0x1f, 0x80]),
        ([0, 0, 0xff], 0xff)
    );
    assert_eq!(
        PlaneFormat::ARGB1555.decode(&[0x00, 0x7c]),
        ([0xff, 0, 0], 0)
    );
    assert_eq!(
        PlaneFormat::ARGB8888.decode(&[0x33, 0x22, 0x11, 0x80]),
        ([0x11, 0x22, 0x33], 0x80)
    );
    assert_eq!(
        PlaneFormat::XRGB8888.decode(&[0x33, 0x22, 0x11, 0x00]),
        ([0x11, 0x22, 0x33], 0xff)
    );
}

#[test]
fn planes_stack_by_priority_and_blend_with_alpha() {
    let (machine, du) = board("");
    let mem = space(&machine);
    // A 4 × 2 window, no blanking to speak of.
    for (reg, value) in [
        (HDSR, 0),
        (HDER, 4),
        (VDSR, 0),
        (VDER, 2),
        (BPOR, 0x0000_00ff),
    ] {
        replay(&mem, &[Op::Write(reg, value)]);
    }
    // Plane 1: XRGB8888 red, 2 × 1 at (0, 0).
    mem.write_bytes(FB, &[0, 0, 0xff, 0, 0, 0, 0xff, 0], MemAttrs::DEFAULT)
        .unwrap();
    // Plane 2: RGB565 white, 2 × 2 at (1, 0), blended at alpha 0x80.
    mem.write_bytes(FB + 0x100, &[0xff; 8], MemAttrs::DEFAULT)
        .unwrap();
    let p1 = |r| plane_reg(1, r);
    let p2 = |r| plane_reg(2, r);
    replay(
        &mem,
        &[
            Op::Write(p1(PNMR), 0x4001),
            Op::Write(p1(PNDDCR4), 0x7766_0002),
            Op::Write(p1(PNDSXR), 2),
            Op::Write(p1(PNDSYR), 1),
            Op::Write(p1(PNDSA0R), FB as u32),
            Op::Write(p2(PNMR), 0x1001),
            Op::Write(p2(PNALPHAR), 0x80),
            Op::Write(p2(PNDSXR), 2),
            Op::Write(p2(PNDSYR), 2),
            Op::Write(p2(PNDPXR), 1),
            Op::Write(p2(PNDSA0R), (FB + 0x100) as u32),
            // Slot 1 (top) is plane 2, slot 2 is plane 1.
            Op::Write(DPPR, 0x8 | 0x1 | (0x8 << 4)),
            Op::Write(DSYSR, DSYSR_DEN),
        ],
    );
    let (w, h, px) = du.read_frame();
    assert_eq!((w, h), (4, 2));
    let half = |under: [u8; 3]| blend(under, [0xff; 3], 0x80);
    assert_eq!(px[0], [0xff, 0, 0], "plane 1 alone");
    assert_eq!(px[1], half([0xff, 0, 0]), "plane 2 over plane 1");
    assert_eq!(px[2], half([0, 0, 0xff]), "plane 2 over the background");
    assert_eq!(px[3], [0, 0, 0xff], "the background alone");
    assert_eq!(px[4], [0, 0, 0xff]);
    assert_eq!(px[5], half([0, 0, 0xff]));
}

#[test]
fn with_dorcr_the_order_comes_from_ds1pr_and_spim_5_blends() {
    // The navi's HMI: DORCR bit 0, DS1PR's nibble 0 on top, PnMR 0x5001.
    let (machine, du) = board("");
    let mem = space(&machine);
    for (reg, value) in [(HDSR, 0), (HDER, 2), (VDSR, 0), (VDER, 1), (BPOR, 0)] {
        replay(&mem, &[Op::Write(reg, value)]);
    }
    mem.write_bytes(FB, &[0x00, 0xf8, 0x00, 0xf8], MemAttrs::DEFAULT) // red 565
        .unwrap();
    mem.write_bytes(FB + 0x100, &[0x1f, 0x00, 0x1f, 0x00], MemAttrs::DEFAULT) // blue
        .unwrap();
    let p3 = |r| plane_reg(3, r);
    let p4 = |r| plane_reg(4, r);
    replay(
        &mem,
        &[
            Op::Write(p3(PNMR), 0x4001),
            Op::Write(p3(PNDSXR), 2),
            Op::Write(p3(PNDSYR), 1),
            Op::Write(p3(PNDSA0R), FB as u32),
            Op::Write(p4(PNMR), 0x5001),
            Op::Write(p4(PNALPHAR), 0x80),
            Op::Write(p4(PNDSXR), 1),
            Op::Write(p4(PNDSYR), 1),
            Op::Write(p4(PNDSA0R), (FB + 0x100) as u32),
            // A stale DPPR naming plane 1, which the routing overrides.
            Op::Write(DPPR, 0x8000_0000),
            Op::Write(DORCR, 1),
            Op::Write(DS1PR, 0x34), // plane 4 on top of plane 3
            Op::Write(DSYSR, DSYSR_DEN),
        ],
    );
    let (_, _, px) = du.read_frame();
    assert_eq!(
        px[0],
        blend([0xff, 0, 0], [0, 0, 0xff], 0x80),
        "plane 4 blended over 3"
    );
    assert_eq!(px[1], [0xff, 0, 0], "plane 3 alone");
}

// ---------------------------------------------------------------------------
// Snapshots and properties
// ---------------------------------------------------------------------------

fn snapshot(du: &Du) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("du", CLASS.name).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("du", CLASS.name, CLASS.version).unwrap();
        du.save(&mut chunk).unwrap();
    }
    w.to_vec().unwrap()
}

#[test]
fn a_snapshot_round_trips_to_identical_state() {
    let saved = tiny();
    write(&saved, u64::from(DIER), 4, DSSR_VBK);
    write(&saved, u64::from(plane_reg(1, PNDSA0R)), 4, 0x9e00_0000);
    write(&saved, 0x0c8, 4, 0x7776_00a8);
    saved.advance_to(137);
    let bytes = snapshot(&saved);

    let restored = unit();
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("du", CLASS.name, CLASS.version, &Migrations::new())
        .unwrap();
    restored.load(&mut chunk.reader()).unwrap();
    // One register lock at a time: two devices' locks share a rank.
    let want = saved.shared.regs.lock().clone();
    assert_eq!(*restored.shared.regs.lock(), want);
    assert_eq!(snapshot(&restored), bytes, "identical state bytes");
    assert_eq!(restored.next_event_tick(), saved.next_event_tick());
    assert_eq!(restored.irq_asserted(), saved.irq_asserted());
    saved.advance_to(10_000);
    restored.advance_to(10_000);
    assert_eq!(restored.frames(), saved.frames());
    // One register lock at a time: two devices' locks share a rank.
    let want = saved.shared.regs.lock().clone();
    assert_eq!(*restored.shared.regs.lock(), want);
}

#[test]
fn reset_returns_to_dres_and_keeps_the_tick() {
    let du = tiny();
    du.advance_to(100);
    du.reset(ResetKind::Cold);
    assert_eq!(du.reg(DSYSR), DSYSR_DRES);
    assert_eq!(du.reg(HCR), 0);
    assert_eq!(du.frames(), 0);
    assert_eq!(du.current_tick(), 100);
}

#[test]
fn properties_are_checked() {
    let du = unit();
    assert_eq!(du.geometry(), (640, 480), "the fallback before timing");
    assert!(Du::new(&Props::new().with("width", 0u64)).is_err());
    assert!(Du::new(&Props::new().with("height", 5000u64)).is_err());
    assert!(Du::new(&Props::new().with("colour", 1u64)).is_err());
    let du = Du::new(&Props::new().with("width", 800u64).with("height", 480u64)).unwrap();
    assert_eq!(du.geometry(), (800, 480));
}
