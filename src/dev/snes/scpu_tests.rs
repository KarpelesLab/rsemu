//! Tests for `snes.scpu`. The beam-driven half (HDMA, the H/V IRQ, NMI and
//! the automatic joypad read) needs a PPU and is proved on the board, in
//! `tests/snes_board.rs`; what is here needs only the register file, the
//! gate and a bus.

use super::*;
use crate::core::space::RamStore;
use crate::core::wire::{Wire, WireIdAllocator};
use crate::dev::snes::input::buttons;

const REGS: u64 = 0x4200;
const DMA: u64 = 0x4300;
const JOY: u64 = 0x4016;

/// An S-CPU with its windows mapped where a board maps them, on an A-bus with
/// RAM in bank 0's low 8 KiB and 256 bytes of RAM standing in for the B-bus
/// at $2100 — which records what DMA wrote.
struct Fixture {
    c: SCpu,
    space: Arc<AddressSpace>,
    low: Arc<RamStore>,
    b: Arc<RamStore>,
}

fn scpu() -> Fixture {
    let c = SCpu::new(&Props::new()).expect("no properties are required to build one");
    let low = Arc::new(RamStore::new(0x2000));
    let b = Arc::new(RamStore::new(0x100));
    let space = Arc::new(AddressSpace::new("abus", 24));
    {
        let mut t = space.topology();
        t.map(MmioRegion::ram("low", Arc::clone(&low)), 0).unwrap();
        t.map(MmioRegion::ram("bbus", Arc::clone(&b)), 0x2100)
            .unwrap();
        t.map(Arc::clone(&c.regs), REGS).unwrap();
        t.map(Arc::clone(&c.dma), DMA).unwrap();
        t.map(Arc::clone(&c.joy), JOY).unwrap();
    }
    *c.shared.space.lock() = Some(Arc::downgrade(&space));
    Fixture { c, space, low, b }
}

fn wr(f: &Fixture, addr: u64, v: u8) {
    f.space
        .write(addr, Width::U8, u64::from(v), MemAttrs::DEFAULT)
        .unwrap();
}

fn rd(f: &Fixture, addr: u64) -> u8 {
    f.space.read(addr, Width::U8, MemAttrs::DEFAULT).unwrap() as u8
}

/// Run the gate until it lets go; the total held clocks it charged.
fn run_dma(f: &Fixture) -> u64 {
    let c = &f.c;
    let mut clocks = 0;
    let held = BusCycle {
        addr: 0,
        pins: Pins(Pins::HELD),
        data: None,
    };
    for _ in 0..100_000 {
        match c.port.arbitrate(0, 0, 0, true) {
            Arbitration::Release => return clocks,
            _ => clocks += c.port.clocks(held),
        }
    }
    panic!("the gate never released");
}

#[test]
fn multiply_and_divide() {
    let f = scpu();
    wr(&f, REGS + 0x02, 200);
    wr(&f, REGS + 0x03, 100);
    assert_eq!(
        u16::from(rd(&f, REGS + 0x16)) | (u16::from(rd(&f, REGS + 0x17)) << 8),
        20_000
    );
    wr(&f, REGS + 0x04, 0x39);
    wr(&f, REGS + 0x05, 0x30); // 12345
    wr(&f, REGS + 0x06, 100);
    let q = u16::from(rd(&f, REGS + 0x14)) | (u16::from(rd(&f, REGS + 0x15)) << 8);
    let r = u16::from(rd(&f, REGS + 0x16)) | (u16::from(rd(&f, REGS + 0x17)) << 8);
    assert_eq!((q, r), (123, 45));
    // By zero: quotient $FFFF, remainder the dividend (fullsnes).
    wr(&f, REGS + 0x06, 0);
    assert_eq!(rd(&f, REGS + 0x14), 0xFF);
    assert_eq!(rd(&f, REGS + 0x15), 0xFF);
    assert_eq!(rd(&f, REGS + 0x16), 0x39);
}

#[test]
fn bus_speed_follows_the_map_and_memsel() {
    assert_eq!(speed(0x00_0000, false), 8, "LowRAM");
    assert_eq!(speed(0x00_2100, false), 6, "the B-bus");
    assert_eq!(speed(0x00_4016, false), 12, "the old joypad ports");
    assert_eq!(speed(0x00_4200, false), 6, "S-CPU registers");
    assert_eq!(speed(0x00_8000, true), 8, "$00-$3F ROM is never fast");
    assert_eq!(speed(0x80_8000, false), 8);
    assert_eq!(speed(0x80_8000, true), 6, "FastROM");
    assert_eq!(speed(0x7E_0000, true), 8, "WRAM");
    assert_eq!(speed(0xC0_0000, true), 6);
    let f = scpu();
    let c = &f.c;
    let read = |addr| BusCycle {
        addr,
        pins: Pins(Pins::VDA),
        data: None,
    };
    assert_eq!(c.port.clocks(read(0x80_8000)), 8);
    wr(&f, REGS + 0x0D, 1);
    assert_eq!(c.port.clocks(read(0x80_8000)), 6, "MEMSEL");
    let internal = BusCycle {
        addr: 0,
        pins: Pins(0),
        data: None,
    };
    assert_eq!(c.port.clocks(internal), 6);
}

#[test]
fn a_gp_dma_copies_ram_to_the_b_bus_and_costs_8_clocks_a_byte() {
    let f = scpu();
    let (low, b) = (&f.low, &f.b);
    for i in 0..4u8 {
        low.write_u8(0x100 + u64::from(i), 0xA0 + i).unwrap();
    }
    // Channel 1, mode 1 (two registers), A to B, increment, 4 bytes from $000100
    // to $2118/$2119.
    wr(&f, DMA + 0x10, 0x01);
    wr(&f, DMA + 0x11, 0x18);
    wr(&f, DMA + 0x12, 0x00);
    wr(&f, DMA + 0x13, 0x01);
    wr(&f, DMA + 0x14, 0x00);
    wr(&f, DMA + 0x15, 4);
    wr(&f, DMA + 0x16, 0);
    wr(&f, REGS + 0x0B, 0x02);
    let clocks = run_dma(&f);
    assert_eq!(clocks, DMA_START + DMA_CHANNEL + 4 * DMA_BYTE);
    // $18 got A0, A2; $19 got A1, A3 — the last of each stands.
    assert_eq!(b.read_u8(0x18).unwrap(), 0xA2);
    assert_eq!(b.read_u8(0x19).unwrap(), 0xA3);
    // The channel's registers are left as the hardware leaves them.
    assert_eq!(rd(&f, DMA + 0x15), 0);
    assert_eq!(rd(&f, DMA + 0x12), 0x04);
    assert_eq!(rd(&f, REGS + 0x0B), 0, "MDMAEN is write-only: open bus");
}

#[test]
fn a_dma_from_the_b_bus_and_a_fixed_address() {
    let f = scpu();
    let (low, b) = (&f.low, &f.b);
    b.write_u8(0x39, 0x5A).unwrap();
    // Channel 0, B to A, fixed A address, 3 bytes into $000200.
    wr(&f, DMA, 0x80 | 0x08);
    wr(&f, DMA + 0x01, 0x39);
    wr(&f, DMA + 0x02, 0x00);
    wr(&f, DMA + 0x03, 0x02);
    wr(&f, DMA + 0x04, 0x00);
    wr(&f, DMA + 0x05, 3);
    wr(&f, DMA + 0x06, 0);
    wr(&f, REGS + 0x0B, 0x01);
    run_dma(&f);
    assert_eq!(low.read_u8(0x200).unwrap(), 0x5A);
    assert_eq!(
        low.read_u8(0x201).unwrap(),
        0,
        "fixed: nothing past the first"
    );
}

#[test]
fn rdnmi_and_timeup_clear_on_read_and_drive_the_lines() {
    #[derive(Debug, Default)]
    struct Probe(AtomicBool);
    impl WireSink for Probe {
        fn set_level(&self, _: WireId, _: u32, level: Level) {
            self.0.store(level.is_high(), Ordering::Relaxed);
        }
    }
    let f = scpu();
    let c = &f.c;
    let ids = WireIdAllocator::new();
    let probe = |port: &str| {
        let id = ids.alloc();
        let p = Arc::new(Probe::default());
        let wire = Wire::builder()
            .source(id)
            .sink(Arc::clone(&p) as Arc<dyn WireSink>, 0)
            .build_shared();
        Device::connect(c, port, WireSource::new(wire, id)).unwrap();
        p
    };
    let nmi = probe(NMI_PIN);
    let vblank = Device::sink(c, VBLANK_PIN, &[WireId::new(99)]).unwrap();
    vblank.sink.set_level(WireId::new(99), 0, Level::High);
    assert!(!nmi.0.load(Ordering::Relaxed), "NMI disabled");
    // Enabling NMI while RDNMI stands raises it.
    wr(&f, REGS, 0x80);
    assert!(nmi.0.load(Ordering::Relaxed));
    assert_eq!(rd(&f, REGS + 0x10) & 0x8F, 0x80 | CPU_VERSION);
    assert!(!nmi.0.load(Ordering::Relaxed), "the read acknowledged it");
    assert_eq!(rd(&f, REGS + 0x10) & 0x80, 0);

    let irq = probe(IRQ_PIN);
    c.shared.state.lock().timeup = true;
    c.shared.drive_lines();
    assert!(irq.0.load(Ordering::Relaxed));
    assert_eq!(rd(&f, REGS + 0x11) & 0x80, 0x80);
    assert!(!irq.0.load(Ordering::Relaxed), "TIMEUP read");
}

#[test]
fn a_reset_releases_the_interrupt_lines_it_was_driving() {
    // NMITIMEN comes out of reset as $00, both sources off (fullsnes, "4200h
    // - NMITIMEN"), so neither line can still be asserted afterwards. The
    // reset zeroed the device's record of what it had driven along with
    // everything else, compared the new state against that record, found no
    // change, and left the wires high: a level-sensitive IRQ with nothing
    // left to acknowledge it, and an NMI line already high for the next
    // vblank, whose edge the CPU therefore never saw.
    #[derive(Debug, Default)]
    struct Probe(AtomicBool);
    impl WireSink for Probe {
        fn set_level(&self, _: WireId, _: u32, level: Level) {
            self.0.store(level.is_high(), Ordering::Relaxed);
        }
    }
    let f = scpu();
    let c = &f.c;
    let ids = WireIdAllocator::new();
    let probe = |port: &str| {
        let id = ids.alloc();
        let p = Arc::new(Probe::default());
        let wire = Wire::builder()
            .source(id)
            .sink(Arc::clone(&p) as Arc<dyn WireSink>, 0)
            .build_shared();
        Device::connect(c, port, WireSource::new(wire, id)).unwrap();
        p
    };
    let nmi = probe(NMI_PIN);
    let irq = probe(IRQ_PIN);
    let vblank = Device::sink(c, VBLANK_PIN, &[WireId::new(99)]).unwrap();
    vblank.sink.set_level(WireId::new(99), 0, Level::High);
    wr(&f, REGS, 0x80);
    c.shared.state.lock().timeup = true;
    c.shared.drive_lines();
    assert!(nmi.0.load(Ordering::Relaxed) && irq.0.load(Ordering::Relaxed));

    Device::reset(c, ResetKind::Warm);
    assert!(!irq.0.load(Ordering::Relaxed), "IRQ is released");
    assert!(!nmi.0.load(Ordering::Relaxed), "and so is NMI");
}

#[test]
fn the_manual_joypad_shifts_b_first_then_ones() {
    let f = scpu();
    let c = &f.c;
    c.pad().set(0, buttons::B | buttons::A | buttons::R);
    wr(&f, JOY, 1);
    wr(&f, JOY, 0);
    let bits: alloc::vec::Vec<u8> = (0..18).map(|_| rd(&f, JOY) & 1).collect();
    assert_eq!(
        bits,
        [1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1],
        "B .. R, four ID zeros, then ones"
    );
    assert_eq!(rd(&f, JOY + 1) & 0x1C, 0x1C, "$4017 bits 4-2 read one");
}

#[test]
fn a_debug_read_changes_nothing() {
    let f = scpu();
    let c = &f.c;
    c.shared.state.lock().rdnmi = true;
    let v = f
        .space
        .read(REGS + 0x10, Width::U8, MemAttrs::DEBUG)
        .unwrap();
    assert_eq!(v & 0x80, 0x80);
    assert!(c.shared.state.lock().rdnmi, "still standing");
}

#[test]
fn a_snapshot_round_trips() {
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    let fs = scpu();
    let saved = &fs.c;
    // With a beam, and part-way into a frame the PPU has already left: the
    // frame this device's tick is in is then state nothing else holds.
    let ppu = crate::dev::snes::ppu::SnesPpu::default();
    *saved.shared.beam.lock() = Some(ppu.beam());
    ppu.advance_to(1364 * 300);
    saved.shared.advance_to(1364 * 100 + 7);
    assert_eq!(
        saved.shared.state.lock().frame,
        Some((0, false, false, 262)),
        "the first frame, not the PPU's second"
    );
    wr(&fs, DMA + 0x32, 0x34);
    wr(&fs, REGS + 0x07, 0x55);
    wr(&fs, REGS + 0x0D, 1);
    let mut shape = MachineShape::new();
    shape.add_device("scpu", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("scpu", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(saved, &mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();
    let fr = scpu();
    let restored = &fr.c;
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("scpu", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(restored, &mut chunk.reader()).unwrap();
    let want = *saved.shared.state.lock();
    let got = *restored.shared.state.lock();
    assert_eq!(got, want);
    assert!(restored.shared.memsel.load(Ordering::Relaxed));

    // The invariant as stated: saving the restored device gives the same
    // bytes.
    let mut shape = MachineShape::new();
    shape.add_device("scpu", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("scpu", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(restored, &mut chunk).unwrap();
    }
    assert_eq!(w.to_vec().unwrap(), bytes, "a round trip is a fixed point");
}

#[test]
fn a_v1_snapshot_loads_and_takes_its_frame_from_the_ppu() {
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    let fs = scpu();
    let ppu = crate::dev::snes::ppu::SnesPpu::default();
    *fs.c.shared.beam.lock() = Some(ppu.beam());
    ppu.advance_to(1364 * 50);
    fs.c.shared.advance_to(1364 * 50);
    wr(&fs, REGS + 0x07, 0x55);
    let chunk_of = |c: &SCpu, version: u32, trim: usize| {
        let mut shape = MachineShape::new();
        shape.add_device("scpu", CLASS_NAME).unwrap();
        let mut body = alloc::vec::Vec::new();
        let mut full = StateWriter::new(shape.clone());
        {
            let mut chunk = full.chunk("scpu", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(c, &mut chunk).unwrap();
        }
        let bytes = full.to_vec().unwrap();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("scpu", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        let mut cr = chunk.reader();
        body.extend_from_slice(cr.take(cr.remaining()).unwrap());
        body.truncate(body.len() - trim);
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("scpu", CLASS_NAME, version).unwrap();
            chunk.write_all(&body).unwrap();
        }
        w.to_vec().unwrap()
    };
    // A v1 chunk is today's without the frame: a flag, a tick, two flags and
    // a line count.
    let v1 = chunk_of(&fs.c, 1, 1 + 8 + 1 + 1 + 2);
    let mut table = Migrations::new();
    migrations(&mut table).unwrap();
    let fr = scpu();
    let reader = StateReader::new(&v1).unwrap();
    let chunk = reader
        .load("scpu", CLASS_NAME, STATE_VERSION, &table)
        .expect("v1 migrates to the shipping version");
    let mut cr = chunk.reader();
    Device::load(&fr.c, &mut cr).unwrap();
    cr.end().expect("every field read");
    let mut want = *fs.c.shared.state.lock();
    assert!(want.frame.is_some());
    want.frame = None;
    assert_eq!(*fr.c.shared.state.lock(), want);
    // And it finds its frame again the first time it needs one.
    *fr.c.shared.beam.lock() = Some(ppu.beam());
    fr.c.shared.advance_to(1364 * 52);
    assert_eq!(fr.c.shared.state.lock().frame, Some((0, false, false, 262)));
}

#[test]
fn starting_a_multiplication_also_loads_the_quotient_register() {
    // nocash, *fullsnes*, "SNES Maths Multiply/Divide", 4203h - WRMPYB: "For
    // some reason, the hardware does additionally set RDDIVL=WRMPYB, and
    // RDDIVH=00h."
    let f = scpu();
    // A division first, so the quotient register holds something else: $1234.
    wr(&f, REGS + 0x04, 0x34);
    wr(&f, REGS + 0x05, 0x12);
    wr(&f, REGS + 0x06, 0x01);
    assert_eq!((rd(&f, REGS + 0x14), rd(&f, REGS + 0x15)), (0x34, 0x12));

    wr(&f, REGS + 0x02, 5);
    wr(&f, REGS + 0x03, 7);
    assert_eq!((rd(&f, REGS + 0x16), rd(&f, REGS + 0x17)), (35, 0));
    assert_eq!(
        (rd(&f, REGS + 0x14), rd(&f, REGS + 0x15)),
        (7, 0),
        "RDDIV is the multiplier, zero-extended"
    );
}

#[test]
fn a_timer_target_the_beam_never_reaches_never_fires() {
    // SNESdev wiki, *MMIO registers*, HTIME: "Note that setting a value larger
    // than the maximum H counter value of 339 will prevent the timer's H
    // condition from being met." And VTIME: "Note that setting a value larger
    // than the maximum V counter value will prevent the timer's V condition
    // from being met. The maximum depends on the region (261 for NTSC, 311 for
    // PAL) and interlacing".
    //
    // The targets were folded into range instead — HTIME clamped to 339, VTIME
    // taken modulo the line count — so the power-on value of $1FF in either
    // register fired an interrupt every line or every frame the moment a
    // program enabled the timer before setting it.
    use crate::dev::snes::ppu::SnesPpu;
    // A little over a frame, with the two devices kept in step the way a
    // scheduler keeps them: the S-CPU measures its events from the beam.
    let run = |f: &Fixture, ppu: &SnesPpu| {
        for tick in (0..1364 * 270).step_by(4) {
            ppu.advance_to(tick);
            f.c.shared.advance_to(tick);
        }
    };
    for (nmitimen, htime, vtime) in [
        (0x20u8, 0u16, 300u16), // V-IRQ at a line NTSC does not have
        (0x20, 0, 262),         // ... including the first one past the end
        (0x10, 511, 0),         // H-IRQ at a dot no line has
        (0x30, 340, 10),        // both, with only H out of range
        (0x30, 10, 400),        // both, with only V out of range
    ] {
        let f = scpu();
        let ppu = SnesPpu::default();
        *f.c.shared.beam.lock() = Some(ppu.beam());
        wr(&f, REGS + 0x07, htime as u8);
        wr(&f, REGS + 0x08, (htime >> 8) as u8);
        wr(&f, REGS + 0x09, vtime as u8);
        wr(&f, REGS + 0x0A, (vtime >> 8) as u8);
        wr(&f, REGS, nmitimen);
        run(&f, &ppu);
        assert_eq!(
            rd(&f, REGS + 0x11) & 0x80,
            0,
            "NMITIMEN=${nmitimen:02x} HTIME={htime} VTIME={vtime}: TIMEUP stays clear"
        );
    }

    // And a target the beam does reach still fires.
    let f = scpu();
    let ppu = SnesPpu::default();
    *f.c.shared.beam.lock() = Some(ppu.beam());
    wr(&f, REGS + 0x07, 100);
    wr(&f, REGS + 0x08, 0);
    wr(&f, REGS + 0x09, 0x05); // 261 = $105
    wr(&f, REGS + 0x0A, 0x01);
    wr(&f, REGS, 0x30);
    run(&f, &ppu);
    assert_eq!(rd(&f, REGS + 0x11) & 0x80, 0x80, "line 261, dot 100 exists");
}

/// Where the PPU stands when the S-CPU is at `tick`, in three flavours: in
/// step, a constant distance ahead, and caught up in strides — which is where
/// it really stands, because the core runs whole bus cycles (and whole DMA
/// stalls) past an S-CPU event before the scheduler delivers it, and anything
/// that touches the PPU on the way catches it up to the core.
#[derive(Debug, Clone, Copy)]
enum Lead {
    Constant(u64),
    Strides(u64),
}

impl Lead {
    fn ppu_at(self, tick: u64) -> u64 {
        match self {
            Lead::Constant(n) => tick + n,
            Lead::Strides(n) => (tick / n + 1) * n,
        }
    }
}

const LEADS: [Lead; 6] = [
    Lead::Constant(0),
    Lead::Constant(8),
    Lead::Strides(40),
    Lead::Strides(600),
    Lead::Strides(1364 * 3 + 100),
    Lead::Strides(1364 * 300),
];

#[test]
fn a_beam_event_fires_on_its_own_tick_wherever_the_ppu_has_got_to() {
    use crate::dev::snes::ppu::SnesPpu;
    use crate::dev::snes::ppu::timing::dot_start;
    // V=10, H=135: one dot after the DRAM refresh event at dot 134. Every
    // event used to be re-derived, each time one fired, as "the PPU's distance
    // to that position, counted from the S-CPU's tick" — two different clocks.
    // With the PPU a constant 8 clocks ahead the interrupt came 8 clocks
    // early; with it ahead by more than the distance to the next occurrence,
    // that occurrence was replaced by the one after (a frame later, for a
    // V-IRQ).
    let want = 10 * 1364 + u64::from(dot_start(135, true));
    for lead in LEADS {
        let f = scpu();
        let ppu = SnesPpu::default();
        *f.c.shared.beam.lock() = Some(ppu.beam());
        f.c.shared.reschedule_all();
        wr(&f, REGS + 0x07, 135);
        wr(&f, REGS + 0x08, 0);
        wr(&f, REGS + 0x09, 10);
        wr(&f, REGS + 0x0A, 0);
        wr(&f, REGS, 0x30);
        let mut fired = None;
        for tick in 0..1364 * 30 {
            ppu.advance_to(lead.ppu_at(tick));
            f.c.shared.advance_to(tick);
            if f.c.shared.state.lock().timeup {
                fired = Some(tick);
                break;
            }
        }
        assert_eq!(fired, Some(want), "with the PPU led by {lead:?}");
    }
}

#[test]
fn hdma_and_refresh_run_once_a_line_wherever_the_ppu_has_got_to() {
    use crate::dev::snes::ppu::SnesPpu;
    // One HDMA channel writing one byte a line to $2100 from a table of
    // single-line entries, so the byte on the B-bus counts the transfers.
    for lead in LEADS {
        let f = scpu();
        let ppu = SnesPpu::default();
        *f.c.shared.beam.lock() = Some(ppu.beam());
        f.c.shared.reschedule_all();
        for i in 0..240u64 {
            f.low.write_u8(0x100 + 2 * i, 1).unwrap();
            f.low.write_u8(0x101 + 2 * i, i as u8).unwrap();
        }
        f.low.write_u8(0x100 + 480, 0).unwrap();
        wr(&f, DMA, 0x00);
        wr(&f, DMA + 1, 0x00);
        wr(&f, DMA + 2, 0x00);
        wr(&f, DMA + 3, 0x01);
        wr(&f, DMA + 4, 0x00);
        wr(&f, REGS + 0x0C, 0x01);
        // Dot 278 of lines 0 to 224 is 225 transfers: the last byte is 224,
        // and vertical blank (line 225 on) adds none. Refresh is every line,
        // blank or not.
        let lines = 240u64;
        let mut refreshes = 0;
        for tick in 0..lines * 1364 {
            ppu.advance_to(lead.ppu_at(tick));
            let before = f.c.shared.owed.load(Ordering::Relaxed);
            f.c.shared.advance_to(tick);
            let added = f.c.shared.owed.load(Ordering::Relaxed) - before;
            // HDMA's cost is 18 + 8 + 8 a line here; refresh is the 40.
            if added == REFRESH_CLOCKS {
                assert_eq!(tick % 1364, 536, "refresh is at dot 134 of its line");
                refreshes += 1;
            }
        }
        assert_eq!(f.b.read_u8(0).unwrap(), 224, "with the PPU led by {lead:?}");
        assert_eq!(refreshes, lines, "with the PPU led by {lead:?}");
    }
}

#[test]
fn wrio_comes_up_high_so_the_counters_can_be_latched() {
    // SNESdev wiki, *MMIO registers*, WRIO ($4201): "On power-on: WRIO = $FF",
    // and "Joypad port 2's I/O bit is also connected to the PPU's /EXTLATCH
    // input, allowing the PPU's H and V counters to be latched when this bit
    // is set to 0 by the CPU or joypad. [...] This should normally be set to 1
    // by the CPU to allow the counters to be latched." fullsnes, SLHV: the
    // $2137 read latches "only if WRIO.Bit7 is (or was) set".
    //
    // The register came up $FF and the PPU's input came up low, because the
    // level only ever reached the PPU from a *write* to $4201 — so a program
    // that left WRIO alone, as most do, read $2137 and latched nothing.
    use crate::dev::snes::ppu::SnesPpu;
    let f = scpu();
    let ppu = SnesPpu::default();
    *f.c.shared.beam.lock() = Some(ppu.beam());
    Device::reset(&f.c, ResetKind::Cold);
    assert_eq!(rd(&f, REGS + 0x13), 0xFF, "RDIO reads the power-on WRIO");
    ppu.advance_to(1364 * 50 + 400);
    ppu.read_register(0x37, 0);
    assert_eq!(
        ppu.read_register(0x3F, 0) & 0x40,
        0x40,
        "STAT78 bit 6: the counters were latched"
    );
    // And the reset itself latched nothing: there was no falling edge.
    let g = scpu();
    let ppu = SnesPpu::default();
    *g.c.shared.beam.lock() = Some(ppu.beam());
    Device::reset(&g.c, ResetKind::Cold);
    assert_eq!(ppu.read_register(0x3F, 0) & 0x40, 0);
    // A reset with WRIO bit 7 low raises the line again without latching.
    wr(&g, REGS + 0x01, 0x00);
    assert_eq!(ppu.read_register(0x3F, 0) & 0x40, 0x40, "1 -> 0 latches");
    Device::reset(&g.c, ResetKind::Warm);
    assert_eq!(ppu.read_register(0x3F, 0) & 0x40, 0);
}
