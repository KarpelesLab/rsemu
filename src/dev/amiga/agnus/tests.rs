//! Agnus as a whole chip: the register block, the pins, the copper against a
//! real custom-chip bus, the blitter against real chip RAM, and the snapshot.
//!
//! Every test here is a lib unit test on purpose. Lock order is only checked in
//! a debug build of the library itself (`core::sync`'s `rank_track`), and the
//! things most worth checking — that the copper's `MOVE` and every pin edge go
//! out with no lock of this chip held — are exactly what an integration test
//! built without it would wave through.

use super::*;
use crate::core::space::{RamStore, Region, RegionRef};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::wire::{Wire, WireId, WireSink};
use crate::dev::amiga::custom::Custom;
use alloc::vec;
use alloc::vec::Vec;

/// 512 KiB of chip RAM.
const CHIP: u64 = 0x8_0000;

/// A stand-in Denise: records every write that reaches it from anything but the
/// processor — the copper's and the DMA's — and where Agnus's beam was when it
/// arrived. Its lock is `DEVICE`-ranked like a real chip's, so
/// a copper that called out while holding its own state lock would trip the
/// rank check in a debug build.
#[derive(Debug)]
struct Recorder {
    dma: Arc<ChipDma>,
    seen: Mutex<Vec<(u16, u16, Origin, BeamPosition)>>,
}

impl CustomChip for Recorder {
    fn which(&self) -> ChipId {
        ChipId::DENISE
    }

    fn read(&self, _: &Reg, _: Origin) -> u16 {
        0
    }

    fn write(&self, reg: &Reg, value: u16, from: Origin) {
        if from.driver == Driver::Cpu {
            // `DMACON` reaches Denise too; the tests' own setup is not news.
            return;
        }
        let beam = self.dma.beam();
        self.seen.lock().push((reg.offset, value, from, beam));
    }
}

/// Counts rising edges on one pin, under a `DEVICE`-ranked lock for the same
/// reason as [`Recorder`].
#[derive(Debug, Default)]
struct Edges {
    state: Mutex<(bool, u64)>,
}

impl Edges {
    fn new() -> Arc<Edges> {
        Arc::new(Edges {
            state: Mutex::with_rank(LockRank::DEVICE, (false, 0)),
        })
    }

    fn rises(&self) -> u64 {
        self.state.lock().1
    }

    fn level(&self) -> bool {
        self.state.lock().0
    }
}

impl WireSink for Edges {
    fn set_level(&self, _: WireId, _: u32, level: Level) {
        let mut st = self.state.lock();
        if level.is_high() && !st.0 {
            st.1 += 1;
        }
        st.0 = level.is_high();
    }
}

/// A board: Agnus, a custom bus with a recording Denise on it, and chip RAM.
struct Board {
    agnus: Agnus,
    custom: Custom,
    denise: Arc<Recorder>,
    ram: Arc<RamStore>,
}

impl Board {
    fn new(std: Standard) -> Board {
        Board::on(Agnus::bare(std))
    }

    fn on(agnus: Agnus) -> Board {
        let custom = Custom::new(&Props::new()).unwrap();
        agnus.attach_bus(custom.bus()).unwrap();
        let ram = Arc::new(RamStore::new(CHIP));
        let region: RegionRef = Arc::new(Region::ram("chip", Arc::clone(&ram)));
        agnus.attach_ram(&region).unwrap();
        let denise = Arc::new(Recorder {
            dma: Arc::clone(agnus.dma()),
            seen: Mutex::with_rank(LockRank::DEVICE, Vec::new()),
        });
        custom
            .bus()
            .attach(Arc::clone(&denise) as Arc<dyn CustomChip>)
            .unwrap();
        Board {
            agnus,
            custom,
            denise,
            ram,
        }
    }

    fn pal() -> Board {
        Board::new(Standard::Pal)
    }

    fn poke(&self, offset: u16, value: u16) {
        assert!(
            self.custom.bus().write(offset, value, Origin::cpu()),
            "a write to ${offset:03x} reached nobody"
        );
    }

    fn peek(&self, offset: u16) -> u16 {
        self.custom.bus().read(offset, Origin::cpu())
    }

    fn store(&self, addr: u32, words: &[u16]) {
        for (i, w) in words.iter().enumerate() {
            let at = u64::from(addr) + 2 * i as u64;
            self.ram
                .write_at(at, &w.to_be_bytes())
                .expect("inside chip RAM");
        }
    }

    fn word(&self, addr: u32) -> u16 {
        let mut b = [0u8; 2];
        self.ram
            .read_at(u64::from(addr), &mut b)
            .expect("inside chip RAM");
        u16::from_be_bytes(b)
    }

    fn wire(&self, pin: &str, id: u64) -> Arc<Edges> {
        let edges = Edges::new();
        let src = WireId::new(id);
        let wire = Wire::builder()
            .source(src)
            .sink(Arc::clone(&edges) as Arc<dyn WireSink>, 0)
            .build_shared();
        self.agnus
            .connect_pin(pin, WireSource::new(wire, src))
            .unwrap();
        edges
    }

    fn run(&self, ticks: u64) {
        let target = self.agnus.ticks() + ticks;
        self.agnus.advance_to(target);
        assert_eq!(self.agnus.ticks(), target);
    }

    /// Point COP1LC at `addr`, strobe COPJMP1, and turn the copper on.
    fn start_copper(&self, addr: u32) {
        self.poke(COP1LCH, (addr >> 16) as u16);
        self.poke(COP1LCL, addr as u16);
        self.poke(COPJMP1, 0);
        self.poke(DMACON, dma::SETCLR | dma::DMAEN | DmaChannel::COPPER.0);
    }

    fn seen(&self) -> Vec<(u16, u16, Origin, BeamPosition)> {
        self.denise.seen.lock().clone()
    }
}

/// PAL counts in a field of 313 lines.
const PAL_FIELD: u64 = 313 * 227;

/// `COLOR00`, a Denise register the copper may always write.
const COLOR00: u16 = 0x180;
const COLOR01: u16 = 0x182;

// ---------------------------------------------------------------------------
// the beam
// ---------------------------------------------------------------------------

#[test]
fn the_beam_counters_read_back_through_the_bus() {
    let b = Board::pal();
    assert_eq!(b.peek(VPOSR), 0x8000, "LOF set, line 0, PAL Agnus id 00");
    assert_eq!(b.peek(VHPOSR), 0x0000);

    // Line $12C, count $E2: 300 whole lines and 226 counts in.
    b.run(300 * 227 + 0xe2);
    assert_eq!(b.peek(VPOSR), 0x8001, "V8");
    assert_eq!(b.peek(VHPOSR), 0x2ce2);

    b.run(11 * 227 + 1);
    assert_eq!(
        (b.peek(VPOSR), b.peek(VHPOSR)),
        (0x8001, 0x3800),
        "line 312"
    );
    b.run(227);
    assert_eq!(
        (b.peek(VPOSR), b.peek(VHPOSR)),
        (0x8000, 0x0000),
        "313 lines"
    );
    assert_eq!(b.agnus.beam().field, 1);
}

#[test]
fn an_ntsc_agnus_says_so_and_counts_its_long_lines() {
    let b = Board::new(Standard::Ntsc);
    assert_eq!(b.peek(VPOSR), 0x9000, "LOF, and id $10 in bits 14-8");
    b.run(227);
    assert_eq!(b.peek(VHPOSR), 0x0100, "line 0 was 227 counts");
    b.run(227);
    assert_eq!(b.peek(VHPOSR), 0x01e3, "line 1 is long: count $E3 exists");
    b.run(1);
    assert_eq!(b.peek(VHPOSR), 0x0200);
}

#[test]
fn vposw_and_vhposw_set_the_counters() {
    let b = Board::pal();
    b.poke(VPOSW, 0x0001);
    b.poke(VHPOSW, 0x2010);
    assert_eq!(b.peek(VPOSR), 0x0001, "LOF cleared, V8 set");
    assert_eq!(b.peek(VHPOSR), 0x2010);
    // A short field now: line $120 is 288, and 312 lines end at 311.
    b.run((311 - 288) * 227 + (227 - 0x10));
    assert_eq!(b.peek(VHPOSR), 0x0000);
    assert_eq!(b.peek(VPOSR), 0x0000, "LOF holds without LACE");
}

#[test]
fn lace_toggles_lof_every_field() {
    let b = Board::pal();
    b.poke(BPLCON0, LACE);
    b.run(PAL_FIELD);
    assert_eq!(
        b.peek(VPOSR) & 0x8000,
        0,
        "a short field follows a long one"
    );
    b.run(312 * 227);
    assert_eq!(b.peek(VPOSR) & 0x8000, 0x8000);
    b.run(PAL_FIELD + 312 * 227);
    assert_eq!(b.agnus.beam().field, 4);
}

// ---------------------------------------------------------------------------
// the pins
// ---------------------------------------------------------------------------

#[test]
fn vsync_rises_once_a_field_and_hsync_once_a_line() {
    let b = Board::pal();
    let vsync = b.wire(VSYNC_PIN, 1);
    let hsync = b.wire(HSYNC_PIN, 2);
    assert!(vsync.level() && hsync.level(), "line 0, count 0: both high");
    // The edge the connection itself announced is not one the field made.
    let (v0, h0) = (vsync.rises(), hsync.rises());

    b.run(10 * PAL_FIELD);
    assert_eq!(vsync.rises() - v0, 10, "one per field: 50 Hz's worth");
    assert_eq!(hsync.rises() - h0, 10 * 313, "one per line");
}

#[test]
fn a_pin_edge_is_an_event_and_an_unconnected_one_is_not() {
    let b = Board::pal();
    assert_eq!(
        b.agnus.next_event_tick(),
        Some(PAL_FIELD),
        "with nothing wired the next thing is the field"
    );
    let _hsync = b.wire(HSYNC_PIN, 1);
    assert_eq!(b.agnus.next_event_tick(), Some(u64::from(HSYNC_COUNTS)));
    b.run(u64::from(HSYNC_COUNTS));
    assert_eq!(b.agnus.next_event_tick(), Some(227), "the next line's rise");
}

#[test]
fn the_same_run_in_one_stride_or_a_thousand_ends_identically() {
    // Additivity: catch-up may be sliced any way the scheduler likes.
    let copper = [0x2c01u16, 0xff00, COLOR00, 0x0fff, 0xffff, 0xfffe];
    let one = Board::pal();
    let many = Board::pal();
    for b in [&one, &many] {
        b.store(0x1000, &copper);
        b.start_copper(0x1000);
    }
    let v_one = one.wire(VSYNC_PIN, 1);
    let v_many = many.wire(VSYNC_PIN, 1);

    one.run(3 * PAL_FIELD + 12_345);
    let mut left = 3 * PAL_FIELD + 12_345;
    let mut step = 1u64;
    while left > 0 {
        let n = step.min(left);
        many.run(n);
        left -= n;
        step = (step * 7 + 3) % 5003 + 1;
    }
    assert_eq!(snapshot(&one.agnus), snapshot(&many.agnus));
    assert_eq!(v_one.rises(), v_many.rises());
    let beams = |b: &Board| b.seen().iter().map(|s| s.3).collect::<Vec<_>>();
    assert_eq!(beams(&one), beams(&many), "every MOVE on the same count");
}

// ---------------------------------------------------------------------------
// DMACON
// ---------------------------------------------------------------------------

#[test]
fn dmacon_sets_and_clears_and_dmaconr_reads_it_back() {
    let b = Board::pal();
    b.poke(DMACON, 0x8000 | 0x0200 | 0x0180 | 0x000f);
    assert_eq!(b.peek(DMACONR), 0x038f);
    b.poke(DMACON, 0x0180);
    assert_eq!(
        b.peek(DMACONR),
        0x020f,
        "a clear takes only the bits written as one"
    );
    b.poke(DMACON, 0xffff);
    assert_eq!(b.peek(DMACONR), 0x07ff, "bits 14-11 are not writable");
    assert_eq!(b.agnus.dma().dmacon(), 0x07ff, "and Paula's view agrees");
}

// ---------------------------------------------------------------------------
// the copper
// ---------------------------------------------------------------------------

#[test]
fn a_move_after_a_wait_lands_three_cycles_after_the_beam_arrives() {
    // The WAIT is satisfied on arrival at count $40 of line $40 — that cycle is
    // its wake-up — then IR1 is fetched at $42 and the MOVE happens with IR2
    // at $44.
    let b = Board::pal();
    b.store(0x2000, &[0x4041, 0xfffe, COLOR00, 0x0f00, 0xffff, 0xfffe]);
    b.start_copper(0x2000);
    b.run(PAL_FIELD);
    let seen = b.seen();
    assert_eq!(seen.len(), 1, "{seen:?}");
    let (offset, value, from, at) = seen[0];
    assert_eq!((offset, value), (COLOR00, 0x0f00));
    assert_eq!(from, Origin::copper(false));
    assert_eq!((at.vpos, at.hpos), (0x40, 0x44));
    assert_eq!(at.tick, 0x40 * 227 + 0x44);
}

#[test]
fn the_copper_restarts_from_cop1lc_at_the_top_of_every_field() {
    let b = Board::pal();
    b.store(0x3000, &[COLOR00, 0x0123, 0xffff, 0xfffe]);
    b.start_copper(0x3000);
    b.run(3 * PAL_FIELD);
    let lines: Vec<(u64, u16, u16)> = b
        .seen()
        .iter()
        .map(|s| (s.3.field, s.3.vpos, s.3.hpos))
        .collect();
    // The start: the strobe lands on count 0, so the first copper cycle is
    // count 2 (IR1) and the MOVE completes on count 4. At each new field,
    // count 0 is itself a cycle, so the MOVE completes on count 2. The run
    // ends on count 0 of field 3, with only IR1 fetched.
    assert_eq!(lines, vec![(0, 0, 4), (1, 0, 2), (2, 0, 2)]);
}

#[test]
fn copjmp2_from_inside_the_list_jumps() {
    // The copper writing its own strobe re-enters this chip from inside its
    // own catch-up, which is the re-entrancy the runner is shaped for.
    let b = Board::pal();
    b.store(
        0x4000,
        &[
            COP2LCL, 0x4100, COPJMP2, 0x0000, COLOR00, 0x0bad, 0xffff, 0xfffe,
        ],
    );
    b.store(0x4100, &[COLOR01, 0x0f0f, 0xffff, 0xfffe]);
    b.start_copper(0x4000);
    b.run(1000);
    let writes: Vec<(u16, u16)> = b.seen().iter().map(|s| (s.0, s.1)).collect();
    assert_eq!(writes, vec![(COLOR01, 0x0f0f)]);
}

#[test]
fn the_seam_holds_the_copper_to_copcon() {
    let b = Board::pal();
    // BLTCON0 needs the danger bit; COPCON never.
    b.store(0x5000, &[BLTCON0, 0x09f0, COPCON, 0x0002, 0xffff, 0xfffe]);
    b.start_copper(0x5000);
    b.run(100);
    assert_eq!(
        b.custom.bus().refused_copper_writes(),
        1,
        "BLTCON0 refused, and the copper stopped there: COPCON was never tried"
    );
    assert_eq!(b.agnus.shared.state.lock().blitter.con0, 0);
    assert_eq!(b.agnus.shared.state.lock().copper.phase, Phase::Halted);

    b.poke(COPCON, CDANG);
    b.poke(COPJMP1, 0);
    b.run(100);
    assert_eq!(
        b.agnus.shared.state.lock().blitter.con0,
        0x09f0,
        "with CDANG"
    );
    assert_eq!(
        b.custom.bus().refused_copper_writes(),
        2,
        "COPCON still refused"
    );
    assert_eq!(b.agnus.shared.state.lock().copper.phase, Phase::Halted);
}

#[test]
fn a_halted_copper_writes_nothing_more_until_the_next_field_restarts_it() {
    // The shape of the list Kickstart 1.3 and AROS hand the copper before
    // their first screen: a pointer's high word, `$0000`, read as a MOVE to
    // `$000`, followed by words that would be writes to real registers.
    let b = Board::pal();
    b.store(0x5000, &[0x0000, 0x1d16, COLOR00, 0x0bad, 0xffff, 0xfffe]);
    b.start_copper(0x5000);
    b.run(100);
    assert!(
        b.seen().is_empty(),
        "nothing after the refused MOVE: {:?}",
        b.seen()
    );
    assert_eq!(b.agnus.shared.state.lock().copper.phase, Phase::Halted);

    // A field later the copper restarts at COP1LC — the same list — and halts
    // again at the same place, so COLOR00 is never written.
    b.run(u64::from(227u16) * 313 + 100);
    assert!(b.seen().is_empty());

    // A list that starts properly runs once the location register says so.
    b.store(0x6000, &[COLOR00, 0x0123, 0xffff, 0xfffe]);
    b.poke(COP1LCH, 0);
    b.poke(COP1LCL, 0x6000);
    b.poke(COPJMP1, 0);
    b.run(100);
    let writes: Vec<(u16, u16)> = b.seen().iter().map(|s| (s.0, s.1)).collect();
    assert_eq!(writes, vec![(COLOR00, 0x0123)]);
}

#[test]
fn skip_skips_and_a_disabled_copper_stands_still() {
    let b = Board::pal();
    b.store(
        0x6000,
        &[
            0x0001, 0xff01, COLOR00, 0x0001, COLOR01, 0x0002, 0xffff, 0xfffe,
        ],
    );
    b.start_copper(0x6000);
    b.run(50);
    assert_eq!(
        b.seen().iter().map(|s| s.0).collect::<Vec<_>>(),
        vec![COLOR01],
        "line 0 >= line 0: skipped"
    );

    b.poke(DMACON, DmaChannel::COPPER.0);
    let pc = b.agnus.copper_pc();
    b.poke(COPJMP1, 0);
    b.run(PAL_FIELD);
    assert_eq!(b.seen().len(), 1, "COPEN clear: nothing moves");
    assert_eq!(
        b.agnus.copper_pc(),
        0x6000,
        "a strobe still loads the counter"
    );
    assert_ne!(pc, 0x6000);
}

#[test]
fn a_wait_can_stride_but_its_move_still_lands_on_its_count() {
    // A WAIT deep in the field is found arithmetically; its MOVE must arrive on
    // the same count it would stepping one count at a time.
    let b = Board::pal();
    b.store(0x7000, &[0xf0e1, 0xfffe, COLOR00, 0x0aaa, 0xffff, 0xfffe]);
    b.start_copper(0x7000);
    assert_eq!(
        b.agnus.next_event_tick(),
        Some(4),
        "IR1 on count 2 has no effect outside; IR2 on count 4 might"
    );
    b.run(20);
    let wake = 0xf0 * 227 + 0xe0;
    assert_eq!(b.agnus.next_event_tick(), Some(wake), "the wake-up count");
    b.run(PAL_FIELD - 20);
    let at = b.seen()[0].3;
    // Woken on $E0, IR1 on $E2 — the last count of a PAL line — and IR2 on
    // count 0 of the next.
    assert_eq!((at.vpos, at.hpos), (0xf1, 0x00));
}

// ---------------------------------------------------------------------------
// the blitter
// ---------------------------------------------------------------------------

/// A 3-word by 2-row A-to-D copy, set up through the registers.
fn copy_blit(b: &Board) {
    b.store(0x8000, &[0x1111, 0x2222, 0x3333, 0x4444, 0x5555, 0x6666]);
    b.poke(BLTCON0, blitter::USEA | blitter::USED | 0xf0);
    b.poke(BLTCON1, 0);
    b.poke(BLTAFWM, 0xffff);
    b.poke(BLTALWM, 0xffff);
    b.poke(0x050, 0x0000); // BLTAPTH
    b.poke(0x052, 0x8000); // BLTAPTL
    b.poke(0x054, 0x0000); // BLTDPTH
    b.poke(BLTDPTL, 0x9000);
    b.poke(0x064, 0); // BLTAMOD
    b.poke(BLTDMOD, 0);
}

#[test]
fn a_blit_takes_the_speed_tables_time_and_pulses_blit_when_done() {
    let b = Board::pal();
    let blit = b.wire(BLIT_PIN, 3);
    copy_blit(&b);
    b.poke(DMACON, dma::SETCLR | dma::DMAEN | DmaChannel::BLITTER.0);
    b.poke(BLTSIZE, (2 << 6) | 3);
    assert_eq!(
        b.peek(DMACONR) & BBUSY,
        BBUSY,
        "busy from the BLTSIZE write"
    );
    assert_eq!(
        b.agnus.next_event_tick(),
        Some(12),
        "six words at four ticks"
    );

    b.run(11);
    assert!(b.agnus.blitter_busy());
    assert_eq!(blit.rises(), 0);
    b.run(1);
    assert!(!b.agnus.blitter_busy());
    assert_eq!(blit.rises(), 1);
    assert!(blit.level(), "high on the count it finished");
    b.run(1);
    assert!(!blit.level(), "for one count");
    assert_eq!(b.peek(DMACONR) & (BBUSY | BZERO), 0, "done, and not zero");
    let copied: Vec<u16> = (0..6).map(|i| b.word(0x9000 + 2 * i)).collect();
    assert_eq!(copied, vec![0x1111, 0x2222, 0x3333, 0x4444, 0x5555, 0x6666]);
}

#[test]
fn a_blit_waits_for_blten_and_the_copper_can_wait_for_it() {
    let b = Board::pal();
    copy_blit(&b);
    b.poke(BLTSIZE, (2 << 6) | 3);
    b.run(1000);
    assert!(b.agnus.blitter_busy(), "BLTEN is clear: the blit is parked");

    // WAIT for line 0 with BFD clear: satisfied only once the blit is done.
    b.store(0xa000, &[0x0001, 0x7ffe, COLOR00, 0x0777, 0xffff, 0xfffe]);
    b.start_copper(0xa000);
    b.run(100);
    assert!(b.seen().is_empty(), "the copper holds for the blitter");
    b.poke(DMACON, dma::SETCLR | DmaChannel::BLITTER.0);
    b.run(12 + 6);
    assert!(!b.agnus.blitter_busy());
    assert_eq!(b.seen().len(), 1, "and moves once it is finished");
}

// ---------------------------------------------------------------------------
// DMA and the export
// ---------------------------------------------------------------------------

#[test]
fn paulas_pointers_are_agnus_registers() {
    let b = Board::pal();
    b.store(0xb000, &[0xcafe, 0xbeef]);
    b.poke(0x0b0, 0x0000); // AUD1LCH
    b.poke(0x0b2, 0xb000); // AUD1LCL
    b.poke(DSKPTH, 0x0000);
    b.poke(DSKPTL, 0xb002);
    b.poke(
        DMACON,
        dma::SETCLR | dma::DMAEN | DmaChannel::AUD1.0 | DmaChannel::DISK.0,
    );

    let export = b.agnus.export(ExportId::CHIP_DMA).expect("published");
    let dma = Arc::clone(export.opaque().unwrap())
        .downcast::<ChipDma>()
        .expect("a ChipDma");
    dma.audio_restart(1);
    assert_eq!(dma.audio_fetch(1), Some(0xcafe));
    assert_eq!(dma.disk_fetch(), Some(0xbeef));
    assert!(b.agnus.export(ExportId::CUSTOM_BUS).is_none());
}

// ---------------------------------------------------------------------------
// reset and snapshots
// ---------------------------------------------------------------------------

#[test]
fn a_reset_disables_every_channel_and_keeps_the_clock() {
    let b = Board::pal();
    b.poke(DMACON, 0x87ff);
    b.poke(COPCON, CDANG);
    b.run(5000);
    Device::reset(&b.agnus, ResetKind::Cold);
    assert_eq!(
        b.agnus.ticks(),
        5000,
        "the scheduler's clock is not rewound"
    );
    assert_eq!(b.peek(DMACONR), 0);
    assert_eq!(b.peek(VHPOSR), 0, "a cold start is the top of a field");
    assert_eq!(b.agnus.dma().dmacon(), 0);
}

fn snapshot(agnus: &Agnus) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("agnus", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("agnus", CLASS_NAME, STATE_VERSION).unwrap();
        Device::save(agnus, &mut chunk).unwrap();
    }
    w.to_vec().unwrap()
}

fn restore(agnus: &Agnus, bytes: &[u8]) {
    let reader = StateReader::new(bytes).unwrap();
    let chunk = reader
        .load("agnus", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    Device::load(agnus, &mut chunk.reader()).unwrap();
}

#[test]
fn a_snapshot_mid_blit_and_mid_copper_round_trips_and_resumes_identically() {
    let run = |b: &Board| {
        b.store(
            0xc000,
            &[
                0x1001, 0xfffe, COLOR00, 0x0f00, 0x2001, 0xfffe, COLOR01, 0x00f0,
            ],
        );
        copy_blit(b);
        b.poke(0x0a0, 0x0001); // AUD0LCH
        b.poke(0x0a2, 0x2344); // AUD0LCL
        b.poke(DMACON, dma::SETCLR | dma::DMAEN | DmaChannel::BLITTER.0);
        b.start_copper(0xc000);
        b.poke(BLTSIZE, (2 << 6) | 3);
    };
    let saved = Board::pal();
    run(&saved);
    saved.run(0x10 * 227 + 5);
    assert!(saved.agnus.shared.state.lock().copper.phase != Phase::Wait || saved.seen().is_empty());
    let bytes = snapshot(&saved.agnus);

    let restored = Board::pal();
    restore(&restored.agnus, &bytes);
    assert_eq!(
        snapshot(&restored.agnus),
        bytes,
        "identical state after a round trip"
    );
    assert_eq!(restored.agnus.ticks(), saved.agnus.ticks());
    assert_eq!(
        restored.agnus.next_event_tick(),
        saved.agnus.next_event_tick()
    );
    assert_eq!(restored.agnus.dma().audio_location(0), 0x0001_2344);

    // Chip RAM is chip RAM's to snapshot; give the restored board the same.
    let mut mem = vec![0u8; CHIP as usize];
    saved.ram.read_at(0, &mut mem).unwrap();
    restored.ram.write_at(0, &mem).unwrap();
    saved.run(PAL_FIELD);
    restored.run(PAL_FIELD);
    assert_eq!(snapshot(&restored.agnus), snapshot(&saved.agnus));
    let tail = |b: &Board| {
        b.seen()
            .iter()
            .rev()
            .take(2)
            .map(|s| (s.0, s.3))
            .collect::<Vec<_>>()
    };
    assert_eq!(tail(&restored), tail(&saved));
}

// ---------------------------------------------------------------------------
// the class
// ---------------------------------------------------------------------------

#[test]
fn the_class_wants_its_links_and_knows_its_standards() {
    use crate::core::props::{Link, Value};
    let links = || {
        Props::new()
            .with("custom", Value::Link(Link::new("custom").unwrap()))
            .with("ram", Value::Link(Link::new("chipram").unwrap()))
    };
    assert_eq!(Agnus::new(&links()).unwrap().standard(), Standard::Pal);
    let ntsc = links().with("standard", Value::from("ntsc"));
    assert_eq!(Agnus::new(&ntsc).unwrap().standard(), Standard::Ntsc);
    assert!(Agnus::new(&links().with("standard", Value::from("secam"))).is_err());
    assert!(Agnus::new(&Props::new()).is_err(), "no links");
    assert!(
        Agnus::bare(Standard::Pal)
            .connect_pin("csync", {
                let src = WireId::new(1);
                WireSource::new(Wire::builder().source(src).build_shared(), src)
            })
            .is_err()
    );
}

// ---------------------------------------------------------------------------
// the joined chipset: a real Denise and a real Paula
// ---------------------------------------------------------------------------

use crate::dev::amiga::denise::{self, Video};
use crate::dev::amiga::paula::{self, Paula};
use crate::host::chardev::{CharDevice, CharPort};

/// Agnus with the two chips it drives, all three on one bus, and chip RAM.
struct Chipset {
    agnus: Agnus,
    custom: Custom,
    video: Arc<Video>,
    paula: Paula,
    ram: Arc<RamStore>,
}

impl Chipset {
    fn pal() -> Chipset {
        Chipset::part(Revision::Ocs, denise::Revision::Ocs, CHIP)
    }

    /// Alice and Lisa on one bus, with `ram` bytes of chip RAM.
    fn aga(ram: u64) -> Chipset {
        Chipset::part(Revision::Aga, denise::Revision::Aga, ram)
    }

    fn part(rev: Revision, dev: denise::Revision, len: u64) -> Chipset {
        let agnus = Agnus::part(rev, Standard::Pal);
        let custom = Custom::new(&Props::new()).unwrap();
        agnus.attach_bus(custom.bus()).unwrap();
        let ram = Arc::new(RamStore::new(len));
        let region: RegionRef = Arc::new(Region::ram("chip", Arc::clone(&ram)));
        agnus.attach_ram(&region).unwrap();

        let video = Arc::new(Video::with_revision(denise::Standard::Pal, dev));
        custom
            .bus()
            .attach(Arc::clone(&video) as Arc<dyn CustomChip>)
            .unwrap();
        agnus.attach_video(Arc::clone(&video));

        let paula = Paula::with_port(
            String::from("custom"),
            Arc::new(CharPort::new()) as Arc<dyn CharDevice>,
            String::from("serial"),
        );
        custom.bus().attach(paula.chip()).unwrap();
        agnus.attach_paula(paula.port());
        Chipset {
            agnus,
            custom,
            video,
            paula,
            ram,
        }
    }

    fn poke(&self, offset: u16, value: u16) {
        assert!(
            self.custom.bus().write(offset, value, Origin::cpu()),
            "a write to ${offset:03x} reached nobody"
        );
    }

    fn store(&self, addr: u32, words: &[u16]) {
        for (i, w) in words.iter().enumerate() {
            self.ram
                .write_at(u64::from(addr) + 2 * i as u64, &w.to_be_bytes())
                .expect("inside chip RAM");
        }
    }

    fn run(&self, ticks: u64) {
        let target = self.agnus.ticks() + ticks;
        self.agnus.advance_to(target);
        // Paula is caught up by every call Agnus makes into it; bring it to the
        // same count for the assertions that read it directly.
        self.paula.advance_to(target);
    }

    fn row(&self, y: u32) -> Vec<u16> {
        let mut row = vec![0u16; denise::WIDTH as usize];
        self.video.read_row(y, &mut row);
        row
    }

    /// One low-resolution plane in the nominal PAL window, every word `$FF00`,
    /// colour 1 red.
    fn one_plane(&self, addr: u32, modulo: u16) {
        self.store(addr, &vec![0xff00u16; 22 * 256]);
        self.poke(BPL1PTH, (addr >> 16) as u16);
        self.poke(BPL1PTH + 2, addr as u16);
        self.poke(BPL1MOD, modulo);
        self.poke(DIWSTRT, 0x2c81);
        self.poke(DIWSTOP, 0x2cc1);
        self.poke(DDFSTRT, 0x38);
        self.poke(DDFSTOP, 0xd0);
        self.poke(0x180, 0x0000); // COLOR00
        self.poke(0x182, 0x0f00); // COLOR01
        self.poke(BPLCON0, 0x1000);
        self.poke(DMACON, dma::SETCLR | dma::DMAEN | DmaChannel::BITPLANE.0);
    }
}

#[test]
fn every_line_and_field_reaches_denise() {
    let c = Chipset::pal();
    c.run(2 * PAL_FIELD + 227);
    assert_eq!(c.video.fields(), 2, "one field call per field");
    assert_eq!(
        c.video.field_clocks(),
        PAL_FIELD,
        "and the lines of the last one add up to 313 × 227 counts"
    );
}

#[test]
fn a_fetched_plane_is_rendered_where_chapter_three_puts_it() {
    let c = Chipset::pal();
    c.one_plane(0x1_0000, 0);
    c.run(PAL_FIELD);

    // Line $2C is the window's first; Denise gives each line two rows from
    // line $1D, and a word fetched at count $38 is displayed from x = 2 × $38 +
    // 17 = $81, which is DIWSTRT's HSTART: column (129 − 64) × 2 = 130.
    let red = |row: &[u16]| row.iter().filter(|&&p| p == 0x0f00).count();
    let y = 2 * (0x2c - 0x1d);
    assert_eq!(red(&c.row(y - 1)), 0, "line $2B is above the window");
    let row = c.row(y);
    assert_eq!(red(&row), 20 * 8 * 2, "twenty words of eight red pixels");
    assert_eq!(row[129], 0x0000, "the border before HSTART");
    assert!(row[130..146].iter().all(|&p| p == 0x0f00), "the first byte");
    assert!(row[146..162].iter().all(|&p| p == 0x0000), "the second");
    assert_eq!(red(&c.row(2 * (0x12b - 0x1d))), 320, "the last line");
    assert_eq!(red(&c.row(2 * (0x12c - 0x1d))), 0, "and not past it");
}

#[test]
fn the_bitplane_pointers_add_the_modulo_every_fetched_line() {
    let c = Chipset::pal();
    c.one_plane(0x1_0000, 4);
    c.run(PAL_FIELD);
    assert_eq!(
        c.agnus.shared.state.lock().bplpt[0],
        0x1_0000 + 256 * (40 + 4),
        "256 lines of twenty words and a four-byte modulo"
    );
}

#[test]
fn line_dma_is_simulated_a_line_at_a_time_but_is_not_an_event() {
    // Bitplanes on and Denise attached: the scheduler still hears only about
    // the field, because nothing a guest can see changes at a line boundary.
    let c = Chipset::pal();
    c.one_plane(0x1_0000, 0);
    assert_eq!(c.agnus.next_event_tick(), Some(PAL_FIELD));
}

#[test]
fn the_pointers_move_the_same_with_or_without_denise() {
    let with = {
        let c = Chipset::pal();
        c.one_plane(0x2_0000, 0);
        c.run(PAL_FIELD / 2);
        c.agnus.shared.state.lock().bplpt
    };
    let without = {
        let b = Board::pal();
        b.store(0x2_0000, &vec![0xff00u16; 22 * 256]);
        b.poke(BPL1PTH, 0x0002);
        b.poke(BPL1PTH + 2, 0x0000);
        b.poke(BPL1MOD, 0);
        b.poke(DIWSTRT, 0x2c81);
        b.poke(DIWSTOP, 0x2cc1);
        b.poke(DDFSTRT, 0x38);
        b.poke(DDFSTOP, 0xd0);
        b.poke(BPLCON0, 0x1000);
        b.poke(DMACON, dma::SETCLR | dma::DMAEN | DmaChannel::BITPLANE.0);
        b.run(PAL_FIELD / 2);
        b.agnus.shared.state.lock().bplpt
    };
    assert_eq!(with, without);
    assert_ne!(with[0], 0x2_0000, "and they did move");
}

#[test]
fn a_copper_colour_split_lands_on_its_line() {
    // WAIT for line $80, set COLOR00 to blue: every picture row from line $80
    // down is blue where the background shows, and every row above is not.
    let c = Chipset::pal();
    c.store(0x3000, &[0x8001, 0xff00, 0x0180, 0x000f, 0xffff, 0xfffe]);
    c.poke(0x180, 0x0000);
    c.poke(COP1LCH, 0);
    c.poke(COP1LCL, 0x3000);
    c.poke(COPJMP1, 0);
    c.poke(DMACON, dma::SETCLR | dma::DMAEN | DmaChannel::COPPER.0);
    // Past line $80 of the first field; the copper restarts next field, and
    // COLOR00 then stays blue until the WAIT again, so read the first field.
    c.run(0x90 * 227);
    let blue = |y: u32| c.row(y).iter().filter(|&&p| p == 0x000f).count();
    assert_eq!(blue(2 * (0x7f - 0x1d)), 0, "line $7F is above the split");
    assert!(blue(2 * (0x81 - 0x1d)) > 0, "line $81 is below it");
}

#[test]
fn a_sprite_follows_chapter_fours_data_structure() {
    // The manual's spaceship and its re-use, from *Reusing Sprite DMA Channels*,
    // shortened to two data lines each.
    let b = Board::pal();
    b.store(
        0x3_0000,
        &[
            0x6d60, 0x6f00, // VSTART $6D, HSTART $60, VSTOP $6F
            0x0990, 0x07e0, // line $6D
            0x13c8, 0x0ff0, // line $6E
            0x8080, 0x8200, // re-use: VSTART $80, VSTOP $82
            0x1818, 0x0000, 0x7e7e, 0x0000, // lines $80, $81
            0x0000, 0x0000, // end of data
        ],
    );
    b.poke(0x120, 0x0003); // SPR0PTH
    b.poke(0x122, 0x0000); // SPR0PTL
    b.poke(DMACON, dma::SETCLR | dma::DMAEN | DmaChannel::SPRITE.0);
    b.run(PAL_FIELD - 1);

    let writes: Vec<(u16, u16, u16, u16)> = b
        .seen()
        .iter()
        .filter(|s| s.2 == Origin::dma() && s.0 < 0x148)
        .map(|s| (s.3.vpos, s.3.hpos, s.0, s.1))
        .collect();
    let (pos, ctl, data, datb) = (0x140, 0x142, 0x144, 0x146);
    assert_eq!(
        writes,
        vec![
            (0x1d, 0, pos, 0x6d60),
            (0x1d, 0, ctl, 0x6f00),
            (0x6d, 0, data, 0x0990),
            (0x6d, 0, datb, 0x07e0),
            (0x6e, 0, data, 0x13c8),
            (0x6e, 0, datb, 0x0ff0),
            (0x6f, 0, pos, 0x8080),
            (0x6f, 0, ctl, 0x8200),
            (0x80, 0, data, 0x1818),
            (0x80, 0, datb, 0x0000),
            (0x81, 0, data, 0x7e7e),
            (0x81, 0, datb, 0x0000),
            (0x82, 0, pos, 0x0000),
            (0x82, 0, ctl, 0x0000),
        ]
    );
    let others: Vec<(u16, u16)> = b
        .seen()
        .iter()
        .filter(|s| s.2 == Origin::dma() && s.0 >= 0x148)
        .map(|s| (s.3.vpos, s.1))
        .collect();
    assert_eq!(
        others.len(),
        7 * 2,
        "the other seven channels: one control pair"
    );
    assert!(
        others
            .iter()
            .all(|&(vpos, value)| vpos == 0x1d && value == 0),
        "fetched from address zero at the end of blanking, and never started"
    );
}

#[test]
fn the_beam_source_answers_from_inside_a_copper_move() {
    #[derive(Debug)]
    struct Asker {
        beam: Mutex<Option<BeamSource>>,
        seen: Mutex<Vec<BeamPosition>>,
    }
    impl CustomChip for Asker {
        fn which(&self) -> ChipId {
            ChipId::PAULA
        }
        fn read(&self, _: &Reg, _: Origin) -> u16 {
            0
        }
        fn write(&self, _: &Reg, _: u16, from: Origin) {
            if from.driver == Driver::Cpu {
                return;
            }
            let beam = self.beam.lock().clone().expect("attached");
            self.seen.lock().push(beam.now());
        }
    }
    let b = Board::pal();
    let asker = Arc::new(Asker {
        beam: Mutex::with_rank(LockRank::LEAF, Some(b.agnus.beam_source())),
        seen: Mutex::with_rank(LockRank::DEVICE, Vec::new()),
    });
    b.custom
        .bus()
        .attach(Arc::clone(&asker) as Arc<dyn CustomChip>)
        .unwrap();
    // WAIT for line $30, then MOVE INTREQ.
    b.store(0x4_0000, &[0x3001, 0xff00, 0x009c, 0x8010, 0xffff, 0xfffe]);
    b.start_copper(0x4_0000);
    b.run(PAL_FIELD / 2);
    let seen = asker.seen.lock().clone();
    assert_eq!(seen.len(), 1);
    assert_eq!((seen[0].vpos, seen[0].hpos), (0x30, 4));
}

#[test]
fn a_beam_source_outliving_its_agnus_reports_the_top_of_the_field() {
    let source = Agnus::bare(Standard::Pal).beam_source();
    assert_eq!(
        source.now(),
        BeamPosition::default(),
        "no cycle kept it alive"
    );
}

#[test]
fn vertb_and_blit_reach_paula_on_their_counts() {
    let c = Chipset::pal();
    c.store(0x8000, &[0x1111, 0x2222]);
    c.poke(BLTCON0, blitter::USEA | blitter::USED | 0xf0);
    c.poke(BLTAFWM, 0xffff);
    c.poke(BLTALWM, 0xffff);
    c.poke(0x052, 0x8000); // BLTAPTL
    c.poke(BLTDPTL, 0x9000);
    c.poke(DMACON, dma::SETCLR | dma::DMAEN | DmaChannel::BLITTER.0);
    c.poke(BLTSIZE, (1 << 6) | 2);

    c.run(3);
    assert_eq!(
        c.paula.intreq() & paula::int::BLIT,
        0,
        "four counts, not three"
    );
    c.run(1);
    assert_eq!(c.paula.intreq() & paula::int::BLIT, paula::int::BLIT);
    assert_eq!(c.paula.intreq() & paula::int::VERTB, 0);

    c.run(PAL_FIELD - 5);
    assert_eq!(c.paula.intreq() & paula::int::VERTB, 0, "one count short");
    c.run(1);
    assert_eq!(c.paula.intreq() & paula::int::VERTB, paula::int::VERTB);
}

#[test]
fn paulas_audio_channel_restarts_and_fetches_through_agnus() {
    let c = Chipset::pal();
    c.store(0x5000, &[0x7f80, 0x0102, 0x0304, 0x0506]);
    c.poke(0x0a0, 0x0000); // AUD0LCH: an Agnus register
    c.poke(0x0a2, 0x5000); // AUD0LCL
    c.poke(0x0a4, 2); // AUD0LEN: Paula's
    c.poke(0x0a6, 124); // AUD0PER
    c.poke(0x0a8, 64); // AUD0VOL
    assert_eq!(
        c.agnus.next_event_tick(),
        Some(PAL_FIELD),
        "nothing enabled"
    );
    c.poke(DMACON, dma::SETCLR | dma::DMAEN | DmaChannel::AUD0.0);
    assert_eq!(c.agnus.next_event_tick(), Some(227), "a slot Paula can see");

    c.run(227);
    let dma = c.agnus.dma();
    assert_eq!(
        dma.audio_pointer(0),
        0x5002,
        "Paula asked for a restart and a word on starting, and got them"
    );
    c.run(20 * 227);
    let p = dma.audio_pointer(0);
    assert!(p > 0x5002, "and keeps asking as it plays: {p:#x}");
    assert!(
        p <= 0x5000 + 2 * 2,
        "a length of two words restarts from AUDxLC: {p:#x}"
    );
    assert_ne!(
        c.paula.intreq() & paula::int::AUD0,
        0,
        "the start interrupt"
    );
}

#[test]
fn a_disk_write_takes_its_words_from_dskpt() {
    let c = Chipset::pal();
    c.store(0x6000, &[0x4489, 0x4489, 0x2aaa, 0x5555]);
    c.poke(DSKPTH, 0x0000);
    c.poke(DSKPTL, 0x6000);
    c.poke(DMACON, dma::SETCLR | dma::DMAEN | DmaChannel::DISK.0);
    // DSKLEN: DKEN, WRITE, four words — "turned on twice".
    c.poke(0x024, 0xc004);
    c.poke(0x024, 0xc004);
    c.run(20 * 227);
    assert_eq!(c.agnus.dma().disk_pointer(), 0x6008, "four words taken");
    assert_ne!(
        c.paula.intreq() & paula::int::DSKBLK,
        0,
        "and Paula finished the block"
    );
}

// ---------------------------------------------------------------------------
// the Enhanced Chip Set
// ---------------------------------------------------------------------------

// Appendix C's ECS register table, the rows this file has no name for yet.
const E_HTOTAL: u16 = 0x1c0;
const E_HSSTOP: u16 = 0x1c2;
const E_HBSTRT: u16 = 0x1c4;
const E_HBSTOP: u16 = 0x1c6;
const E_VTOTAL: u16 = 0x1c8;
const E_VSSTOP: u16 = 0x1ca;
const E_VBSTRT: u16 = 0x1cc;
const E_VBSTOP: u16 = 0x1ce;
const E_HSSTRT: u16 = 0x1de;
const E_VSSTRT: u16 = 0x1e0;

/// An 8372A (1 MiB) or an 8375 (2 MiB).
fn ecs_board(std: Standard, reach: u64) -> Board {
    Board::on(Agnus::part(Revision::Ecs { reach }, std))
}

#[test]
fn an_ecs_part_identifies_itself_in_vposr_with_lol_and_v10_v9() {
    // "8368 (hr) or 8372 (fat-hr) = 20 for PAL, 30 for NTSC", and the later
    // 2 MiB part's 21 and 31.
    for (std, reach, id) in [
        (Standard::Pal, ecs::MIB, 0x20),
        (Standard::Ntsc, ecs::MIB, 0x30),
        (Standard::Pal, 2 * ecs::MIB, 0x21),
        (Standard::Ntsc, 2 * ecs::MIB, 0x31),
    ] {
        let b = ecs_board(std, reach);
        assert_eq!(b.peek(VPOSR), 0x8000 | (id << 8), "{std:?} {reach}");
    }
    // "LOF I6-I0 LOL -- -- -- -- v10 v9 V8": NTSC's second line is long.
    let b = ecs_board(Standard::Ntsc, ecs::MIB);
    b.run(227);
    assert_eq!(b.peek(VPOSR), 0xb080, "LOL on the long line");
    // VPOSW writes V10-V8 on an ECS part, V8 alone on the original.
    b.poke(VPOSW, 0x8006);
    assert_eq!(b.peek(VPOSR) & 7, 6);
    let ocs = Board::new(Standard::Ntsc);
    ocs.poke(VPOSW, 0x8006);
    assert_eq!(ocs.peek(VPOSR) & 7, 0, "V10 and V9 are not there to write");
}

#[test]
fn beamcon0_comes_out_of_reset_with_the_strap_and_switches_the_standard() {
    let b = ecs_board(Standard::Pal, ecs::MIB);
    assert_eq!(b.agnus.next_event_tick(), Some(PAL_FIELD), "a PAL field");
    // PAL clear: 263 lines, alternating 227 and 228 from a short first line.
    b.poke(BEAMCON0, 0);
    assert_eq!(b.agnus.next_event_tick(), Some(263 * 227 + 131));
    b.poke(BEAMCON0, ecs::LOLDIS);
    assert_eq!(
        b.agnus.next_event_tick(),
        Some(263 * 227),
        "LOLDIS stops the toggle"
    );
    assert_eq!(b.peek(VPOSR) >> 8 & 0x7f, 0x20, "the strap is still PAL");
    b.poke(BEAMCON0, ecs::PAL);
    assert_eq!(b.agnus.next_event_tick(), Some(PAL_FIELD));
}

#[test]
fn varbeamen_builds_productivity_modes_beam_from_htotal_and_vtotal() {
    let b = ecs_board(Standard::Pal, ecs::MIB);
    b.poke(E_HTOTAL, 113);
    b.poke(E_VTOTAL, 524);
    b.poke(BEAMCON0, ecs::PAL | ecs::VARBEAMEN | ecs::LOLDIS);
    assert_eq!(b.agnus.next_event_tick(), Some(525 * 114));
    b.run(520 * 114 + 3);
    assert_eq!(b.peek(VHPOSR), 0x0803, "line 520 = $208, count 3");
    assert_eq!(b.peek(VPOSR) & 7, 2, "V9 of line $208");
    b.run(5 * 114 - 3);
    assert_eq!((b.agnus.beam().vpos, b.agnus.beam().field), (0, 1));
    // Interlaced, the long field is VTOTAL + 2 lines and the short one + 1.
    b.poke(BPLCON0, LACE);
    assert_eq!(b.agnus.next_event_tick(), Some(b.agnus.ticks() + 526 * 114));
}

#[test]
fn an_original_part_holds_the_ecs_registers_and_counts_as_it_always_did() {
    let b = Board::pal();
    for (offset, value) in [
        (E_HTOTAL, 113),
        (E_VTOTAL, 524),
        (E_HSSTRT, 100),
        (E_HSSTOP, 110),
        (BEAMCON0, 0x1ba0),
        (DIWHIGH, 0x0100),
    ] {
        b.poke(offset, value);
    }
    assert_eq!(b.agnus.next_event_tick(), Some(PAL_FIELD));
    assert_eq!(b.peek(VPOSR), 0x8000, "an 8371 still");
    let st = b.agnus.shared.state.lock();
    assert_eq!(st.ecs[ecs::BEAMCON0], 0x1ba0, "held, for a snapshot");
    assert!(!st.diwhigh_on, "and not acted on");
}

#[test]
fn programmed_sync_moves_both_pins() {
    let b = ecs_board(Standard::Pal, ecs::MIB);
    b.poke(E_HSSTRT, 100);
    b.poke(E_HSSTOP, 110);
    b.poke(E_VSSTRT, 10);
    b.poke(E_VSSTOP, 12);
    b.poke(BEAMCON0, ecs::PAL | ecs::VARHSYEN | ecs::VARVSYEN);
    // Wired now: out of reset the hardwired windows hold both pins high at
    // count 0 of line 0, and those would be edges of their own.
    let hsync = b.wire(HSYNC_PIN, 1);
    let vsync = b.wire(VSYNC_PIN, 2);
    assert!(
        !hsync.level() && !vsync.level(),
        "count 0 of line 0 is outside both"
    );
    b.run(100);
    assert!(hsync.level(), "HSSTRT");
    b.run(10);
    assert!(!hsync.level(), "HSSTOP");
    b.run(10 * 227 - 110);
    assert!(vsync.level(), "VSSTRT");
    b.run(2 * 227);
    assert!(!vsync.level(), "VSSTOP");
    b.run(PAL_FIELD - 12 * 227);
    assert_eq!((hsync.rises(), vsync.rises()), (313, 1), "a field of each");
}

#[test]
fn superhires_fetches_four_words_a_block_on_an_ecs_part_only() {
    for (board, words) in [(ecs_board(Standard::Pal, ecs::MIB), 40), (Board::pal(), 10)] {
        board.poke(DDFSTRT, 0x18);
        board.poke(DDFSTOP, 0x60);
        board.poke(BPLCON0, 0x2240); // two planes, COLOR, SHRES
        assert_eq!(
            board.agnus.shared.state.lock().fetch_window(),
            Some((0x18, words))
        );
    }
}

#[test]
fn diwhigh_moves_the_vertical_fetch_window_until_diwstrt_or_diwstop_is_written() {
    let window = |b: &Board, v| b.agnus.shared.state.lock().in_vertical_window(v);
    let b = ecs_board(Standard::Pal, ecs::MIB);
    b.poke(DIWSTRT, 0x1e35);
    b.poke(DIWSTOP, 0xfed5);
    assert!(
        !window(&b, 0x1fd),
        "the old scheme: $FE has V7 set, so V8 is clear"
    );
    b.poke(DIWHIGH, 0x0100);
    assert!(
        window(&b, 0x1fd) && !window(&b, 0x1fe),
        "stop V8 from DIWHIGH"
    );
    b.poke(DIWSTRT, 0x1e35);
    assert!(!window(&b, 0x1fd), "written again: the old scheme");

    let ocs = Board::pal();
    ocs.poke(DIWSTRT, 0x1e35);
    ocs.poke(DIWSTOP, 0xfed5);
    ocs.poke(DIWHIGH, 0x0100);
    assert!(!window(&ocs, 0x1fd), "an 8371 has no DIWHIGH");
}

#[test]
fn an_ecs_copper_has_appendix_cs_permission() {
    let b = ecs_board(Standard::Pal, ecs::MIB);
    // Without CDANG the blitter block is open, and $3E down is not.
    b.store(0x5000, &[BLTCON0, 0x09f0, COPCON, 0x0002, 0xffff, 0xfffe]);
    b.start_copper(0x5000);
    b.run(100);
    assert_eq!(
        b.agnus.shared.state.lock().blitter.con0,
        0x09f0,
        "BLTCON0 without CDANG"
    );
    assert_eq!(
        b.agnus.shared.state.lock().copper.phase,
        Phase::Halted,
        "and COPCON halts it"
    );
    assert_eq!(b.custom.bus().refused_copper_writes(), 1);
    // With it, "all of the Amiga chip registers": DSKPTH among them.
    b.store(0x5100, &[DSKPTH, 0x0001, 0xffff, 0xfffe]);
    b.poke(COPCON, CDANG);
    b.poke(COP1LCL, 0x5100);
    b.poke(COPJMP1, 0);
    b.run(100);
    assert_eq!(b.agnus.dma().disk_pointer(), 0x0001_0000);
    assert_eq!(
        b.custom.bus().refused_copper_writes(),
        1,
        "nothing more refused"
    );
}

#[test]
fn the_reach_is_the_parts_and_a_bigger_ram_is_refused() {
    use crate::core::props::{Link, Value};
    let props = |rev: &str| {
        Props::new()
            .with("custom", Value::Link(Link::new("custom").unwrap()))
            .with("ram", Value::Link(Link::new("chipram").unwrap()))
            .with("revision", Value::from(rev))
    };
    assert_eq!(
        Agnus::new(&props("ecs")).unwrap().revision(),
        Revision::Ecs { reach: ecs::MIB },
        "an 8372A unless told otherwise"
    );
    let two = props("ecs").with("reach", Value::Size(2 * ecs::MIB));
    assert_eq!(
        Agnus::new(&two).unwrap().revision(),
        Revision::Ecs {
            reach: 2 * ecs::MIB
        }
    );
    assert!(Agnus::new(&props("ecs").with("reach", Value::Size(4 * ecs::MIB))).is_err());
    assert!(Agnus::new(&props("ocs").with("reach", Value::Size(ecs::MIB))).is_err());

    // Alice's pointers are twenty bits, so her reach is 2 MiB and is not a
    // property: the value may be written, and only that value.
    assert_eq!(Agnus::new(&props("aga")).unwrap().revision(), Revision::Aga);
    assert_eq!(
        Agnus::new(&props("aga").with("reach", Value::Size(2 * ecs::MIB)))
            .unwrap()
            .revision(),
        Revision::Aga
    );
    assert!(Agnus::new(&props("aga").with("reach", Value::Size(ecs::MIB))).is_err());
    assert_eq!(Revision::Aga.reach(), 2 * ecs::MIB);

    let agnus = Agnus::part(Revision::Ecs { reach: ecs::MIB }, Standard::Pal);
    let ram: RegionRef = Arc::new(Region::ram("chip", Arc::new(RamStore::new(2 * ecs::MIB))));
    assert!(agnus.attach_ram(&ram).is_err(), "2 MiB on an 8372A");
    // Alice takes it.
    let alice = Agnus::part(Revision::Aga, Standard::Pal);
    assert!(alice.attach_ram(&ram).is_ok());
    let four: RegionRef = Arc::new(Region::ram("chip", Arc::new(RamStore::new(4 * ecs::MIB))));
    assert!(
        Agnus::part(Revision::Aga, Standard::Pal)
            .attach_ram(&four)
            .is_err(),
        "4 MiB is past twenty bits of pointer"
    );
}

#[test]
fn an_ecs_snapshot_round_trips_with_its_beam_and_window() {
    let setup = |b: &Board| {
        b.poke(E_HTOTAL, 113);
        b.poke(E_VTOTAL, 524);
        b.poke(E_HBSTRT, 110);
        b.poke(E_HBSTOP, 20);
        b.poke(E_VBSTRT, 510);
        b.poke(E_VSSTOP, 5);
        b.poke(BEAMCON0, ecs::PAL | ecs::VARBEAMEN | ecs::LOLDIS);
        b.poke(DIWSTRT, 0x1e35);
        b.poke(DIWSTOP, 0xfed5);
        b.poke(DIWHIGH, 0x0100);
    };
    let saved = ecs_board(Standard::Pal, 2 * ecs::MIB);
    setup(&saved);
    saved.run(300 * 114 + 7);
    let bytes = snapshot(&saved.agnus);

    let restored = ecs_board(Standard::Pal, 2 * ecs::MIB);
    restore(&restored.agnus, &bytes);
    assert_eq!(
        snapshot(&restored.agnus),
        bytes,
        "identical after a round trip"
    );
    assert!(restored.agnus.shared.state.lock().diwhigh_on);
    assert_eq!(
        restored.agnus.next_event_tick(),
        saved.agnus.next_event_tick(),
        "the programmed field survives"
    );
    saved.run(1000);
    restored.run(1000);
    assert_eq!(snapshot(&restored.agnus), snapshot(&saved.agnus));
}

#[test]
fn an_ecs_part_hands_denise_the_raster_it_programmed() {
    use crate::dev::amiga::denise::Raster;
    let agnus = Agnus::part(Revision::Ecs { reach: ecs::MIB }, Standard::Pal);
    let custom = Custom::new(&Props::new()).unwrap();
    agnus.attach_bus(custom.bus()).unwrap();
    let ram: RegionRef = Arc::new(Region::ram("chip", Arc::new(RamStore::new(CHIP))));
    agnus.attach_ram(&ram).unwrap();
    let video = Arc::new(Video::with_revision(
        denise::Standard::Pal,
        denise::Revision::Ecs,
    ));
    custom
        .bus()
        .attach(Arc::clone(&video) as Arc<dyn CustomChip>)
        .unwrap();
    agnus.attach_video(Arc::clone(&video));
    let poke = |o: u16, v: u16| assert!(custom.bus().write(o, v, Origin::cpu()));

    // Out of reset: the original chip set's picture exactly.
    agnus.advance_to(PAL_FIELD);
    assert_eq!(
        video.current_raster(),
        Raster::standard(denise::Standard::Pal)
    );
    assert_eq!(video.geometry(), (800, 568));

    poke(E_HTOTAL, 113);
    poke(E_VTOTAL, 524);
    poke(E_HBSTRT, 110);
    poke(E_HBSTOP, 20);
    poke(E_VBSTRT, 510);
    poke(E_VBSTOP, 30);
    poke(
        BEAMCON0,
        ecs::PAL | ecs::VARBEAMEN | ecs::VARVBEN | ecs::HARDDIS | ecs::LOLDIS,
    );
    // The rest of this PAL field, then one programmed one.
    agnus.advance_to(2 * PAL_FIELD + 525 * 114);
    let raster = video.current_raster();
    assert_eq!(
        (
            raster.first_line,
            raster.lines,
            raster.first_clock,
            raster.clocks
        ),
        (30, 480, 20, 90)
    );
    assert_eq!(video.geometry(), (2 * 180, 480), "high-resolution columns");
    assert_eq!(video.field_clocks(), 525 * 114);
}

// ---------------------------------------------------------------------------
// Alice: the AA chip set's Agnus
//
// Every expectation is the *Specification for the Advanced Amiga (AA) Chip
// Set*'s (Commodore-Amiga), cited by section. `aga`'s own unit tests cover the
// arithmetic; these drive the whole chip, and the ones with a picture drive it
// into a real Lisa.
// ---------------------------------------------------------------------------

/// `FMODE`, Alice's and Lisa's both (§3: "FMODE p 1FC W A D").
const E_FMODE: u16 = 0x1fc;
const BPL7PTH: u16 = 0x0f8;
const BPL8PTH: u16 = 0x0fc;
const SPR0PTR: u16 = 0x120;
/// `BPLCON3`, whose `BANK` and `PF2OF` the colour-table setup below writes.
const BPLCON3: u16 = 0x106;
/// `BPLCON3`'s reset `PF2OF = 011` (§4).
const PF2OF: u16 = 0b011 << 10;

fn alice(std: Standard) -> Board {
    Board::on(Agnus::part(Revision::Aga, std))
}

#[test]
fn alice_identifies_herself_in_vposr() {
    // §4, VPOSR: "8374(alice) = 22 PAL, 32 NTSC".
    assert_eq!(alice(Standard::Pal).peek(VPOSR), 0x8000 | 0x22 << 8);
    assert_eq!(alice(Standard::Ntsc).peek(VPOSR), 0x8000 | 0x32 << 8);
    // And she has everything an ECS part has: V10 and V9 beside V8.
    let b = alice(Standard::Pal);
    b.poke(VPOSW, 0x0007);
    assert_eq!(b.peek(VPOSR) & 0x0007, 0x0007, "V10-V8 all written");
}

#[test]
fn the_bitplane_seven_and_eight_pointers_are_alices_alone() {
    // §3: BPL7PTH $0F8 … BPL8PTL $0FE, with `P` in the rev column. An older
    // part has no register there, so the write is held nowhere.
    let aga = alice(Standard::Pal);
    let ecs = ecs_board(Standard::Pal, 2 * ecs::MIB);
    for b in [&aga, &ecs] {
        b.poke(BPL7PTH, 0x0002);
        b.poke(BPL7PTH + 2, 0x0000);
        b.poke(BPL8PTH, 0x0003);
        b.poke(BPL8PTH + 2, 0x0000);
    }
    assert_eq!(
        aga.agnus.shared.state.lock().bplpt[6..],
        [0x2_0000, 0x3_0000],
        "Alice holds them"
    );
    assert_eq!(
        ecs.agnus.shared.state.lock().bplpt[6..],
        [0, 0],
        "an 8375 does not"
    );
}

#[test]
fn eight_planes_are_fetched_with_their_own_pointers_and_the_two_modulos() {
    let c = Chipset::aga(CHIP);
    // One word a line for each of eight planes, each plane's data marked with
    // its own number so the streams cannot be confused.
    for p in 0..8u32 {
        let base = 0x1_0000 + 0x1000 * p;
        c.store(base, &[0x1000 + p as u16, 0x2000 + p as u16]);
        c.poke(BPL1PTH + 4 * p as u16, (base >> 16) as u16);
        c.poke(BPL1PTH + 4 * p as u16 + 2, base as u16);
    }
    c.poke(DIWSTRT, 0x2c81);
    c.poke(DIWSTOP, 0x2cc1);
    c.poke(DDFSTRT, 0x38);
    c.poke(DDFSTOP, 0x38); // one eight-count block: one word a line
    c.poke(BPL1MOD, 0u16.wrapping_sub(2)); // odd planes stay put
    c.poke(BPL2MOD, 0); // even planes walk forward
    // BPU = 8: BPU2-BPU0 are zero and BPU3 is bit 4 (§4, BPLCON0).
    c.poke(BPLCON0, 1 << 4);
    c.poke(DMACON, dma::SETCLR | dma::DMAEN | DmaChannel::BITPLANE.0);
    c.run(PAL_FIELD);

    let st = c.agnus.shared.state.lock();
    for p in 0..8usize {
        let base = 0x1_0000 + 0x1000 * p as u32;
        // 256 lines of one word. An odd plane (1, 3, 5, 7 — index 0, 2, 4, 6)
        // adds BPL1MOD = −2 after each two-byte word and comes back to where
        // it started; an even one adds nothing and has walked 512 bytes.
        let want = if p % 2 == 0 { base } else { base + 2 * 256 };
        assert_eq!(st.bplpt[p], want, "plane {}", p + 1);
    }
}

#[test]
fn bpu_eight_puts_both_new_planes_in_the_picture() {
    let c = Chipset::aga(CHIP);
    // Planes 1-7 all zero, plane 8 all ones: colour 128 everywhere in the
    // window, which is the 256-entry table's second half (§2, *Bitplanes*).
    for p in 0..8u32 {
        let base = 0x1_0000 + 0x2000 * p;
        c.store(base, &vec![if p == 7 { 0xffff } else { 0 }; 32 * 256]);
        c.poke(BPL1PTH + 4 * p as u16, (base >> 16) as u16);
        c.poke(BPL1PTH + 4 * p as u16 + 2, base as u16);
    }
    c.poke(DIWSTRT, 0x2c81);
    c.poke(DIWSTOP, 0x2cc1);
    c.poke(DDFSTRT, 0x38);
    c.poke(DDFSTOP, 0xd0);
    c.poke(BPL1MOD, 0);
    c.poke(BPL2MOD, 0);
    // COLOR00 black, and colour 128 — bank 4, entry 0 — red.
    c.poke(BPLCON3, PF2OF);
    c.poke(COLOR00, 0x0000);
    c.poke(BPLCON3, 4 << 13 | PF2OF);
    c.poke(COLOR00, 0x0f00);
    c.poke(BPLCON3, PF2OF);
    c.poke(BPLCON0, 1 << 4);
    c.poke(DMACON, dma::SETCLR | dma::DMAEN | DmaChannel::BITPLANE.0);
    c.run(PAL_FIELD);

    // Lisa's picture is 35 ns quarters: the window's first pixel is at
    // (0x81 − 64) × 4.
    let mut row = vec![0u32; 1600];
    c.video.read_row_rgb(2 * (0x2c - 0x1d), &mut row);
    let at = (0x81 - 64) * 4;
    assert_eq!(row[at - 1], 0x0000_0000, "the border is colour 0");
    assert!(
        row[at..at + 320 * 4].iter().all(|&p| p == 0x00ff_0000),
        "plane 8 alone selects colour 128"
    );
}

#[test]
fn each_fmode_width_moves_that_many_words_a_transfer() {
    // §4's FMODE table, and `aga`'s rounding inference: the word count a line
    // is the window's, rounded up to a whole transfer.
    for (fmode, f) in [
        (0u16, 1u16),
        (aga::BPL32, 2),
        (aga::BPAGEM, 2),
        (aga::BPAGEM | aga::BPL32, 4),
    ] {
        let b = alice(Standard::Pal);
        b.poke(E_FMODE, fmode);
        b.poke(DDFSTRT, 0x38);
        b.poke(DDFSTOP, 0xd0); // twenty eight-count blocks: 20 words in LORES
        let st = b.agnus.shared.state.lock();
        assert_eq!(st.fetch_words(), f, "FMODE ${fmode:04x}");
        let (_, words) = st.fetch_window().expect("a window");
        assert_eq!(words, 20u16.div_ceil(f) * f, "FMODE ${fmode:04x}");
        assert_eq!(words % f, 0, "a transfer is indivisible");
    }
    // A window that does not divide: 21 blocks with a four-word transfer is
    // 24 words, six transfers.
    let b = alice(Standard::Pal);
    b.poke(E_FMODE, aga::BPAGEM | aga::BPL32);
    b.poke(DDFSTRT, 0x38);
    b.poke(DDFSTOP, 0xd8);
    assert_eq!(b.agnus.shared.state.lock().fetch_window(), Some((0x38, 24)));
}

#[test]
fn a_wide_fetch_is_the_same_stream_and_moves_the_pointer_by_the_words() {
    // §4, BPLxDAT: a fetch of any width is that many consecutive pixels, "MSB
    // … always on the left". So a 64-bit fetch of four words shows exactly
    // what four 16-bit fetches of the same words show.
    let narrow = Chipset::aga(CHIP);
    let wide = Chipset::aga(CHIP);
    let row = |c: &Chipset| {
        let mut row = vec![0u32; 1600];
        c.video.read_row_rgb(2 * (0x2c - 0x1d), &mut row);
        row
    };
    for (c, fmode) in [(&narrow, 0u16), (&wide, aga::BPAGEM | aga::BPL32)] {
        c.store(0x1_0000, &vec![0xf0f0u16; 8 * 256]);
        c.poke(BPL1PTH, 0x0001);
        c.poke(BPL1PTH + 2, 0x0000);
        c.poke(DIWSTRT, 0x2c81);
        c.poke(DIWSTOP, 0x2cc1);
        c.poke(DDFSTRT, 0x38);
        c.poke(DDFSTOP, 0x50); // four eight-count blocks: four words
        c.poke(BPL1MOD, 0);
        c.poke(BPLCON3, PF2OF);
        c.poke(COLOR00, 0x0000);
        c.poke(COLOR01, 0x0f00);
        c.poke(BPLCON0, 1 << 12);
        c.poke(E_FMODE, fmode);
        c.poke(DMACON, dma::SETCLR | dma::DMAEN | DmaChannel::BITPLANE.0);
        c.run(PAL_FIELD);
    }
    assert_eq!(
        row(&narrow),
        row(&wide),
        "one transfer or four, same pixels"
    );
    // And both pointers have walked the same four words a line. Read one at a
    // time: two `DEVICE`-ranked locks at once is a rank violation, and the
    // rank check is half of why these tests are unit tests.
    let one = narrow.agnus.shared.state.lock().bplpt[0];
    let two = wide.agnus.shared.state.lock().bplpt[0];
    assert_eq!(one, two);
}

#[test]
fn an_older_part_has_no_fmode_and_fetches_as_it_always_did() {
    // The register decodes on every part — it is in the address map — but only
    // Alice acts on it. This is the regression gate for every OCS and ECS
    // golden.
    let all = aga::BPAGEM | aga::BPL32 | aga::BSCAN2 | aga::SSCAN2;
    for b in [Board::pal(), ecs_board(Standard::Pal, 2 * ecs::MIB)] {
        b.poke(E_FMODE, all);
        b.poke(DDFSTRT, 0x3a);
        b.poke(DDFSTOP, 0xd8);
        b.poke(BPLCON0, 7 << 12 | 1 << 4);
        let st = b.agnus.shared.state.lock();
        assert_eq!(st.fetch_words(), 1);
        assert_eq!(st.fetch_window(), Some((0x38, 21)), "H8-H3, 21 words");
        assert_eq!(st.planes(), 6, "BPU3 is not a bit this part has");
        assert_eq!(st.fmode, all, "held, so a snapshot carries it");
    }
}

#[test]
fn an_aa_fetch_window_decodes_h2() {
    // §4, DDFSTRT: "H8 H7 H6 H5 H4 H3 H2 X" against bits 7-0, one further
    // down than the Enhanced Chip Set's. H2 is two colour clocks.
    let b = alice(Standard::Pal);
    b.poke(DDFSTRT, 0x3a);
    b.poke(DDFSTOP, 0xd0);
    assert_eq!(b.agnus.shared.state.lock().fetch_window(), Some((0x3a, 20)));
}

#[test]
fn bscan2_takes_the_modulus_from_the_lines_parity() {
    // §2: "When V0 bit of DIWSTRT matches V0 of vertical beam counter, BPL1MOD
    // contains the modulus for the display line, else BPL2MOD is used. When
    // scan-doubled both odd and even bitplanes use the same modulus."
    let c = Chipset::aga(CHIP);
    for p in 0..2u32 {
        let base = 0x1_0000 + 0x4000 * p;
        c.store(base, &vec![0u16; 4096]);
        c.poke(BPL1PTH + 4 * p as u16, (base >> 16) as u16);
        c.poke(BPL1PTH + 4 * p as u16 + 2, base as u16);
    }
    c.poke(DIWSTRT, 0x2c81); // V0 of $2C is 0: even lines are primary
    c.poke(DIWSTOP, 0x2ec1);
    // DIWSTOP's ninth bit is the complement of its eighth, so $2E alone would
    // mean line $12E; DIWHIGH written last gives V10-V8 directly (Appendix C),
    // and two display lines, $2C and $2D.
    c.poke(DIWHIGH, 0x0000);
    c.poke(DDFSTRT, 0x38);
    c.poke(DDFSTOP, 0x38); // one word a line
    c.poke(BPL1MOD, 0u16.wrapping_sub(2)); // primary: stay put
    c.poke(BPL2MOD, 30); // alternate: skip on
    c.poke(E_FMODE, aga::BSCAN2);
    c.poke(BPLCON0, 2 << 12);
    c.poke(DMACON, dma::SETCLR | dma::DMAEN | DmaChannel::BITPLANE.0);
    c.run(PAL_FIELD);

    let st = c.agnus.shared.state.lock();
    for p in 0..2usize {
        let base = 0x1_0000 + 0x4000 * p as u32;
        // Line $2C is primary (−2 after a two-byte word: no movement); line
        // $2D is alternate (+30 after two bytes: 32 on). Both planes alike.
        assert_eq!(st.bplpt[p], base + 32, "plane {}", p + 1);
    }
}

#[test]
fn without_bscan2_the_modulus_is_still_the_planes_own() {
    let c = Chipset::aga(CHIP);
    for p in 0..2u32 {
        let base = 0x1_0000 + 0x4000 * p;
        c.store(base, &vec![0u16; 4096]);
        c.poke(BPL1PTH + 4 * p as u16, (base >> 16) as u16);
        c.poke(BPL1PTH + 4 * p as u16 + 2, base as u16);
    }
    c.poke(DIWSTRT, 0x2c81);
    c.poke(DIWSTOP, 0x2ec1);
    c.poke(DIWHIGH, 0x0000); // lines $2C and $2D, as above
    c.poke(DDFSTRT, 0x38);
    c.poke(DDFSTOP, 0x38);
    c.poke(BPL1MOD, 0u16.wrapping_sub(2));
    c.poke(BPL2MOD, 30);
    c.poke(BPLCON0, 2 << 12);
    c.poke(DMACON, dma::SETCLR | dma::DMAEN | DmaChannel::BITPLANE.0);
    c.run(PAL_FIELD);
    let st = c.agnus.shared.state.lock();
    assert_eq!(st.bplpt[0], 0x1_0000, "plane 1: BPL1MOD twice over");
    assert_eq!(st.bplpt[1], 0x1_4000 + 64, "plane 2: BPL2MOD twice over");
}

#[test]
fn a_wide_sprite_is_fetched_whole_and_reaches_lisa_left_justified() {
    // §4, FMODE: a sprite fetch moves 2, 4 or 8 bytes, and §5: "Sprites are
    // either 16, 32, or 64 bits wide". The words go to Lisa through
    // `Video::sprite_dma`, "MSB first on the left" (§4, SPRxDATA).
    for (fmode, f) in [
        (0u16, 1usize),
        (aga::SPR32, 2),
        (aga::SPAGEM, 2),
        (aga::SPAGEM | aga::SPR32, 4),
    ] {
        let c = Chipset::aga(CHIP);
        let base = 0x2_0000u32;
        // Control words for one line at $2C, then one line of data: `f` words
        // of A and `f` words of B, each marked so the order is visible.
        let mut words = vec![0u16; 2 * f];
        words[0] = 0x2c40; // SPRxPOS: VSTART $2C, HSTART $40
        words[f] = 0x2d00; // SPRxCTL: VSTOP $2D
        let mut data = vec![0u16; 2 * f];
        for i in 0..f {
            data[i] = 0xa000 + i as u16;
            data[f + i] = 0xb000 + i as u16;
        }
        c.store(base, &words);
        c.store(base + 4 * f as u32, &data);
        c.poke(E_FMODE, fmode);
        c.poke(SPR0PTR, (base >> 16) as u16);
        c.poke(SPR0PTR + 2, base as u16);
        c.poke(DMACON, dma::SETCLR | dma::DMAEN | DmaChannel::SPRITE.0);
        c.run(PAL_FIELD);

        // Six transfers: the control pair at vertical blank's end, the data
        // pair on line $2C, and the control pair again on the VSTOP line.
        assert_eq!(
            c.agnus.shared.state.lock().sprpt[0],
            base + 12 * f as u32,
            "FMODE ${fmode:04x}: six transfers of {f} words"
        );
    }
}

#[test]
fn sscan2_skips_a_sprite_fetch_on_a_line_of_the_wrong_parity() {
    // §2, *Sprites*: "When V0 bit of SPRxPOS register matches V0 bit of
    // vertical beam counter, the given sprite's DMA is allowed to proceed as
    // before. If they don't match, then sprite DMA is disabled and LISA reuses
    // the sprite data from the previous line."
    for (fmode, sh10, lines) in [
        (aga::SSCAN2, 1u16, 2u32), // doubled: four lines, two fetches
        (aga::SSCAN2, 0, 4),       // SH10 clear: this sprite is not doubled
        (0, 1, 4),                 // SSCAN2 clear: nor is any
    ] {
        let c = Chipset::aga(CHIP);
        let base = 0x2_0000u32;
        // VSTART $2C, VSTOP $30 — four display lines, same parity (§2's note).
        c.store(base, &[0x2c40 | sh10, 0x3000]);
        c.store(base + 4, &[0u16; 64]);
        c.poke(E_FMODE, fmode);
        c.poke(SPR0PTR, (base >> 16) as u16);
        c.poke(SPR0PTR + 2, base as u16);
        c.poke(DMACON, dma::SETCLR | dma::DMAEN | DmaChannel::SPRITE.0);
        c.run(PAL_FIELD);
        // One control pair at each end plus one data pair per fetching line,
        // four bytes each.
        assert_eq!(
            c.agnus.shared.state.lock().sprpt[0],
            base + 8 + 4 * lines,
            "FMODE ${fmode:04x}, SH10 {sh10}"
        );
    }
}

#[test]
fn two_mib_of_chip_ram_is_addressed_by_twenty_bit_pointers() {
    // §3's preamble: "PTL,PTH=20 bit Pointer that addresses DMA data … (old
    // chips- 18 bits)", so the pair carries address bits 1-20 and reaches
    // 2 MiB.
    let c = Chipset::aga(2 * ecs::MIB);
    let base = 0x1f_0000u32;
    c.store(base, &[0xffffu16; 64]);
    c.poke(BPL1PTH, (base >> 16) as u16);
    c.poke(BPL1PTH + 2, base as u16);
    c.poke(DIWSTRT, 0x2c81);
    c.poke(DIWSTOP, 0x2cc1);
    c.poke(DDFSTRT, 0x38);
    c.poke(DDFSTOP, 0x38);
    c.poke(BPL1MOD, 0u16.wrapping_sub(2));
    c.poke(BPLCON3, PF2OF);
    c.poke(COLOR00, 0x0000);
    c.poke(COLOR01, 0x0f00);
    c.poke(BPLCON0, 1 << 12);
    c.poke(DMACON, dma::SETCLR | dma::DMAEN | DmaChannel::BITPLANE.0);
    c.run(PAL_FIELD);
    let mut row = vec![0u32; 1600];
    c.video.read_row_rgb(2 * (0x2c - 0x1d), &mut row);
    let at = (0x81 - 64) * 4;
    assert!(
        row[at..at + 16 * 4].iter().all(|&p| p == 0x00ff_0000),
        "a word fetched from $1F0000 is on the screen"
    );
}

#[test]
fn an_alice_snapshot_round_trips_with_fmode_and_eight_pointers() {
    let setup = |b: &Board| {
        b.poke(
            E_FMODE,
            aga::BPAGEM | aga::BPL32 | aga::BSCAN2 | aga::SSCAN2,
        );
        for p in 0..8u16 {
            b.poke(BPL1PTH + 4 * p, 0x0001);
            b.poke(BPL1PTH + 4 * p + 2, 0x1000 + 0x100 * p);
        }
        b.poke(DIWSTRT, 0x2c81);
        b.poke(DIWSTOP, 0x2cc1);
        b.poke(BPLCON0, 1 << 4);
        b.poke(DMACON, dma::SETCLR | dma::DMAEN | DmaChannel::BITPLANE.0);
    };
    let saved = alice(Standard::Pal);
    setup(&saved);
    saved.run(300 * 227 + 11);
    let bytes = snapshot(&saved.agnus);

    let restored = alice(Standard::Pal);
    restore(&restored.agnus, &bytes);
    assert_eq!(snapshot(&restored.agnus), bytes, "identical state");
    {
        let st = restored.agnus.shared.state.lock();
        assert_eq!(
            st.fmode,
            aga::BPAGEM | aga::BPL32 | aga::BSCAN2 | aga::SSCAN2
        );
        for p in 0..8usize {
            assert_eq!(st.bplpt[p], 0x1_1000 + 0x100 * p as u32, "plane {p}");
        }
    }
    // And the two run on identically from there.
    saved.run(2 * PAL_FIELD);
    restored.run(2 * PAL_FIELD);
    assert_eq!(snapshot(&restored.agnus), snapshot(&saved.agnus));
}
