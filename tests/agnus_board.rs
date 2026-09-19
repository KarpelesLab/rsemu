//! Agnus on a board: the colour clock the machine file gives it, the sync wires
//! into the two CIAs' `TOD` inputs, and the copper and blitter reached through
//! the `$DFF000` window and chip RAM the processor wrote.
//!
//! `src/dev/amiga/agnus/tests.rs` has the unit tests, which drive the chip
//! directly and check the lock order. What they cannot prove is that any of it
//! survives the machine layer: that `clock = clk / 8` really is the horizontal
//! count, that `wire agnus.vsync -> cia_a.tod` counts fields on a real 8520,
//! and that the scheduler catches the chip up so a copper `WAIT` lands on its
//! line while a 68000 runs beside it.
//!
//! The board is `machines/tests/agnus-board.machine`. The ROM is built here: a
//! reset vector pair and `BRA *`. No Kickstart is in this repository.
//!
//! Every assertion is written from the *Amiga Hardware Reference Manual*: the
//! colour clock and line counts from chapter 2, the register layouts from
//! Appendix A, the CIA register numbers from the 8520 appendix.

#![cfg(all(
    feature = "cpu-m68k",
    feature = "dev-amiga-agnus",
    feature = "dev-mos8520"
))]

use std::sync::Arc;

use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ExportId;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::dev::amiga::custom::CustomBus;
use rsemu::dev::amiga::dma::ChipDma;
use rsemu::machine::{Machine, catalog};

/// The board.
const BOARD: &str = include_str!("../machines/tests/agnus-board.machine");

/// The custom-chip base.
const CUSTOM: u64 = 0xDF_F000;
const DMACONR: u64 = CUSTOM + 0x002;
const VPOSR: u64 = CUSTOM + 0x004;
const VHPOSR: u64 = CUSTOM + 0x006;
const BLTCON0: u64 = CUSTOM + 0x040;
const BLTCON1: u64 = CUSTOM + 0x042;
const BLTAFWM: u64 = CUSTOM + 0x044;
const BLTALWM: u64 = CUSTOM + 0x046;
const BLTAPTH: u64 = CUSTOM + 0x050;
const BLTDPTH: u64 = CUSTOM + 0x054;
const BLTSIZE: u64 = CUSTOM + 0x058;
const COP1LCH: u64 = CUSTOM + 0x080;
const COPJMP1: u64 = CUSTOM + 0x088;
const DMACON: u64 = CUSTOM + 0x096;
/// `BPL1MOD`, at `$108`: Agnus's, and **write-only** (Appendix B). Reading it
/// is the access that has nothing to answer it.
const BPL1MOD: u64 = CUSTOM + 0x108;

/// CIA-A answers at `$BFEr01` and CIA-B at `$BFDr00` (8520 appendix).
const CIA_A: u64 = 0xBF_E001;
const CIA_B: u64 = 0xBF_D000;
/// `PRA`, `DDRA`, and the TOD counter's three bytes, LSB first.
const PRA: u64 = 0x0;
const DDRA: u64 = 0x2;
const TOD_LSB: u64 = 0x8;
const TOD_MID: u64 = 0x9;
const TOD_MSB: u64 = 0xA;

/// The PAL colour clock: "3,546,895 Hz" (chapter 2).
const COLOUR_CLOCK: u64 = 3_546_895;
/// A PAL line, and a long PAL field, in counts.
const LINE: u64 = 227;
const FIELD: u64 = 313 * LINE;

/// `DMACON` bits: set/clear, `BLTPRI`, `DMAEN`, `COPEN`, `BLTEN`.
const SETCLR: u16 = 0x8000;
const BLTPRI: u16 = 0x0400;
const DMAEN: u16 = 0x0200;
const COPEN: u16 = 0x0080;
const BLTEN: u16 = 0x0040;
/// `DMACONR` bit 14.
const BBUSY: u16 = 0x4000;

/// A 512 KiB ROM: the supervisor stack at the top of chip RAM, the program
/// counter in the ROM's own window, and `BRA *` there (MC68000 user's manual,
/// *Instruction Set Details*). Pointing the PC at `$F80008` rather than into the
/// overlay means clearing `OVL` does not pull the code out from under the
/// processor.
fn rom() -> Vec<u8> {
    let mut image = vec![0u8; 512 * 1024];
    image[0..4].copy_from_slice(&0x0008_0000u32.to_be_bytes());
    image[4..8].copy_from_slice(&0x00F8_0008u32.to_be_bytes());
    image[8..10].copy_from_slice(&0x60FEu16.to_be_bytes());
    image
}

fn boot(tag: &str) -> Machine {
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("kickstart", rom());
    let registry = catalog::registry().expect("a registry");
    rsemu::machine::build(tag, BOARD, &registry, &options)
        .unwrap_or_else(|e| panic!("the board does not realize: {e}"))
}

fn peek_byte(m: &Machine, addr: u64) -> u8 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U8, MemAttrs::DEFAULT)
        .expect("a mapped byte") as u8
}

fn poke_byte(m: &Machine, addr: u64, value: u8) {
    m.space("mem")
        .expect("the memory space")
        .write(addr, Width::U8, u64::from(value), MemAttrs::DEFAULT)
        .expect("a mapped byte");
}

fn peek(m: &Machine, addr: u64) -> u16 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U16, MemAttrs::DEFAULT)
        .expect("a mapped word") as u16
}

fn poke(m: &Machine, addr: u64, value: u16) {
    m.space("mem")
        .expect("the memory space")
        .write(addr, Width::U16, u64::from(value), MemAttrs::DEFAULT)
        .expect("a mapped word");
}

fn poke_long(m: &Machine, addr: u64, value: u32) {
    poke(m, addr, (value >> 16) as u16);
    poke(m, addr + 2, value as u16);
}

/// Clear `OVL` the way Kickstart does: CIA-A's `PA0` made an output and driven
/// low, so chip RAM answers at zero.
fn clear_overlay(m: &Machine) {
    poke_byte(m, CIA_A + (DDRA << 8), 0x01);
    poke_byte(m, CIA_A + (PRA << 8), 0x00);
}

/// A CIA's TOD counter, read MSB first so the latch holds all three bytes and
/// the LSB read releases it (8520 data sheet).
fn tod(m: &Machine, cia: u64) -> u32 {
    let msb = peek_byte(m, cia + (TOD_MSB << 8));
    let mid = peek_byte(m, cia + (TOD_MID << 8));
    let lsb = peek_byte(m, cia + (TOD_LSB << 8));
    u32::from_be_bytes([0, msb, mid, lsb])
}

fn agnus_ticks(m: &Machine) -> u64 {
    m.device("agnus")
        .expect("the board has an agnus")
        .device()
        .current_tick()
}

fn dma(m: &Machine) -> Arc<ChipDma> {
    let export = m
        .device("agnus")
        .expect("the board has an agnus")
        .device()
        .export(ExportId::CHIP_DMA)
        .expect("Agnus publishes its DMA handle");
    Arc::clone(export.opaque().expect("an opaque handle"))
        .downcast::<ChipDma>()
        .expect("a ChipDma")
}

fn custom_bus(m: &Machine) -> Arc<CustomBus> {
    let export = m
        .device("custom")
        .expect("the board has a custom")
        .device()
        .export(ExportId::CUSTOM_BUS)
        .expect("published");
    Arc::clone(export.opaque().expect("an opaque handle"))
        .downcast::<CustomBus>()
        .expect("a CustomBus")
}

#[test]
fn one_second_of_the_board_is_the_pal_colour_clock_exactly() {
    let mut m = boot("agnus.clock");
    m.run_for(GlobalTime::from_nanos(1_000_000_000))
        .expect("it runs");
    assert_eq!(
        agnus_ticks(&m),
        COLOUR_CLOCK,
        "clk / 8 of 28 375 160 Hz, counted rather than computed"
    );
    // And the beam is where that many counts put it.
    let into_field = COLOUR_CLOCK % FIELD;
    let (vpos, hpos) = (into_field / LINE, into_field % LINE);
    assert_eq!(u64::from(peek(&m, VHPOSR) >> 8), vpos & 0xff);
    assert_eq!(u64::from(peek(&m, VHPOSR) & 0xff), hpos);
    assert_eq!(u64::from(peek(&m, VPOSR) & 1), vpos >> 8);
}

#[test]
fn vsync_counts_fields_on_cia_a_and_hsync_counts_lines_on_cia_b() {
    let mut m = boot("agnus.tod");
    let (a0, b0) = (tod(&m, CIA_A), tod(&m, CIA_B));

    // Just over a second: long enough for the fiftieth field to begin.
    m.run_for(GlobalTime::from_nanos(1_002_000_000))
        .expect("it runs");
    let ticks = agnus_ticks(&m);
    assert!(ticks >= 50 * FIELD, "{ticks}");

    assert_eq!(
        u64::from(tod(&m, CIA_A) - a0),
        ticks / FIELD,
        "one TOD count per field: fifty"
    );
    assert_eq!(
        u64::from(tod(&m, CIA_B) - b0),
        ticks / LINE,
        "one TOD count per line"
    );
    assert_eq!(ticks / FIELD, 50);
}

#[test]
fn a_copper_list_the_processor_wrote_runs_on_its_line() {
    let mut m = boot("agnus.copper");
    clear_overlay(&m);

    // WAIT for line $80, set BLTPRI through DMACON, and stop.
    let list: u64 = 0x1000;
    for (i, word) in [0x8001u16, 0xff00, 0x0096, SETCLR | BLTPRI, 0xffff, 0xfffe]
        .iter()
        .enumerate()
    {
        poke(&m, list + 2 * i as u64, *word);
    }
    poke_long(&m, COP1LCH, list as u32);
    poke(&m, COPJMP1, 0);
    poke(&m, DMACON, SETCLR | DMAEN | COPEN);

    // Line $80 begins at count 128 × 227 = 29 056: 8.19 ms in.
    m.run_for(GlobalTime::from_nanos(8_000_000))
        .expect("it runs");
    assert_eq!(peek(&m, DMACONR) & BLTPRI, 0, "not yet at line $80");
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");
    assert_eq!(
        peek(&m, DMACONR),
        BLTPRI | DMAEN | COPEN,
        "the MOVE landed, read back through the bus"
    );
    let at = dma(&m).beam();
    assert!(at.vpos >= 0x80, "{at:?}");
    assert_eq!(
        custom_bus(&m).unclaimed(),
        0,
        "every access on this board reached a chip"
    );
}

#[test]
fn a_blit_through_the_bus_copies_chip_ram_and_reports_busy_until_done() {
    let mut m = boot("agnus.blit");
    clear_overlay(&m);
    for i in 0..6u64 {
        poke(&m, 0x2000 + 2 * i, 0x1111 * (i as u16 + 1));
    }
    poke(&m, DMACON, SETCLR | DMAEN | BLTEN);
    poke(&m, BLTCON0, 0x0900 | 0xf0); // USEA, USED, D = A
    poke(&m, BLTCON1, 0);
    poke(&m, BLTAFWM, 0xffff);
    poke(&m, BLTALWM, 0xffff);
    poke_long(&m, BLTAPTH, 0x2000);
    poke_long(&m, BLTDPTH, 0x3000);
    poke(&m, BLTSIZE, (2 << 6) | 3);
    assert_eq!(peek(&m, DMACONR) & BBUSY, BBUSY, "busy from the write");

    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");
    assert_eq!(
        peek(&m, DMACONR) & BBUSY,
        0,
        "six words at four ticks is done"
    );
    let copied: Vec<u16> = (0..6).map(|i| peek(&m, 0x3000 + 2 * i)).collect();
    assert_eq!(copied, vec![0x1111, 0x2222, 0x3333, 0x4444, 0x5555, 0x6666]);
}

#[test]
fn the_board_runs_the_same_in_one_span_or_many() {
    let mut whole = boot("agnus.whole");
    let mut parts = boot("agnus.parts");
    for m in [&whole, &parts] {
        clear_overlay(m);
        poke(m, 0x1000, 0x4001);
        poke(m, 0x1002, 0xff00);
        poke(m, 0x1004, 0x0096);
        poke(m, 0x1006, SETCLR | BLTPRI);
        poke(m, 0x1008, 0xffff);
        poke(m, 0x100a, 0xfffe);
        poke_long(m, COP1LCH, 0x1000);
        poke(m, COPJMP1, 0);
        poke(m, DMACON, SETCLR | DMAEN | COPEN);
    }
    // Absolute deadlines, so that nanosecond rounding in the arithmetic of the
    // spans is not what gets compared.
    let end = GlobalTime::from_nanos(300_000_000);
    whole.run_until(end).expect("it runs");
    for nanos in [
        1,
        1_000_000,
        18_000_000,
        18_000_003,
        100_000_003,
        200_000_000,
    ] {
        parts
            .run_until(GlobalTime::from_nanos(nanos))
            .expect("it runs");
    }
    parts.run_until(end).expect("it runs");
    assert_eq!(whole.now(), parts.now());
    assert_eq!(agnus_ticks(&whole), agnus_ticks(&parts));
    assert_eq!(
        whole.state_hash().expect("hashable"),
        parts.state_hash().expect("hashable"),
        "run_for is additive on this board too (ROADMAP.md §11.6)"
    );
}

// ---------------------------------------------------------------------------
// the chip data bus
// ---------------------------------------------------------------------------

/// Reading a write-only register answers with the word the last chip-bus cycle
/// left on `D15`–`D0` — **once**.
///
/// Appendix B says which registers are readable and nothing about the rest;
/// Appendix C (p. 299), describing an 8362's absent `DENISEID`, calls the
/// answer "whatever value is left over on the bus from the last cycle" and
/// makes reading it the way to tell an 8362 from an 8373. Chapter 6, *Blitter
/// Operations and System DMA*, says whose cycles those are: Agnus arbitrates
/// every access to chip memory and hands the 68000 what the channels do not
/// want.
///
/// Here the last cycle is a **blitter D write of a word this test chose**, and
/// `BPL1MOD` — Agnus's own, write-only, so the chip that owns it is present
/// and still cannot answer — reads back exactly that word. The *second* read
/// gets nothing: that read was itself a cycle in which nothing drove the
/// lines, which is what makes Appendix C's test work.
#[test]
fn a_write_only_register_reads_back_the_last_cycles_word_once() {
    let mut m = boot("agnus.floating");
    clear_overlay(&m);
    let bus = custom_bus(&m);

    // Nothing has run a chip-bus cycle yet, so nothing is on the lines.
    assert_eq!(bus.floating(), 0, "no cycle has happened");
    assert_eq!(peek(&m, BPL1MOD), 0x0000);
    assert_eq!(bus.unclaimed(), 1, "the read fell through and was counted");

    // One word of source, and a one-word blit that copies it: A is read, D is
    // written, and the D write is the last cycle of the blit. Every register
    // write below puts its own word on the lines on the way past; the blit is
    // what leaves the last one.
    const WORD: u16 = 0xa55a;
    poke(&m, 0x2000, WORD);
    poke(&m, DMACON, SETCLR | DMAEN | BLTEN);
    poke(&m, BLTCON0, 0x0900 | 0xf0); // USEA, USED, D = A
    poke(&m, BLTCON1, 0);
    poke(&m, BLTAFWM, 0xffff);
    poke(&m, BLTALWM, 0xffff);
    poke_long(&m, BLTAPTH, 0x2000);
    poke_long(&m, BLTDPTH, 0x3000);
    poke(&m, BLTSIZE, (1 << 6) | 1);
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");

    // The word the blitter moved is what an unanswered read latches...
    assert_eq!(bus.floating(), WORD, "the blitter's D write");
    assert_eq!(peek(&m, BPL1MOD), WORD);
    // ...and that read left the lines floating, so the next one gets nothing.
    assert_eq!(peek(&m, BPL1MOD), 0x0000, "a second read is not the first");
    assert_eq!(bus.unclaimed(), 3);

    // A register a chip *does* answer drives the lines, so the next
    // write-only read finds that chip's word. `DMACONR` is Agnus's here.
    let dmaconr = peek(&m, DMACONR);
    assert_eq!(dmaconr, DMAEN | BLTEN, "the channels this test enabled");
    assert_eq!(peek(&m, BPL1MOD), dmaconr);
    // And a byte read keeps the half its strobe selects of that whole word.
    poke(&m, CUSTOM + 0x100, WORD); // BPLCON0: the write is the cycle
    assert_eq!(peek_byte(&m, BPL1MOD), (WORD >> 8) as u8);
    poke(&m, CUSTOM + 0x100, WORD);
    assert_eq!(peek_byte(&m, BPL1MOD + 1), WORD as u8);

    assert_eq!(peek(&m, 0x3000), WORD, "and the blit did copy it");
}

/// A refresh slot transfers no data, so the lines keep what the last cycle
/// left, however long it goes on.
///
/// Chapter 6 gives memory refresh its own slots at the start of every line and
/// Appendix B gives it `REFPTR`, but refresh is a `RAS`-only DRAM cycle: `CAS`
/// is never asserted and the DRAMs never enable their outputs, so no data
/// crosses the bus. With every `DMACON` channel off, refresh is the only DMA a
/// real Agnus still runs — and after thirty PAL lines of it the lines still
/// hold what the last cycle put there. (The manual does not say this; it is
/// the DRAM cycle's own definition, and it is marked as an inference in
/// `dev::amiga::dma::ChipDataBus`.)
///
/// This is the other half of the rule the test above shows: a cycle that
/// drives nothing *and selects nothing* changes nothing, while a read that
/// nothing answers opens the buffers onto the lines and leaves them floating.
#[test]
fn refresh_slots_transfer_no_data_and_leave_the_word_alone() {
    let mut m = boot("agnus.refresh");
    clear_overlay(&m);
    let bus = custom_bus(&m);

    const WORD: u16 = 0x3c0f;
    poke(&m, 0x2000, WORD);
    poke(&m, DMACON, SETCLR | DMAEN | BLTEN);
    poke(&m, BLTCON0, 0x0900 | 0xf0);
    poke(&m, BLTCON1, 0);
    poke(&m, BLTAFWM, 0xffff);
    poke(&m, BLTALWM, 0xffff);
    poke_long(&m, BLTAPTH, 0x2000);
    poke_long(&m, BLTDPTH, 0x3000);
    poke(&m, BLTSIZE, (1 << 6) | 1);
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");
    assert_eq!(bus.floating(), WORD);

    // Every channel off. That write is itself a cycle and leaves its own word
    // on the lines, which is what the rest of this test watches.
    const OFF: u16 = DMAEN | BLTEN | COPEN;
    poke(&m, DMACON, OFF);
    assert_eq!(bus.floating(), OFF);

    // Two milliseconds is thirty PAL lines, so a hundred and twenty refresh
    // slots and nothing else.
    m.run_for(GlobalTime::from_nanos(2_000_000))
        .expect("it runs");
    assert_eq!(bus.floating(), OFF, "refresh drove nothing");

    // A debugger's read is not a cycle at all: it neither takes the word nor
    // moves the counter, however many times it is made. A guest's read does
    // both, which is the other half of the `MemAttrs::debug` contract.
    let counted = bus.unclaimed();
    let space = m.space("mem").expect("the memory space");
    for _ in 0..3 {
        let seen = space
            .read(BPL1MOD, Width::U16, MemAttrs::DEBUG)
            .expect("a word") as u16;
        assert_eq!(seen, OFF, "a debugger sees what the guest would");
    }
    assert_eq!(bus.unclaimed(), counted, "and is not counted");
    assert_eq!(peek(&m, BPL1MOD), OFF, "the guest's read gets it");
    assert_eq!(bus.unclaimed(), counted + 1, "and is counted");
    assert_eq!(bus.floating(), 0, "and left the lines floating");
}
