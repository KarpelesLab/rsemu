//! Hand-written tests for the 68030 and 68EC030 models.
//!
//! There is no 68030 corpus. The expected values here come from the manuals —
//! MC68030UM §6.3.1 for `CACR`, §8 for the frames, §12.1.3 for what the part
//! dropped, and MC68EC030UM §9 and Appendix A for the cut-down package — and
//! `conformance.rs` replays the whole 68000 corpus through the 68030 on top
//! of them.

use alloc::sync::Arc;

use crate::core::error::Result;
use crate::core::space::{AddressSpace, RamStore, Region};
use crate::core::value::Endian;

use super::tests_68010::{Board, restore, snapshot};
use super::{Config, M68k, Model, Reg, flags, vector};

/// A 68030 board with 64 KiB at zero and 4 KiB at 16 MiB, so an access that
/// needs the top eight address lines has somewhere to land.
struct HighBoard {
    cpu: Arc<M68k>,
    low: Arc<RamStore>,
    high: Arc<RamStore>,
}

/// Where the second region sits: the first address a 24-pin part cannot
/// reach.
const HIGH: u64 = 0x0100_0000;

impl HighBoard {
    fn new(model: Model) -> HighBoard {
        let low = Arc::new(RamStore::new(0x1_0000));
        let high = Arc::new(RamStore::new(0x1000));
        let bits = if model.address_mask() == u32::MAX {
            32
        } else {
            24
        };
        let space = AddressSpace::new("cpu", bits).with_endian(Endian::Big);
        space
            .topology()
            .map(Region::ram("low", low.clone()).with_endian(Endian::Big), 0)
            .expect("64 KiB at zero");
        if bits == 32 {
            space
                .topology()
                .map(
                    Region::ram("high", high.clone()).with_endian(Endian::Big),
                    HIGH,
                )
                .expect("4 KiB at 16 MiB");
        }
        let cpu = Arc::new(M68k::new(Config::default().with_model(model)));
        cpu.attach_space(Arc::new(space));
        HighBoard { cpu, low, high }
    }

    fn poke_word(&self, addr: u64, value: u16) {
        self.low.write_u8(addr, (value >> 8) as u8).unwrap();
        self.low.write_u8(addr + 1, value as u8).unwrap();
    }

    fn boot(&self, words: &[u16]) {
        self.poke_word(2, 0x2000);
        self.poke_word(6, 0x0400);
        for (i, word) in words.iter().enumerate() {
            self.poke_word(0x400 + 2 * i as u64, *word);
        }
        self.cpu.step();
    }
}

#[test]
fn callm_and_rtm_are_unimplemented_on_a_68030() {
    // MC68030UM §12.1.3: "the MC68030 does not support the CALLM and RTM
    // instructions of the MC68020. If code is executed on the MC68030 using
    // either the CALLM or RTM instructions, an unimplemented instruction
    // exception is taken." Their encodings are in line 0, whose unimplemented
    // exception is vector 4.
    for (words, what) in [
        (&[0x06d0u16, 0x0000][..], "CALLM #0,(A0)"),
        (&[0x06c3, 0x0000][..], "RTM A3"),
    ] {
        let board = Board::new(Model::M68030);
        board.boot(words);
        board.handler(0, vector::ILLEGAL, 0x0c00);
        board.cpu.step();
        assert_eq!(
            board.cpu.last_exception(),
            Some(vector::ILLEGAL),
            "{what} on a 68030"
        );
        // And the 68020 still runs it.
        let twenty = Board::new(Model::M68020);
        twenty.boot(words);
        twenty.handler(0, vector::ILLEGAL, 0x0c00);
        twenty.cpu.step();
        assert_ne!(
            twenty.cpu.last_exception(),
            Some(vector::ILLEGAL),
            "{what} on a 68020"
        );
    }
}

#[test]
fn the_cache_control_register_keeps_the_68030s_bits() {
    // MC68030UM Figure 6-14: EI(0), FI(1), IBE(4), ED(8), FD(9), DBE(12) and
    // WA(13) have storage. CEI(2), CI(3), CED(10) and CD(11) act on contents
    // and read as zero, and bits 31-14 and 7-5 are reserved.
    let board = Board::new(Model::M68030);
    // MOVE.L #$ffffffff,D0 ; MOVEC D0,CACR ; MOVEC CACR,D1
    board.boot(&[0x203c, 0xffff, 0xffff, 0x4e7b, 0x0002, 0x4e7a, 0x1002]);
    for _ in 0..3 {
        board.cpu.step();
    }
    assert_eq!(board.cpu.regs().cacr, 0x3313);
    assert_eq!(board.cpu.regs().d[1], 0x3313);

    // A 68020 keeps only E and F (MC68020UM Figure 4-2).
    let twenty = Board::new(Model::M68020);
    twenty.boot(&[0x203c, 0xffff, 0xffff, 0x4e7b, 0x0002, 0x4e7a, 0x1002]);
    for _ in 0..3 {
        twenty.cpu.step();
    }
    assert_eq!(twenty.cpu.regs().cacr, 0x3);
}

#[test]
fn both_68030_packages_drive_all_32_address_lines() {
    // MC68EC030UM §1: the EC part drops the MMU, not the address bus — which
    // is the opposite of the 68EC020, whose only difference *is* the pins.
    assert_eq!(Model::M68030.address_mask(), u32::MAX);
    assert_eq!(Model::M68EC030.address_mask(), u32::MAX);
    assert_eq!(Model::M68EC020.address_mask(), 0x00ff_ffff);

    for model in [Model::M68030, Model::M68EC030] {
        let board = HighBoard::new(model);
        // MOVE.W #$1234,($01000010).L
        board.boot(&[0x33fc, 0x1234, 0x0100, 0x0010]);
        board.cpu.step();
        assert_eq!(
            board.high.read_u8(0x10).unwrap(),
            0x12,
            "{model} reached 16 MiB"
        );
        assert_eq!(board.high.read_u8(0x11).unwrap(), 0x34);
    }
}

#[test]
fn a_68030_snapshot_round_trips_its_control_registers() -> Result<()> {
    let board = Board::new(Model::M68030);
    board.boot(&[0x7001, 0x4e71]);
    board.with_regs(|r| {
        r.vbr = 0x1234_5678;
        r.cacr = 0x3313;
        r.caar = 0x0000_0ff0;
        r.msp = 0x0000_1f00;
    });
    board.cpu.step();
    let bytes = snapshot(&board.cpu)?;
    let other = M68k::new(Config::MC68030);
    restore(&other, &bytes)?;
    assert_eq!(other.regs(), board.cpu.regs());
    assert_eq!(snapshot(&other)?, bytes, "a round trip is a fixed point");
    // The model is in the chunk, so an EC030 refuses a 68030's snapshot.
    assert!(restore(&M68k::new(Config::MC68EC030), &bytes).is_err());
    Ok(())
}

#[test]
fn the_68030s_registers_can_be_named() {
    let names = Reg::all_for(Model::M68030);
    assert!(names.contains(&Reg::Cacr));
    assert!(names.contains(&Reg::Msp));
    assert!(names.contains(&Reg::Vbr));
    assert!(names.contains(&Reg::Tc));
    assert!(names.contains(&Reg::CrpHi));
    // The EC part has the access control unit and nothing more
    // (MC68EC030UM §9.3).
    let ec = Reg::all_for(Model::M68EC030);
    assert!(ec.contains(&Reg::Tt0));
    assert!(ec.contains(&Reg::Mmusr));
    assert!(!ec.contains(&Reg::Tc));
    assert!(!ec.contains(&Reg::CrpHi));
    // "To access AC0 in the MC68EC030, use TT0 in the MC68030 assembler."
    assert_eq!(Reg::from_name("ac0"), Some(Reg::Tt0));
    assert_eq!(Reg::from_name("acusr"), Some(Reg::Mmusr));
    for reg in names {
        assert_eq!(Reg::from_name(&alloc::format!("{reg}")), Some(reg));
    }
}

#[test]
fn instruction_lengths_agree_with_the_disassembler_on_a_68030() {
    // The same sweep the 68020 gets: for every encoding that decodes and
    // runs, the bytes the disassembler claims are the bytes the program
    // counter moved.
    use super::isa::Op;
    let measured = super::tests_68010::length_sweep(
        Model::M68030,
        &[
            Op::Mull,
            Op::Divl,
            Op::Bfextu,
            Op::Bfins,
            Op::Cas,
            Op::Pack,
            Op::Unpk,
            Op::Extb,
            Op::Trapcc,
        ],
    );
    assert!(measured > 100, "only {measured} encodings measured");
}

// ---------------------------------------------------------------------------
// The paged memory management unit
// ---------------------------------------------------------------------------
//
// Every tree below is the same shape, because it is the smallest one that
// exercises two levels of table and a page offset: `TC` = `$80cc4400`, which
// is `PS` 12 (4K pages), `IS` 12, `TIA` 4 and `TIB` 4 — twelve plus twelve
// plus four plus four is thirty-two, which is the consistency check of
// MC68030UM §9.7.2. The effective logical address is therefore twenty bits:
// bits 19-16 index the A table, bits 15-12 the B table, and bits 11-0 are the
// offset.
//
// `TT0` transparently translates every function code in `$00000000`-
// `$00ffffff`, which is where the code, the stack and the tables live, so the
// processor can keep running while the tree maps a page somewhere else
// (§9.3). The page under test is at logical `$01042000`, outside that block.

/// The translation control value every tree below uses.
const TC: u32 = 0x80cc_4400;
/// `TT0`: enabled, read/write masked, every function code, `$00xxxxxx`.
const TT0: u32 = 0x0000_8107;
/// Where the A-level table goes.
const TABLE_A: u32 = 0x1300;
/// Where the B-level table goes.
const TABLE_B: u32 = 0x1400;
/// The physical page the tree maps to.
const FRAME: u32 = 0x8000;

/// A root pointer with the limit suppressed: **L/U** clear and **LIMIT** all
/// ones, **DT** = valid four byte, table at [`TABLE_A`] (MC68030UM §9.7.1).
const ROOT: u64 = (0x7fff << 48) | (2 << 32) | (TABLE_A as u64);

/// A 68030 with its MMU registers loaded from memory and a two-level tree.
struct Mapped {
    board: Board,
}

impl Mapped {
    /// Build the tree with `a` at the A level and `b` at the B level, and run
    /// the three `PMOVE`s that switch translation on.
    fn new(model: Model, a: u32, b: u32, crp: u64) -> Mapped {
        let board = Board::new(model);
        // PMOVE ($1200).L,TT0 ; PMOVE ($1208).L,CRP ; PMOVE ($1204).L,TC
        board.boot(&[
            0xf039, 0x0800, 0x0000, 0x1200, //
            0xf039, 0x4c00, 0x0000, 0x1208, //
            0xf039, 0x4000, 0x0000, 0x1204, //
        ]);
        board.poke_long(0x1200, TT0);
        board.poke_long(0x1204, TC);
        board.poke_long(0x1208, (crp >> 32) as u32);
        board.poke_long(0x120c, crp as u32);
        board.poke_long(u64::from(TABLE_A) + 4 * 4, a);
        board.poke_long(u64::from(TABLE_B) + 2 * 4, b);
        for _ in 0..3 {
            board.cpu.step();
        }
        Mapped { board }
    }

    /// The ordinary case: a short table descriptor and a short page
    /// descriptor with no protection.
    fn plain(model: Model) -> Mapped {
        Mapped::new(model, TABLE_B | 2, FRAME | 1, ROOT)
    }

    /// Assemble `words` at `$500` and run them.
    fn run(&self, words: &[u16], count: usize) {
        self.board.load(0x500, words);
        self.board.at(0x500);
        for _ in 0..count {
            self.board.cpu.step();
        }
    }
}

#[test]
fn a_two_level_tree_maps_a_page() {
    let m = Mapped::plain(Model::M68030);
    assert_eq!(m.board.cpu.regs().tc, TC, "TC took the value");
    assert_eq!(m.board.cpu.regs().tt[0], TT0);
    assert_eq!(m.board.cpu.regs().crp, ROOT);
    // MOVE.W #$1234,($01042010).L — through the tree to physical $8010.
    m.run(&[0x33fc, 0x1234, 0x0104, 0x2010], 1);
    assert_eq!(m.board.peek_word(u64::from(FRAME) + 0x10), 0x1234);
    // And back again, through the cache this time.
    m.run(&[0x3039, 0x0104, 0x2010], 1); // MOVE.W ($01042010).L,D0
    assert_eq!(m.board.cpu.regs().d[0] & 0xffff, 0x1234);
}

#[test]
fn the_used_and_modified_bits_are_written_back() {
    // §9.5.1.1: "This bit is automatically set by the processor when a
    // descriptor is accessed in which the U bit is clear", and "The MC68030
    // sets the M bit in the corresponding page descriptor before a write
    // operation to a page for which the M bit is zero".
    let m = Mapped::plain(Model::M68030);
    // A read first: U is set everywhere it was clear, M is left alone.
    m.run(&[0x3039, 0x0104, 0x2010], 1);
    let a = m.board.peek_long(u64::from(TABLE_A) + 16);
    let b = m.board.peek_long(u64::from(TABLE_B) + 8);
    assert_eq!(a & 0x08, 0x08, "the table descriptor's U bit");
    assert_eq!(b & 0x08, 0x08, "the page descriptor's U bit");
    assert_eq!(b & 0x10, 0, "and M is untouched by a read");
    // Then a write to the same page: the entry is in the cache with M clear,
    // so the access is aborted, a search sets M, and it is retried (§9.4).
    m.run(&[0x33fc, 0x1234, 0x0104, 0x2010], 1);
    assert_eq!(
        m.board.peek_long(u64::from(TABLE_B) + 8) & 0x10,
        0x10,
        "M after the write"
    );
    assert_eq!(m.board.peek_word(u64::from(FRAME) + 0x10), 0x1234);
}

#[test]
fn an_invalid_descriptor_is_a_bus_error() {
    // DT = $0 at the B level. "This bit indicates an invalid translation",
    // and the ATC entry gets its B bit, which faults on the retry (§9.4).
    let m = Mapped::new(Model::M68030, TABLE_B | 2, FRAME, ROOT);
    m.board.handler(0, vector::BUS_ERROR, 0x0c00);
    m.run(&[0x3039, 0x0104, 0x2010], 1);
    assert_eq!(m.board.cpu.last_exception(), Some(vector::BUS_ERROR));
}

#[test]
fn write_protection_stops_a_write_and_not_a_read() {
    // §9.5.1.1, WP: "When WP is set, the MC68030 does not allow the logical
    // address space mapped by that descriptor to be written by any program
    // (i.e., this protection is absolute)."
    let m = Mapped::new(Model::M68030, TABLE_B | 2, FRAME | 4 | 1, ROOT);
    m.board.handler(0, vector::BUS_ERROR, 0x0c00);
    m.run(&[0x3039, 0x0104, 0x2010], 1); // a read
    assert_eq!(m.board.cpu.last_exception(), None, "a read is allowed");
    m.run(&[0x33fc, 0x1234, 0x0104, 0x2010], 1);
    assert_eq!(m.board.cpu.last_exception(), Some(vector::BUS_ERROR));
    assert_eq!(m.board.peek_word(u64::from(FRAME) + 0x10), 0, "not written");
    // And M was not set on the way, because "the MC68030 sets the M bit ...
    // except after a descriptor with the WP bit set is encountered".
    assert_eq!(m.board.peek_long(u64::from(TABLE_B) + 8) & 0x10, 0);
}

#[test]
fn a_limit_violation_aborts_the_search() {
    // The root pointer's limit is an upper one of two and the A index is
    // four, so the search stops before it reaches a table (Figure 9-28).
    let capped = (2u64 << 48) | (2 << 32) | u64::from(TABLE_A);
    let m = Mapped::new(Model::M68030, TABLE_B | 2, FRAME | 1, capped);
    m.board.handler(0, vector::BUS_ERROR, 0x0c00);
    m.run(&[0x3039, 0x0104, 0x2010], 1);
    assert_eq!(m.board.cpu.last_exception(), Some(vector::BUS_ERROR));
    // An index inside the limit goes through: logical $01002000 indexes A[0].
    let ok = Mapped::new(Model::M68030, TABLE_B | 2, FRAME | 1, capped);
    ok.board.poke_long(u64::from(TABLE_A), TABLE_B | 2);
    ok.board.handler(0, vector::BUS_ERROR, 0x0c00);
    ok.run(&[0x3039, 0x0100, 0x2010], 1);
    assert_eq!(ok.board.cpu.last_exception(), None);
}

#[test]
fn a_supervisor_only_page_is_refused_from_user_state() {
    // The S bit lives in bit 8 of a *long* format descriptor's status word
    // (Figure 9-11), so the A level is a long-format table here: DT = $3 in
    // the root, and the A entry is two long words.
    let root = (0x7fffu64 << 48) | (3 << 32) | u64::from(TABLE_A);
    let board = Board::new(Model::M68030);
    board.boot(&[
        0xf039, 0x0800, 0x0000, 0x1200, //
        0xf039, 0x4c00, 0x0000, 0x1208, //
        0xf039, 0x4000, 0x0000, 0x1204, //
    ]);
    board.poke_long(0x1200, TT0);
    board.poke_long(0x1204, TC);
    board.poke_long(0x1208, (root >> 32) as u32);
    board.poke_long(0x120c, root as u32);
    // A[4], long format: a status word with S set and DT = valid four byte,
    // then the table address.
    board.poke_long(u64::from(TABLE_A) + 4 * 8, 0x7fff_0000 | 0x100 | 2);
    board.poke_long(u64::from(TABLE_A) + 4 * 8 + 4, TABLE_B);
    board.poke_long(u64::from(TABLE_B) + 2 * 4, FRAME | 1);
    for _ in 0..3 {
        board.cpu.step();
    }
    board.handler(0, vector::BUS_ERROR, 0x0c00);
    board.load(0x500, &[0x3039, 0x0104, 0x2010]);
    board.at(0x500);
    board.cpu.step();
    assert_eq!(board.cpu.last_exception(), None, "supervisor may");
    // User: refused. Its own function code makes its own cache entry, so
    // nothing the supervisor left behind can answer for it.
    board.load(0x500, &[0x3039, 0x0104, 0x2010]);
    board.at(0x500);
    board.with_regs(|r| r.sr &= !flags::S);
    board.cpu.step();
    assert_eq!(board.cpu.last_exception(), Some(vector::BUS_ERROR));
}

#[test]
fn a_transparent_block_needs_no_tree_at_all() {
    // §9.3: "Logical addresses in a transparently translated block are used
    // as physical addresses, without modification and without protection
    // checking."
    let m = Mapped::new(Model::M68030, 0, 0, ROOT);
    // $00001800 is inside TT0's block, so it is reached although the tree
    // maps nothing at all.
    m.run(&[0x33fc, 0x4321, 0x0000, 0x1800], 1);
    assert_eq!(m.board.peek_word(0x1800), 0x4321);
    assert_eq!(m.board.cpu.last_exception(), None);
}

#[test]
fn ptest_reports_the_translation_it_found() {
    let m = Mapped::plain(Model::M68030);
    // PTESTR #5,($01042010).L,#7 — a full tree search.
    m.run(&[0xf039, 0x9e15, 0x0104, 0x2010], 1);
    let mmusr = m.board.cpu.regs().mmusr;
    assert_eq!(mmusr & 0x0007, 2, "two tables were read");
    assert_eq!(mmusr & 0x0400, 0, "not invalid");
    assert_eq!(mmusr & 0x0800, 0, "not write protected");
    // PTESTR #5,($01042010).L,#0 — the cache, which the search above filled.
    m.run(&[0xf039, 0x8215, 0x0104, 0x2010], 1);
    assert_eq!(m.board.cpu.regs().mmusr & 0x0400, 0, "resident");
    // An address nothing maps.
    m.run(&[0xf039, 0x8215, 0x0105, 0x2010], 1);
    assert_eq!(m.board.cpu.regs().mmusr & 0x0400, 0x0400, "not resident");
    // And a transparent one sets T and nothing else (Table 9-3).
    m.run(&[0xf039, 0x8215, 0x0000, 0x1800], 1);
    assert_eq!(m.board.cpu.regs().mmusr, 0x0040);
}

#[test]
fn pflush_invalidates_what_it_selects() {
    let m = Mapped::plain(Model::M68030);
    m.run(&[0x3039, 0x0104, 0x2010], 1);
    let resident = |m: &Mapped| {
        m.board.load(0x560, &[0xf039, 0x8215, 0x0104, 0x2010]);
        m.board.at(0x560);
        m.board.cpu.step();
        m.board.cpu.regs().mmusr & 0x0400 == 0
    };
    assert!(resident(&m), "the read left an entry");
    // PFLUSH #1,#7 — supervisor data is function code 5, so a flush that
    // selects code 1 alone leaves it.
    m.run(&[0xf000, 0x30f1], 1);
    assert!(resident(&m), "another function code was flushed");
    // PFLUSHA takes everything.
    m.run(&[0xf000, 0x2400], 1);
    assert!(!resident(&m), "PFLUSHA emptied the cache");
}

#[test]
fn pload_fills_the_cache_without_an_access() {
    let m = Mapped::plain(Model::M68030);
    // PLOADR #5,($01042010).L
    m.run(&[0xf039, 0x2215, 0x0104, 0x2010], 1);
    m.run(&[0xf039, 0x8215, 0x0104, 0x2010], 1);
    assert_eq!(
        m.board.cpu.regs().mmusr & 0x0400,
        0,
        "PLOAD left a resident entry"
    );
}

#[test]
fn an_inconsistent_translation_control_takes_the_configuration_exception() {
    // §9.7.2: "If the sum is not equal to 32, the PMOVE instruction causes an
    // MMU configuration exception ... the TC register is updated with the
    // data, and the E bit is cleared."
    let board = Board::new(Model::M68030);
    board.boot(&[0xf039, 0x4000, 0x0000, 0x1204]);
    board.handler(0, vector::MMU_CONFIG, 0x0c00);
    let bad = TC + 0x0000_0100; // TIB one bit too wide
    board.poke_long(0x1204, bad);
    board.cpu.step();
    assert_eq!(board.cpu.last_exception(), Some(vector::MMU_CONFIG));
    assert_eq!(board.cpu.regs().tc, bad & !0x8000_0000, "E cleared");
}

#[test]
fn an_invalid_root_pointer_takes_the_configuration_exception() {
    // §9.7.5.3: "A PMOVE instruction that loads either the CRP or the SRP
    // causes an MMU configuration exception if the new value of the DT field
    // is zero (invalid). In this case, the register is loaded with the new
    // value before the exception is taken."
    let board = Board::new(Model::M68030);
    board.boot(&[0xf039, 0x4c00, 0x0000, 0x1208]);
    board.handler(0, vector::MMU_CONFIG, 0x0c00);
    board.poke_long(0x1208, 0x7fff_0000);
    board.poke_long(0x120c, TABLE_A);
    board.cpu.step();
    assert_eq!(board.cpu.last_exception(), Some(vector::MMU_CONFIG));
    assert_eq!(
        board.cpu.regs().crp,
        (0x7fffu64 << 48) | u64::from(TABLE_A),
        "loaded anyway"
    );
}

#[test]
fn an_ec030_has_the_access_control_unit_and_no_more() {
    // MC68EC030UM §9.4: PFLUSH, PMOVEFD and PLOAD, and PMOVE for the paged
    // unit's registers, are unimplemented F-line instructions.
    for words in [
        &[0xf039u16, 0x4000, 0x0000, 0x1204][..], // PMOVE (xxx).L,TC
        &[0xf039, 0x4c00, 0x0000, 0x1208][..],    // PMOVE (xxx).L,CRP
        &[0xf000, 0x2400][..],                    // PFLUSHA
        &[0xf039, 0x2215, 0x0104, 0x2010][..],    // PLOADR
        &[0xf039, 0x9e15, 0x0104, 0x2010][..],    // PTEST with a level
    ] {
        let board = Board::new(Model::M68EC030);
        board.boot(words);
        board.handler(0, vector::LINE_F, 0x0c00);
        board.cpu.step();
        assert_eq!(
            board.cpu.last_exception(),
            Some(vector::LINE_F),
            "{words:04x?}"
        );
    }
    // What it does have: AC0, AC1 and the ACUSR, spelled TT0, TT1 and MMUSR.
    let board = Board::new(Model::M68EC030);
    board.boot(&[
        0xf039, 0x0800, 0x0000, 0x1200, //
        0xf039, 0x8215, 0x0000, 0x1800,
    ]);
    board.poke_long(0x1200, TT0);
    board.cpu.step();
    assert_eq!(board.cpu.regs().tt[0], TT0);
    board.cpu.step();
    // "The AC bit is set if a match occurs in either (or both) of the access
    // control registers" — bit 6, where the 68030 puts T.
    assert_eq!(board.cpu.regs().mmusr, 0x0040);
}

#[test]
fn an_mmu_snapshot_carries_the_registers_and_not_the_cache() -> Result<()> {
    let m = Mapped::plain(Model::M68030);
    m.run(&[0x3039, 0x0104, 0x2010], 1);
    let bytes = snapshot(&m.board.cpu)?;
    let other = M68k::new(Config::MC68030);
    restore(&other, &bytes)?;
    let regs = other.regs();
    assert_eq!(regs.tc, TC);
    assert_eq!(regs.crp, ROOT);
    assert_eq!(regs.tt[0], TT0);
    assert_eq!(regs, m.board.cpu.regs());
    assert_eq!(snapshot(&other)?, bytes, "a round trip is a fixed point");
    Ok(())
}

#[test]
fn the_disassembler_speaks_the_memory_management_instructions() {
    use super::disasm::disassemble_for;
    let text = |words: &[u16]| alloc::format!("{}", disassemble_for(Model::M68030, 0x400, words));
    assert_eq!(text(&[0xf010, 0x4000]), "PMOVE (A0),tc");
    assert_eq!(text(&[0xf010, 0x4200]), "PMOVE tc,(A0)");
    assert_eq!(text(&[0xf010, 0x4c00]), "PMOVE (A0),crp");
    assert_eq!(text(&[0xf010, 0x0800]), "PMOVE (A0),tt0");
    assert_eq!(text(&[0xf010, 0x6000]), "PMOVE (A0),mmusr");
    assert_eq!(text(&[0xf000, 0x2400]), "PFLUSHA");
    assert_eq!(text(&[0xf000, 0x30f1]), "PFLUSH #1,#$7");
    assert_eq!(text(&[0xf000, 0x30e1]), "PFLUSH dfc,#$7");
    assert_eq!(text(&[0xf010, 0x38f5]), "PFLUSH #5,#$7,(A0)");
    assert_eq!(text(&[0xf010, 0x2215]), "PLOADR #5,(A0)");
    assert_eq!(text(&[0xf010, 0x2015]), "PLOADW #5,(A0)");
    assert_eq!(text(&[0xf010, 0x9e15]), "PTESTR #5,(A0),#7");
    assert_eq!(text(&[0xf010, 0x9f55]), "PTESTR #5,(A0),#7,A2");
    // A 68020 sees the whole F line as a trap instead.
    assert_eq!(
        alloc::format!(
            "{}",
            disassemble_for(Model::M68020, 0x400, &[0xf010, 0x4000])
        ),
        "LINEF $f010"
    );
    // The lengths come from the same walk the interpreter makes.
    assert_eq!(disassemble_for(Model::M68030, 0, &[0xf010, 0x4000]).len, 4);
    assert_eq!(
        disassemble_for(Model::M68030, 0, &[0xf039, 0x4000, 1, 2]).len,
        8
    );
}
