//! Hand-written tests for the 68010 model.
//!
//! There is no 68010 corpus, so the expected values here are computed from the
//! manuals — the frame layouts from MC68000UM Figures 6-6 and 6-8, the
//! instruction semantics from the M68000PRM pages — and the differential run in
//! `conformance.rs` accounts for every way the 68010 departs from the 68000 on
//! the 68000's own vectors.

use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::Device;
use crate::core::error::Result;
use crate::core::space::{AccessConstraints, AddressSpace, RamStore, Region};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::value::Endian;

use super::{CLASS, Config, M68k, Model, Reg, Regs, flags, vector};

/// A core of some model with 64 KiB of big-endian RAM, and 4 KiB at `$10000`
/// that only a supervisor access may touch.
pub(super) struct Board {
    pub(super) cpu: Arc<M68k>,
    pub(super) ram: Arc<RamStore>,
    pub(super) guarded: Arc<RamStore>,
}

impl Board {
    pub(super) fn new(model: Model) -> Board {
        Board::with_fpu(model, super::Coprocessor::None)
    }

    /// The same board with a floating-point coprocessor attached.
    pub(super) fn with_fpu(model: Model, fpu: super::Coprocessor) -> Board {
        let ram = Arc::new(RamStore::new(0x1_0000));
        let guarded = Arc::new(RamStore::new(0x1000));
        let bits = if model.address_mask() == u32::MAX {
            32
        } else {
            24
        };
        let space = AddressSpace::new("cpu", bits).with_endian(Endian::Big);
        space
            .topology()
            .map(Region::ram("ram", ram.clone()).with_endian(Endian::Big), 0)
            .expect("64 KiB fits");
        space
            .topology()
            .map(
                Region::ram("guarded", guarded.clone())
                    .with_endian(Endian::Big)
                    .with_constraints(
                        AccessConstraints::ANY
                            .with_endian(Endian::Big)
                            .with_privileged_only(true),
                    ),
                0x1_0000,
            )
            .expect("4 KiB fits");
        let cpu = Arc::new(M68k::new(Config::default().with_model(model).with_fpu(fpu)));
        cpu.attach_space(Arc::new(space));
        Board { cpu, ram, guarded }
    }

    pub(super) fn poke_word(&self, addr: u64, value: u16) {
        self.ram.write_u8(addr, (value >> 8) as u8).unwrap();
        self.ram.write_u8(addr + 1, value as u8).unwrap();
    }

    pub(super) fn poke_long(&self, addr: u64, value: u32) {
        self.poke_word(addr, (value >> 16) as u16);
        self.poke_word(addr + 2, value as u16);
    }

    pub(super) fn peek_word(&self, addr: u64) -> u16 {
        (u16::from(self.ram.read_u8(addr).unwrap()) << 8)
            | u16::from(self.ram.read_u8(addr + 1).unwrap())
    }

    pub(super) fn peek_long(&self, addr: u64) -> u32 {
        (u32::from(self.peek_word(addr)) << 16) | u32::from(self.peek_word(addr + 2))
    }

    /// Assemble `words` at `$400`, point the reset vector at it, and run the
    /// reset sequence.
    pub(super) fn boot(&self, words: &[u16]) {
        self.poke_long(0, 0x2000);
        self.poke_long(4, 0x0400);
        self.load(0x400, words);
        self.cpu.step();
    }

    /// Put `words` at `addr`.
    pub(super) fn load(&self, addr: u64, words: &[u16]) {
        for (i, word) in words.iter().enumerate() {
            self.poke_word(addr + 2 * i as u64, *word);
        }
    }

    /// Point `vector` at `handler` in a table at `base`, and put a `NOP` there
    /// so arriving is observable.
    pub(super) fn handler(&self, base: u32, vector: u8, handler: u32) {
        self.poke_long(u64::from(base) + u64::from(vector) * 4, handler);
        self.poke_word(u64::from(handler), 0x4e71);
    }

    /// Start executing at `addr`.
    ///
    /// The prefetch queue is part of the register file, so moving the program
    /// counter alone would leave the core about to execute whatever two words
    /// it last fetched. The queue's invariant is that `prefetch[0]` is the
    /// word at `pc`, so this places both.
    pub(super) fn at(&self, addr: u32) {
        let queue = [
            self.peek_word(u64::from(addr)),
            self.peek_word(u64::from(addr) + 2),
        ];
        self.with_regs(|r| {
            r.pc = addr;
            r.prefetch = queue;
        });
        self.cpu.set_reset_pending(false);
    }

    pub(super) fn with_regs(&self, edit: impl FnOnce(&mut Regs)) {
        let mut regs = self.cpu.regs();
        edit(&mut regs);
        self.cpu.set_regs(regs);
    }
}

#[test]
fn the_model_property_chooses_the_processor() {
    use crate::core::props::Props;

    for (name, model) in [
        ("68000", Model::M68000),
        ("68010", Model::M68010),
        ("68020", Model::M68020),
        ("68ec020", Model::M68EC020),
        ("68030", Model::M68030),
        ("68ec030", Model::M68EC030),
        ("68040", Model::M68040),
        ("68lc040", Model::M68LC040),
        ("68ec040", Model::M68EC040),
    ] {
        let cpu = M68k::from_props(&Props::new().with("model", name)).unwrap();
        assert_eq!(cpu.model(), model);
        assert_eq!(cpu.config().model, model);
    }
    assert_eq!(
        M68k::from_props(&Props::new()).unwrap().model(),
        Model::M68000
    );
    assert!(
        M68k::from_props(&Props::new().with("model", "68060")).is_err(),
        "a processor this core does not model is an error, not a 68000"
    );
    // The validator knows the same list.
    let schema = super::schema();
    assert!(alloc::format!("{schema:?}").contains("68ec020"));
}

#[test]
fn the_vector_table_moves_with_vbr() {
    let board = Board::new(Model::M68010);
    // MOVE.L #$8000,D0 ; MOVEC D0,VBR ; TRAP #3
    board.boot(&[0x203c, 0x0000, 0x8000, 0x4e7b, 0x0801, 0x4e43]);
    board.handler(0x8000, vector::TRAP_BASE + 3, 0x0c00);
    // A handler at the old table's slot proves the old table is not read.
    board.handler(0, vector::TRAP_BASE + 3, 0x0d00);
    for _ in 0..3 {
        board.cpu.step();
    }
    assert_eq!(board.cpu.regs().vbr, 0x8000);
    assert_eq!(board.cpu.regs().pc, 0x0c00);
}

#[test]
fn a_trap_pushes_the_four_word_format_0_frame() {
    // MC68000UM Figure 6-6: SR, PC high, PC low, then the format in bits
    // 15-12 and the vector offset in bits 11-0 — so TRAP #3 (vector 35,
    // offset $08C) stacks $008C.
    let board = Board::new(Model::M68010);
    board.boot(&[0x4e43]);
    board.handler(0, vector::TRAP_BASE + 3, 0x0c00);
    let sr = board.cpu.regs().sr;
    board.cpu.step();
    let sp = u64::from(board.cpu.regs().a[7]);
    assert_eq!(sp, 0x2000 - 8, "four words");
    assert_eq!(board.peek_word(sp), sr, "+0 status register");
    assert_eq!(board.peek_long(sp + 2), 0x402, "+2 the next instruction");
    assert_eq!(
        board.peek_word(sp + 6),
        0x008c,
        "+6 format 0, vector offset"
    );
}

#[test]
fn a_68000_pushes_no_format_word() {
    let board = Board::new(Model::M68000);
    board.boot(&[0x4e43]);
    board.handler(0, vector::TRAP_BASE + 3, 0x0c00);
    board.cpu.step();
    assert_eq!(board.cpu.regs().a[7], 0x2000 - 6);
}

#[test]
fn an_address_error_pushes_the_29_word_format_8_frame() {
    // MC68000UM Figure 6-8, byte by byte.
    let board = Board::new(Model::M68010);
    // MOVE.W D0,(A0) with A0 odd.
    board.boot(&[0x3080]);
    board.handler(0, vector::ADDRESS_ERROR, 0x0800);
    // Paint the frame's area so the three reserved words that are not
    // written can be seen not to have been.
    for addr in (0x2000 - 58..0x2000).step_by(2) {
        board.poke_word(addr, 0xaaaa);
    }
    board.with_regs(|r| {
        r.a[0] = 0x1001;
        r.d[0] = 0x1234_5678;
    });
    let sr = board.cpu.regs().sr;
    board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.pc, 0x800, "the handler");
    let sp = u64::from(regs.a[7]);
    assert_eq!(sp, 0x2000 - 58, "the stack pointer drops by 29 words");
    assert_eq!(board.peek_word(sp), sr, "+$00 status register");
    // Where the prefetch had got to, which the manual lets run up to five
    // words ahead of the instruction; a register-to-memory MOVE writes before
    // it prefetches, so here it has not moved at all.
    assert_eq!(board.peek_long(sp + 2), 0x400, "+$02 program counter");
    assert_eq!(
        board.peek_word(sp + 6),
        0x800c,
        "+$06 format 8, offset $00C"
    );
    // Special status word (Figure 6-9): a data write in supervisor data
    // space — RR, IF, DF, RM, HB, BY and RW all clear, FC 5.
    assert_eq!(board.peek_word(sp + 8), 0x0005, "+$08 special status word");
    assert_eq!(board.peek_long(sp + 10), 0x1001, "+$0A fault address");
    assert_eq!(
        board.peek_word(sp + 14),
        0xaaaa,
        "+$0E reserved, not written"
    );
    assert_eq!(board.peek_word(sp + 16), 0x5678, "+$10 data output buffer");
    assert_eq!(
        board.peek_word(sp + 18),
        0xaaaa,
        "+$12 reserved, not written"
    );
    assert_eq!(board.peek_word(sp + 20), 0x0000, "+$14 data input buffer");
    assert_eq!(
        board.peek_word(sp + 22),
        0xaaaa,
        "+$16 reserved, not written"
    );
    assert_eq!(
        board.peek_word(sp + 24),
        0x3080,
        "+$18 instruction input buffer"
    );
    assert_eq!(
        (board.peek_word(sp + 26) >> 10) & 0xf,
        super::exec::VERSION_68010,
        "+$1A version number, bits 13-10"
    );
}

#[test]
fn a_read_fault_says_so_in_the_special_status_word() {
    let board = Board::new(Model::M68010);
    // MOVE.W (A0),D1 with A0 odd: a data read.
    board.boot(&[0x3210]);
    board.handler(0, vector::ADDRESS_ERROR, 0x0800);
    board.with_regs(|r| r.a[0] = 0x1001);
    board.cpu.step();
    let sp = u64::from(board.cpu.regs().a[7]);
    // DF (bit 12) for a data fetch, RW (bit 8) for a read.
    assert_eq!(board.peek_word(sp + 8), 0x1105);
}

#[test]
fn rte_restarts_an_instruction_a_handler_completed_in_software() {
    // A handler that emulates a misaligned read: it puts the word in the
    // data input buffer, sets RR so the processor does not rerun the cycle,
    // and returns (MC68000UM §6.3.9.2). The instruction then completes with
    // the handler's data, and the postincrement its first attempt had
    // already made is not made twice.
    let board = Board::new(Model::M68010);
    // MOVE.W (A0)+,D1 at $400; then NOP.
    board.boot(&[0x3218, 0x4e71]);
    board.poke_long(u64::from(vector::ADDRESS_ERROR) * 4, 0x0800);
    // The handler: MOVE.W #$BEEF,($14,A7) ; ORI.W #$8000,(8,A7) ; RTE.
    board.load(
        0x800,
        &[
            0x3f7c, 0xbeef, 0x0014, // MOVE.W #$BEEF,$14(A7)
            0x006f, 0x8000, 0x0008, // ORI.W #$8000,$8(A7)
            0x4e73, // RTE
        ],
    );
    board.with_regs(|r| r.a[0] = 0x1001);
    board.cpu.step(); // the MOVE faults
    assert_eq!(board.cpu.regs().pc, 0x800);
    // The 68000-like register state the handler sees: the postincrement
    // happened as the address was calculated.
    assert_eq!(board.cpu.regs().a[0], 0x1003);
    board.cpu.step();
    board.cpu.step();
    board.cpu.step(); // RTE
    assert_eq!(board.cpu.regs().pc, 0x400, "the MOVE is restarted");
    assert_eq!(board.cpu.regs().a[7], 0x2000, "the frame is gone");
    board.cpu.step(); // the MOVE again
    let regs = board.cpu.regs();
    assert_eq!(regs.d[1] & 0xffff, 0xbeef, "the handler's word");
    assert_eq!(regs.a[0], 0x1003, "incremented once, not twice");
    assert_eq!(regs.pc, 0x402);
}

#[test]
fn rte_rejects_a_format_it_does_not_know() {
    let board = Board::new(Model::M68010);
    board.boot(&[0x4e73]); // RTE
    board.handler(0, vector::FORMAT_ERROR, 0x0900);
    // A four-word frame with format 3, which the 68010 does not define.
    board.poke_word(0x1ff8, 0x2700);
    board.poke_long(0x1ffa, 0x0500);
    board.poke_word(0x1ffe, 0x3000);
    board.with_regs(|r| {
        r.a[7] = 0x1ff8;
        r.ssp = 0x1ff8;
    });
    board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.pc, 0x900, "format error");
    // The faulty frame is left where it was and the new one goes under it,
    // pushing the RTE's own address (MC68000UM §6.4).
    assert_eq!(regs.a[7], 0x1ff8 - 8);
    assert_eq!(board.peek_long(0x1ff8 - 6), 0x400);
    assert_eq!(board.peek_word(0x1ff8 - 2), 0x0038, "format 0, offset $038");
    assert_eq!(board.peek_word(0x1ffe), 0x3000, "the bad frame is intact");
}

#[test]
fn rte_rejects_a_long_frame_from_another_processor() {
    let board = Board::new(Model::M68010);
    board.boot(&[0x4e73]);
    board.handler(0, vector::FORMAT_ERROR, 0x0900);
    let sp = 0x1000u64;
    board.poke_word(sp, 0x2700);
    board.poke_long(sp + 2, 0x0500);
    board.poke_word(sp + 6, 0x8008);
    // A version number that is not this core's.
    board.poke_word(sp + 26, 0x3c00);
    board.with_regs(|r| {
        r.a[7] = sp as u32;
        r.ssp = sp as u32;
    });
    board.cpu.step();
    assert_eq!(board.cpu.regs().pc, 0x900);
    assert_eq!(board.cpu.regs().a[7], sp as u32 - 8);
}

#[test]
fn move_from_sr_is_privileged_on_a_68010_and_not_on_a_68000() {
    for (model, trapped) in [(Model::M68000, false), (Model::M68010, true)] {
        let board = Board::new(model);
        board.boot(&[0x40c0]); // MOVE SR,D0
        board.handler(0, vector::PRIVILEGE, 0x0a00);
        board.with_regs(|r| {
            r.sr &= !flags::S;
            r.usp = 0x1800;
        });
        board.cpu.step();
        assert_eq!(board.cpu.regs().pc == 0xa00, trapped, "{model}");
    }
}

#[test]
fn move_from_ccr_is_for_user_code() {
    let board = Board::new(Model::M68010);
    board.boot(&[0x42c0]); // MOVE CCR,D0
    board.with_regs(|r| {
        r.sr = flags::X | flags::Z | flags::C; // user state
        r.usp = 0x1800;
        r.d[0] = 0xffff_ffff;
    });
    board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.pc, 0x402);
    // A word: the CCR zero-extended, the register's high half untouched
    // (M68000PRM, MOVE from CCR).
    assert_eq!(regs.d[0], 0xffff_0015);
    // And the 68000 has no such instruction.
    let old = Board::new(Model::M68000);
    old.boot(&[0x42c0]);
    old.handler(0, vector::ILLEGAL, 0x0b00);
    old.cpu.step();
    assert_eq!(old.cpu.regs().pc, 0xb00);
}

#[test]
fn movec_reaches_every_68010_control_register_and_no_other() {
    let board = Board::new(Model::M68010);
    board.boot(&[
        0x7005, // MOVEQ #5,D0
        0x4e7b, 0x0000, // MOVEC D0,SFC
        0x7003, // MOVEQ #3,D0
        0x4e7b, 0x0001, // MOVEC D0,DFC
        0x207c, 0x0000, 0x1700, // MOVEA.L #$1700,A0
        0x4e7b, 0x8800, // MOVEC A0,USP
        0x4e7a, 0x1000, // MOVEC SFC,D1
        0x4e7a, 0x2001, // MOVEC DFC,D2
        0x4e7a, 0xb800, // MOVEC USP,A3
        0x4e7a, 0x0002, // MOVEC CACR,D0 — a 68020 register
    ]);
    board.handler(0, vector::ILLEGAL, 0x0b00);
    for _ in 0..9 {
        board.cpu.step();
    }
    let regs = board.cpu.regs();
    assert_eq!((regs.sfc, regs.dfc), (5, 3));
    assert_eq!(regs.usp, 0x1700);
    assert_eq!((regs.d[1], regs.d[2], regs.a[3]), (5, 3, 0x1700));
    board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.pc, 0xb00, "CACR is not a 68010 register");
    assert_eq!(
        board.peek_long(u64::from(regs.a[7]) + 2),
        0x422,
        "the MOVEC's own address"
    );
}

#[test]
fn movec_is_privileged() {
    let board = Board::new(Model::M68010);
    board.boot(&[0x4e7a, 0x0801]); // MOVEC VBR,D0
    board.handler(0, vector::PRIVILEGE, 0x0a00);
    board.with_regs(|r| r.sr &= !flags::S);
    board.cpu.step();
    assert_eq!(board.cpu.regs().pc, 0xa00);
}

#[test]
fn moves_uses_the_function_code_registers() {
    // MOVES reaches the space DFC or SFC names, whatever state the processor
    // is in. With a user function code, a supervisor-only region refuses the
    // access; with a supervisor one it accepts it.
    let board = Board::new(Model::M68010);
    board.boot(&[
        0x7001, // MOVEQ #1,D0
        0x4e7b, 0x0001, // MOVEC D0,DFC     — user data
        0x207c, 0x0001, 0x0010, // MOVEA.L #$10010,A0
        0x323c, 0x1234, // MOVE.W #$1234,D1
        0x0e50, 0x1800, // MOVES.W D1,(A0)
    ]);
    board.handler(0, vector::BUS_ERROR, 0x0800);
    for _ in 0..5 {
        board.cpu.step();
    }
    assert_eq!(
        board.cpu.regs().pc,
        0x800,
        "a user write to supervisor space"
    );
    let sp = u64::from(board.cpu.regs().a[7]);
    assert_eq!(
        board.peek_word(sp + 8) & 7,
        1,
        "the fault reports DFC's code"
    );

    let board = Board::new(Model::M68010);
    board.boot(&[
        0x7005, // MOVEQ #5,D0
        0x4e7b, 0x0001, // MOVEC D0,DFC     — supervisor data
        0x207c, 0x0001, 0x0010, // MOVEA.L #$10010,A0
        0x323c, 0x1234, // MOVE.W #$1234,D1
        0x0e50, 0x1800, // MOVES.W D1,(A0)
        0x4e7b, 0x0000, // MOVEC D0,SFC
        0x0e50, 0xa000, // MOVES.W (A0),A2
    ]);
    for _ in 0..7 {
        board.cpu.step();
    }
    assert_eq!(board.guarded.read_u8(0x10).unwrap(), 0x12);
    assert_eq!(board.guarded.read_u8(0x11).unwrap(), 0x34);
    // Into an address register a word is sign-extended.
    assert_eq!(board.cpu.regs().a[2], 0x0000_1234);
}

#[test]
fn rtd_returns_and_drops_its_parameters() {
    let board = Board::new(Model::M68010);
    board.boot(&[0x4e74, 0x000c]); // RTD #12
    board.poke_long(0x1ff0, 0x0600);
    board.poke_word(0x600, 0x4e71);
    board.with_regs(|r| {
        r.a[7] = 0x1ff0;
        r.ssp = 0x1ff0;
    });
    let used = board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.pc, 0x600);
    assert_eq!(regs.a[7], 0x1ff0 + 4 + 12);
    // RTS's time: 16(4/0) (MC68000UM Table 9-18).
    assert_eq!(used, 16);
}

#[test]
fn bkpt_with_nothing_to_answer_is_an_illegal_instruction() {
    let board = Board::new(Model::M68010);
    board.boot(&[0x484b]); // BKPT #3
    board.handler(0, vector::ILLEGAL, 0x0b00);
    board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.pc, 0xb00);
    assert_eq!(board.peek_long(u64::from(regs.a[7]) + 2), 0x400);
}

#[test]
fn clr_does_not_read_its_destination_on_a_68010() {
    // CLR.W (A0): 12(2/1) on a 68000, 8(1/1) on a 68010 (MC68000UM Tables
    // 8-6 and 9-10) — the read is gone.
    for (model, cycles) in [(Model::M68000, 12), (Model::M68010, 8)] {
        let board = Board::new(model);
        board.boot(&[0x4250]);
        board.with_regs(|r| r.a[0] = 0x1000);
        assert_eq!(board.cpu.step(), cycles, "{model}");
    }
}

#[test]
fn reset_clears_the_vector_base() {
    let board = Board::new(Model::M68010);
    board.boot(&[0x4e71]);
    board.with_regs(|r| r.vbr = 0x4000);
    assert_eq!(board.cpu.regs().vbr, 0x4000);
    board.cpu.request_reset();
    board.cpu.step();
    assert_eq!(board.cpu.regs().vbr, 0);
}

#[test]
fn an_interrupt_vectors_through_vbr_with_a_format_word() {
    let board = Board::new(Model::M68010);
    board.boot(&[0x4e71]);
    board.handler(0x6000, vector::AUTOVECTOR_BASE + 3, 0x0e00);
    board.with_regs(|r| {
        r.vbr = 0x6000;
        r.sr = flags::S; // mask 0
    });
    board.cpu.set_ipl(3);
    let used = board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.pc, 0xe00);
    assert_eq!(regs.a[7], 0x2000 - 8);
    assert_eq!(board.peek_word(0x2000 - 2), 0x006c, "format 0, offset $06C");
    // 46(5/4) (MC68000UM Table 9-19).
    assert_eq!(used, 46);
}

/// Save one core into a snapshot's `cpu` chunk.
pub(super) fn snapshot(cpu: &M68k) -> Result<Vec<u8>> {
    let mut shape = MachineShape::new();
    shape.add_device("cpu", CLASS.name)?;
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("cpu", CLASS.name, CLASS.version)?;
        cpu.save(&mut chunk)?;
    }
    w.to_vec()
}

/// Load one core out of such a snapshot.
pub(super) fn restore(cpu: &M68k, bytes: &[u8]) -> Result<()> {
    let reader = StateReader::new(bytes)?;
    let chunk = reader.load("cpu", CLASS.name, CLASS.version, &Migrations::new())?;
    let mut r = chunk.reader();
    cpu.load(&mut r)?;
    r.end()
}

#[test]
fn a_68010_snapshot_round_trips_its_control_registers() -> Result<()> {
    let board = Board::new(Model::M68010);
    board.boot(&[0x7001, 0x4e71]);
    board.with_regs(|r| {
        r.vbr = 0x1234_5678;
        r.sfc = 2;
        r.dfc = 6;
        r.usp = 0x0bad_f00d;
    });
    board.cpu.step();
    let bytes = snapshot(&board.cpu)?;
    let other = M68k::new(Config::MC68010);
    restore(&other, &bytes)?;
    assert_eq!(other.regs(), board.cpu.regs());
    assert_eq!(snapshot(&other)?, bytes, "a round trip is a fixed point");
    // A 68000 cannot take it: the tail is not a shape it knows.
    assert!(restore(&M68k::new(Config::MC68000), &bytes).is_err());
    Ok(())
}

#[test]
fn registers_a_68010_adds_can_be_named() {
    let names = Reg::all_for(Model::M68010);
    assert!(names.contains(&Reg::Vbr));
    assert!(!names.contains(&Reg::Cacr));
    assert_eq!(Reg::from_name("vbr"), Some(Reg::Vbr));
    assert_eq!(Reg::from_name("isp"), Some(Reg::Ssp));
    for reg in Reg::all_for(Model::M68020) {
        assert_eq!(Reg::from_name(&alloc::format!("{reg}")), Some(reg));
    }
}

#[test]
fn the_disassembler_speaks_68010() {
    use super::disasm::disassemble_for;
    let text = |words: &[u16]| alloc::format!("{}", disassemble_for(Model::M68010, 0x400, words));
    assert_eq!(text(&[0x4e7a, 0x0801]), "MOVEC VBR,D0");
    assert_eq!(text(&[0x4e7b, 0x9800]), "MOVEC A1,USP");
    assert_eq!(text(&[0x4e7b, 0x0000]), "MOVEC D0,SFC");
    assert_eq!(text(&[0x4e7a, 0x7123]), "MOVEC $123,D7");
    assert_eq!(text(&[0x0e50, 0x1800]), "MOVES.W D1,(A0)");
    assert_eq!(text(&[0x0e98, 0xa000]), "MOVES.L (A0)+,A2");
    assert_eq!(text(&[0x0e28, 0x3000, 0x0010]), "MOVES.B $10(A0),D3");
    assert_eq!(text(&[0x4e74, 0x000c]), "RTD #$c");
    assert_eq!(text(&[0x42c0]), "MOVE CCR,D0");
    assert_eq!(text(&[0x484b]), "BKPT #$3");
    // The lengths come from the same walk the interpreter makes.
    assert_eq!(
        disassemble_for(Model::M68010, 0, &[0x0e28, 0x3000, 0x0010]).len,
        6
    );
    assert_eq!(disassemble_for(Model::M68010, 0, &[0x4e7a, 0x0801]).len, 4);
    // And a 68000 sees data.
    assert_eq!(
        alloc::format!("{}", super::disasm::disassemble(0x400, &[0x4e7a, 0x0801])),
        "DC.W $4e7a"
    );
}

/// For every encoding `model` decodes to one of `ops` and runs to
/// completion, the bytes the disassembler says it occupies must be the bytes
/// the program counter moved. Returns how many encodings were measured.
pub(super) fn length_sweep(model: Model, ops: &[super::isa::Op]) -> usize {
    use super::disasm::disassemble_for;
    use super::isa::decode_for;

    let board = Board::new(model);
    board.poke_long(0, 0x2000);
    board.poke_long(4, 0x0400);
    board.cpu.step();
    let mut base = Regs {
        d: [0x0000_0004; 8],
        a: [0x0000_1000; 8],
        usp: 0x0000_1800,
        ssp: 0x0000_2000,
        pc: 0x400,
        sr: flags::S | flags::IPL,
        ..Regs::default()
    };
    base.a[7] = 0x2000;
    let mut checked = 0usize;
    for opcode in 0..=u16::MAX {
        if !ops.contains(&decode_for(model, opcode).op) {
            continue;
        }
        // Extension words of small even values keep every mode inside RAM;
        // for a 68020 full-format word they also mean "null displacements,
        // no indirection" more often than not.
        for i in 0..11u64 {
            board.poke_word(0x400 + 2 * i, 0x0010);
        }
        board.poke_word(0x400, opcode);
        board.cpu.set_regs(base);
        board.cpu.request_reset();
        board.cpu.step();
        board.cpu.set_regs(Regs {
            prefetch: board.cpu.regs().prefetch,
            ..base
        });
        let mut words = [0u16; 11];
        for (i, word) in words.iter_mut().enumerate() {
            *word = board.peek_word(0x400 + 2 * i as u64);
        }
        let expected = disassemble_for(model, 0x400, &words);
        board.cpu.step();
        let after = board.cpu.regs().pc;
        if !(0x400..=0x420).contains(&after) || board.cpu.last_exception().is_some() {
            continue;
        }
        assert_eq!(
            after - 0x400,
            u32::from(expected.len),
            "{model} {opcode:04x} {expected}: the disassembler says {} bytes, the \
             interpreter consumed {}",
            expected.len,
            after - 0x400
        );
        checked += 1;
    }
    checked
}

#[test]
fn the_disassembler_and_the_68010_agree_on_lengths() {
    use super::isa::Op;
    let checked = length_sweep(
        Model::M68010,
        &[
            Op::MoveFromCcr,
            Op::Movec,
            Op::Moves,
            Op::MoveFromSr,
            Op::Clr,
            Op::Scc,
        ],
    );
    assert!(checked > 500, "only {checked} encodings were exercised");
}

#[test]
fn a_machine_file_can_ask_for_a_later_processor() {
    // The property travels through the machine layer, and the validator
    // rejects a model this core does not have before anything is built.
    use crate::core::Captured;

    let cores: Arc<Captured<M68k>> = Arc::new(Captured::new());
    let kept = Arc::clone(&cores);
    let mut options = crate::machine::BuildOptions::new();
    options.classes.insert(super::schema());
    for schema in crate::machine::builtin::schemas() {
        options.classes.insert(schema);
    }
    options
        .bindings
        .bind("cpu.m68k", move |props| {
            let cpu = Arc::new(M68k::from_props(props)?);
            kept.push(&cpu);
            Ok(cpu)
        })
        .expect("nothing else claims cpu.m68k");
    crate::machine::builtin::bind(&mut options.bindings).expect("ram and rom");
    let mut registry = crate::core::Registry::new();
    crate::machine::builtin::register(&mut registry).expect("ram and rom");
    super::register(&mut registry).expect("nothing else claims cpu.m68k");

    let board = |model: &str| {
        alloc::format!(
            "machine \"m\" {{\n  osc x = 14000000 Hz\n  \
             space mem {{ width = 24, endian = big }}\n  \
             object dram \"ram\" {{ size = 64K }}\n  \
             object cpu \"cpu.m68k\" {{ clock = x, space = mem, model = \"{model}\" }}\n  \
             map mem 0 size 64K = dram {{ endian = \"big\" }}\n}}\n"
        )
    };
    crate::machine::build("t.machine", &board("68020"), &registry, &options)
        .expect("a 68020 board builds");
    assert_eq!(cores.last().expect("the core").model(), Model::M68020);

    let err = crate::machine::build("t.machine", &board("68060"), &registry, &options)
        .expect_err("a processor this core does not model");
    assert!(alloc::format!("{err}").contains("model"), "{err}");
}
