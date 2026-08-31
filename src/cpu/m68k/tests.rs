//! Hand-written tests for the MC68000 core.
//!
//! The conformance corpus next door covers instruction semantics far more
//! thoroughly than anything written by hand could; what lives here is what the
//! corpus does *not* reach — reset, interrupts, the privilege split, the trace
//! bit, the prefetch queue's invariant, and the snapshot round trip.

use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::Device;
use crate::core::error::Result;
use crate::core::space::{AddressSpace, RamStore, Region};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::value::Endian;

use super::{CLASS, Config, M68k, Reg, Regs, flags, isa, vector};

/// A 68000 with 64 KiB of big-endian RAM and nothing else.
struct Board {
    cpu: Arc<M68k>,
    ram: Arc<RamStore>,
}

impl Board {
    fn new() -> Board {
        let ram = Arc::new(RamStore::new(0x1_0000));
        let space = AddressSpace::new("cpu", 24).with_endian(Endian::Big);
        space
            .topology()
            .map(Region::ram("ram", ram.clone()).with_endian(Endian::Big), 0)
            .expect("64 KiB fits in 24 bits");
        let cpu = Arc::new(M68k::new(Config::default()));
        cpu.attach_space(Arc::new(space));
        Board { cpu, ram }
    }

    fn poke_word(&self, addr: u64, value: u16) {
        self.ram.write_u8(addr, (value >> 8) as u8).unwrap();
        self.ram.write_u8(addr + 1, value as u8).unwrap();
    }

    fn poke_long(&self, addr: u64, value: u32) {
        self.poke_word(addr, (value >> 16) as u16);
        self.poke_word(addr + 2, value as u16);
    }

    fn peek_word(&self, addr: u64) -> u16 {
        (u16::from(self.ram.read_u8(addr).unwrap()) << 8)
            | u16::from(self.ram.read_u8(addr + 1).unwrap())
    }

    fn peek_long(&self, addr: u64) -> u32 {
        (u32::from(self.peek_word(addr)) << 16) | u32::from(self.peek_word(addr + 2))
    }

    /// Assemble `words` at `$400`, point the reset vector at it, and run the
    /// reset sequence.
    fn boot(&self, words: &[u16]) {
        self.poke_long(0, 0x2000);
        self.poke_long(4, 0x0400);
        for (i, word) in words.iter().enumerate() {
            self.poke_word(0x400 + 2 * i as u64, *word);
        }
        self.cpu.step();
    }
}

#[test]
fn reset_loads_the_stack_pointer_and_program_counter() {
    let board = Board::new();
    board.boot(&[0x4e71]);
    let regs = board.cpu.regs();
    assert_eq!(regs.ssp, 0x2000);
    assert_eq!(regs.a[7], 0x2000);
    assert_eq!(regs.pc, 0x400);
    assert!(regs.supervisor());
    assert_eq!(regs.ipl_mask(), 7);
    // Both prefetch words are loaded, which is the invariant the whole
    // interpreter rests on.
    assert_eq!(regs.prefetch[0], 0x4e71);
    assert_eq!(regs.prefetch[1], board.peek_word(0x402));
}

#[test]
fn the_prefetch_queue_holds_the_words_at_pc_and_pc_plus_two() {
    let board = Board::new();
    // NOP, NOP, MOVEQ #1,D0, NOP
    board.boot(&[0x4e71, 0x4e71, 0x7001, 0x4e71]);
    for _ in 0..3 {
        let regs = board.cpu.regs();
        assert_eq!(regs.prefetch[0], board.peek_word(u64::from(regs.pc)));
        assert_eq!(regs.prefetch[1], board.peek_word(u64::from(regs.pc) + 2));
        board.cpu.step();
    }
    assert_eq!(board.cpu.regs().d[0], 1);
}

#[test]
fn nop_costs_one_bus_cycle() {
    let board = Board::new();
    board.boot(&[0x4e71]);
    let before = board.cpu.cycles();
    let used = board.cpu.step();
    assert_eq!(used, 4, "a NOP is one prefetch and nothing else");
    assert_eq!(board.cpu.cycles() - before, 4);
}

#[test]
fn an_odd_word_access_takes_an_address_error() {
    let board = Board::new();
    // MOVE.W D0,(A0) with A0 odd.
    board.boot(&[0x3080]);
    board.poke_long(u64::from(vector::ADDRESS_ERROR) * 4, 0x0800);
    board.poke_word(0x800, 0x4e71);
    let mut regs = board.cpu.regs();
    regs.a[0] = 0x1001;
    board.cpu.set_regs(regs);
    board.cpu.step();

    let regs = board.cpu.regs();
    assert_eq!(regs.pc, 0x800, "the address-error handler was entered");
    // The fourteen-byte group-0 frame, in the layout of MC68000UM figure 6-6.
    let sp = regs.a[7];
    assert_eq!(sp, 0x2000 - 14);
    assert_eq!(board.peek_long(u64::from(sp) + 2), 0x1001, "access address");
    assert_eq!(board.peek_word(u64::from(sp) + 6), 0x3080, "instruction");
    // The special status word's low five bits describe the failed access:
    // supervisor data space, a write.
    let ssw = board.peek_word(u64::from(sp));
    assert_eq!(ssw & 0x1f, 0x05, "supervisor data, write");
}

#[test]
fn a_trap_pushes_the_six_byte_frame_and_vectors() {
    let board = Board::new();
    board.boot(&[0x4e4f]); // TRAP #15
    board.poke_long(u64::from(vector::TRAP_BASE + 15) * 4, 0x0900);
    board.poke_word(0x900, 0x4e73); // RTE
    let sr_before = board.cpu.regs().sr;
    board.cpu.step();

    let regs = board.cpu.regs();
    assert_eq!(regs.pc, 0x900);
    let sp = regs.a[7];
    assert_eq!(sp, 0x2000 - 6);
    assert_eq!(board.peek_word(u64::from(sp)), sr_before);
    assert_eq!(board.peek_long(u64::from(sp) + 2), 0x402);

    // And RTE puts everything back.
    board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.pc, 0x402);
    assert_eq!(regs.sr, sr_before);
    assert_eq!(regs.a[7], 0x2000);
}

#[test]
fn user_mode_cannot_touch_the_status_register() {
    let board = Board::new();
    board.boot(&[0x46fc, 0x2700]); // MOVE #$2700,SR
    board.poke_long(u64::from(vector::PRIVILEGE) * 4, 0x0a00);
    board.poke_word(0xa00, 0x4e71);
    let mut regs = board.cpu.regs();
    regs.sr &= !flags::S; // drop to user state
    regs.usp = 0x1800;
    board.cpu.set_regs(regs);
    assert_eq!(board.cpu.regs().a[7], 0x1800);

    board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.pc, 0xa00);
    assert!(regs.supervisor(), "the handler runs in supervisor state");
    // The frame went on the supervisor stack, and the user stack is untouched.
    assert_eq!(regs.usp, 0x1800);
    assert_eq!(regs.ssp, 0x2000 - 6);
    assert_eq!(board.peek_long(u64::from(regs.ssp) + 2), 0x400);
}

#[test]
fn the_two_stack_pointers_are_separate_registers() {
    let board = Board::new();
    board.boot(&[0x4e71]);
    let mut regs = board.cpu.regs();
    regs.sr &= !flags::S;
    regs.usp = 0x1000;
    regs.ssp = 0x2000;
    board.cpu.set_regs(regs);
    assert_eq!(board.cpu.reg(Reg::A(7)), 0x1000);

    // Returning to supervisor state swaps the bank back.
    let mut regs = board.cpu.regs();
    regs.sr |= flags::S;
    board.cpu.set_regs(regs);
    assert_eq!(board.cpu.reg(Reg::A(7)), 0x2000);
    assert_eq!(board.cpu.reg(Reg::Usp), 0x1000);
}

#[test]
fn an_illegal_encoding_traps_through_vector_four() {
    let board = Board::new();
    board.boot(&[0x4afc]); // ILLEGAL
    board.poke_long(u64::from(vector::ILLEGAL) * 4, 0x0b00);
    board.poke_word(0xb00, 0x4e71);
    board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.pc, 0xb00);
    // An illegal instruction pushes its own address, not the next one.
    assert_eq!(board.peek_long(u64::from(regs.ssp) + 2), 0x400);
}

#[test]
fn the_a_and_f_lines_have_their_own_vectors() {
    for (opcode, vec, target) in [
        (0xa000u16, vector::LINE_A, 0xc00u32),
        (0xf000, vector::LINE_F, 0xd00),
    ] {
        let board = Board::new();
        board.boot(&[opcode]);
        board.poke_long(u64::from(vec) * 4, target);
        board.poke_word(u64::from(target), 0x4e71);
        board.cpu.step();
        assert_eq!(board.cpu.regs().pc, target, "opcode {opcode:04x}");
    }
}

#[test]
fn an_interrupt_above_the_mask_is_taken_between_instructions() {
    let board = Board::new();
    board.boot(&[0x4e71, 0x4e71]);
    board.poke_long(u64::from(vector::AUTOVECTOR_BASE + 4) * 4, 0x0e00);
    board.poke_word(0xe00, 0x4e73);
    // Drop the mask so level 4 gets through.
    let mut regs = board.cpu.regs();
    regs.sr = (regs.sr & !flags::IPL) | (2 << 8);
    board.cpu.set_regs(regs);

    board.cpu.set_ipl(4);
    board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.pc, 0xe00, "the level 4 autovector");
    assert_eq!(regs.ipl_mask(), 4, "the mask rises to the level serviced");
}

#[test]
fn an_interrupt_at_or_below_the_mask_is_ignored() {
    let board = Board::new();
    board.boot(&[0x4e71]);
    let mut regs = board.cpu.regs();
    regs.sr = (regs.sr & !flags::IPL) | (5 << 8);
    board.cpu.set_regs(regs);
    board.cpu.set_ipl(5);
    board.cpu.step();
    assert_eq!(
        board.cpu.regs().pc,
        0x402,
        "the NOP ran, no vector was taken"
    );
}

#[test]
fn level_seven_is_not_maskable() {
    let board = Board::new();
    board.boot(&[0x4e71]);
    board.poke_long(u64::from(vector::AUTOVECTOR_BASE + 7) * 4, 0x0f00);
    board.poke_word(0xf00, 0x4e73);
    assert_eq!(board.cpu.regs().ipl_mask(), 7);
    board.cpu.set_ipl(7);
    board.cpu.step();
    assert_eq!(board.cpu.regs().pc, 0xf00);
}

#[test]
fn stop_waits_for_an_interrupt() {
    let board = Board::new();
    board.boot(&[0x4e72, 0x2000]); // STOP #$2000 — supervisor, mask 0
    board.poke_long(u64::from(vector::AUTOVECTOR_BASE + 1) * 4, 0x0e80);
    board.poke_word(0xe80, 0x4e73);
    board.cpu.step();
    assert!(board.cpu.is_stopped());
    // Stepping a stopped core makes time pass and nothing else.
    let pc = board.cpu.regs().pc;
    assert!(board.cpu.step() > 0);
    assert_eq!(board.cpu.regs().pc, pc);

    board.cpu.set_ipl(1);
    board.cpu.step();
    assert!(!board.cpu.is_stopped());
    assert_eq!(board.cpu.regs().pc, 0xe80);
}

#[test]
fn the_trace_bit_takes_an_exception_after_each_instruction() {
    let board = Board::new();
    board.boot(&[0x7001]); // MOVEQ #1,D0
    board.poke_long(u64::from(vector::TRACE) * 4, 0x0f80);
    board.poke_word(0xf80, 0x4e73);
    let mut regs = board.cpu.regs();
    regs.sr |= flags::T;
    board.cpu.set_regs(regs);

    board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.d[0], 1, "the instruction completed first");
    assert_eq!(regs.pc, 0xf80);
    assert!(!regs.flag(flags::T), "tracing is off inside the handler");
    assert_eq!(board.peek_long(u64::from(regs.ssp) + 2), 0x402);
}

#[test]
fn addresses_reach_the_bus_modulo_sixteen_megabytes() {
    let board = Board::new();
    // MOVE.W #$1234,($FF001000).L — the high byte has no pin to drive.
    board.boot(&[0x33fc, 0x1234, 0xff00, 0x1000]);
    board.cpu.step();
    assert_eq!(board.peek_word(0x1000), 0x1234);
}

#[test]
fn a_double_fault_halts_the_processor() {
    let board = Board::new();
    // MOVE.W D0,(A0) with A0 odd, and an address-error vector that is also
    // odd: the exception's own fetch faults, which is the double bus fault.
    board.boot(&[0x3080]);
    board.poke_long(u64::from(vector::ADDRESS_ERROR) * 4, 0x0801);
    let mut regs = board.cpu.regs();
    regs.a[0] = 0x1001;
    board.cpu.set_regs(regs);
    board.cpu.step();
    assert!(board.cpu.is_halted());
    assert_eq!(board.cpu.step(), 0, "a halted core consumes no time");
}

#[test]
fn arithmetic_sets_the_flags_the_manual_gives_it() {
    let board = Board::new();
    // ADD.W D1,D0 with $8000 + $8000: zero, overflow, carry and extend.
    board.boot(&[0xd041]);
    let mut regs = board.cpu.regs();
    regs.d[0] = 0x8000;
    regs.d[1] = 0x8000;
    board.cpu.set_regs(regs);
    board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.d[0] & 0xffff, 0);
    assert!(regs.flag(flags::Z));
    assert!(regs.flag(flags::V));
    assert!(regs.flag(flags::C));
    assert!(regs.flag(flags::X));
    assert!(!regs.flag(flags::N));
}

#[test]
fn addx_only_ever_clears_the_zero_flag() {
    let board = Board::new();
    // ADDX.W D1,D0, adding zero to zero with X clear: Z must stay as it was.
    board.boot(&[0xd141, 0xd141]);
    let mut regs = board.cpu.regs();
    regs.sr |= flags::Z;
    regs.d[0] = 0;
    regs.d[1] = 0;
    board.cpu.set_regs(regs);
    board.cpu.step();
    assert!(
        board.cpu.regs().flag(flags::Z),
        "a zero step leaves Z alone"
    );

    let mut regs = board.cpu.regs();
    regs.sr &= !flags::Z;
    board.cpu.set_regs(regs);
    board.cpu.step();
    assert!(
        !board.cpu.regs().flag(flags::Z),
        "and a clear Z stays clear, which is the point of the rule"
    );
}

#[test]
fn abcd_adds_in_decimal() {
    let board = Board::new();
    board.boot(&[0xc300]); // ABCD D0,D1
    let mut regs = board.cpu.regs();
    regs.d[0] = 0x28;
    regs.d[1] = 0x34;
    regs.sr &= !flags::X;
    regs.sr |= flags::Z;
    board.cpu.set_regs(regs);
    board.cpu.step();
    assert_eq!(board.cpu.regs().d[1] & 0xff, 0x62);
    assert!(!board.cpu.regs().flag(flags::C));
}

#[test]
fn abcd_carries_out_of_the_high_nibble() {
    let board = Board::new();
    board.boot(&[0xc300]);
    let mut regs = board.cpu.regs();
    regs.d[0] = 0x99;
    regs.d[1] = 0x01;
    regs.sr &= !flags::X;
    board.cpu.set_regs(regs);
    board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.d[1] & 0xff, 0x00);
    assert!(regs.flag(flags::C));
    assert!(regs.flag(flags::X));
}

#[test]
fn a_byte_access_through_a7_keeps_the_stack_even() {
    let board = Board::new();
    board.boot(&[0x1f00]); // MOVE.B D0,-(A7)
    board.cpu.step();
    assert_eq!(
        board.cpu.regs().a[7],
        0x2000 - 2,
        "the stack pointer steps by two even for a byte"
    );
}

#[test]
fn movem_round_trips_a_register_list() {
    let board = Board::new();
    // MOVEM.L D0-D2,-(A7) then MOVEM.L (A7)+,D5-D7
    board.boot(&[0x48e7, 0xe000, 0x4cdf, 0x00e0]);
    let mut regs = board.cpu.regs();
    regs.d[0] = 0x1111_1111;
    regs.d[1] = 0x2222_2222;
    regs.d[2] = 0x3333_3333;
    board.cpu.set_regs(regs);
    board.cpu.step();
    assert_eq!(board.cpu.regs().a[7], 0x2000 - 12);
    board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.a[7], 0x2000);
    assert_eq!(regs.d[5], 0x1111_1111);
    assert_eq!(regs.d[6], 0x2222_2222);
    assert_eq!(regs.d[7], 0x3333_3333);
}

#[test]
fn jsr_and_rts_agree_on_the_return_address() {
    let board = Board::new();
    // JSR ($0500).W ; NOP ... at $500: RTS
    board.boot(&[0x4eb8, 0x0500]);
    board.poke_word(0x500, 0x4e75);
    board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.pc, 0x500);
    assert_eq!(board.peek_long(u64::from(regs.a[7])), 0x404);
    board.cpu.step();
    assert_eq!(board.cpu.regs().pc, 0x404);
}

/// Save one core into a snapshot's `cpu` chunk.
fn snapshot(cpu: &M68k) -> Result<Vec<u8>> {
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
fn restore(cpu: &M68k, bytes: &[u8]) -> Result<()> {
    let reader = StateReader::new(bytes)?;
    let chunk = reader.load("cpu", CLASS.name, CLASS.version, &Migrations::new())?;
    let mut r = chunk.reader();
    cpu.load(&mut r)?;
    r.end()
}

#[test]
fn a_snapshot_round_trips_to_an_identical_state() -> Result<()> {
    let board = Board::new();
    board.boot(&[0x7001, 0x7202, 0x4e71]);
    board.cpu.step();
    board.cpu.step();
    board.cpu.set_ipl(3);

    let bytes = snapshot(&board.cpu)?;
    let other = M68k::new(Config::default());
    restore(&other, &bytes)?;

    assert_eq!(other.regs(), board.cpu.regs());
    assert_eq!(other.cycles(), board.cpu.cycles());
    assert_eq!(other.ipl(), 3);
    // The invariant the roadmap actually asks for: a round trip is a fixed
    // point, byte for byte.
    assert_eq!(snapshot(&other)?, bytes);
    Ok(())
}

#[test]
fn a_status_register_with_impossible_bits_cannot_be_set() {
    let board = Board::new();
    board.boot(&[0x4e71]);
    let mut regs = board.cpu.regs();
    regs.sr = 0xffff;
    board.cpu.set_regs(regs);
    assert_eq!(
        board.cpu.regs().sr & !flags::IMPLEMENTED,
        0,
        "bits 11, 12 and 14 have no storage on a 68000"
    );
}

#[test]
fn registers_can_be_named() {
    assert_eq!(Reg::from_name("d3"), Some(Reg::D(3)));
    assert_eq!(Reg::from_name("a7"), Some(Reg::A(7)));
    assert_eq!(Reg::from_name("usp"), Some(Reg::Usp));
    assert_eq!(Reg::from_name("pc"), Some(Reg::Pc));
    assert_eq!(Reg::from_name("d9"), None);
    assert_eq!(Reg::from_name("nope"), None);
    for reg in Reg::ALL {
        assert_eq!(Reg::from_name(&alloc::format!("{reg}")), Some(*reg));
    }
}

#[test]
fn the_register_display_is_readable() {
    let regs = Regs {
        sr: flags::S | flags::IPL | flags::Z,
        ..Regs::default()
    };
    let text = alloc::format!("{regs}");
    assert!(text.contains("D0:00000000"));
    assert!(text.contains("-S--Z--"));
    assert!(text.ends_with("I7"));
}

#[test]
fn the_isa_description_covers_every_row() {
    let text = super::describe_isa();
    assert_eq!(text.lines().count(), isa::TABLE.len());
    assert!(text.contains("MOVE"));
}

#[test]
fn construction_from_properties_validates_what_it_is_given() {
    use crate::core::props::Props;
    use crate::core::registry::Registry;
    use crate::core::space::RequesterId;

    let cpu = M68k::from_props(&Props::new().with("requester", 7u64)).unwrap();
    assert_eq!(cpu.config().requester, RequesterId(7));
    // A property nothing here accepts is an error, not a shrug: a typo that is
    // silently ignored is an afternoon lost.
    assert!(M68k::from_props(&Props::new().with("decimal", true)).is_err());

    let mut registry = Registry::new();
    super::register(&mut registry).unwrap();
    assert!(super::register(&mut registry).is_err(), "no double claims");
    let device = registry.create("cpu.m68k", &Props::new()).unwrap();
    assert_eq!(device.class().name, "cpu.m68k");
}

#[test]
fn realize_does_nothing_outward_because_the_space_has_not_arrived_yet() {
    use crate::core::device::{Deferred, RealizeCtx};
    use crate::core::space::RequesterId;

    // The check that a core has an address space used to live in `realize`. It
    // cannot: the realizer runs `realize` for every device *before* it binds
    // any of them, so a core that refused here would refuse every machine. The
    // check is in `Instance::bind`, and the test for it is below.
    let cpu = M68k::new(Config::default());
    let mut deferred = Deferred::new();
    let mut ctx = RealizeCtx::new("cpu", RequesterId::ANONYMOUS, &mut deferred);
    assert!(cpu.realize(&mut ctx).is_ok());
}

/// A `BuildOptions` and registry that know about this core and `ram`/`rom`.
fn machine_layer() -> (crate::core::Registry, crate::machine::BuildOptions) {
    let mut options = crate::machine::BuildOptions::new();
    options.classes.insert(super::schema());
    for schema in crate::machine::builtin::schemas() {
        options.classes.insert(schema);
    }
    super::bind(&mut options.bindings).expect("nothing else claims cpu.m68k");
    crate::machine::builtin::bind(&mut options.bindings).expect("ram and rom");

    let mut registry = crate::core::Registry::new();
    crate::machine::builtin::register(&mut registry).expect("ram and rom");
    super::register(&mut registry).expect("nothing else claims cpu.m68k");
    (registry, options)
}

#[test]
fn binding_a_core_with_no_address_space_is_a_machine_error() {
    let (registry, options) = machine_layer();
    let text = "machine \"m\" {\n  osc x = 8000000 Hz\n  \
                space mem { width = 24, endian = big }\n  \
                object dram \"ram\" { size = 64K }\n  object cpu \"cpu.m68k\" { clock = x }\n  \
                map mem 0 size 64K = dram\n}\n";
    let err = crate::machine::build("t.machine", text, &registry, &options)
        .expect_err("a core with no `space =` cannot fetch");
    let text = alloc::format!("{err}");
    assert!(text.contains("address space"), "{text}");
}

#[test]
fn the_three_ipl_pins_are_one_sink_seen_through_three_ports() {
    use crate::core::device::Device;
    use crate::core::wire::{Level, Wire, WireId};

    // The shape a 68000 forces: the pins encode a *level*, so nothing can
    // resolve one of them alone, and all three ports therefore have to reach
    // the same object. Three separate sinks would each see a third of the
    // answer and drive a third of the level.
    let board = Board::new();
    let mut wires = Vec::new();
    let mut pins = Vec::new();
    for (line, port) in ["ipl0", "ipl1", "ipl2"].iter().enumerate() {
        let src = WireId::new(line as u64 + 1);
        let pin = board.cpu.sink(port, &[src]).expect("an IPL pin");
        assert_eq!(pin.line, line as u32, "the port names which line it is");
        let wire = Wire::builder()
            .source(src)
            .sink_weak(Arc::downgrade(&pin.sink), pin.line)
            .build();
        pins.push(pin);
        wires.push((wire, src));
    }

    assert_eq!(board.cpu.ipl(), 0);
    wires[1].0.set(wires[1].1, Level::High);
    assert_eq!(board.cpu.ipl(), 2, "IPL1 alone encodes level 2");
    wires[0].0.set(wires[0].1, Level::High);
    assert_eq!(board.cpu.ipl(), 3, "and with IPL0, level 3");
    wires[2].0.set(wires[2].1, Level::High);
    assert_eq!(board.cpu.ipl(), 7, "all three is the non-maskable level");
    wires[1].0.set(wires[1].1, Level::Low);
    assert_eq!(board.cpu.ipl(), 5);
}

#[test]
fn the_pins_a_machine_file_may_name_are_exactly_these_four() {
    use crate::core::device::Device;
    let cpu = M68k::new(Config::default());
    for port in ["ipl0", "ipl1", "ipl2", "reset"] {
        assert!(cpu.sink(port, &[]).is_some(), "`{port}` should be a pin");
    }
    for port in ["irq", "ipl", "ipl3", ""] {
        assert!(
            cpu.sink(port, &[]).is_none(),
            "`{port}` is not a pin this core has"
        );
    }
}

#[test]
fn the_reset_pin_latches_and_the_next_step_runs_the_sequence() {
    use crate::core::device::Device;
    use crate::core::wire::{Level, Wire, WireId};

    let board = Board::new();
    board.boot(&[0x4e71]); // NOP
    board.cpu.step();
    assert_eq!(board.cpu.regs().pc, 0x402);

    let src = WireId::new(1);
    let pin = board.cpu.sink("reset", &[src]).expect("a reset pin");
    let wire = Wire::builder()
        .source(src)
        .sink_weak(Arc::downgrade(&pin.sink), pin.line)
        .build();
    // The latch lives outside the execution lock, so it becomes execution
    // state on the step that consumes it — which is what keeps a device
    // asserting reset from inside an access this core issued out of the core's
    // own critical section.
    wire.set(src, Level::High);
    board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.pc, 0x400, "the sequence re-read vector 1");
    assert_eq!(regs.a[7], 0x2000, "and vector 0");
    assert_eq!(regs.ipl_mask(), 7);
}

/// A controller that answers the acknowledge cycle with a vector, the way a
/// 68000 peripheral that does *not* assert `VPA` does.
#[derive(Debug)]
struct Vectoring {
    vector: u8,
    acknowledged: crate::core::sync::AtomicU32,
}

impl crate::core::wire::IntAck for Vectoring {
    fn acknowledge(&self) -> u32 {
        self.acknowledged
            .fetch_add(1, crate::core::sync::Ordering::Relaxed);
        u32::from(self.vector)
    }
}

#[test]
fn a_controller_that_answers_the_acknowledge_vectors_and_one_that_does_not_autovectors() {
    use crate::core::device::Device;
    use crate::core::sync::{AtomicU32, Ordering};
    use crate::core::wire::IntAck;

    // Autovector first: nothing attached, so the level picks the vector.
    let board = Board::new();
    board.boot(&[0x4e71, 0x4e71]);
    board.poke_long(u64::from(vector::AUTOVECTOR_BASE + 5) * 4, 0x0800);
    // Drop the mask, which a reset leaves at 7, so level 5 gets through.
    let mut regs = board.cpu.regs();
    regs.sr = (regs.sr & !flags::IPL) | (1 << 8);
    board.cpu.set_regs(regs);
    board.cpu.set_ipl(5);
    board.cpu.step();
    assert_eq!(
        board.cpu.regs().pc,
        0x0800,
        "with nothing answering, `VPA` and the autovector for level 5"
    );

    // And now with a controller on the net.
    let board = Board::new();
    board.boot(&[0x4e71, 0x4e71]);
    board.poke_long(64 * 4, 0x0900);
    let mut regs = board.cpu.regs();
    regs.sr = (regs.sr & !flags::IPL) | (1 << 8);
    board.cpu.set_regs(regs);
    let device: Arc<Vectoring> = Arc::new(Vectoring {
        vector: 64,
        acknowledged: AtomicU32::new(0),
    });
    let weak: alloc::sync::Weak<dyn IntAck> = Arc::downgrade(&device) as _;
    board.cpu.attach_int_ack("ipl2", weak);
    board.cpu.set_ipl(5);
    board.cpu.step();
    assert_eq!(
        device.acknowledged.load(Ordering::Relaxed),
        1,
        "the CPU must run the acknowledge cycle"
    );
    assert_eq!(
        board.cpu.regs().pc,
        0x0900,
        "and take the vector the controller drove, not the autovector"
    );
}

#[test]
fn the_scheduler_budget_is_never_overshot_and_the_debt_is_paid_back() {
    // `MOVEM.L` of every register is far longer than a one-cycle budget, so
    // this is the case where a plain `run` reports more than it was handed —
    // which the scheduler rejects outright.
    let board = Board::new();
    board.boot(&[0x48e7, 0xffff, 0x60fa]); // MOVEM.L d0-a7,-(a7) ; BRA .-4
    let before = board.cpu.cycles();
    let mut total = 0u64;
    for _ in 0..64 {
        let used = board.cpu.run_budget(1);
        assert!(used <= 1, "a budget of one cycle reported {used}");
        total += used;
    }
    assert_eq!(total, 64, "every tick of every budget was granted and used");
    assert_eq!(
        board.cpu.cycles() - before,
        total + board.cpu.cycle_debt(),
        "cycles executed but not yet reported are exactly the debt"
    );
}

#[test]
fn a_core_without_a_space_does_nothing_rather_than_panicking() {
    let cpu = M68k::new(Config::default());
    assert_eq!(cpu.step(), 0);
    assert!(cpu.disassemble(0, 4).is_empty());
}

// ---------------------------------------------------------------------------
// The quirks. Each of these is a place where the obvious implementation is
// wrong, so each gets a test that says what the hardware actually does.
// ---------------------------------------------------------------------------

#[test]
fn link_a7_pushes_the_stack_pointer_it_has_just_moved() {
    let board = Board::new();
    board.boot(&[0x4e57, 0xfff0]); // LINK A7,#-16
    board.cpu.step();
    let regs = board.cpu.regs();
    // A7 was decremented to $1ffc before it was read, so that is what landed
    // on the stack — not the $2000 it held on entry.
    assert_eq!(board.peek_long(0x1ffc), 0x1ffc);
    assert_eq!(regs.a[7], 0x1ffc - 16);
}

#[test]
fn unlk_a7_ends_up_holding_what_it_popped() {
    let board = Board::new();
    board.boot(&[0x4e5f]); // UNLK A7
    board.poke_long(0x2000, 0x0000_1234);
    board.cpu.step();
    assert_eq!(
        board.cpu.regs().a[7],
        0x1234,
        "the register is restored after the stack pointer, so it wins"
    );
}

#[test]
fn a_shift_past_the_operand_width_runs_out_of_carry() {
    let board = Board::new();
    // ASR.B D1,D0 with a count of 12 on a negative byte.
    board.boot(&[0xe220, 0xe220]);
    let mut regs = board.cpu.regs();
    regs.d[0] = 0x0000_00f3;
    regs.d[1] = 12;
    regs.sr &= !(flags::X | flags::C);
    board.cpu.set_regs(regs);
    board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.d[0] & 0xff, 0xff, "the result is all sign bits");
    assert!(regs.flag(flags::N));
    assert!(
        !regs.flag(flags::C) && !regs.flag(flags::X),
        "the operand ran out before the count did, so nothing was shifted out"
    );

    // Exactly eight, and the sign bit *is* the last bit out.
    let mut regs = board.cpu.regs();
    regs.d[0] = 0x0000_00f3;
    regs.d[1] = 8;
    board.cpu.set_regs(regs);
    board.cpu.step();
    let regs = board.cpu.regs();
    assert!(regs.flag(flags::C) && regs.flag(flags::X));
}

#[test]
fn a_plain_rotate_leaves_the_extend_flag_alone() {
    let board = Board::new();
    board.boot(&[0xe358]); // ROL.W #1,D0
    let mut regs = board.cpu.regs();
    regs.d[0] = 0x0000_8001;
    regs.sr &= !flags::X;
    board.cpu.set_regs(regs);
    board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.d[0] & 0xffff, 0x0003);
    assert!(regs.flag(flags::C), "the bit rotated out reaches carry");
    assert!(!regs.flag(flags::X), "but never reaches extend");
}

#[test]
fn abcd_decides_its_carry_before_correcting_the_units() {
    let board = Board::new();
    board.boot(&[0xc300]); // ABCD D0,D1
    let mut regs = board.cpu.regs();
    regs.d[0] = 0x2d;
    regs.d[1] = 0x69;
    regs.sr &= !flags::X;
    board.cpu.set_regs(regs);
    board.cpu.step();
    let regs = board.cpu.regs();
    // $2d + $69 is $96 in binary, below $99, so no carry — and only then does
    // the units correction take it to $9c. Testing after the correction would
    // wrongly carry and give $fc.
    assert_eq!(regs.d[1] & 0xff, 0x9c);
    assert!(!regs.flag(flags::C));
}

#[test]
fn sbcd_carries_out_of_the_units_correction() {
    let board = Board::new();
    board.boot(&[0x8300]); // SBCD D0,D1
    let mut regs = board.cpu.regs();
    regs.d[0] = 0xef;
    regs.d[1] = 0xf0;
    regs.sr &= !flags::X;
    board.cpu.set_regs(regs);
    board.cpu.step();
    let regs = board.cpu.regs();
    // $f0 - $ef borrows nothing in binary and needs no tens correction, but
    // subtracting the units correction takes it below zero: the result is $fb
    // *and* the carry is set.
    assert_eq!(regs.d[1] & 0xff, 0xfb);
    assert!(regs.flag(flags::C) && regs.flag(flags::X));
}

#[test]
fn movem_stores_the_register_it_is_walking_as_it_found_it() {
    let board = Board::new();
    // MOVEM.L D0/A7,-(A7): the mask is read backwards for a predecrement, so
    // A7 is bit 0 and D0 is bit 15.
    board.boot(&[0x48e7, 0x8001]);
    board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.a[7], 0x2000 - 8);
    assert_eq!(
        board.peek_long(0x2000 - 4),
        0x2000,
        "A7 is stored with the value it had before the instruction, not the \
         value it has reached by its turn"
    );
    assert_eq!(board.peek_long(0x2000 - 8), regs.d[0]);
}

#[test]
fn movem_reads_one_word_past_the_end_of_its_list() {
    let board = Board::new();
    // MOVEM.W (A0)+,D0 — one register, but two words are read.
    board.boot(&[0x4c98, 0x0001]);
    board.poke_word(0x1000, 0x1234);
    board.poke_word(0x1002, 0x5678);
    let mut regs = board.cpu.regs();
    regs.a[0] = 0x1000;
    board.cpu.set_regs(regs);
    let used = board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.d[0], 0x1234, "sign-extended from the word read");
    assert_eq!(regs.a[0], 0x1002, "the discarded word does not advance A0");
    // The register-list fetch, the transfer, the discarded word, and the
    // final prefetch: four bus cycles and no internal time at all.
    assert_eq!(used, 16);
}

#[test]
fn a_jump_takes_its_last_extension_word_out_of_the_queue() {
    let board = Board::new();
    // JMP $10(A0). The displacement is already prefetched, so the only bus
    // cycles are the two that reload the queue at the target.
    board.boot(&[0x4ee8, 0x0010]);
    board.poke_word(0x1010, 0x4e71);
    let mut regs = board.cpu.regs();
    regs.a[0] = 0x1000;
    board.cpu.set_regs(regs);
    let used = board.cpu.step();
    assert_eq!(board.cpu.regs().pc, 0x1010);
    assert_eq!(
        used, 10,
        "two bus cycles and two internal, not three cycles"
    );
}

#[test]
fn a_branch_to_an_odd_address_pushes_the_target_minus_four() {
    let board = Board::new();
    // BRA to an odd address: the queue reload takes the address error, and by
    // then the program counter is four behind the word it was fetching.
    board.boot(&[0x6011]);
    board.poke_long(u64::from(vector::ADDRESS_ERROR) * 4, 0x0800);
    board.poke_word(0x800, 0x4e71);
    board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.pc, 0x800);
    let sp = regs.a[7];
    assert_eq!(board.peek_long(u64::from(sp) + 2), 0x413, "the odd target");
    assert_eq!(board.peek_long(u64::from(sp) + 10), 0x413 - 4);
    // A fetch, so the special status word says program space.
    assert_eq!(board.peek_word(u64::from(sp)) & 0x1f, 0x1e);
}

#[test]
fn the_reset_instruction_pulses_the_line_without_resetting_the_core() {
    let board = Board::new();
    board.boot(&[0x4e70]); // RESET
    let pulses = board.cpu.reset_pulses();
    let used = board.cpu.step();
    assert_eq!(board.cpu.reset_pulses(), pulses + 1);
    assert_eq!(board.cpu.regs().pc, 0x402, "the processor carries on");
    assert_eq!(used, 132, "124 clocks of RESET asserted, plus the prefetch");
}

#[test]
fn move_usp_reaches_the_bank_that_is_not_in_use() {
    let board = Board::new();
    // MOVE USP,A0 then MOVE A1,USP.
    board.boot(&[0x4e68, 0x4e61]);
    let mut regs = board.cpu.regs();
    regs.usp = 0x1234;
    regs.a[1] = 0x5678;
    board.cpu.set_regs(regs);
    board.cpu.step();
    assert_eq!(board.cpu.regs().a[0], 0x1234);
    board.cpu.step();
    let regs = board.cpu.regs();
    assert_eq!(regs.usp, 0x5678);
    assert_eq!(regs.a[7], 0x2000, "the supervisor stack is untouched");
}

#[test]
fn level_seven_is_edge_triggered_and_fires_once() {
    let board = Board::new();
    board.boot(&[0x4e71, 0x4e71, 0x4e71]);
    board.poke_long(u64::from(vector::AUTOVECTOR_BASE + 7) * 4, 0x0f00);
    board.poke_word(0xf00, 0x4e71);

    board.cpu.set_ipl(7);
    board.cpu.step();
    assert_eq!(board.cpu.regs().pc, 0xf00, "the transition is recognised");

    // The pins are still at seven, and nothing more happens: the edge is gone.
    board.cpu.step();
    assert_eq!(board.cpu.regs().pc, 0xf02, "the NOP in the handler ran");

    // Dropping and re-raising is a new edge.
    board.cpu.set_ipl(0);
    board.cpu.set_ipl(7);
    board.cpu.step();
    assert_eq!(board.cpu.regs().pc, 0xf00);
}

#[test]
fn an_interrupt_during_stop_returns_after_the_stop() {
    let board = Board::new();
    // STOP #$2000, then a MOVEQ that must be what RTE comes back to.
    board.boot(&[0x4e72, 0x2000, 0x7001]);
    board.poke_long(u64::from(vector::AUTOVECTOR_BASE + 3) * 4, 0x0e80);
    board.poke_word(0xe80, 0x4e73); // RTE
    board.cpu.step();
    assert!(board.cpu.is_stopped());

    board.cpu.set_ipl(3);
    board.cpu.step();
    let regs = board.cpu.regs();
    assert!(!board.cpu.is_stopped());
    assert_eq!(regs.pc, 0xe80);
    assert_eq!(
        board.peek_long(u64::from(regs.ssp) + 2),
        0x404,
        "the address pushed is the instruction after the STOP, not inside it"
    );
    // The source has to stop asking, or a level-sensitive interrupt is simply
    // taken again the moment RTE lowers the mask.
    board.cpu.set_ipl(0);
    board.cpu.step(); // RTE
    board.cpu.step(); // MOVEQ #1,D0
    assert_eq!(board.cpu.regs().d[0], 1);
}

#[test]
fn a_reset_from_user_state_does_not_lose_a_stack_pointer() {
    let board = Board::new();
    board.boot(&[0x4e71]);
    let mut regs = board.cpu.regs();
    regs.sr &= !flags::S;
    regs.usp = 0x1500;
    regs.ssp = 0x1900;
    board.cpu.set_regs(regs);
    assert_eq!(board.cpu.reg(Reg::A(7)), 0x1500);

    board.cpu.request_reset();
    board.cpu.step();
    let regs = board.cpu.regs();
    assert!(regs.supervisor());
    // Vector 0 loaded the supervisor stack pointer, and the user one is still
    // in its own bank rather than having been overwritten by it.
    assert_eq!(regs.ssp, 0x2000);
    assert_eq!(regs.a[7], 0x2000);
    assert_eq!(regs.usp, 0x1500);
}

#[test]
fn a_bus_error_is_an_exception_rather_than_a_shrug() {
    // Nothing is mapped above 64 KiB on this board, and the space faults on an
    // unassigned access — which is what a 68000 sees as BERR.
    let board = Board::new();
    board.boot(&[0x2050]); // MOVEA.L (A0),A0
    board.poke_long(u64::from(vector::BUS_ERROR) * 4, 0x0700);
    board.poke_word(0x700, 0x4e71);
    let mut regs = board.cpu.regs();
    regs.a[0] = 0x0080_0000;
    board.cpu.set_regs(regs);
    board.cpu.step();

    let regs = board.cpu.regs();
    assert_eq!(regs.pc, 0x700);
    assert_eq!(board.cpu.bus_faults().0, 1);
    let sp = regs.a[7];
    assert_eq!(sp, 0x2000 - 14, "a bus error uses the group 0 frame too");
    assert_eq!(board.peek_long(u64::from(sp) + 2), 0x0080_0000);
    assert_eq!(
        board.peek_word(u64::from(sp)) & 0x1f,
        0x15,
        "supervisor data, read"
    );
}

#[test]
fn the_disassembler_and_the_interpreter_agree_on_every_instruction_length() {
    // The strongest thing that can be said about a table two consumers read:
    // sweep all 65 536 encodings, and for every one that both decodes and runs
    // to completion, check that the bytes the disassembler says an instruction
    // occupies are the bytes the program counter actually moved. A length
    // computed twice is a length that drifts, and this is what stops it.
    let board = Board::new();
    board.poke_long(0, 0x2000);
    board.poke_long(4, 0x0400);
    board.cpu.step();

    // A register file that keeps every addressing mode inside RAM and away
    // from an odd address, so the sweep measures decode rather than exceptions.
    let mut base = Regs {
        d: [0x0000_0004; 8],
        a: [0x0000_1000; 8],
        usp: 0x0000_1800,
        ssp: 0x0000_2000,
        pc: 0x400,
        sr: flags::S | flags::IPL,
        prefetch: [0, 0],
    };
    base.a[7] = 0x2000;

    let mut checked = 0usize;
    for opcode in 0..=u16::MAX {
        let insn = isa::decode(opcode);
        // Anything that moves the program counter somewhere of its own choosing
        // has no "length" to compare against, and anything that traps measures
        // the exception rather than the instruction.
        if !matches!(
            insn.op,
            isa::Op::Move
                | isa::Op::Movea
                | isa::Op::Moveq
                | isa::Op::Movem
                | isa::Op::Movep
                | isa::Op::Add
                | isa::Op::Addi
                | isa::Op::Addq
                | isa::Op::Adda
                | isa::Op::Addx
                | isa::Op::Sub
                | isa::Op::Subi
                | isa::Op::Subq
                | isa::Op::Suba
                | isa::Op::Subx
                | isa::Op::And
                | isa::Op::Andi
                | isa::Op::Or
                | isa::Op::Ori
                | isa::Op::Eor
                | isa::Op::Eori
                | isa::Op::Cmp
                | isa::Op::Cmpi
                | isa::Op::Cmpa
                | isa::Op::Cmpm
                | isa::Op::Abcd
                | isa::Op::Sbcd
                | isa::Op::Nbcd
                | isa::Op::Clr
                | isa::Op::Neg
                | isa::Op::Negx
                | isa::Op::Not
                | isa::Op::Tst
                | isa::Op::Tas
                | isa::Op::Ext
                | isa::Op::Swap
                | isa::Op::Exg
                | isa::Op::Lea
                | isa::Op::Muls
                | isa::Op::Mulu
                | isa::Op::Btst
                | isa::Op::Bchg
                | isa::Op::Bclr
                | isa::Op::Bset
                | isa::Op::Scc
                | isa::Op::Asl
                | isa::Op::Asr
                | isa::Op::Lsl
                | isa::Op::Lsr
                | isa::Op::Rol
                | isa::Op::Ror
                | isa::Op::Roxl
                | isa::Op::Roxr
                | isa::Op::Nop
                | isa::Op::OriToCcr
                | isa::Op::AndiToCcr
                | isa::Op::EoriToCcr
                | isa::Op::MoveFromSr
                | isa::Op::MoveToCcr
        ) {
            continue;
        }

        // Four extension words of small even values: legal for every mode, and
        // they keep an index or a displacement pointing into RAM.
        for i in 0..5u64 {
            board.poke_word(0x400 + 2 * i, 0x0010);
        }
        board.poke_word(0x400, opcode);
        board.cpu.set_regs(base);
        board.cpu.request_reset();
        board.cpu.step();
        assert_eq!(board.cpu.regs().pc, 0x400);
        board.cpu.set_regs(Regs {
            prefetch: board.cpu.regs().prefetch,
            ..base
        });

        let expected = board.cpu.disassemble(0x400, 1);
        board.cpu.step();
        let after = board.cpu.regs().pc;
        // An instruction that faulted went somewhere else entirely; it has
        // nothing to say about lengths.
        if !(0x400..=0x420).contains(&after) {
            continue;
        }
        assert_eq!(
            after - 0x400,
            u32::from(expected[0].len),
            "{opcode:04x} {}: the disassembler says {} bytes, the interpreter \
             consumed {}",
            expected[0],
            expected[0].len,
            after - 0x400
        );
        checked += 1;
    }
    assert!(checked > 20_000, "only {checked} encodings were exercised");
}

#[test]
fn a_level_seven_pulse_is_not_lost_between_steps() {
    let board = Board::new();
    board.boot(&[0x4e71]);
    board.poke_long(u64::from(vector::AUTOVECTOR_BASE + 7) * 4, 0x0f00);
    board.poke_word(0xf00, 0x4e71);

    // A step covers many clocks, so a source that raises the non-maskable
    // level and lets go again inside one is ordinary, not a race. The edge is
    // latched and must survive to the next instruction boundary.
    board.cpu.set_ipl(7);
    board.cpu.set_ipl(0);
    board.cpu.step();
    assert_eq!(board.cpu.regs().pc, 0xf00);
    assert_eq!(
        board.cpu.ipl(),
        0,
        "and the pins read what they were left at"
    );
}

#[test]
fn an_interrupt_costs_what_the_manual_says() {
    let board = Board::new();
    board.boot(&[0x4e71]);
    board.poke_long(u64::from(vector::AUTOVECTOR_BASE + 4) * 4, 0x0e00);
    board.poke_word(0xe00, 0x4e71);
    let mut regs = board.cpu.regs();
    regs.sr = (regs.sr & !flags::IPL) | (1 << 8);
    board.cpu.set_regs(regs);
    board.cpu.set_ipl(4);
    // MC68000UM Table 8-14: 44(5/3). Three of those five reads and all three
    // writes are on the bus here; the acknowledge cycle is charged rather than
    // driven, because CPU space is a function code the framework has no room
    // for yet.
    assert_eq!(board.cpu.step(), 44);
}

#[test]
fn a_supplied_interrupt_vector_is_consumed_by_the_acknowledge() {
    let board = Board::new();
    board.boot(&[0x4e73, 0x4e73]); // two RTEs, so each handler returns
    board.poke_long(64 * 4, 0x0e00); // the vector the controller supplies
    board.poke_long(u64::from(vector::AUTOVECTOR_BASE + 4) * 4, 0x0f00);
    board.poke_word(0xe00, 0x4e71);
    board.poke_word(0xf00, 0x4e71);
    let mut regs = board.cpu.regs();
    regs.sr = (regs.sr & !flags::IPL) | (1 << 8);
    board.cpu.set_regs(regs);

    board.cpu.set_interrupt_vector(Some(64));
    board.cpu.set_ipl(4);
    board.cpu.step();
    assert_eq!(board.cpu.regs().pc, 0xe00, "the controller's vector");
    assert_eq!(board.cpu.interrupt_vector(), None, "and it was consumed");

    // The next acknowledge autovectors, exactly as it would if no device
    // answered that cycle.
    let mut regs = board.cpu.regs();
    regs.sr = (regs.sr & !flags::IPL) | (1 << 8);
    board.cpu.set_regs(regs);
    board.cpu.step();
    assert_eq!(board.cpu.regs().pc, 0xf00);
}

#[test]
fn a_core_can_be_placed_mid_program_without_reaching_inside_it() {
    // A fresh core owes a reset sequence, which would throw away a register
    // file written before the first step. Anything that resumes a loaded image
    // or replays a trace turns that off through the public API.
    let board = Board::new();
    board.poke_word(0x1000, 0x7042); // MOVEQ #$42,D0
    board.cpu.set_reset_pending(false);
    board.cpu.set_regs(Regs {
        pc: 0x1000,
        sr: flags::S | flags::IPL,
        prefetch: [0x7042, board.peek_word(0x1002)],
        ..Regs::default()
    });
    board.cpu.step();
    assert_eq!(board.cpu.regs().d[0], 0x42);
    assert_eq!(board.cpu.regs().pc, 0x1002);
}

#[test]
fn resume_restarts_a_halted_core() {
    let board = Board::new();
    board.boot(&[0x3080]);
    board.poke_long(u64::from(vector::ADDRESS_ERROR) * 4, 0x0801);
    let mut regs = board.cpu.regs();
    regs.a[0] = 0x1001;
    board.cpu.set_regs(regs);
    board.cpu.step();
    assert!(board.cpu.is_halted());

    board.cpu.resume();
    assert!(!board.cpu.is_halted());
    assert!(board.cpu.step() > 0);
}
