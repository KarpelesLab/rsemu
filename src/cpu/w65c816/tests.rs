//! Hand-written tests for what `SingleStepTests/65816` cannot reach.
//!
//! The corpus starts every vector mid-program with the interrupt lines idle,
//! so the reset sequence, IRQ, NMI and ABORT, the `WAI` wake-up rules, `STP`,
//! `/RDY`, the timing hook, open bus and the snapshot pair are all covered
//! here instead. Everything the corpus does cover — 5 080 400 vectors with
//! pin-level bus traces — is checked there.

use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::{Arbitration, CycleGate, Device, ResetKind};
use crate::core::props::Props;
use crate::core::space::{AddressSpace, RamStore, Region, UnassignedPolicy};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::sync;

use super::{AccessTiming, BusCycle, CLASS, Interrupt, Pins, Regs, W65c816, flags};

/// Banks 0 and 1 of RAM; everything above faults.
struct Machine {
    cpu: W65c816,
    ram: Arc<RamStore>,
}

impl Machine {
    /// A machine whose reset vector points at `$8000`, with `code` there.
    fn new(code: &[u8]) -> Machine {
        let ram = Arc::new(RamStore::new(0x2_0000));
        let m = Machine {
            cpu: W65c816::new(),
            ram,
        };
        m.load(0x8000, code);
        m.load(0xfffc, &[0x00, 0x80]);
        let space = AddressSpace::new("cpu", 24).with_unassigned(UnassignedPolicy::FAULT);
        space
            .topology()
            .map(Region::ram("ram", Arc::clone(&m.ram)), 0)
            .unwrap();
        m.cpu.attach_space(Arc::new(space));
        m
    }

    fn load(&self, at: u64, bytes: &[u8]) {
        for (i, b) in bytes.iter().enumerate() {
            self.ram.write_u8(at + i as u64, *b).unwrap();
        }
    }

    fn peek(&self, at: u64) -> u8 {
        self.ram.read_u8(at).unwrap()
    }

    /// Reset, then `n` more steps.
    fn run(&self, n: usize) {
        for _ in 0..=n {
            self.cpu.step();
        }
    }
}

/// `CLC; XCE` — into native mode.
const NATIVE: [u8; 2] = [0x18, 0xfb];

#[test]
fn reset_enters_emulation_mode_through_fffc() {
    let m = Machine::new(&[0xea]);
    m.cpu.set_regs(Regs {
        e: false,
        p: 0x08,
        d: 0x1234,
        pbr: 5,
        dbr: 6,
        s: 0x3456,
        ..Regs::new()
    });
    let clocks = m.cpu.step();
    let r = m.cpu.regs();
    assert_eq!(clocks, 7);
    assert!(r.e);
    assert_eq!((r.pbr, r.pc, r.dbr, r.d), (0, 0x8000, 0, 0));
    assert_eq!(r.s & 0xff00, 0x0100);
    assert_eq!(r.p & (flags::M | flags::X | flags::I | flags::D), 0x34);
}

#[test]
fn sixteen_bit_decimal_arithmetic_carries_across_all_four_digits() {
    // CLC; XCE; REP #$20; SED; LDA #$1999; CLC; ADC #$0001; SBC #$0999
    let m = Machine::new(&[
        0x18, 0xfb, 0xc2, 0x20, 0xf8, 0xa9, 0x99, 0x19, 0x18, 0x69, 0x01, 0x00, 0xe9, 0x99, 0x09,
    ]);
    m.run(7);
    assert_eq!(m.cpu.regs().a, 0x2000);
    m.cpu.step();
    // C was clear after the ADC, so SBC borrows one: 2000 - 0999 - 1.
    assert_eq!(m.cpu.regs().a, 0x1000);
    assert_ne!(m.cpu.regs().p & flags::C, 0, "no borrow out");
}

#[test]
fn a_native_irq_pushes_the_program_bank_and_returns_through_rti() {
    // CLC; XCE; CLI; NOP ... handler at $9000: RTI.
    let m = Machine::new(&[0x18, 0xfb, 0x58, 0xea, 0xea]);
    m.load(0xffee, &[0x00, 0x90]);
    m.load(0x9000, &[0x40]);
    m.cpu.set_regs(Regs {
        s: 0x01ff,
        ..Regs::new()
    });
    m.run(3); // reset, CLC, XCE, CLI
    m.cpu.set_irq(true);
    m.cpu.step(); // NOP: the poll before its last cycle latches IRQ
    assert_eq!(m.cpu.pending_interrupt(), Some(Interrupt::Irq));
    let before = m.cpu.cycles();
    m.cpu.step();
    assert_eq!(m.cpu.cycles() - before, 8, "native interrupts are 8 cycles");
    let r = m.cpu.regs();
    assert_eq!((r.pbr, r.pc), (0, 0x9000));
    assert_ne!(r.p & flags::I, 0);
    // PBR, PCH, PCL, P on the stack.
    assert_eq!(r.s, 0x01fb);
    assert_eq!(m.peek(0x1ff), 0);
    assert_eq!(m.peek(0x1fe), 0x80);
    assert_eq!(m.peek(0x1fd), 0x04);
    m.cpu.set_irq(false);
    m.cpu.step(); // RTI
    assert_eq!(m.cpu.regs().pc, 0x8004);
    assert_eq!(m.cpu.regs().s, 0x01ff);
}

#[test]
fn an_emulation_mode_irq_pushes_b_clear_and_brk_pushes_it_set() {
    let m = Machine::new(&[0x58, 0xea, 0x00, 0x00]);
    m.load(0xfffe, &[0x00, 0x90]);
    m.load(0x9000, &[0xea]);
    m.run(1); // CLI
    m.cpu.set_irq(true);
    m.cpu.step(); // NOP
    m.cpu.step(); // IRQ
    assert_eq!(m.cpu.regs().pc, 0x9000);
    let s = m.cpu.regs().s;
    assert_eq!(m.peek(u64::from(s) + 1) & flags::B, 0);
    m.cpu.set_irq(false);

    let m = Machine::new(&[0x00, 0x00]);
    m.load(0xfffe, &[0x00, 0x90]);
    m.run(1); // BRK
    let s = m.cpu.regs().s;
    assert_ne!(m.peek(u64::from(s) + 1) & flags::B, 0);
    assert_eq!(
        m.peek(u64::from(s) + 2),
        0x02,
        "BRK returns past its signature"
    );
}

#[test]
fn nmi_is_edge_triggered_and_taken_once() {
    let m = Machine::new(&[0xea; 8]);
    m.load(0xfffa, &[0x00, 0x90]);
    m.load(0x9000, &[0xea; 4]);
    m.run(0);
    m.cpu.set_nmi(true);
    m.cpu.step(); // NOP, edge seen
    m.cpu.step(); // NMI
    assert_eq!(m.cpu.regs().pc, 0x9000);
    // Still held high: no second edge, so no second NMI.
    m.cpu.step();
    m.cpu.step();
    assert_eq!(m.cpu.regs().pc, 0x9002);
    m.cpu.set_nmi(false);
}

#[test]
fn irq_is_masked_by_i() {
    let m = Machine::new(&[0xea; 4]);
    m.run(0);
    m.cpu.set_irq(true);
    m.run(2);
    assert_eq!(m.cpu.regs().pc, 0x8003);
}

#[test]
fn wai_with_i_set_wakes_on_irq_and_continues_without_the_handler() {
    // SEI; WAI; INX
    let m = Machine::new(&[0x78, 0xcb, 0xe8]);
    m.run(2);
    assert!(m.cpu.is_waiting());
    let before = m.cpu.regs().pc;
    assert!(m.cpu.step() > 0, "waiting still passes time");
    assert_eq!(m.cpu.regs().pc, before);
    m.cpu.set_irq(true);
    m.cpu.step(); // wakes and runs INX
    assert!(!m.cpu.is_waiting());
    assert_eq!(m.cpu.regs().x, 1);
    assert_eq!(m.cpu.regs().pc, 0x8003);
}

#[test]
fn wai_with_i_clear_takes_the_interrupt() {
    // CLI; WAI
    let m = Machine::new(&[0x58, 0xcb, 0xe8]);
    m.load(0xfffe, &[0x00, 0x90]);
    m.run(2);
    m.cpu.set_irq(true);
    m.cpu.step();
    assert_eq!(m.cpu.regs().pc, 0x9000);
    let s = m.cpu.regs().s;
    assert_eq!(m.peek(u64::from(s) + 2), 0x02, "returns after the WAI");
}

#[test]
fn stp_stops_until_reset() {
    let m = Machine::new(&[0xdb]);
    m.run(1);
    assert!(m.cpu.is_halted());
    assert_eq!(m.cpu.step(), 0);
    m.cpu.request_reset();
    assert_eq!(m.cpu.step(), 7);
    assert!(!m.cpu.is_halted());
}

#[test]
fn abort_rolls_back_the_registers_and_returns_to_the_aborted_opcode() {
    // CLC; XCE; INX; STA $0200
    let m = Machine::new(&[0x18, 0xfb, 0xe8, 0x8d, 0x00, 0x02]);
    m.load(0xffe8, &[0x00, 0x90]);
    m.run(2);
    let s = m.cpu.regs().s;
    m.cpu.set_abort(true);
    m.cpu.step(); // INX runs its cycles, but is undone
    assert_eq!(m.cpu.regs().x, 0);
    assert_eq!(m.cpu.regs().pc, 0x8002);
    assert_eq!(m.cpu.pending_interrupt(), Some(Interrupt::Abort));
    m.cpu.step();
    let r = m.cpu.regs();
    assert_eq!(r.pc, 0x9000);
    // The return address is the aborted instruction, not the one after.
    assert_eq!(m.peek(u64::from(s) - 1), 0x80);
    assert_eq!(m.peek(u64::from(s) - 2), 0x02);
    m.cpu.set_abort(false);
}

#[test]
fn xce_into_emulation_forces_the_widths_and_the_stack_page() {
    // CLC; XCE; REP #$30; LDX #$1234; LDA #$5678; TCS; SEC; XCE
    let m = Machine::new(&[
        0x18, 0xfb, 0xc2, 0x30, 0xa2, 0x34, 0x12, 0xa9, 0x78, 0x56, 0x1b, 0x38, 0xfb,
    ]);
    m.run(8);
    let r = m.cpu.regs();
    assert!(r.e);
    assert_eq!(r.x, 0x34, "XH is cleared");
    assert_eq!(r.a, 0x5678, "B survives");
    assert_eq!(r.s, 0x0178);
    assert_eq!(r.p & (flags::M | flags::X), flags::M | flags::X);
}

#[test]
fn a_block_move_reexecutes_until_the_count_runs_out() {
    // CLC; XCE; REP #$30; LDA #2; LDX #$1000; LDY #$2000; MVN $00,$00
    let m = Machine::new(&[
        0x18, 0xfb, 0xc2, 0x30, 0xa9, 0x02, 0x00, 0xa2, 0x00, 0x10, 0xa0, 0x00, 0x20, 0x54, 0x00,
        0x00,
    ]);
    m.load(0x1000, &[1, 2, 3, 4]);
    m.run(6);
    for _ in 0..3 {
        m.cpu.step();
    }
    let r = m.cpu.regs();
    assert_eq!(r.a, 0xffff);
    assert_eq!((r.x, r.y), (0x1003, 0x2003));
    assert_eq!(r.pc, 0x8010);
    assert_eq!([m.peek(0x2000), m.peek(0x2001), m.peek(0x2002)], [1, 2, 3]);
    assert_eq!(m.peek(0x2003), 0);
}

/// SNES-shaped timing: ROM (`$8000+`) is 8 clocks, the rest 12, internal 6.
#[derive(Debug, Default)]
struct Snesish(sync::Mutex<Vec<BusCycle>>);

impl AccessTiming for Snesish {
    fn clocks(&self, c: BusCycle) -> u64 {
        self.0.lock().push(c);
        if c.pins.is_internal() {
            6
        } else if c.addr & 0xffff >= 0x8000 {
            8
        } else {
            12
        }
    }
}

#[test]
fn the_timing_hook_prices_every_cycle_and_sees_the_pins() {
    // LDA $0010; NOP
    let m = Machine::new(&[0xad, 0x10, 0x00, 0xea]);
    m.run(0);
    let t = Arc::new(Snesish::default());
    m.cpu.attach_timing(t.clone());
    assert_eq!(m.cpu.step(), 8 + 8 + 8 + 12);
    assert_eq!(m.cpu.step(), 8 + 6);
    let seen = t.0.lock().clone();
    assert!(seen[0].pins.is_opcode_fetch());
    assert!(seen[1].pins.has(Pins::VPA) && !seen[1].pins.has(Pins::VDA));
    assert_eq!(seen[3].addr, 0x0010);
    assert!(seen[3].pins.has(Pins::VDA));
    assert!(seen[5].pins.is_internal());
    assert!(seen.iter().all(|c| c.pins.has(Pins::E)));
    let total = m.cpu.clocks();
    assert_eq!(total, 7 + 36 + 14);
}

/// Holds the core for `n` cycles after the first write it sees.
#[derive(Debug)]
struct Dma(sync::Mutex<(bool, u32)>);

impl CycleGate for Dma {
    fn arbitrate(&self, _cycle: u64, _held: u64, _bus: u8, write: bool) -> Arbitration {
        let mut s = self.0.lock();
        if write && !s.0 {
            s.0 = true;
            return Arbitration::Release;
        }
        if s.0 && s.1 > 0 {
            s.1 -= 1;
            return Arbitration::Steal(0x5a);
        }
        Arbitration::Release
    }
}

#[test]
fn rdy_holds_the_core_and_the_held_cycles_cost_time() {
    // STA $0010; LDA $124000 (unmapped: open bus)
    let m = Machine::new(&[0x8d, 0x10, 0x00, 0xaf, 0x00, 0x40, 0x12]);
    m.run(0);
    m.cpu
        .attach_rdy(Arc::new(Dma(sync::Mutex::new((false, 5)))));
    // The write is the STA's last cycle, so the DMA it starts holds the core
    // from the LDA's opcode fetch on.
    assert_eq!(m.cpu.step(), 4);
    assert_eq!(m.cpu.step(), 5 + 5);
    assert_eq!(m.cpu.cycles(), 7 + 4 + 5, "held cycles are not bus cycles");
    // The DMA left $5a on the bus, then the LDA's own operand fetch drove
    // the bank byte $12 last; that is what the unmapped read returns.
    assert_eq!(m.cpu.regs().a & 0xff, 0x12);
}

#[test]
fn an_unmapped_read_is_open_bus_and_counted() {
    // LDA $123456 (nothing there): the last byte on the bus was the bank.
    let m = Machine::new(&[0xaf, 0x56, 0x34, 0x12]);
    m.run(1);
    assert_eq!(m.cpu.regs().a & 0xff, 0x12);
    assert_eq!(m.cpu.open_bus(), 0x12);
    assert_eq!(m.cpu.bus_faults(), (1, 0x12_3456));
}

#[test]
fn a_16_bit_absolute_read_carries_into_the_next_bank() {
    // CLC; XCE; REP #$20; LDA $ffff with DBR = 0 reads $00ffff and $010000.
    let m = Machine::new(&[0x18, 0xfb, 0xc2, 0x20, 0xad, 0xff, 0xff]);
    m.load(0xffff, &[0xaa]);
    m.load(0x1_0000, &[0xbb]);
    m.run(4);
    assert_eq!(m.cpu.regs().a, 0xbbaa);
}

fn snapshot(cpu: &W65c816) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("cpu", CLASS.name).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("cpu", CLASS.name, CLASS.version).unwrap();
        cpu.save(&mut chunk).unwrap();
    }
    w.to_vec().unwrap()
}

#[test]
fn state_round_trips_through_a_snapshot() {
    let m = Machine::new(&NATIVE);
    m.run(2);
    m.cpu.set_irq(true);
    m.cpu.set_nmi(true);
    m.cpu.step();
    let bytes = snapshot(&m.cpu);

    let restored = W65c816::new();
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("cpu", CLASS.name, CLASS.version, &Migrations::new())
        .unwrap();
    let mut cr = chunk.reader();
    restored.load(&mut cr).unwrap();
    cr.end()
        .expect("the loader read every field the saver wrote");
    assert_eq!(restored.regs(), m.cpu.regs());
    assert_eq!(restored.clocks(), m.cpu.clocks());
    assert!(restored.nmi_pending());
    assert_eq!(
        snapshot(&restored),
        bytes,
        "a round trip must be a fixed point"
    );
}

#[test]
fn a_snapshot_naming_an_unknown_interrupt_is_rejected() {
    let m = Machine::new(&NATIVE);
    // A distinctive register file, so the chunk can be found in the file.
    m.cpu.set_regs(Regs {
        a: 0xa5c3,
        x: 0x5a,
        y: 0x3c,
        e: false,
        p: 0x30,
        ..Regs::new()
    });
    let mut bytes = snapshot(&m.cpu);
    let start = bytes
        .windows(6)
        .position(|w| w == [0xc3, 0xa5, 0x5a, 0x00, 0x3c, 0x00])
        .expect("the chunk payload is in the file");
    // a x y s d pc (12), pbr dbr p e (4), cycles and clocks (16), then the
    // three state bools: the pending tag is byte 35.
    bytes[start + 35] = 9;
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("cpu", CLASS.name, CLASS.version, &Migrations::new())
        .unwrap();
    assert!(W65c816::new().load(&mut chunk.reader()).is_err());
}

#[test]
fn a_cold_reset_restores_power_on() {
    let m = Machine::new(&NATIVE);
    m.run(2);
    m.cpu.reset(ResetKind::Cold);
    assert!(m.cpu.reset_pending());
    assert!(m.cpu.regs().e);
    assert_eq!(m.cpu.clocks(), 0);
}

#[test]
fn props_accept_the_documented_links_and_reject_typos() {
    let props = Props::new().with("engine", "interp");
    assert!(W65c816::from_props(&props).is_ok());
    let props = Props::new().with("timming", "x");
    assert!(W65c816::from_props(&props).is_err());
}

#[test]
fn the_isa_description_covers_every_opcode() {
    let text = super::describe_isa();
    assert_eq!(text.lines().count(), 256);
    assert!(text.contains("54 MVN"));
}

#[test]
fn disassembly_follows_the_live_widths() {
    // CLC; XCE; REP #$20; LDA #$1234
    let m = Machine::new(&[0x18, 0xfb, 0xc2, 0x20, 0xa9, 0x34, 0x12]);
    m.run(3);
    let d = m.cpu.disassemble_here(1);
    assert_eq!(alloc::format!("{}", d[0]), "LDA #$1234");
}

/// A board in a machine file: ROM at `$8000` of bank 0, RAM below it, the
/// core on a 24-bit space with `unassigned = open-bus`.
const BOARD: &str = r#"
machine "w65c816-test" {
  osc clk = 1000000 Hz
  space bus { width = 24, unassigned = open-bus }
  object cpu "cpu.w65c816" { clock = clk, space = bus }
  object rom "rom" { size = 32K, image = "firmware" }
  object ram "ram" { size = 32K }
  map bus 0x000000 size 0x8000 = ram
  map bus 0x008000 size 0x8000 = rom
}
"#;

fn board(src: &str) -> crate::core::Result<crate::machine::Machine> {
    use crate::machine::{BuildOptions, catalog};
    // INC $10; BRA -4, entered from the reset vector.
    let mut image = alloc::vec![0u8; 0x8000];
    image[..4].copy_from_slice(&[0xe6, 0x10, 0x80, 0xfc]);
    image[0x7ffc] = 0x00;
    image[0x7ffd] = 0x80;
    let mut options = BuildOptions::new()
        .with_classes(catalog::classes())
        .with_bindings(catalog::bindings()?);
    options.realize.media.insert("firmware", image.as_slice());
    let registry = catalog::registry()?;
    crate::machine::build("w65c816-test.machine", src, &registry, &options)
}

#[test]
fn the_core_runs_on_a_board_built_from_a_machine_file() {
    use crate::core::clock::GlobalTime;
    use crate::core::space::MemAttrs;
    use crate::core::value::Width;

    let mut m = board(BOARD).expect("the board builds");
    // A millisecond at 1 MHz: a thousand cycles, about 125 trips round a
    // five-cycle INC and a three-cycle BRA.
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");
    let bus = m.space("bus").expect("the space");
    let count = bus.read(0x10, Width::U8, MemAttrs::DEBUG).unwrap();
    assert!(count > 100, "INC ran {count} times");
}

#[test]
fn a_timing_link_to_an_object_that_publishes_none_is_a_config_error() {
    let src = BOARD.replace("space = bus }", "space = bus, timing = ram }");
    let err = board(&src).expect_err("a RAM is not a timing provider");
    let text = alloc::format!("{err}");
    assert!(text.contains("bus access timing"), "{text}");
}
