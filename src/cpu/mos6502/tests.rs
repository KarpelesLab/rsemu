//! Tests for the 6502 core.
//!
//! The interesting ones are not "does `LDA` load" — they are about the bus.
//! Every test that says something about timing asserts the *trace*: which
//! address was touched, in which order, read or written. A cycle count alone
//! would pass with the dummy accesses in the wrong place, and the dummy
//! accesses are the part the NES depends on.

use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;

use crate::core::device::Device;
use crate::core::error::Result;
use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, UnassignedPolicy,
};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::sync::{self, LockRank};
use crate::core::value::Width;
use crate::core::wire::{Wire, WireSource};

use super::*;

// ---------------------------------------------------------------------------
// Harness
// ---------------------------------------------------------------------------

/// One bus access as the CPU made it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Cycle {
    addr: u16,
    value: u8,
    write: bool,
}

impl Cycle {
    const fn r(addr: u16, value: u8) -> Cycle {
        Cycle {
            addr,
            value,
            write: false,
        }
    }

    const fn w(addr: u16, value: u8) -> Cycle {
        Cycle {
            addr,
            value,
            write: true,
        }
    }
}

#[derive(Debug)]
struct MemState {
    ram: Vec<u8>,
    log: Vec<Cycle>,
    /// Reading this address asserts IRQ, mid-instruction — the only way to
    /// test *when* the lines are sampled.
    irq_on_read: Option<u16>,
    cpu: Weak<Mos6502>,
}

/// 64 KiB of RAM that records every access.
#[derive(Debug)]
struct TestBus(sync::Mutex<MemState>);

impl TestBus {
    fn new() -> TestBus {
        TestBus(sync::Mutex::with_rank(
            // Below the CPU's own BUS-ranked lock, which is held across the
            // access — exactly the nesting the ladder is drawn for.
            LockRank::DEVICE,
            MemState {
                ram: alloc::vec![0; 0x1_0000],
                log: Vec::new(),
                irq_on_read: None,
                cpu: Weak::new(),
            },
        ))
    }

    /// Write without logging or side effects, the way a loader would.
    fn poke(&self, addr: u16, bytes: &[u8]) {
        let mut m = self.0.lock();
        for (i, b) in bytes.iter().enumerate() {
            m.ram[(addr as usize + i) & 0xffff] = *b;
        }
    }

    fn peek(&self, addr: u16) -> u8 {
        self.0.lock().ram[addr as usize]
    }

    fn take_log(&self) -> Vec<Cycle> {
        core::mem::take(&mut self.0.lock().log)
    }
}

impl MemOps for TestBus {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let addr = offset as u16;
        let mut fire = None;
        {
            let mut m = self.0.lock();
            for (i, slot) in dst.iter_mut().enumerate() {
                *slot = m.ram[(addr as usize + i) & 0xffff];
                // A debug read must leave no trace at all — not in the log a
                // person is reading, and not in the hooks.
                if !attrs.debug {
                    let value = *slot;
                    m.log.push(Cycle::r(addr.wrapping_add(i as u16), value));
                }
            }
            if !attrs.debug && m.irq_on_read == Some(addr) {
                fire = m.cpu.upgrade();
            }
        }
        // Outward call *after* the critical section, per the re-entrancy
        // contract — and it reaches back into the CPU that is mid-access.
        if let Some(cpu) = fire {
            cpu.set_irq(true);
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let addr = offset as u16;
        let mut m = self.0.lock();
        for (i, b) in src.iter().enumerate() {
            m.ram[(addr as usize + i) & 0xffff] = *b;
            if !attrs.debug {
                m.log.push(Cycle::w(addr.wrapping_add(i as u16), *b));
            }
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::ANY
    }
}

/// A CPU, its bus, and the shortcuts every test wants.
struct Harness {
    cpu: Arc<Mos6502>,
    bus: Arc<TestBus>,
}

impl Harness {
    fn with_config(cfg: Config) -> Harness {
        let bus = Arc::new(TestBus::new());
        let space = AddressSpace::new("cpu", 16).with_unassigned(UnassignedPolicy::FAULT);
        space
            .topology()
            .map(Region::io("ram", 0x1_0000, bus.clone()), 0)
            .expect("64 KiB fits in a 16-bit space");
        let cpu = Arc::new(Mos6502::new(cfg));
        cpu.attach_space(Arc::new(space));
        bus.0.lock().cpu = Arc::downgrade(&cpu);
        Harness { cpu, bus }
    }

    /// A core sitting at `$c000` with `program` loaded there, reset already
    /// done and the trace cleared.
    fn running(program: &[u8]) -> Harness {
        Harness::running_with(Config::default(), program)
    }

    fn running_with(cfg: Config, program: &[u8]) -> Harness {
        let h = Harness::with_config(cfg);
        h.bus.poke(0xfffc, &[0x00, 0xc0]);
        h.bus.poke(0xc000, program);
        let used = h.cpu.step();
        assert_eq!(used, 7, "the reset sequence is seven cycles");
        h.bus.take_log();
        h
    }

    fn step(&self) -> u64 {
        self.cpu.step()
    }

    /// Run one instruction and return its bus trace.
    fn trace(&self) -> Vec<Cycle> {
        self.bus.take_log();
        self.cpu.step();
        self.bus.take_log()
    }

    fn regs(&self) -> Regs {
        self.cpu.regs()
    }

    fn set_regs(&self, f: impl FnOnce(&mut Regs)) {
        let mut r = self.cpu.regs();
        f(&mut r);
        self.cpu.set_regs(r);
    }
}

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------

#[test]
fn the_reset_sequence_reads_the_stack_without_writing_it() {
    let h = Harness::with_config(Config::default());
    h.bus.poke(0xfffc, &[0x34, 0x12]);
    assert!(h.cpu.reset_pending());
    let used = h.cpu.step();
    assert_eq!(used, 7);

    let log = h.bus.take_log();
    assert_eq!(
        log,
        [
            Cycle::r(0x0000, 0),
            Cycle::r(0x0000, 0),
            // Three stack accesses that are reads, not pushes: this is why a
            // 6502 comes up with S = $fd.
            Cycle::r(0x0100, 0),
            Cycle::r(0x01ff, 0),
            Cycle::r(0x01fe, 0),
            Cycle::r(0xfffc, 0x34),
            Cycle::r(0xfffd, 0x12),
        ]
    );
    let regs = h.cpu.regs();
    assert_eq!(regs.pc, 0x1234);
    assert_eq!(regs.s, 0xfd);
    assert!(regs.flag(flags::I));
    assert!(regs.flag(flags::U));
    assert!(!h.cpu.reset_pending());
}

#[test]
fn a_warm_reset_keeps_the_registers_a_cold_one_clears() {
    let h = Harness::running(&[0xea]);
    h.set_regs(|r| {
        r.a = 0x55;
        r.x = 0x66;
    });
    h.cpu.reset(ResetKind::Warm);
    assert_eq!(h.cpu.regs().a, 0x55);
    h.cpu.step();
    assert_eq!(h.cpu.regs().a, 0x55, "a warm reset is a pulse, not a wipe");

    h.cpu.reset(ResetKind::Cold);
    assert_eq!(h.cpu.regs().a, 0x00);
}

#[test]
fn a_reset_drops_the_nmi_latch_but_not_the_input_levels() {
    // The latch is inside the CPU; the levels belong to whatever drives them,
    // and a reset that cleared those would make the machine lie.
    let h = Harness::running(&[0xea]);
    h.cpu.set_irq(true);
    h.cpu.set_nmi(true);
    h.cpu.reset(ResetKind::Warm);
    assert!(h.cpu.irq_asserted(), "the driver still holds IRQ");
    assert!(!h.cpu.nmi_pending(), "but the edge latch is internal");
}

// ---------------------------------------------------------------------------
// Flags
// ---------------------------------------------------------------------------

#[test]
fn loads_set_negative_and_zero() {
    let h = Harness::running(&[0xa9, 0x00, 0xa9, 0x80, 0xa9, 0x01]);
    h.step();
    assert!(h.regs().flag(flags::Z) && !h.regs().flag(flags::N));
    h.step();
    assert!(!h.regs().flag(flags::Z) && h.regs().flag(flags::N));
    h.step();
    assert!(!h.regs().flag(flags::Z) && !h.regs().flag(flags::N));
}

#[test]
fn adc_sets_carry_and_overflow_from_signs() {
    // The four interesting sign combinations, from the classic overflow table.
    let cases = [
        // a, m, carry-in, result, C, V
        (0x50u8, 0x10u8, false, 0x60u8, false, false),
        (0x50, 0x50, false, 0xa0, false, true),
        (0xd0, 0x90, false, 0x60, true, true),
        (0xd0, 0x10, false, 0xe0, false, false),
        (0xff, 0x01, false, 0x00, true, false),
        (0xff, 0x00, true, 0x00, true, false),
    ];
    for (a, m, carry, want, want_c, want_v) in cases {
        let h = Harness::running(&[0x69, m]);
        h.set_regs(|r| {
            r.a = a;
            r.p = if carry { flags::U | flags::C } else { flags::U };
        });
        h.step();
        let regs = h.regs();
        assert_eq!(regs.a, want, "{a:02x} + {m:02x} + {}", u8::from(carry));
        assert_eq!(regs.flag(flags::C), want_c, "carry of {a:02x}+{m:02x}");
        assert_eq!(regs.flag(flags::V), want_v, "overflow of {a:02x}+{m:02x}");
    }
}

#[test]
fn sbc_borrows_through_carry() {
    // Carry set means no borrow, which is the convention that trips everyone.
    let h = Harness::running(&[0xe9, 0x01]);
    h.set_regs(|r| {
        r.a = 0x00;
        r.p = flags::U | flags::C;
    });
    h.step();
    assert_eq!(h.regs().a, 0xff);
    assert!(!h.regs().flag(flags::C), "the subtract borrowed");
    assert!(h.regs().flag(flags::N));

    let h = Harness::running(&[0xe9, 0x01]);
    h.set_regs(|r| {
        r.a = 0x01;
        r.p = flags::U; // carry clear: an extra borrow
    });
    h.step();
    assert_eq!(h.regs().a, 0xff);
    assert!(!h.regs().flag(flags::C));
}

#[test]
fn compare_sets_carry_when_the_register_is_not_smaller() {
    for (reg, m, c, z, n) in [
        (0x10u8, 0x10u8, true, true, false),
        (0x10, 0x0f, true, false, false),
        (0x10, 0x11, false, false, true),
        (0x80, 0x01, true, false, false),
    ] {
        let h = Harness::running(&[0xc9, m]);
        h.set_regs(|r| r.a = reg);
        h.step();
        let regs = h.regs();
        assert_eq!(regs.flag(flags::C), c, "CMP {reg:02x},{m:02x} carry");
        assert_eq!(regs.flag(flags::Z), z, "CMP {reg:02x},{m:02x} zero");
        assert_eq!(regs.flag(flags::N), n, "CMP {reg:02x},{m:02x} negative");
    }
}

#[test]
fn bit_takes_n_and_v_from_the_operand_not_the_result() {
    let h = Harness::running(&[0x24, 0x10]);
    h.bus.poke(0x0010, &[0xc0]);
    h.set_regs(|r| r.a = 0x01);
    h.step();
    let regs = h.regs();
    assert!(regs.flag(flags::Z), "A AND M is zero");
    assert!(regs.flag(flags::N), "bit 7 of M");
    assert!(regs.flag(flags::V), "bit 6 of M");
    assert_eq!(regs.a, 0x01, "BIT does not touch the accumulator");
}

#[test]
fn shifts_move_the_end_bit_into_carry() {
    let h = Harness::running(&[0x0a, 0x4a, 0x2a, 0x6a]);
    h.set_regs(|r| r.a = 0x81);
    h.step(); // ASL A
    assert_eq!(h.regs().a, 0x02);
    assert!(h.regs().flag(flags::C));

    h.set_regs(|r| r.a = 0x03);
    h.step(); // LSR A
    assert_eq!(h.regs().a, 0x01);
    assert!(h.regs().flag(flags::C));

    h.set_regs(|r| {
        r.a = 0x80;
        r.p |= flags::C;
    });
    h.step(); // ROL A, carry in
    assert_eq!(h.regs().a, 0x01);
    assert!(h.regs().flag(flags::C));

    h.set_regs(|r| {
        r.a = 0x01;
        r.p |= flags::C;
    });
    h.step(); // ROR A, carry in
    assert_eq!(h.regs().a, 0x80);
    assert!(h.regs().flag(flags::C));
}

#[test]
fn txs_is_the_transfer_that_sets_no_flags() {
    let h = Harness::running(&[0x9a, 0xba]);
    h.set_regs(|r| {
        r.x = 0x00;
        r.p = flags::U;
    });
    h.step(); // TXS
    assert_eq!(h.regs().s, 0x00);
    assert!(!h.regs().flag(flags::Z), "TXS sets no flags");
    h.step(); // TSX
    assert!(h.regs().flag(flags::Z), "TSX does");
}

// ---------------------------------------------------------------------------
// Timing and dummy accesses
// ---------------------------------------------------------------------------

#[test]
fn absolute_indexed_reads_pay_for_a_page_cross_only_when_it_happens() {
    // No cross: four cycles, and nothing is read but the operand.
    let h = Harness::running(&[0xbd, 0x00, 0x20]); // LDA $2000,X
    h.set_regs(|r| r.x = 0x01);
    h.bus.poke(0x2001, &[0x42]);
    let log = h.trace();
    assert_eq!(
        log,
        [
            Cycle::r(0xc000, 0xbd),
            Cycle::r(0xc001, 0x00),
            Cycle::r(0xc002, 0x20),
            Cycle::r(0x2001, 0x42),
        ]
    );

    // Crossing: five, and the fifth is preceded by a read of the *unfixed*
    // address — the one with the old high byte. Hardware really does touch
    // $2000 here, which is how a mapper sees a phantom access.
    let h = Harness::running(&[0xbd, 0xff, 0x20]); // LDA $20ff,X
    h.set_regs(|r| r.x = 0x01);
    h.bus.poke(0x2100, &[0x42]);
    h.bus.poke(0x2000, &[0x99]);
    let log = h.trace();
    assert_eq!(
        log,
        [
            Cycle::r(0xc000, 0xbd),
            Cycle::r(0xc001, 0xff),
            Cycle::r(0xc002, 0x20),
            Cycle::r(0x2000, 0x99), // dummy, at the unfixed address
            Cycle::r(0x2100, 0x42),
        ]
    );
    assert_eq!(h.regs().a, 0x42);
}

#[test]
fn indexed_writes_always_spend_the_fix_up_cycle() {
    // STA $2000,X does not cross, and still reads $2001 first: the CPU cannot
    // know in advance, so it always pays.
    let h = Harness::running(&[0x9d, 0x00, 0x20]);
    h.set_regs(|r| {
        r.a = 0x42;
        r.x = 0x01;
    });
    let log = h.trace();
    assert_eq!(
        log,
        [
            Cycle::r(0xc000, 0x9d),
            Cycle::r(0xc001, 0x00),
            Cycle::r(0xc002, 0x20),
            Cycle::r(0x2001, 0x00), // dummy read
            Cycle::w(0x2001, 0x42),
        ]
    );
}

#[test]
fn read_modify_write_writes_the_old_value_back_first() {
    // The NMOS double write. `INC $2000` is five cycles: fetch, two operand
    // bytes, read, write-back, write — and a device mapped there sees *two*
    // writes.
    let h = Harness::running(&[0xee, 0x00, 0x20]);
    h.bus.poke(0x2000, &[0x41]);
    let log = h.trace();
    assert_eq!(
        log,
        [
            Cycle::r(0xc000, 0xee),
            Cycle::r(0xc001, 0x00),
            Cycle::r(0xc002, 0x20),
            Cycle::r(0x2000, 0x41),
            Cycle::w(0x2000, 0x41), // the unmodified value, back on the bus
            Cycle::w(0x2000, 0x42),
        ]
    );
}

#[test]
fn zero_page_indexing_reads_the_unindexed_address_and_wraps() {
    let h = Harness::running(&[0xb5, 0xff]); // LDA $ff,X
    h.set_regs(|r| r.x = 0x02);
    h.bus.poke(0x0001, &[0x42]);
    h.bus.poke(0x00ff, &[0x99]);
    let log = h.trace();
    assert_eq!(
        log,
        [
            Cycle::r(0xc000, 0xb5),
            Cycle::r(0xc001, 0xff),
            Cycle::r(0x00ff, 0x99), // dummy at the un-indexed address
            Cycle::r(0x0001, 0x42), // $ff + 2 wraps inside page zero
        ]
    );
    assert_eq!(h.regs().a, 0x42);
}

#[test]
fn indirect_x_reads_the_pointer_twice_and_wraps_in_page_zero() {
    let h = Harness::running(&[0xa1, 0xff]); // LDA ($ff,X)
    h.set_regs(|r| r.x = 0x01);
    // Pointer at $00/$01 because $ff + 1 wraps.
    h.bus.poke(0x0000, &[0x34]);
    h.bus.poke(0x0001, &[0x12]);
    h.bus.poke(0x1234, &[0x42]);
    let log = h.trace();
    assert_eq!(
        log,
        [
            Cycle::r(0xc000, 0xa1),
            Cycle::r(0xc001, 0xff),
            Cycle::r(0x00ff, 0x00), // dummy at the un-indexed pointer
            Cycle::r(0x0000, 0x34),
            Cycle::r(0x0001, 0x12),
            Cycle::r(0x1234, 0x42),
        ]
    );
}

#[test]
fn indirect_y_crosses_pages_like_absolute_indexed() {
    let h = Harness::running(&[0xb1, 0x10]); // LDA ($10),Y
    h.set_regs(|r| r.y = 0x01);
    h.bus.poke(0x0010, &[0xff, 0x20]);
    h.bus.poke(0x2100, &[0x42]);
    let log = h.trace();
    assert_eq!(log.len(), 6, "five cycles plus the page-cross fix-up");
    assert_eq!(log[3], Cycle::r(0x0011, 0x20));
    assert_eq!(log[4], Cycle::r(0x2000, 0x00), "unfixed address");
    assert_eq!(log[5], Cycle::r(0x2100, 0x42));

    // A store through the same mode is six cycles either way.
    let h = Harness::running(&[0x91, 0x10]); // STA ($10),Y
    h.set_regs(|r| {
        r.y = 0x01;
        r.a = 0x42;
    });
    h.bus.poke(0x0010, &[0x00, 0x20]);
    let log = h.trace();
    assert_eq!(log.len(), 6);
    assert_eq!(log[4], Cycle::r(0x2001, 0x00), "dummy read, no cross");
    assert_eq!(log[5], Cycle::w(0x2001, 0x42));
}

#[test]
fn a_branch_costs_two_three_or_four_cycles() {
    // Not taken.
    let h = Harness::running(&[0xd0, 0x10]);
    h.set_regs(|r| r.p |= flags::Z);
    assert_eq!(h.step(), 2);
    assert_eq!(h.regs().pc, 0xc002);

    // Taken, same page.
    let h = Harness::running(&[0xd0, 0x10]);
    assert_eq!(h.step(), 3);
    assert_eq!(h.regs().pc, 0xc012);

    // Taken, crossing a page backwards.
    let h = Harness::running(&[]);
    h.bus.poke(0xc000, &[0x4c, 0x02, 0xc1]); // JMP $c102
    h.bus.poke(0xc102, &[0xd0, 0x80]); // BNE $c084
    h.step();
    let log = h.trace();
    assert_eq!(log.len(), 4);
    assert_eq!(log[2], Cycle::r(0xc104, 0x00), "dummy opcode fetch");
    assert_eq!(log[3], Cycle::r(0xc184, 0x00), "read at the half-fixed PC");
    assert_eq!(h.regs().pc, 0xc084);
}

#[test]
fn jsr_and_rts_agree_about_what_was_pushed() {
    let h = Harness::running(&[0x20, 0x00, 0xd0]); // JSR $d000
    h.bus.poke(0xd000, &[0x60]); // RTS
    let log = h.trace();
    assert_eq!(
        log,
        [
            Cycle::r(0xc000, 0x20),
            Cycle::r(0xc001, 0x00),
            Cycle::r(0x01fd, 0x00), // the internal cycle, visible as a stack read
            Cycle::w(0x01fd, 0xc0), // return address high
            Cycle::w(0x01fc, 0x02), // ... and low: the *last* byte of the JSR
            Cycle::r(0xc002, 0xd0),
        ]
    );
    assert_eq!(h.regs().pc, 0xd000);
    assert_eq!(h.regs().s, 0xfb);

    let log = h.trace();
    assert_eq!(log.len(), 6);
    assert_eq!(h.regs().pc, 0xc003, "RTS returns past the pushed address");
    assert_eq!(h.regs().s, 0xfd);
}

#[test]
fn jmp_indirect_reproduces_the_page_wrap_bug() {
    let h = Harness::running(&[0x6c, 0xff, 0x30]); // JMP ($30ff)
    h.bus.poke(0x30ff, &[0x34]);
    h.bus.poke(0x3000, &[0x12]); // the high byte hardware actually uses
    h.bus.poke(0x3100, &[0x99]); // ... not this one
    let log = h.trace();
    assert_eq!(log.len(), 5);
    assert_eq!(log[4], Cycle::r(0x3000, 0x12));
    assert_eq!(h.regs().pc, 0x1234);
}

#[test]
fn the_stack_wraps_inside_page_one() {
    let h = Harness::running(&[0x48, 0x48]); // PHA twice
    h.set_regs(|r| {
        r.s = 0x00;
        r.a = 0x42;
    });
    h.step();
    assert_eq!(h.bus.peek(0x0100), 0x42);
    assert_eq!(h.regs().s, 0xff);
    h.step();
    assert_eq!(h.bus.peek(0x01ff), 0x42, "S wrapped to the top of page one");
}

// ---------------------------------------------------------------------------
// Interrupts
// ---------------------------------------------------------------------------

#[test]
fn an_irq_is_taken_between_instructions_and_pushes_b_clear() {
    let h = Harness::running(&[0xea, 0xea]);
    h.bus.poke(0xfffe, &[0x00, 0xe0]);
    h.set_regs(|r| r.p = flags::U); // I clear
    h.cpu.set_irq(true);

    // The poll happens inside the NOP; the sequence runs on the next step.
    assert_eq!(h.step(), 2);
    assert_eq!(h.cpu.pending_interrupt(), Some(Interrupt::Irq));

    let log = h.trace();
    assert_eq!(log.len(), 7, "the interrupt sequence is seven cycles");
    assert_eq!(log[0], Cycle::r(0xc001, 0xea), "the discarded opcode fetch");
    assert_eq!(log[1], Cycle::r(0xc001, 0xea), "read again, PC unchanged");
    assert_eq!(log[2], Cycle::w(0x01fd, 0xc0));
    assert_eq!(log[3], Cycle::w(0x01fc, 0x01));
    assert_eq!(log[4], Cycle::w(0x01fb, flags::U));
    assert_eq!(log[5], Cycle::r(0xfffe, 0x00));
    assert_eq!(log[6], Cycle::r(0xffff, 0xe0));
    assert_eq!(h.regs().pc, 0xe000);
    assert!(h.regs().flag(flags::I), "the sequence sets I");
    let pushed = h.bus.peek(0x01fb);
    assert_eq!(pushed & flags::B, 0, "B is clear for a hardware interrupt");
}

#[test]
fn brk_pushes_b_set_and_returns_two_bytes_on() {
    let h = Harness::running(&[0x00, 0xff]); // BRK, then its signature byte
    h.bus.poke(0xfffe, &[0x00, 0xe0]);
    assert_eq!(h.step(), 7);
    assert_eq!(h.regs().pc, 0xe000);
    assert_eq!(h.bus.peek(0x01fd), 0xc0);
    assert_eq!(h.bus.peek(0x01fc), 0x02, "BRK returns to PC + 2");
    assert_ne!(h.bus.peek(0x01fb) & flags::B, 0, "B is set by a BRK");
}

#[test]
fn an_nmi_hijacks_a_brk_but_leaves_the_pushed_flags_alone() {
    // NESdev, CPU interrupts: an NMI asserted during the first four cycles of
    // a BRK steals the vector while the sequence otherwise runs unchanged.
    let h = Harness::running(&[0x00, 0xff]);
    h.bus.poke(0xfffa, &[0x00, 0xf0]); // NMI vector
    h.bus.poke(0xfffe, &[0x00, 0xe0]); // IRQ/BRK vector
    h.cpu.set_nmi(true);
    assert_eq!(h.step(), 7);
    assert_eq!(h.regs().pc, 0xf000, "the NMI vector won");
    assert_ne!(h.bus.peek(0x01fb) & flags::B, 0, "still a BRK on the stack");
    assert!(!h.cpu.nmi_pending(), "the latch was consumed");
}

#[test]
fn an_nmi_arriving_after_the_pushes_does_not_hijack() {
    // The line is sampled once, at the end of the fourth cycle. Assert it
    // afterwards and the BRK keeps its own vector; the NMI is serviced next.
    let h = Harness::running(&[0x00, 0xff, 0xea]);
    h.bus.poke(0xfffa, &[0x00, 0xf0]);
    h.bus.poke(0xfffe, &[0x00, 0xe0]);
    h.step();
    assert_eq!(h.regs().pc, 0xe000);
    h.cpu.set_nmi(true);
    assert!(h.cpu.nmi_pending());
}

#[test]
fn an_nmi_is_edge_triggered_and_latches_until_serviced() {
    let h = Harness::running(&[0xea, 0xea, 0xea]);
    h.bus.poke(0xfffa, &[0x00, 0xf0]);
    h.cpu.set_nmi(true);
    h.cpu.set_nmi(false); // a pulse: the latch survives the line dropping
    assert!(h.cpu.nmi_pending());
    h.step(); // NOP polls and latches
    assert_eq!(h.cpu.pending_interrupt(), Some(Interrupt::Nmi));
    assert_eq!(h.step(), 7);
    assert_eq!(h.regs().pc, 0xf000);
    assert!(!h.cpu.nmi_pending());

    // A level that never falls produces no second interrupt.
    h.cpu.set_nmi(true);
    h.cpu.set_nmi(true);
    assert!(h.cpu.nmi_pending(), "the first edge latched");
    h.cpu.set_nmi(false);
    h.cpu.set_nmi(false);
}

#[test]
fn an_nmi_outranks_a_simultaneous_irq() {
    let h = Harness::running(&[0xea, 0xea]);
    h.bus.poke(0xfffa, &[0x00, 0xf0]);
    h.bus.poke(0xfffe, &[0x00, 0xe0]);
    h.set_regs(|r| r.p = flags::U);
    h.cpu.set_irq(true);
    h.cpu.set_nmi(true);
    h.step();
    assert_eq!(h.cpu.pending_interrupt(), Some(Interrupt::Nmi));
    h.step();
    assert_eq!(h.regs().pc, 0xf000);
}

#[test]
fn cli_delays_the_irq_by_one_instruction() {
    // NESdev, CPU interrupts: CLI, SEI and PLP change I *after* the poll, so
    // an IRQ pending across a CLI is not taken until the instruction after it.
    let h = Harness::running(&[0x58, 0xea, 0xea]); // CLI, NOP, NOP
    h.bus.poke(0xfffe, &[0x00, 0xe0]);
    h.set_regs(|r| r.p = flags::U | flags::I);
    h.cpu.set_irq(true);

    h.step(); // CLI: polled while I was still set
    assert!(!h.regs().flag(flags::I));
    assert_eq!(
        h.cpu.pending_interrupt(),
        None,
        "delayed by one instruction"
    );

    h.step(); // NOP: now the poll sees I clear
    assert_eq!(h.cpu.pending_interrupt(), Some(Interrupt::Irq));
}

#[test]
fn rti_affects_the_irq_immediately() {
    // RTI pulls P on its fourth cycle, which is before the final poll — so
    // unlike PLP it takes effect at once.
    let h = Harness::running(&[0x40]); // RTI
    h.bus.poke(0xfffe, &[0x00, 0xe0]);
    h.set_regs(|r| {
        r.p = flags::U | flags::I;
        r.s = 0xfa;
    });
    // Stack: P (I clear), then the return address.
    h.bus.poke(0x01fb, &[flags::U, 0x00, 0xc0]);
    h.cpu.set_irq(true);
    assert_eq!(h.step(), 6);
    assert_eq!(h.regs().pc, 0xc000);
    assert_eq!(h.cpu.pending_interrupt(), Some(Interrupt::Irq));
}

#[test]
fn plp_delays_the_irq_the_way_cli_does() {
    let h = Harness::running(&[0x28, 0xea]); // PLP, NOP
    h.bus.poke(0xfffe, &[0x00, 0xe0]);
    h.set_regs(|r| {
        r.p = flags::U | flags::I;
        r.s = 0xfc;
    });
    h.bus.poke(0x01fd, &[flags::U]); // a status byte with I clear
    h.cpu.set_irq(true);
    h.step();
    assert!(!h.regs().flag(flags::I));
    assert_eq!(h.cpu.pending_interrupt(), None);
    h.step();
    assert_eq!(h.cpu.pending_interrupt(), Some(Interrupt::Irq));
}

#[test]
fn a_taken_branch_does_not_poll_before_its_last_cycle() {
    // "Interrupts are always polled before the second CPU cycle (the operand
    // fetch), but not before the third CPU cycle on a taken branch" — NESdev.
    // The IRQ is asserted by a read of the operand byte, which happens *after*
    // the second cycle's poll, so only a third-cycle poll could catch it.
    let h = Harness::running(&[0xd0, 0x02, 0xea, 0xea, 0xea]);
    h.bus.poke(0xfffe, &[0x00, 0xe0]);
    h.set_regs(|r| r.p = flags::U);
    h.bus.0.lock().irq_on_read = Some(0xc001); // the branch's operand byte

    h.step(); // BNE, taken, three cycles
    assert!(h.cpu.irq_asserted(), "the operand read raised IRQ");
    assert_eq!(
        h.cpu.pending_interrupt(),
        None,
        "the third cycle of a taken branch does not poll"
    );

    // The same trigger on a three-cycle non-branch instruction *is* caught,
    // which is what makes the branch case a quirk rather than a bug here.
    let h = Harness::running(&[0xa5, 0x10]); // LDA $10, three cycles
    h.set_regs(|r| r.p = flags::U);
    h.bus.0.lock().irq_on_read = Some(0xc001);
    h.step();
    assert_eq!(h.cpu.pending_interrupt(), Some(Interrupt::Irq));
}

#[test]
fn the_interrupt_sequence_itself_does_not_poll() {
    // At least one instruction of the handler runs before another interrupt.
    let h = Harness::running(&[0xea]);
    h.bus.poke(0xfffe, &[0x00, 0xe0]);
    h.bus.poke(0xe000, &[0xea]);
    h.set_regs(|r| r.p = flags::U);
    h.cpu.set_irq(true);
    h.step(); // NOP, latches
    h.step(); // the sequence
    assert_eq!(h.cpu.pending_interrupt(), None);
    assert!(
        h.regs().flag(flags::I),
        "and I is set, so it stays that way"
    );
}

// ---------------------------------------------------------------------------
// Decimal mode
// ---------------------------------------------------------------------------

#[test]
fn decimal_mode_is_a_property_of_the_part_not_a_build_flag() {
    // The same opcode, the same flags, two different chips.
    let plain = Harness::running_with(Config::NMOS_6502, &[0x69, 0x01]);
    plain.set_regs(|r| {
        r.a = 0x09;
        r.p = flags::U | flags::D;
    });
    plain.step();
    assert_eq!(plain.regs().a, 0x10, "BCD: 09 + 01 = 10");

    let nes = Harness::running_with(Config::RP2A03, &[0x69, 0x01]);
    nes.set_regs(|r| {
        r.a = 0x09;
        r.p = flags::U | flags::D;
    });
    nes.step();
    assert_eq!(nes.regs().a, 0x0a, "the RP2A03 has no BCD adder");
}

#[test]
fn decimal_adc_carries_out_of_the_high_nibble() {
    let h = Harness::running(&[0x69, 0x01]);
    h.set_regs(|r| {
        r.a = 0x99;
        r.p = flags::U | flags::D;
    });
    h.step();
    assert_eq!(h.regs().a, 0x00);
    assert!(h.regs().flag(flags::C));
    // Z comes from the *binary* sum, which is $9a — not zero. This asymmetry
    // is the one thing everybody gets wrong (Bruce Clark, 6502.org).
    assert!(!h.regs().flag(flags::Z));
}

#[test]
fn decimal_sbc_takes_every_flag_from_the_binary_result() {
    let h = Harness::running(&[0xe9, 0x01]);
    h.set_regs(|r| {
        r.a = 0x00;
        r.p = flags::U | flags::D | flags::C;
    });
    h.step();
    assert_eq!(h.regs().a, 0x99, "BCD: 00 - 01 = 99 with a borrow");
    assert!(!h.regs().flag(flags::C));
    assert!(h.regs().flag(flags::N), "N is the binary result's sign");
    assert!(!h.regs().flag(flags::Z));
}

#[test]
fn the_decimal_flag_still_exists_on_a_part_without_decimal_mode() {
    let h = Harness::running_with(Config::RP2A03, &[0xf8, 0x08]); // SED, PHP
    h.step();
    assert!(h.regs().flag(flags::D), "SED still sets the flag");
    h.step();
    assert_ne!(h.bus.peek(0x01fd) & flags::D, 0, "and PHP still pushes it");
}

// ---------------------------------------------------------------------------
// Undocumented opcodes
// ---------------------------------------------------------------------------

#[test]
fn lax_loads_both_registers_and_sax_stores_their_and() {
    let h = Harness::running(&[0xa7, 0x10, 0x87, 0x11]); // LAX $10, SAX $11
    h.bus.poke(0x0010, &[0x80]);
    h.step();
    assert_eq!(h.regs().a, 0x80);
    assert_eq!(h.regs().x, 0x80);
    assert!(h.regs().flag(flags::N));

    h.set_regs(|r| {
        r.a = 0xf0;
        r.x = 0x3c;
    });
    h.step();
    assert_eq!(h.bus.peek(0x0011), 0x30);
}

#[test]
fn the_combined_read_modify_writes_do_both_halves() {
    // SLO: ASL the memory, then OR it into A. Five cycles for zero page, and
    // the double write is still there.
    let h = Harness::running(&[0x07, 0x10]);
    h.bus.poke(0x0010, &[0x41]);
    h.set_regs(|r| r.a = 0x02);
    let log = h.trace();
    assert_eq!(log.len(), 5);
    assert_eq!(log[3], Cycle::w(0x0010, 0x41));
    assert_eq!(log[4], Cycle::w(0x0010, 0x82));
    assert_eq!(h.regs().a, 0x82);
    assert!(!h.regs().flag(flags::C));

    // ISC: increment, then subtract.
    let h = Harness::running(&[0xe7, 0x10]);
    h.bus.poke(0x0010, &[0x0f]);
    h.set_regs(|r| {
        r.a = 0x20;
        r.p = flags::U | flags::C;
    });
    h.step();
    assert_eq!(h.bus.peek(0x0010), 0x10);
    assert_eq!(h.regs().a, 0x10);

    // DCP: decrement, then compare.
    let h = Harness::running(&[0xc7, 0x10]);
    h.bus.poke(0x0010, &[0x43]);
    h.set_regs(|r| r.a = 0x42);
    h.step();
    assert_eq!(h.bus.peek(0x0010), 0x42);
    assert!(h.regs().flag(flags::Z) && h.regs().flag(flags::C));
}

#[test]
fn the_immediate_illegals_fold_a_shift_into_the_and() {
    // ANC: AND, then carry takes the sign.
    let h = Harness::running(&[0x0b, 0xff]);
    h.set_regs(|r| r.a = 0x80);
    h.step();
    assert_eq!(h.regs().a, 0x80);
    assert!(h.regs().flag(flags::C) && h.regs().flag(flags::N));

    // ALR: AND, then LSR.
    let h = Harness::running(&[0x4b, 0xff]);
    h.set_regs(|r| r.a = 0x03);
    h.step();
    assert_eq!(h.regs().a, 0x01);
    assert!(h.regs().flag(flags::C));

    // ARR: AND, then a rotate through the adder — C is bit 6 of the result
    // and V is bit 6 XOR bit 5.
    let h = Harness::running(&[0x6b, 0xff]);
    h.set_regs(|r| {
        r.a = 0xc0;
        r.p = flags::U;
    });
    h.step();
    assert_eq!(h.regs().a, 0x60);
    assert!(h.regs().flag(flags::C), "bit 6 of the result");
    assert!(!h.regs().flag(flags::V), "bits 6 and 5 agree");

    // SBX: (A AND X) - imm into X, flags like a compare.
    let h = Harness::running(&[0xcb, 0x02]);
    h.set_regs(|r| {
        r.a = 0xff;
        r.x = 0x05;
    });
    h.step();
    assert_eq!(h.regs().x, 0x03);
    assert!(h.regs().flag(flags::C));
}

#[test]
fn ane_and_lxa_use_the_configured_magic_constant() {
    // Documented-unstable: the constant is a property, and $ee is what
    // SingleStepTests was generated with.
    let h = Harness::running(&[0x8b, 0xff]); // ANE #$ff
    h.set_regs(|r| {
        r.a = 0x00;
        r.x = 0xff;
    });
    h.step();
    assert_eq!(h.regs().a, 0xee, "(0 | $ee) & $ff & $ff");

    let h = Harness::running_with(Config::NMOS_6502.with_magic(0x00), &[0x8b, 0xff]);
    h.set_regs(|r| {
        r.a = 0x00;
        r.x = 0xff;
    });
    h.step();
    assert_eq!(h.regs().a, 0x00, "a different chip, a different constant");

    let h = Harness::running(&[0xab, 0x0f]); // LXA #$0f
    h.set_regs(|r| r.a = 0x00);
    h.step();
    assert_eq!(h.regs().a, 0x0e);
    assert_eq!(h.regs().x, 0x0e);
}

#[test]
fn las_ands_memory_with_the_stack_pointer_into_three_registers() {
    let h = Harness::running(&[0xbb, 0x00, 0x20]); // LAS $2000,Y
    h.bus.poke(0x2000, &[0xf0]);
    h.set_regs(|r| {
        r.y = 0x00;
        r.s = 0x3f;
    });
    h.step();
    assert_eq!(h.regs().a, 0x30);
    assert_eq!(h.regs().x, 0x30);
    assert_eq!(h.regs().s, 0x30);
}

#[test]
fn the_unstable_stores_and_the_high_address_byte_into_the_value() {
    // SHY $2000,X with X = 0: stores Y AND ($20 + 1) = Y AND $21.
    let h = Harness::running(&[0x9c, 0x00, 0x20]);
    h.set_regs(|r| {
        r.y = 0xff;
        r.x = 0x00;
    });
    h.step();
    assert_eq!(h.bus.peek(0x2000), 0x21);

    // SHX $20ff,Y with Y = 1 crosses a page. The value being stored is on the
    // bus while the high address byte is driven, so the store lands at
    // (value << 8) | low instead of at $2100.
    let h = Harness::running(&[0x9e, 0xff, 0x20]);
    h.set_regs(|r| {
        r.x = 0x0f;
        r.y = 0x01;
    });
    h.step();
    let value = 0x0f & 0x21;
    assert_eq!(h.bus.peek(u16::from(value) << 8), value);
    assert_eq!(h.bus.peek(0x2100), 0x00, "not the arithmetic address");
}

#[test]
fn tas_loads_the_stack_pointer_whatever_else_it_does() {
    let h = Harness::running(&[0x9b, 0x00, 0x20]); // TAS $2000,Y
    h.set_regs(|r| {
        r.a = 0xf0;
        r.x = 0x3f;
        r.y = 0x00;
    });
    h.step();
    assert_eq!(h.regs().s, 0x30, "S = A AND X");
    assert_eq!(h.bus.peek(0x2000), 0x30 & 0x21);
}

#[test]
fn jam_freezes_the_core_until_reset() {
    let h = Harness::running(&[0x02]);
    let log = h.trace();
    assert!(h.cpu.is_halted());
    // Two fetch cycles, then the stuck bus pattern the corpus records.
    assert_eq!(log[0], Cycle::r(0xc000, 0x02));
    assert_eq!(log[1], Cycle::r(0xc001, 0x00));
    assert_eq!(log[2].addr, 0xffff);
    assert_eq!(log[3].addr, 0xfffe);
    assert_eq!(log[4].addr, 0xfffe);
    assert!(log[5..].iter().all(|c| c.addr == 0xffff && !c.write));
    assert_eq!(h.regs().pc, 0xc001, "PC advanced past the opcode only");
    assert_eq!(h.step(), 0, "a jammed core charges nothing");
    assert_eq!(h.cpu.run(1000), 0, "and cannot be run out of it");

    h.cpu.reset(ResetKind::Warm);
    assert!(!h.cpu.is_halted());
    assert_eq!(h.cpu.step(), 7, "reset is the only way out");
}

#[test]
fn the_undocumented_nops_still_cost_their_cycles() {
    // Their whole reason for existing in real code: a two-byte, four-cycle
    // delay that touches nothing.
    let h = Harness::running(&[0x1c, 0xff, 0x20]); // NOP $20ff,X
    h.set_regs(|r| r.x = 0x01);
    let log = h.trace();
    assert_eq!(log.len(), 5, "and it pays for the page cross, like a read");
    assert_eq!(h.regs().pc, 0xc003);
}

// ---------------------------------------------------------------------------
// The bus edges
// ---------------------------------------------------------------------------

#[test]
fn a_refused_access_reads_open_bus_and_is_counted() {
    // Nothing is mapped above $8000 here, and the space faults. A 6502 has no
    // bus-error input, so the read returns the last value on the bus — but the
    // counter says it happened.
    let bus = Arc::new(TestBus::new());
    let space = AddressSpace::new("cpu", 16).with_unassigned(UnassignedPolicy::FAULT);
    space
        .topology()
        .map(Region::io("ram", 0x8000, bus.clone()), 0)
        .unwrap();
    let cpu = Mos6502::new(Config::default());
    cpu.attach_space(Arc::new(space));
    bus.poke(0x0000, &[0xad, 0x00, 0x90]); // LDA $9000, from $0000
    cpu.set_regs(Regs {
        pc: 0x0000,
        ..Regs::new()
    });
    cpu.request_reset();
    // Skip the reset sequence: its vector fetch is itself unmapped here.
    cpu.session.lock().state.reset_pending = false;
    cpu.step();
    let (faults, last) = cpu.bus_faults();
    assert_eq!(faults, 1);
    assert_eq!(last, 0x9000);
    assert_eq!(cpu.regs().a, 0x90, "the last byte that was on the bus");
}

#[test]
fn a_debug_read_leaves_no_trace() {
    let h = Harness::running(&[0xa9, 0x42]);
    h.bus.take_log();
    let listing = h.cpu.disassemble(0xc000, 1);
    assert_eq!(alloc::format!("{}", listing[0]), "LDA #$42");
    assert!(
        h.bus.take_log().is_empty(),
        "disassembly must not disturb the machine"
    );
}

// ---------------------------------------------------------------------------
// Device plumbing
// ---------------------------------------------------------------------------

#[test]
fn save_and_load_round_trip_to_an_identical_state() -> Result<()> {
    let h = Harness::running(&[0xa9, 0x42, 0xaa, 0x48, 0x58]);
    h.cpu.set_irq(true);
    h.cpu.set_nmi(true);
    h.cpu.run(8);

    let mut shape = MachineShape::new();
    shape.add_device("cpu", CLASS.name)?;
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("cpu", CLASS.name, CLASS.version)?;
        h.cpu.save(&mut chunk)?;
    }
    let bytes = w.to_vec()?;

    // A fresh core with the same configuration, loaded from those bytes.
    let restored = Mos6502::new(h.cpu.config());
    let reader = StateReader::new(&bytes)?;
    let chunk = reader.load("cpu", CLASS.name, CLASS.version, &Migrations::new())?;
    let mut cr = chunk.reader();
    restored.load(&mut cr)?;
    cr.end()?;

    assert_eq!(restored.regs(), h.cpu.regs());
    assert_eq!(restored.cycles(), h.cpu.cycles());
    assert_eq!(restored.pending_interrupt(), h.cpu.pending_interrupt());
    assert_eq!(restored.nmi_pending(), h.cpu.nmi_pending());
    assert_eq!(restored.irq_asserted(), h.cpu.irq_asserted());

    // The hash the invariant actually asks for: save the restored core and
    // compare the bytes.
    let mut shape2 = MachineShape::new();
    shape2.add_device("cpu", CLASS.name)?;
    let mut w2 = StateWriter::new(shape2);
    {
        let mut chunk = w2.chunk("cpu", CLASS.name, CLASS.version)?;
        restored.save(&mut chunk)?;
    }
    assert_eq!(w2.to_vec()?, bytes, "a round trip must be a fixed point");
    Ok(())
}

#[test]
fn construction_from_properties_validates_what_it_is_given() {
    let cpu = Mos6502::from_props(&Props::new().with("decimal", false)).unwrap();
    assert!(!cpu.config().decimal);
    assert_eq!(cpu.config().magic, 0xee);

    let cpu = Mos6502::from_props(&Props::new().with("magic", 0u64)).unwrap();
    assert_eq!(cpu.config().magic, 0x00);

    // A typo is an error, not a shrug.
    let err = Mos6502::from_props(&Props::new().with("decimel", true)).unwrap_err();
    assert!(alloc::format!("{err}").contains("decimel"));

    // And so is a value that cannot be a byte.
    assert!(Mos6502::from_props(&Props::new().with("magic", 0x100u64)).is_err());
}

#[test]
fn realize_does_nothing_outward_and_bind_is_what_needs_a_space() {
    // Realize runs before the machine hands anything out, so it cannot check
    // for an address space — the check belongs where the space arrives. A CPU
    // with nowhere to fetch from is still a config error, just one instant
    // later (`ROADMAP.md` §4.4).
    let cpu = Mos6502::new(Config::default());
    let mut deferred = crate::core::device::Deferred::new();
    let mut ctx = RealizeCtx::new("cpu", RequesterId::ANONYMOUS, &mut deferred);
    assert!(cpu.realize(&mut ctx).is_ok());
    assert!(cpu.space().is_none(), "realize attaches nothing");
}

#[test]
fn the_class_is_registrable_and_constructs_through_the_registry() {
    let mut reg = Registry::new();
    register(&mut reg).unwrap();
    assert!(reg.get("cpu.mos6502").is_some());
    assert!(register(&mut reg).is_err(), "twice is a collision");

    let device = reg.create("cpu.mos6502", &Props::new()).unwrap();
    assert_eq!(device.class().name, "cpu.mos6502");
}

#[test]
fn the_register_file_is_addressable_by_name() {
    let h = Harness::running(&[]);
    h.cpu.set_reg(Reg::A, 0x42);
    assert_eq!(h.cpu.reg(Reg::A), 0x42);
    h.cpu.set_reg(Reg::Pc, 0x1234);
    assert_eq!(h.cpu.reg(Reg::Pc), 0x1234);
    assert_eq!(Reg::from_name("pc"), Some(Reg::Pc));
    assert_eq!(Reg::from_name("q"), None);
    assert_eq!(Reg::Pc.width(), Width::U16);
    assert_eq!(Reg::A.width(), Width::U8);
    assert_eq!(Reg::ALL.len(), 6);
}

#[test]
fn an_interrupt_pin_wire_ors_its_sources() {
    let h = Harness::running(&[]);
    let apu = WireId::new(1);
    let cart = WireId::new(2);
    let pin = InterruptPin::new(h.cpu.clone(), Interrupt::Irq, &[apu, cart]);

    pin.set_level(apu, 0, Level::High);
    assert!(h.cpu.irq_asserted());
    pin.set_level(cart, 0, Level::High);
    pin.set_level(apu, 0, Level::Low);
    assert!(h.cpu.irq_asserted(), "the cartridge still holds the line");
    pin.set_level(cart, 0, Level::Low);
    assert!(!h.cpu.irq_asserted());
    assert_eq!(pin.which(), Interrupt::Irq);
}

#[test]
fn the_isa_description_covers_every_encoding() {
    let text = describe_isa();
    assert_eq!(text.lines().count(), 256);
    assert!(text.contains("a9  LDA"));
    assert!(text.contains("03 *SLO"));
    assert!(text.contains("8b !ANE"));
}

#[test]
fn a_short_program_runs_to_a_known_state() {
    // The end-to-end check: a loop, a store, a subroutine, an interrupt.
    let program = [
        0xa2, 0x03, // LDX #$03
        0xa9, 0x00, // LDA #$00
        0x18, // CLC
        0x69, 0x05, // loop: ADC #$05
        0xca, // DEX
        0xd0, 0xfb, // BNE loop
        0x8d, 0x00, 0x02, // STA $0200
        0x02, // JAM
    ];
    let h = Harness::running(&program);
    let used = h.cpu.run(1000);
    assert!(h.cpu.is_halted());
    assert_eq!(h.bus.peek(0x0200), 0x0f);
    assert_eq!(h.regs().x, 0x00);
    // 2+2+2 setup, two full loops at 2+2+3, a last one at 2+2+2 where the
    // branch is not taken, 4 for the store, and 11 for the JAM.
    assert_eq!(used, 41);
}

// ---------------------------------------------------------------------------
// The machine-layer connection surface
// ---------------------------------------------------------------------------

#[test]
fn the_device_publishes_its_three_input_pins() {
    let h = Harness::running(&[]);
    let src = WireId::new(1);
    let device: &dyn Device = h.cpu.as_ref();
    for port in ["irq", "nmi", "reset"] {
        assert!(device.sink(port, &[src]).is_some(), "no `{port}` pin");
    }
    assert!(device.sink("rdy", &[src]).is_none(), "a pin nobody models");
    // A 6502 drives no line this core models, so every port is input-only.
    let net = Wire::builder().sources(&[src]).build_shared();
    assert!(
        device.connect("irq", WireSource::new(net, src)).is_err(),
        "an input pin cannot be a wire's source"
    );
    assert!(device.is_runnable(), "a cpu takes execution budgets");
}

#[test]
fn a_wire_driven_through_the_sink_reaches_the_interrupt_latches() {
    let h = Harness::running(&[]);
    let apu = WireId::new(1);
    let cart = WireId::new(2);
    let device: &dyn Device = h.cpu.as_ref();

    // Two sources on one net, as the NES's IRQ line really has: the APU and
    // the cartridge, wire-ORed.
    let pin = device.sink("irq", &[apu, cart]).expect("an irq pin");
    pin.sink.set_level(apu, pin.line, Level::High);
    assert!(h.cpu.irq_asserted());
    pin.sink.set_level(cart, pin.line, Level::High);
    pin.sink.set_level(apu, pin.line, Level::Low);
    assert!(h.cpu.irq_asserted(), "the cartridge still holds the line");
    pin.sink.set_level(cart, pin.line, Level::Low);
    assert!(!h.cpu.irq_asserted());

    // NMI is edge-sensitive: the latch survives the level going away.
    let nmi = device.sink("nmi", &[apu]).expect("an nmi pin");
    nmi.sink.set_level(apu, nmi.line, Level::High);
    nmi.sink.set_level(apu, nmi.line, Level::Low);
    assert!(h.cpu.nmi_pending(), "a high-going edge latches");
}

#[test]
fn the_reset_pin_restarts_a_jammed_core() {
    // A `JAM` freezes the CPU until reset, and `/RES` is the only way out —
    // which is why the pin cannot reach for the execution lock: the core may
    // be inside a bus access when the line moves.
    let h = Harness::running(&[0x02]);
    h.cpu.run(100);
    assert!(h.cpu.is_halted());
    assert_eq!(h.cpu.run(100), 0, "a jammed core consumes nothing");

    let button = WireId::new(1);
    let device: &dyn Device = h.cpu.as_ref();
    let pin = device.sink("reset", &[button]).expect("a reset pin");
    pin.sink.set_level(button, pin.line, Level::High);
    // A signal, not a method call: the sequence runs on the next step, which
    // is when the CPU can read the vector.
    assert!(h.cpu.is_halted(), "nothing has executed yet");
    h.cpu.step();
    assert!(!h.cpu.is_halted(), "the reset sequence un-jammed it");
    assert_eq!(h.cpu.regs().pc, 0xc000, "and it fetched the vector");
}

#[test]
fn a_budget_is_never_overrun_and_never_loses_a_cycle() {
    // The scheduler treats an overrun as fatal, and a 6502 cannot stop
    // mid-instruction — so `run_budget` reports the budget and carries the
    // overshoot. Over many budgets the two counts must stay in step.
    let program = [0xea; 8]; // NOPs, two cycles each
    let h = Harness::running(&program);
    let start = h.cpu.cycles(); // the reset sequence already ran
    let mut granted = 0u64;
    for _ in 0..100 {
        // Three is not a multiple of two, so every other budget ends inside an
        // instruction — the case the debt exists for.
        let used = h.cpu.run_budget(3);
        assert_eq!(used, 3, "a running core consumes its budget in full");
        granted += used;
    }
    assert_eq!(granted, 300);
    // The core executed whole instructions, so its own count is the budget
    // total plus whatever it currently owes.
    assert_eq!(h.cpu.cycles() - start - h.cpu.cycle_debt(), granted);
    assert!(h.cpu.cycle_debt() <= 7);
}

#[test]
fn a_halted_core_reports_what_it_actually_used() {
    let h = Harness::running(&[0x02]); // JAM: eleven cycles, then nothing
    let used = h.cpu.run_budget(1000);
    assert!(h.cpu.is_halted());
    assert_eq!(used, 11, "a halted core must not claim the whole budget");
    assert_eq!(h.cpu.run_budget(1000), 0, "and nothing after that");
}

#[test]
fn the_cycle_debt_survives_a_snapshot() -> Result<()> {
    let h = Harness::running(&[0xea; 4]);
    h.cpu.run_budget(3);
    let debt = h.cpu.cycle_debt();
    assert_eq!(
        debt, 1,
        "a two-cycle NOP overshoots a three-cycle budget by one"
    );

    let mut shape = MachineShape::new();
    shape.add_device("cpu", CLASS.name)?;
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("cpu", CLASS.name, CLASS.version)?;
        h.cpu.save(&mut chunk)?;
    }
    let bytes = w.to_vec()?;

    let other = Harness::running(&[]);
    let reader = StateReader::new(&bytes)?;
    let chunk = reader.load("cpu", CLASS.name, CLASS.version, &Migrations::new())?;
    other.cpu.load(&mut chunk.reader())?;
    // Dropping it would resume a cycle ahead of where the save was taken,
    // which is the kind of drift a replay cannot survive.
    assert_eq!(other.cpu.cycle_debt(), debt);
    Ok(())
}

#[test]
fn properties_carry_the_nes_variant_and_the_dsl_spelling() {
    let props = Props::new().with("decimal", false).with("engine", "interp");
    let cpu = Mos6502::from_props(&props).expect("the NES's RP2A03");
    assert!(!cpu.config().decimal);
    assert_eq!(cpu.config().magic, 0xee);

    // An engine that does not exist yet is a config error, not a silent
    // fallback to the interpreter.
    let props = Props::new().with("engine", "jit");
    assert!(Mos6502::from_props(&props).is_err());
}

// ---------------------------------------------------------------------------
// The W65C02S
//
// The bus behaviour of every CMOS encoding is measured against
// SingleStepTests/65x02's `wdc65c02` corpus next door, which is where the
// double reads and the shortened indexed shifts are actually proved. What
// lives here is what that corpus cannot express: the interrupt lines, the
// reset sequence, `WAI`, `STP`, and the fact that the whole thing is a
// construction property rather than a build flag.
// ---------------------------------------------------------------------------

/// A CMOS core at `$c000` with `program` loaded there.
fn cmos(program: &[u8]) -> Harness {
    Harness::running_with(Config::W65C02S, program)
}

#[test]
fn the_variant_is_a_construction_property_and_changes_the_decode() {
    // The same byte, two parts, two instructions — and no `#[cfg]` anywhere
    // near it: both cores exist in one build and one test.
    let nmos = Harness::running(&[0xa9, 0x01, 0x3a]);
    nmos.step();
    nmos.step();
    assert_eq!(nmos.regs().a, 0x01, "$3a is a one-byte NOP on an NMOS part");

    let wdc = cmos(&[0xa9, 0x01, 0x3a]);
    wdc.step();
    wdc.step();
    assert_eq!(wdc.regs().a, 0x00, "and DEC A on the CMOS one");
    assert!(wdc.regs().flag(flags::Z));
}

#[test]
fn inc_a_and_dec_a_cost_two_cycles_and_touch_no_memory() {
    let h = cmos(&[0x1a, 0x3a]);
    h.set_regs(|r| r.a = 0x7f);
    // The dummy read of the byte after the opcode, which PC does not advance
    // over: exactly the shape of every other implied instruction.
    assert_eq!(h.trace(), [Cycle::r(0xc000, 0x1a), Cycle::r(0xc001, 0x3a)]);
    assert_eq!(h.regs().a, 0x80);
    assert!(h.regs().flag(flags::N), "0x7f + 1 is negative");
    h.step();
    assert_eq!(h.regs().a, 0x7f);
    assert!(!h.regs().flag(flags::N));
    // Neither touches carry, which is what separates them from ADC/SBC.
    assert!(!h.regs().flag(flags::C));
}

#[test]
fn a_read_modify_write_reads_twice_instead_of_writing_twice() {
    // The single most load-bearing difference on the bus, and the reason NMOS
    // mapper tricks do not port to a CMOS board.
    let nmos = Harness::running(&[0xee, 0x00, 0x02]);
    nmos.bus.poke(0x0200, &[0x41]);
    assert_eq!(
        nmos.trace(),
        [
            Cycle::r(0xc000, 0xee),
            Cycle::r(0xc001, 0x00),
            Cycle::r(0xc002, 0x02),
            Cycle::r(0x0200, 0x41),
            Cycle::w(0x0200, 0x41),
            Cycle::w(0x0200, 0x42),
        ]
    );

    let wdc = cmos(&[0xee, 0x00, 0x02]);
    wdc.bus.poke(0x0200, &[0x41]);
    assert_eq!(
        wdc.trace(),
        [
            Cycle::r(0xc000, 0xee),
            Cycle::r(0xc001, 0x00),
            Cycle::r(0xc002, 0x02),
            Cycle::r(0x0200, 0x41),
            Cycle::r(0x0200, 0x41),
            Cycle::w(0x0200, 0x42),
        ]
    );
    assert_eq!(wdc.bus.peek(0x0200), 0x42);
}

#[test]
fn an_indexed_write_re_reads_the_operand_instead_of_the_unfixed_address() {
    // `STA $20ff,X` with X = 1 lands at $2100. The NMOS part reads $2000 on the
    // way — which is what makes it dangerous next to memory-mapped hardware —
    // and the CMOS one reads its own last operand byte instead.
    let nmos = Harness::running(&[0x9d, 0xff, 0x20]);
    nmos.set_regs(|r| {
        r.a = 0x5a;
        r.x = 0x01;
    });
    assert_eq!(nmos.trace()[3], Cycle::r(0x2000, 0x00));

    let wdc = cmos(&[0x9d, 0xff, 0x20]);
    wdc.set_regs(|r| {
        r.a = 0x5a;
        r.x = 0x01;
    });
    let trace = wdc.trace();
    assert_eq!(trace[3], Cycle::r(0xc002, 0x20), "the high operand byte");
    assert_eq!(trace[4], Cycle::w(0x2100, 0x5a));
    assert_eq!(trace.len(), 5);
}

#[test]
fn jmp_indirect_no_longer_wraps_but_still_makes_the_wrong_access() {
    // The bug is fixed by *adding a cycle*: the CMOS part reads the same wrong
    // address the NMOS one stopped at, then reads the right one and uses that.
    let h = cmos(&[0x6c, 0xff, 0x02]);
    h.bus.poke(0x02ff, &[0x34]);
    h.bus.poke(0x0200, &[0xbb]);
    h.bus.poke(0x0300, &[0x12]);
    assert_eq!(
        h.trace(),
        [
            Cycle::r(0xc000, 0x6c),
            Cycle::r(0xc001, 0xff),
            Cycle::r(0xc002, 0x02),
            Cycle::r(0x02ff, 0x34),
            Cycle::r(0x0200, 0xbb),
            Cycle::r(0x0300, 0x12),
        ]
    );
    assert_eq!(h.regs().pc, 0x1234, "and $0300 wins, not $0200");
}

#[test]
fn the_new_addressing_modes_reach_the_right_byte() {
    // `LDA ($40)` — page-zero indirect, the mode the NMOS ALU group never had.
    let h = cmos(&[0xb2, 0x40]);
    h.bus.poke(0x0040, &[0x00, 0x03]);
    h.bus.poke(0x0300, &[0x77]);
    assert_eq!(
        h.trace(),
        [
            Cycle::r(0xc000, 0xb2),
            Cycle::r(0xc001, 0x40),
            Cycle::r(0x0040, 0x00),
            Cycle::r(0x0041, 0x03),
            Cycle::r(0x0300, 0x77),
        ]
    );
    assert_eq!(h.regs().a, 0x77);

    // And its pointer wraps inside page zero, like every other zero-page mode.
    let h = cmos(&[0xb2, 0xff]);
    h.bus.poke(0x00ff, &[0x00]);
    h.bus.poke(0x0000, &[0x04]);
    h.bus.poke(0x0400, &[0x99]);
    h.step();
    assert_eq!(h.regs().a, 0x99);

    // `JMP ($1200,X)`, whose fix-up cycle re-reads the *first* operand byte.
    let h = cmos(&[0x7c, 0x00, 0x12]);
    h.set_regs(|r| r.x = 0x04);
    h.bus.poke(0x1204, &[0xcd, 0xab]);
    let trace = h.trace();
    assert_eq!(trace[3], Cycle::r(0xc001, 0x00));
    assert_eq!(h.regs().pc, 0xabcd);
    assert_eq!(trace.len(), 6);
}

#[test]
fn stz_stores_a_zero_nobody_had_to_load() {
    let h = cmos(&[0x9c, 0x34, 0x12]);
    h.set_regs(|r| {
        r.a = 0xff;
        r.x = 0xff;
        r.y = 0xff;
    });
    h.bus.poke(0x1234, &[0xff]);
    h.step();
    assert_eq!(h.bus.peek(0x1234), 0x00);
    // No register was disturbed, which is the entire point of the instruction.
    assert_eq!((h.regs().a, h.regs().x, h.regs().y), (0xff, 0xff, 0xff));
}

#[test]
fn trb_and_tsb_report_the_old_bits_in_z_alone() {
    // TSB sets the bits A selects; TRB clears them. Z says whether any of them
    // were already set — and N and V are left alone, which is what separates
    // these from BIT.
    let h = cmos(&[0x0c, 0x00, 0x02, 0x1c, 0x00, 0x02]);
    h.bus.poke(0x0200, &[0x0f]);
    h.set_regs(|r| {
        r.a = 0x30;
        r.p = flags::U | flags::N | flags::V;
    });
    h.step();
    assert_eq!(h.bus.peek(0x0200), 0x3f, "TSB set both");
    assert!(h.regs().flag(flags::Z), "neither was set before");
    assert!(h.regs().flag(flags::N), "N is untouched");
    assert!(h.regs().flag(flags::V), "so is V");

    h.step();
    assert_eq!(h.bus.peek(0x0200), 0x0f, "TRB cleared them again");
    assert!(!h.regs().flag(flags::Z), "and this time they were set");
}

#[test]
fn bit_immediate_moves_z_and_nothing_else() {
    let h = cmos(&[0x89, 0xc0, 0x24, 0x40]);
    h.bus.poke(0x0040, &[0xc0]);
    h.set_regs(|r| {
        r.a = 0x0f;
        r.p = flags::U;
    });
    assert_eq!(h.trace().len(), 2, "two cycles, no memory");
    assert!(h.regs().flag(flags::Z), "$0f AND $c0 is zero");
    assert!(!h.regs().flag(flags::N), "N did not come from the operand");
    assert!(!h.regs().flag(flags::V), "nor V");

    // The memory forms still do take N and V from the operand's top two bits.
    h.step();
    assert!(h.regs().flag(flags::N));
    assert!(h.regs().flag(flags::V));
}

#[test]
fn the_bit_group_sets_clears_and_branches_on_one_bit() {
    // SMB3 $40 / RMB3 $40, then BBR3 and BBS3 over the same byte.
    let h = cmos(&[0xb7, 0x40, 0x37, 0x40]);
    h.bus.poke(0x0040, &[0x00]);
    h.set_regs(|r| r.p = flags::U | flags::N | flags::Z | flags::C);
    let before = h.regs().p;
    h.step();
    assert_eq!(h.bus.peek(0x0040), 0x08, "SMB3 set bit 3");
    assert_eq!(h.regs().p, before, "and touched no flag at all");
    h.step();
    assert_eq!(h.bus.peek(0x0040), 0x00, "RMB3 cleared it");

    // BBR3 $40,+2 with bit 3 clear: taken, six cycles, and the page-zero byte
    // is read twice on the way.
    let h = cmos(&[0x3f, 0x40, 0x02]);
    h.bus.poke(0x0040, &[0x00]);
    assert_eq!(
        h.trace(),
        [
            Cycle::r(0xc000, 0x3f),
            Cycle::r(0xc001, 0x40),
            Cycle::r(0x0040, 0x00),
            Cycle::r(0x0040, 0x00),
            Cycle::r(0xc002, 0x02),
            Cycle::r(0xc003, 0x00),
        ]
    );
    assert_eq!(h.regs().pc, 0xc005);

    // The same encoding with the bit set: not taken, five cycles, three bytes.
    let h = cmos(&[0x3f, 0x40, 0x02]);
    h.bus.poke(0x0040, &[0x08]);
    assert_eq!(h.trace().len(), 5);
    assert_eq!(h.regs().pc, 0xc003);
}

#[test]
fn decimal_arithmetic_costs_a_cycle_and_gets_its_flags_right() {
    // $99 + $01 in BCD is $00 with a carry. The NMOS part leaves Z describing
    // the *binary* sum and N describing an intermediate; the CMOS part latches
    // both after the correction, so they describe the answer.
    let nmos = Harness::running(&[0xf8, 0x18, 0xa9, 0x99, 0x69, 0x01]);
    for _ in 0..3 {
        nmos.step();
    }
    let trace = nmos.trace();
    assert_eq!(trace.len(), 2, "no correction cycle on an NMOS part");
    assert_eq!(nmos.regs().a, 0x00);
    assert!(nmos.regs().flag(flags::C));
    assert!(!nmos.regs().flag(flags::Z), "binary $99 + $01 is $9a");

    let wdc = cmos(&[0xf8, 0x18, 0xa9, 0x99, 0x69, 0x01]);
    for _ in 0..3 {
        wdc.step();
    }
    let trace = wdc.trace();
    assert_eq!(trace.len(), 3, "the correction cycle is on the bus");
    assert_eq!(trace[2], Cycle::r(0xc005, 0x01), "re-reading the operand");
    assert_eq!(wdc.regs().a, 0x00);
    assert!(wdc.regs().flag(flags::C));
    assert!(wdc.regs().flag(flags::Z), "and Z describes the answer");
}

#[test]
fn decimal_subtraction_disagrees_with_the_nmos_part_on_invalid_bcd() {
    // $10 - $fc with carry set. `$fc` is not a BCD pair, so the two parts'
    // different correction orders become visible: the NMOS nibble-at-a-time
    // path gives $be, the CMOS correct-the-difference path gives $ae.
    let program = [0xf8, 0x38, 0xa9, 0x10, 0xe9, 0xfc];
    let nmos = Harness::running(&program);
    for _ in 0..4 {
        nmos.step();
    }
    assert_eq!(nmos.regs().a, 0xbe);

    let wdc = cmos(&program);
    for _ in 0..4 {
        wdc.step();
    }
    assert_eq!(wdc.regs().a, 0xae);
    assert!(wdc.regs().flag(flags::N), "N from the corrected value");
}

#[test]
fn an_interrupt_clears_the_decimal_flag_on_the_cmos_part() {
    // The reason a CMOS handler no longer has to open with CLD. The *pushed*
    // byte still carries the old D — the flag is cleared after the push.
    let h = cmos(&[0xf8, 0x00, 0x00]);
    h.bus.poke(0xfffe, &[0x00, 0xd0]);
    h.step();
    assert!(h.regs().flag(flags::D));
    let s_before = h.regs().s;
    h.step();
    assert_eq!(h.regs().pc, 0xd000);
    assert!(!h.regs().flag(flags::D), "D is clear inside the handler");
    // PCH, PCL, then P: the status byte is two below where S started.
    let pushed = h.bus.peek(0x0100 | u16::from(s_before.wrapping_sub(2)));
    assert_ne!(pushed & flags::D, 0, "but the stacked copy still has it");
    assert_ne!(pushed & flags::B, 0, "and B, because this was a BRK");

    // The NMOS part leaves D exactly where it was, which is the bug.
    let n = Harness::running(&[0xf8, 0x00, 0x00]);
    n.bus.poke(0xfffe, &[0x00, 0xd0]);
    n.step();
    n.step();
    assert!(n.regs().flag(flags::D));
}

#[test]
fn a_cmos_reset_clears_decimal_mode_too() {
    let h = Harness::with_config(Config::W65C02S);
    h.bus.poke(0xfffc, &[0x00, 0xc0]);
    h.cpu.set_regs(Regs {
        p: flags::U | flags::D,
        ..Regs::new()
    });
    h.step();
    assert!(!h.cpu.regs().flag(flags::D));
    assert!(
        h.cpu.regs().flag(flags::I),
        "and sets I, as every part does"
    );
}

#[test]
fn stp_halts_the_core_until_reset() {
    // Three cycles, then nothing — not even the jammed bus pattern a `JAM`
    // leaves behind, because a stopped oscillator has no cycles to spend.
    let h = cmos(&[0xdb]);
    assert_eq!(
        h.trace(),
        [
            Cycle::r(0xc000, 0xdb),
            Cycle::r(0xc001, 0x00),
            Cycle::r(0xc001, 0x00),
        ]
    );
    assert!(h.cpu.is_halted());
    assert_eq!(h.step(), 0, "and it stays stopped");
    assert!(h.bus.take_log().is_empty());

    // An interrupt does not wake it; only a reset does.
    h.cpu.pulse_nmi();
    assert_eq!(h.step(), 0);
    h.cpu.request_reset();
    assert_eq!(h.step(), 7);
    assert!(!h.cpu.is_halted());
}

#[test]
fn wai_stalls_without_a_bus_access_and_any_interrupt_releases_it() {
    let h = cmos(&[0xcb, 0xe8]);
    assert_eq!(h.trace().len(), 3, "WAI itself is three cycles");
    assert!(h.cpu.is_waiting());
    assert!(!h.cpu.is_halted(), "stalled is not halted");

    // Time passes and the bus stays quiet: a WAIing part holds RDY low rather
    // than fetching, and a scheduler that saw zero cycles would call the
    // machine dead instead of merely idle.
    let before = h.cpu.cycles();
    assert_eq!(h.step(), 1);
    assert_eq!(h.step(), 1);
    assert_eq!(h.cpu.cycles(), before + 2);
    assert!(h.bus.take_log().is_empty());

    // IRQ releases it even though I is set — what the mask decides is whether
    // the handler runs, not whether the part wakes up.
    h.set_regs(|r| r.p |= flags::I);
    h.cpu.set_irq(true);
    h.step();
    assert!(!h.cpu.is_waiting());
    assert_eq!(h.cpu.pending_interrupt(), None, "masked, so no handler");
    assert_eq!(h.regs().x, 0x01, "execution resumed at the INX");
}

#[test]
fn wai_takes_the_interrupt_when_it_is_not_masked() {
    let h = cmos(&[0xcb, 0xe8]);
    h.bus.poke(0xfffe, &[0x00, 0xd0]);
    h.set_regs(|r| r.p &= !flags::I);
    h.step();
    assert!(h.cpu.is_waiting());
    h.cpu.set_irq(true);
    // Waking and taking the interrupt are one step: the stall ends where the
    // sequence begins, which is the whole reason to use `WAI` over a poll loop.
    assert_eq!(h.step(), 7, "the interrupt sequence, not a stall cycle");
    assert!(!h.cpu.is_waiting());
    assert_eq!(h.regs().pc, 0xd000);

    // And a reset releases a WAI as surely as it does an STP.
    let h = cmos(&[0xcb]);
    h.step();
    assert!(h.cpu.is_waiting());
    h.cpu.request_reset();
    h.step();
    assert!(!h.cpu.is_waiting());
}

#[test]
fn the_cmos_one_cycle_nops_really_do_take_one_cycle() {
    // $x3 and $xB are one byte and one bus cycle — the only instructions in
    // this family that finish inside their own opcode fetch.
    let h = cmos(&[0x03, 0x0b, 0x5c, 0x34, 0x12]);
    assert_eq!(h.trace(), [Cycle::r(0xc000, 0x03)]);
    assert_eq!(h.regs().pc, 0xc001);
    assert_eq!(h.trace(), [Cycle::r(0xc001, 0x0b)]);

    // $5c is three bytes and four cycles, the fourth spent re-reading the last
    // operand byte. Nothing at $1234 is touched.
    assert_eq!(
        h.trace(),
        [
            Cycle::r(0xc002, 0x5c),
            Cycle::r(0xc003, 0x34),
            Cycle::r(0xc004, 0x12),
            Cycle::r(0xc004, 0x12),
        ]
    );
    assert_eq!(h.regs().pc, 0xc005);
}

#[test]
fn bra_is_a_branch_that_is_always_taken() {
    let h = cmos(&[0x80, 0x02]);
    h.set_regs(|r| r.p = flags::U | flags::Z | flags::N | flags::C | flags::V);
    assert_eq!(h.trace().len(), 3, "three cycles, no page cross");
    assert_eq!(h.regs().pc, 0xc004, "and no flag could have stopped it");
}

#[test]
fn the_stack_gains_x_and_y() {
    let h = cmos(&[0xda, 0x5a, 0xfa, 0x7a]);
    h.set_regs(|r| {
        r.x = 0x11;
        r.y = 0x22;
    });
    let s = h.regs().s;
    h.step();
    h.step();
    assert_eq!(h.bus.peek(0x0100 | u16::from(s)), 0x11);
    assert_eq!(h.bus.peek(0x0100 | u16::from(s.wrapping_sub(1))), 0x22);
    h.set_regs(|r| {
        r.x = 0;
        r.y = 0;
    });
    h.step(); // PLX pulls what PHY pushed
    assert_eq!(h.regs().x, 0x22);
    assert!(!h.regs().flag(flags::N));
    h.step(); // PLY pulls what PHX pushed
    assert_eq!(h.regs().y, 0x11);
    assert_eq!(h.regs().s, s, "and the stack is back where it started");
}

#[test]
fn the_cmos_disassembler_spells_the_new_modes() {
    use super::disasm::{disassemble, disassemble_as};
    let cases: &[(&[u8], &str)] = &[
        (&[0x3a], "DEC A"),
        (&[0x1a], "INC A"),
        (&[0x80, 0x05], "BRA $c007"),
        (&[0xb2, 0x40], "LDA ($40)"),
        (&[0x7c, 0x34, 0x12], "JMP ($1234,X)"),
        (&[0x9c, 0x34, 0x12], "STZ $1234"),
        (&[0x04, 0x40], "TSB $40"),
        (&[0x14, 0x40], "TRB $40"),
        (&[0x89, 0x42], "BIT #$42"),
        (&[0x3c, 0x34, 0x12], "BIT $1234,X"),
        (&[0xda], "PHX"),
        (&[0x7a], "PLY"),
        (&[0x27, 0x40], "RMB2 $40"),
        (&[0xb7, 0x40], "SMB3 $40"),
        (&[0x8f, 0x40, 0x05], "BBS0 $40,$c008"),
        (&[0x0f, 0x40, 0xfb], "BBR0 $40,$bffe"),
        (&[0xcb], "WAI"),
        (&[0xdb], "STP"),
        (&[0x03], "NOP"),
        (&[0x5c, 0x34, 0x12], "NOP $1234"),
    ];
    for (bytes, want) in cases {
        let d = disassemble_as(Variant::Wdc65C02, 0xc000, bytes);
        assert_eq!(alloc::format!("{d}"), *want);
        assert_eq!(usize::from(d.len), bytes.len(), "{want}");
        assert!(!d.is_undocumented(), "{want} is a documented CMOS encoding");
    }
    // And the disassembler follows the core it belongs to, so a listing of the
    // same bytes is not the same listing.
    assert_eq!(alloc::format!("{}", disassemble(0xc000, &[0x3a])), "NOP");
}

#[test]
fn a_core_disassembles_with_its_own_variant() {
    let h = cmos(&[0x3a, 0x80, 0xfd]);
    let listing = h.cpu.disassemble(0xc000, 2);
    assert_eq!(alloc::format!("{}", listing[0]), "DEC A");
    assert_eq!(alloc::format!("{}", listing[1]), "BRA $c000");
}

#[test]
fn properties_name_the_part_and_default_its_adder() {
    let props = Props::new().with("variant", "65c02");
    let cpu = Mos6502::from_props(&props).expect("a W65C02S");
    assert_eq!(cpu.config().variant, Variant::Wdc65C02);
    assert!(cpu.config().decimal, "the CMOS part has a BCD adder");

    // The NES's part defaults its own adder off, so a machine file naming the
    // variant need not also remember to say `decimal = false`.
    let props = Props::new().with("variant", "2a03");
    let cpu = Mos6502::from_props(&props).expect("an RP2A03");
    assert_eq!(cpu.config().variant, Variant::Ricoh2A03);
    assert!(!cpu.config().decimal);
    // ... but it stays independently settable, because a test may want to ask
    // what the missing adder would have done.
    let props = Props::new().with("variant", "2a03").with("decimal", true);
    assert!(
        Mos6502::from_props(&props)
            .expect("still valid")
            .config()
            .decimal
    );

    // A part nobody has implemented is a config error, not a silent 6502.
    let props = Props::new().with("variant", "65816");
    assert!(Mos6502::from_props(&props).is_err());
}

#[test]
fn a_wai_survives_a_snapshot() -> Result<()> {
    let h = cmos(&[0xcb]);
    h.step();
    assert!(h.cpu.is_waiting());

    let mut shape = MachineShape::new();
    shape.add_device("cpu", CLASS.name)?;
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("cpu", CLASS.name, CLASS.version)?;
        h.cpu.save(&mut chunk)?;
    }
    let bytes = w.to_vec()?;

    let other = Harness::running_with(Config::W65C02S, &[]);
    let reader = StateReader::new(&bytes)?;
    let chunk = reader.load("cpu", CLASS.name, CLASS.version, &Migrations::new())?;
    other.cpu.load(&mut chunk.reader())?;
    // A restore that dropped this would resume by executing whatever the WAI
    // was waiting in front of, which is not where the machine was.
    assert!(other.cpu.is_waiting());
    assert_eq!(other.cpu.regs(), h.cpu.regs());
    Ok(())
}
