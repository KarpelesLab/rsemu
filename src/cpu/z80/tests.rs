//! Hand-written tests for the parts the corpus cannot reach.
//!
//! `SingleStepTests/z80` starts every vector mid-program with the interrupt
//! lines idle, so it says nothing about `RESET`, `NMI`, `INT` in any mode, the
//! `HALT` state, or the `EI` delay — and it never exercises the snapshot pair.
//! Those are exactly what this file covers. Everything the corpus *does* cover
//! is checked there instead of duplicated here, because 1 604 000 vectors with
//! full bus traces is a better test than any assertion written by hand.

use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::{Deferred, Device, RealizeCtx, ResetKind};
use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, RamStore, Region, RequesterId,
};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::sync::{self, LockRank};

use super::isa::{Index, decode, index_substitute};
use super::{Config, Interrupt, MCycle, Reg, Regs, Z80, flags};

/// A machine: 64 KiB of RAM and, optionally, an I/O space that logs.
struct Machine {
    cpu: Arc<Z80>,
    ram: Arc<RamStore>,
    ports: Arc<PortLog>,
}

/// One port transaction: address, byte, and whether it was a write.
type Transaction = (u16, u8, bool);

/// An I/O space that answers every port with one value and records the
/// traffic.
#[derive(Debug)]
struct PortLog(sync::Mutex<(u8, Vec<Transaction>)>);

impl MemOps for PortLog {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let mut m = self.0.lock();
        let value = m.0;
        for slot in dst.iter_mut() {
            *slot = value;
        }
        m.1.push((offset as u16, value, false));
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], _attrs: MemAttrs) -> MemResult {
        let mut m = self.0.lock();
        for (i, byte) in src.iter().enumerate() {
            m.1.push(((offset as u16).wrapping_add(i as u16), *byte, true));
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::ANY
    }
}

impl Machine {
    fn new() -> Machine {
        Machine::with_config(Config::NMOS)
    }

    fn with_config(cfg: Config) -> Machine {
        let ram = Arc::new(RamStore::new(0x1_0000));
        let space = AddressSpace::new("cpu", 16);
        space
            .topology()
            .map(Region::ram("ram", ram.clone()), 0)
            .expect("64 KiB fits");

        let ports = Arc::new(PortLog(sync::Mutex::with_rank(
            LockRank::DEVICE,
            (0xff, Vec::new()),
        )));
        let io = AddressSpace::new("io", 16);
        io.topology()
            .map(Region::io("ports", 0x1_0000, ports.clone()), 0)
            .expect("64 KiB fits");

        let cpu = Arc::new(Z80::new(cfg));
        cpu.attach_space(Arc::new(space));
        cpu.attach_io_space(Arc::new(io));
        Machine { cpu, ram, ports }
    }

    /// Load `code` at `at` and start there with the reset already done.
    fn load(&self, at: u16, code: &[u8]) {
        for (i, byte) in code.iter().enumerate() {
            self.ram
                .write_u8(u64::from(at) + i as u64, *byte)
                .expect("fits");
        }
    }

    /// Skip the reset sequence and start at `pc` with a usable stack.
    fn start(&self, pc: u16) {
        self.cpu.step();
        self.cpu.set_reg(Reg::Pc, pc);
        self.cpu.set_reg(Reg::Sp, 0xf000);
    }

    fn peek(&self, at: u16) -> u8 {
        self.ram.read_u8(u64::from(at)).expect("mapped")
    }

    fn port_traffic(&self) -> Vec<Transaction> {
        self.ports.0.lock().1.clone()
    }
}

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------

#[test]
fn reset_clears_the_registers_the_manual_says_it_clears() {
    let m = Machine::new();
    m.cpu.set_regs(Regs {
        pc: 0x1234,
        i: 0x55,
        r: 0x66,
        ..Regs::new()
    });
    m.cpu.set_iff(true, true);
    m.cpu.set_interrupt_mode(2).expect("mode 2 exists");
    m.cpu.request_reset();

    let t = m.cpu.step();
    // UM0080 §"Reset": PC, I and R are cleared, both flip-flops are reset and
    // mode 0 is selected. There is no vector fetch — a Z80 starts at $0000.
    assert_eq!(t, 3);
    let regs = m.cpu.regs();
    assert_eq!(regs.pc, 0);
    assert_eq!(regs.i, 0);
    assert_eq!(regs.r, 0);
    assert_eq!(m.cpu.iff(), (false, false));
    assert_eq!(m.cpu.interrupt_mode(), 0);
    assert!(!m.cpu.reset_pending());
}

#[test]
fn a_warm_reset_keeps_the_general_registers() {
    let m = Machine::new();
    m.start(0x0000);
    m.cpu.set_regs(Regs {
        a: 0x42,
        b: 0x99,
        ..m.cpu.regs()
    });
    m.cpu.reset(ResetKind::Warm);
    m.cpu.step();
    assert_eq!(m.cpu.regs().a, 0x42, "a warm reset is a pin, not a wipe");
    assert_eq!(m.cpu.regs().b, 0x99);
    assert_eq!(m.cpu.regs().pc, 0);
}

// ---------------------------------------------------------------------------
// The refresh register
// ---------------------------------------------------------------------------

#[test]
fn r_counts_seven_bits_and_leaves_the_eighth_alone() {
    let m = Machine::new();
    m.load(0x0000, &[0x00, 0x00, 0x00]);
    m.start(0x0000);
    // Bit 7 is a latch the program owns: the hardware increment carries
    // within bits 0-6 and never into it (UM0080 §"CPU Registers").
    m.cpu.set_reg(Reg::R, 0xff);
    m.cpu.step();
    assert_eq!(m.cpu.reg(Reg::R), 0x80);
    m.cpu.step();
    assert_eq!(m.cpu.reg(Reg::R), 0x81);
}

#[test]
fn each_prefix_costs_its_own_fetch_and_its_own_refresh() {
    let m = Machine::new();
    // DD DD DD 00: three redundant prefixes and a NOP, which is one
    // instruction of four M1 cycles.
    m.load(0x0000, &[0xdd, 0xdd, 0xdd, 0x00]);
    m.start(0x0000);
    m.cpu.set_reg(Reg::R, 0x00);
    let t = m.cpu.step();
    assert_eq!(t, 16, "four fetches at four T-states each");
    assert_eq!(m.cpu.reg(Reg::R), 0x04);
    assert_eq!(m.cpu.regs().pc, 0x0004);
    let log = m.cpu.last_cycles();
    assert_eq!(log.cycles().len(), 4);
    assert!(log.cycles().iter().all(|c| c.kind == MCycle::Fetch));
}

// ---------------------------------------------------------------------------
// HALT
// ---------------------------------------------------------------------------

#[test]
fn halt_keeps_refreshing_and_only_an_interrupt_ends_it() {
    let m = Machine::new();
    m.load(0x0000, &[0x76]);
    m.start(0x0000);
    m.cpu.set_iff(true, true);
    m.cpu.set_interrupt_mode(1).expect("mode 1 exists");

    assert_eq!(m.cpu.step(), 4);
    assert!(m.cpu.is_halted());
    assert_eq!(m.cpu.regs().pc, 0x0001, "PC is already past the HALT");

    // A halted Z80 is not stopped: it keeps issuing M1 cycles so dynamic RAM
    // stays refreshed, and it re-fetches the HALT itself.
    let before = m.cpu.reg(Reg::R);
    assert_eq!(m.cpu.step(), 4);
    assert_eq!(m.cpu.reg(Reg::R), before + 1);
    assert_eq!(m.cpu.regs().pc, 0x0001);
    assert_eq!(m.cpu.last_cycles().cycles()[0].addr, 0x0000);

    m.cpu.set_int(true);
    m.cpu.step();
    assert!(!m.cpu.is_halted());
    assert_eq!(m.cpu.regs().pc, 0x0038, "mode 1 vectors through $0038");
    // The return address is the instruction after the HALT, which is why PC
    // moved past it when the HALT executed.
    assert_eq!(m.peek(0xeffe), 0x01);
    assert_eq!(m.peek(0xefff), 0x00);
}

// ---------------------------------------------------------------------------
// Interrupts
// ---------------------------------------------------------------------------

#[test]
fn nmi_pushes_saves_iff1_in_iff2_and_vectors_through_0066() {
    let m = Machine::new();
    m.load(0x1000, &[0x00]);
    m.start(0x1000);
    m.cpu.set_iff(true, true);
    m.cpu.pulse_nmi();
    assert!(m.cpu.nmi_pending());

    let t = m.cpu.step();
    assert_eq!(t, 11, "acknowledge plus two pushes");
    assert_eq!(m.cpu.regs().pc, 0x0066);
    assert_eq!(m.cpu.regs().wz, 0x0066);
    // IFF1 goes to IFF2 so RETN can put it back; IFF1 itself is cleared, which
    // is what makes an NMI handler non-reentrant by default.
    assert_eq!(m.cpu.iff(), (false, true));
    assert!(!m.cpu.nmi_pending(), "the edge latch was consumed");
    assert_eq!(m.peek(0xeffe), 0x00);
    assert_eq!(m.peek(0xefff), 0x10);
}

#[test]
fn retn_restores_iff1_from_its_nmi_backup() {
    let m = Machine::new();
    m.load(0x1000, &[0x00]);
    m.load(0x0066, &[0xed, 0x45]); // RETN
    m.start(0x1000);
    m.cpu.set_iff(true, true);
    m.cpu.pulse_nmi();
    m.cpu.step(); // take the NMI
    assert_eq!(m.cpu.iff(), (false, true));
    let t = m.cpu.step(); // RETN
    assert_eq!(t, 14);
    assert_eq!(m.cpu.regs().pc, 0x1000);
    assert_eq!(m.cpu.iff(), (true, true));
}

#[test]
fn an_nmi_is_edge_triggered_and_survives_until_it_is_serviced() {
    let m = Machine::new();
    m.load(0x1000, &[0x00, 0x00, 0x00]);
    m.start(0x1000);
    // Holding the line asserted latches exactly one edge.
    m.cpu.set_nmi(true);
    m.cpu.step();
    assert_eq!(m.cpu.regs().pc, 0x0066);
    m.cpu.set_reg(Reg::Pc, 0x1000);
    m.cpu.step();
    assert_ne!(m.cpu.regs().pc, 0x0066, "a level does not re-trigger");
}

#[test]
fn int_is_masked_by_iff1_and_deferred_one_instruction_by_ei() {
    let m = Machine::new();
    m.load(0x1000, &[0xfb, 0x00, 0x00]); // EI ; NOP ; NOP
    m.start(0x1000);
    m.cpu.set_iff(false, false);
    m.cpu.set_interrupt_mode(1).expect("mode 1 exists");
    m.cpu.set_int(true);

    m.cpu.step(); // EI: not taken, IFF1 was clear at the sample point
    assert_eq!(m.cpu.regs().pc, 0x1001);
    m.cpu.step(); // the one instruction EI defers by
    assert_eq!(m.cpu.regs().pc, 0x1002, "EI hides INT for one instruction");
    let t = m.cpu.step();
    assert_eq!(t, 13, "mode 1 is an acknowledge plus two pushes");
    assert_eq!(m.cpu.regs().pc, 0x0038);
    assert_eq!(m.cpu.iff(), (false, false));
}

#[test]
fn mode_2_reads_its_vector_from_the_table_i_points_at() {
    let m = Machine::new();
    m.load(0x1000, &[0x00]);
    // I = $80, device byte $40, so the pointer lives at $8040.
    m.load(0x8040, &[0x34, 0x12]);
    m.start(0x1000);
    m.cpu.set_reg(Reg::I, 0x80);
    m.cpu.set_iff(true, true);
    m.cpu.set_interrupt_mode(2).expect("mode 2 exists");
    m.cpu.set_interrupt_vector(0x40);
    m.cpu.set_int(true);

    let t = m.cpu.step();
    assert_eq!(t, 19, "acknowledge, two pushes and the vector fetch");
    assert_eq!(m.cpu.regs().pc, 0x1234);
    assert_eq!(m.cpu.regs().wz, 0x1234);
}

#[test]
fn mode_0_executes_the_restart_the_device_put_on_the_bus() {
    let m = Machine::new();
    m.load(0x1000, &[0x00]);
    m.start(0x1000);
    m.cpu.set_iff(true, true);
    m.cpu.set_interrupt_mode(0).expect("mode 0 exists");
    // $ff is `RST 38`, which is what an undriven bus with pull-ups produces —
    // and the reason so many Z80 boards use $0038 with no interrupt hardware
    // at all.
    m.cpu.set_interrupt_vector(0xff);
    m.cpu.set_int(true);

    let t = m.cpu.step();
    assert_eq!(t, 13);
    assert_eq!(m.cpu.regs().pc, 0x0038);

    m.cpu.set_reg(Reg::Pc, 0x1000);
    m.cpu.set_iff(true, true);
    m.cpu.set_interrupt_vector(0xcf); // RST 08
    m.cpu.step();
    assert_eq!(m.cpu.regs().pc, 0x0008);
}

#[test]
fn an_interrupt_during_ld_a_i_clears_the_parity_it_had_just_copied() {
    let m = Machine::new();
    m.load(0x1000, &[0xed, 0x57]); // LD A,I
    m.start(0x1000);
    m.cpu.set_reg(Reg::I, 0x01);
    m.cpu.set_iff(true, true);
    m.cpu.set_interrupt_mode(1).expect("mode 1 exists");

    m.cpu.step();
    // IFF2 was set, so P/V came out set...
    assert!(m.cpu.regs().flag(flags::PV));
    m.cpu.set_int(true);
    m.cpu.step();
    // ...and the interrupt that lands while the instruction is finishing takes
    // it away again, which is the classic reason to test P/V twice.
    assert!(!m.cpu.regs().flag(flags::PV));
    assert_eq!(m.cpu.regs().pc, 0x0038);
}

// ---------------------------------------------------------------------------
// The I/O space
// ---------------------------------------------------------------------------

#[test]
fn io_is_a_separate_space_addressed_sixteen_bits_wide() {
    let m = Machine::new();
    // IN A,($fe) ; OUT ($fe),A ; IN B,(C) ; OUT (C),B
    m.load(0x1000, &[0xdb, 0xfe, 0xd3, 0xfe, 0xed, 0x40, 0xed, 0x41]);
    m.start(0x1000);
    m.cpu.set_regs(Regs {
        a: 0x7f,
        b: 0x12,
        c: 0x34,
        ..m.cpu.regs()
    });
    m.ports.0.lock().0 = 0xa5;

    m.cpu.step(); // IN A,($fe): A supplies the high half of the address
    assert_eq!(m.cpu.regs().a, 0xa5);
    m.cpu.step(); // OUT ($fe),A
    m.cpu.step(); // IN B,(C): the whole of BC addresses the port, B included
    assert_eq!(m.cpu.regs().b, 0xa5);
    m.cpu.step(); // OUT (C),B

    assert_eq!(
        m.port_traffic(),
        [
            (0x7ffe, 0xa5, false),
            (0xa5fe, 0xa5, true),
            (0x1234, 0xa5, false),
            (0xa534, 0xa5, true),
        ]
    );
    // Nothing reached the memory space: the two are genuinely separate.
    assert_eq!(m.peek(0xfe), 0x00);
}

#[test]
fn a_machine_with_no_io_space_reads_a_floating_bus_rather_than_faulting() {
    let ram = Arc::new(RamStore::new(0x1_0000));
    ram.write_u8(0x0000, 0xdb).expect("fits"); // IN A,($00)
    ram.write_u8(0x0001, 0x00).expect("fits");
    let space = AddressSpace::new("cpu", 16);
    space
        .topology()
        .map(Region::ram("ram", ram), 0)
        .expect("fits");
    let cpu = Z80::new(Config::NMOS);
    cpu.attach_space(Arc::new(space));
    cpu.step(); // reset
    cpu.step(); // IN A,($00)
    assert_eq!(cpu.regs().a, 0xff);
    assert_eq!(cpu.bus_faults().0, 0, "an absent space is not a fault");
}

#[test]
fn out_c_zero_writes_what_the_part_family_writes() {
    for (cfg, expected) in [(Config::NMOS, 0x00u8), (Config::CMOS, 0xff)] {
        let m = Machine::with_config(cfg);
        m.load(0x1000, &[0xed, 0x71]); // OUT (C),0
        m.start(0x1000);
        m.cpu.set_regs(Regs {
            b: 0x00,
            c: 0x10,
            ..m.cpu.regs()
        });
        m.cpu.step();
        assert_eq!(m.port_traffic(), [(0x0010, expected, true)]);
    }
}

// ---------------------------------------------------------------------------
// Timing, spot-checked against UM0080's published figures
// ---------------------------------------------------------------------------

#[test]
fn instruction_timings_match_the_manuals_figures() {
    // The corpus checks every instruction's trace T-state by T-state; this is
    // the human-readable version of the same claim, so a regression says what
    // it broke rather than printing a hundred pin states.
    for (code, tstates, note) in [
        (&[0x00u8][..], 4u64, "NOP"),
        (&[0x3e, 0x00], 7, "LD A,n"),
        (&[0x21, 0x00, 0x00], 10, "LD HL,nn"),
        (&[0x34], 11, "INC (HL)"),
        (&[0x36, 0x00], 10, "LD (HL),n"),
        (&[0x09], 11, "ADD HL,BC"),
        (&[0xf9], 6, "LD SP,HL"),
        (&[0xc5], 11, "PUSH BC"),
        (&[0xc1], 10, "POP BC"),
        (&[0xcd, 0x00, 0x20], 17, "CALL nn"),
        (&[0xc9], 10, "RET"),
        (&[0xc7], 11, "RST 00"),
        (&[0xe3], 19, "EX (SP),HL"),
        (&[0xdb, 0x00], 11, "IN A,(n)"),
        (&[0xcb, 0x00], 8, "RLC B"),
        (&[0xcb, 0x06], 15, "RLC (HL)"),
        (&[0xcb, 0x46], 12, "BIT 0,(HL)"),
        (&[0xed, 0x44], 8, "NEG"),
        (&[0xed, 0x40], 12, "IN B,(C)"),
        (&[0xed, 0x42], 15, "SBC HL,BC"),
        (&[0xed, 0x67], 18, "RRD"),
        (&[0xed, 0x43, 0x00, 0x20], 20, "LD (nn),BC"),
        (&[0xdd, 0x21, 0x00, 0x00], 14, "LD IX,nn"),
        (&[0xdd, 0x09], 15, "ADD IX,BC"),
        (&[0xdd, 0x7e, 0x00], 19, "LD A,(IX+d)"),
        (&[0xdd, 0x34, 0x00], 23, "INC (IX+d)"),
        (&[0xdd, 0x36, 0x00, 0x00], 19, "LD (IX+d),n"),
        (&[0xdd, 0xcb, 0x00, 0x06], 23, "RLC (IX+d)"),
        (&[0xdd, 0xcb, 0x00, 0x46], 20, "BIT 0,(IX+d)"),
    ] {
        let m = Machine::new();
        m.load(0x1000, code);
        m.start(0x1000);
        let t = m.cpu.step();
        assert_eq!(t, tstates, "{note}");
        assert_eq!(
            u64::from(m.cpu.last_cycles().tstates()),
            tstates,
            "{note}: the log and the charge disagree"
        );
    }
}

#[test]
fn a_conditional_pays_for_the_branch_only_when_it_takes_it() {
    for (code, taken, not_taken, note) in [
        (&[0x20u8, 0x00][..], 12u64, 7u64, "JR NZ,e"),
        (&[0x10, 0x00], 13, 8, "DJNZ e"),
        (&[0xc0], 11, 5, "RET NZ"),
        (&[0xc4, 0x00, 0x20], 17, 10, "CALL NZ,nn"),
    ] {
        let m = Machine::new();
        m.load(0x1000, code);
        m.start(0x1000);
        // Z clear, and B = 2 so DJNZ has somewhere to go.
        m.cpu.set_regs(Regs {
            f: 0,
            b: 2,
            ..m.cpu.regs()
        });
        assert_eq!(m.cpu.step(), taken, "{note} taken");

        let m = Machine::new();
        m.load(0x1000, code);
        m.start(0x1000);
        m.cpu.set_regs(Regs {
            f: flags::Z,
            b: 1,
            ..m.cpu.regs()
        });
        assert_eq!(m.cpu.step(), not_taken, "{note} not taken");
    }
}

#[test]
fn a_block_instruction_costs_more_when_it_repeats() {
    let m = Machine::new();
    m.load(0x1000, &[0xed, 0xb0]); // LDIR
    m.start(0x1000);
    m.cpu.set_regs(Regs {
        b: 0x00,
        c: 0x02,
        h: 0x20,
        l: 0x00,
        d: 0x30,
        e: 0x00,
        ..m.cpu.regs()
    });
    assert_eq!(m.cpu.step(), 21, "one byte left to go, so it repeats");
    // Backing PC up by two is how the Z80 stays interruptible mid-block.
    assert_eq!(m.cpu.regs().pc, 0x1000);
    assert_eq!(m.cpu.regs().wz, 0x1001);
    assert_eq!(m.cpu.step(), 16, "the last iteration falls through");
    assert_eq!(m.cpu.regs().pc, 0x1002);
}

// ---------------------------------------------------------------------------
// The internal registers, in the small
// ---------------------------------------------------------------------------

#[test]
fn bit_through_hl_takes_its_undocumented_bits_from_memptr() {
    let m = Machine::new();
    // LD A,($2028) leaves WZ = $2029, then BIT 0,(HL) reads bits 3 and 5 of
    // the *latch*, not of the byte tested.
    m.load(0x1000, &[0x3a, 0x28, 0x20, 0xcb, 0x46]);
    m.load(0x2028, &[0x01]);
    m.start(0x1000);
    m.cpu.set_regs(Regs {
        h: 0x20,
        l: 0x28,
        ..m.cpu.regs()
    });
    m.cpu.step();
    assert_eq!(m.cpu.regs().wz, 0x2029);
    m.cpu.step();
    // W = $20, so bit 5 is set and bit 3 is clear.
    assert!(m.cpu.regs().flag(flags::YF));
    assert!(!m.cpu.regs().flag(flags::XF));
}

#[test]
fn scf_sees_whether_the_previous_instruction_wrote_flags() {
    // With Q live (the previous instruction wrote flags) the undocumented bits
    // come from A alone; with Q clear they are OR'd with the old F. `LD A,n`
    // writes no flags, `OR A` does, and that is the whole difference.
    let after_flagless = {
        let m = Machine::new();
        m.load(0x1000, &[0x3e, 0x00, 0x37]); // LD A,0 ; SCF
        m.start(0x1000);
        m.cpu.set_regs(Regs {
            f: flags::YF | flags::XF,
            ..m.cpu.regs()
        });
        m.cpu.step();
        m.cpu.step();
        m.cpu.regs().f & flags::XY
    };
    let after_flagged = {
        let m = Machine::new();
        m.load(0x1000, &[0xb7, 0x37]); // OR A ; SCF
        m.start(0x1000);
        m.cpu.set_regs(Regs {
            a: 0x00,
            f: flags::YF | flags::XF,
            ..m.cpu.regs()
        });
        m.cpu.step();
        m.cpu.step();
        m.cpu.regs().f & flags::XY
    };
    assert_eq!(after_flagless, flags::XY, "F | A, and F had both bits");
    assert_eq!(after_flagged, 0, "A alone, and A is zero");
}

#[test]
fn an_index_prefix_clears_q_all_by_itself() {
    // The prefix is its own M1 cycle, and Q is cleared by the fetch rather
    // than by the instruction — so `DD 37` sees Q clear even when the
    // instruction before it wrote flags.
    let m = Machine::new();
    m.load(0x1000, &[0xb7, 0xdd, 0x37]); // OR A ; DD SCF
    m.start(0x1000);
    m.cpu.set_regs(Regs {
        a: 0x00,
        f: flags::YF | flags::XF,
        ..m.cpu.regs()
    });
    m.cpu.step();
    let carried = m.cpu.regs().f & flags::XY;
    m.cpu.step();
    assert_eq!(carried, 0, "OR A of zero leaves both bits clear");
    assert_eq!(
        m.cpu.regs().f & flags::XY,
        0,
        "and F is what the prefix-cleared Q makes SCF read"
    );
}

// ---------------------------------------------------------------------------
// The undocumented pages, in the small
// ---------------------------------------------------------------------------

#[test]
fn the_index_halves_are_h_and_l_seen_through_a_prefix() {
    let m = Machine::new();
    // LD IXH,$12 ; LD IXL,$34 ; LD A,IXH
    m.load(0x1000, &[0xdd, 0x26, 0x12, 0xdd, 0x2e, 0x34, 0xdd, 0x7c]);
    m.start(0x1000);
    m.cpu.step();
    m.cpu.step();
    assert_eq!(m.cpu.regs().ix, 0x1234);
    assert_eq!(m.cpu.regs().h, 0x00, "the real H is untouched");
    m.cpu.step();
    assert_eq!(m.cpu.regs().a, 0x12);
}

#[test]
fn a_displaced_form_leaves_the_halves_alone() {
    let m = Machine::new();
    // LD H,(IX+1): the displacement wins, so H is the real H.
    m.load(0x1000, &[0xdd, 0x66, 0x01]);
    m.load(0x2001, &[0x5a]);
    m.start(0x1000);
    m.cpu.set_regs(Regs {
        ix: 0x2000,
        ..m.cpu.regs()
    });
    m.cpu.step();
    assert_eq!(m.cpu.regs().h, 0x5a);
    assert_eq!(m.cpu.regs().ix, 0x2000, "IXH did not move");
    assert_eq!(m.cpu.regs().wz, 0x2001, "the effective address is latched");
}

#[test]
fn ex_de_hl_ignores_an_index_prefix() {
    let m = Machine::new();
    m.load(0x1000, &[0xdd, 0xeb]);
    m.start(0x1000);
    m.cpu.set_regs(Regs {
        d: 0x11,
        e: 0x22,
        h: 0x33,
        l: 0x44,
        ix: 0x5566,
        ..m.cpu.regs()
    });
    m.cpu.step();
    assert_eq!(m.cpu.regs().de(), 0x3344);
    assert_eq!(m.cpu.regs().hl(), 0x1122);
    assert_eq!(m.cpu.regs().ix, 0x5566, "the prefix did nothing at all");
}

#[test]
fn sll_shifts_a_one_in_at_the_bottom() {
    let m = Machine::new();
    m.load(0x1000, &[0xcb, 0x30]); // SLL B
    m.start(0x1000);
    m.cpu.set_regs(Regs {
        b: 0x80,
        ..m.cpu.regs()
    });
    m.cpu.step();
    assert_eq!(m.cpu.regs().b, 0x01);
    assert!(m.cpu.regs().flag(flags::C));
}

#[test]
fn the_ddcb_forms_write_the_register_as_well_as_the_memory() {
    let m = Machine::new();
    // DD CB 01 00 is RLC (IX+1), and the low three bits still select B.
    m.load(0x1000, &[0xdd, 0xcb, 0x01, 0x00]);
    m.load(0x2001, &[0x81]);
    m.start(0x1000);
    m.cpu.set_regs(Regs {
        ix: 0x2000,
        ..m.cpu.regs()
    });
    m.cpu.step();
    assert_eq!(m.peek(0x2001), 0x03);
    assert_eq!(m.cpu.regs().b, 0x03, "the encoded register gets it too");

    // BIT has no result, so it copies nothing.
    let m = Machine::new();
    m.load(0x1000, &[0xdd, 0xcb, 0x01, 0x41]); // BIT 0,(IX+1), r = C
    m.load(0x2001, &[0x01]);
    m.start(0x1000);
    m.cpu.set_regs(Regs {
        ix: 0x2000,
        c: 0x99,
        ..m.cpu.regs()
    });
    m.cpu.step();
    assert_eq!(m.cpu.regs().c, 0x99);
}

#[test]
fn the_ed_pages_holes_are_two_fetches_and_nothing_else() {
    let m = Machine::new();
    m.load(0x1000, &[0xed, 0x00]);
    m.start(0x1000);
    let before = m.cpu.regs();
    assert_eq!(m.cpu.step(), 8);
    let after = m.cpu.regs();
    assert_eq!(after.pc, 0x1002);
    assert_eq!(after.a, before.a);
    assert_eq!(after.f, before.f);
}

// ---------------------------------------------------------------------------
// The table, the disassembler and the device surface
// ---------------------------------------------------------------------------

#[test]
fn the_disassembler_agrees_with_the_program_counter() {
    // If a row's operand count were wrong the two would drift, and this is the
    // cheapest place to notice.
    let code: &[&[u8]] = &[
        &[0x00],
        &[0x3e, 0x42],
        &[0x21, 0x34, 0x12],
        &[0xcb, 0x06],
        &[0xed, 0x43, 0x00, 0x20],
        &[0xdd, 0x36, 0x01, 0x02],
        &[0xdd, 0xcb, 0x01, 0x06],
        &[0xfd, 0xe5],
    ];
    let m = Machine::new();
    let mut at = 0x1000u16;
    for chunk in code {
        m.load(at, chunk);
        at = at.wrapping_add(chunk.len() as u16);
    }
    m.start(0x1000);
    let listing = m.cpu.disassemble(0x1000, code.len());
    assert_eq!(listing.len(), code.len());
    for (d, chunk) in listing.iter().zip(code) {
        assert_eq!(usize::from(d.len), chunk.len(), "{d}");
        let before = m.cpu.regs().pc;
        m.cpu.step();
        assert_eq!(
            m.cpu.regs().pc,
            before.wrapping_add(u16::from(d.len)),
            "{d}: the disassembler and the fetch disagree on length"
        );
    }
}

#[test]
fn every_encoding_the_tables_hold_can_be_executed() {
    // A row whose operands the interpreter cannot handle panics on an
    // `unreachable!`, so running all of them is a real check on the match arms
    // rather than a smoke test. The prefix rows are covered through the pages
    // they lead to.
    let m = Machine::new();
    for lead in [None, Some(0xcb), Some(0xed), Some(0xdd), Some(0xfd)] {
        for opcode in 0..=255u8 {
            let mut code = Vec::new();
            if let Some(prefix) = lead {
                code.push(prefix);
            }
            code.push(opcode);
            if lead == Some(0xdd) || lead == Some(0xfd) {
                // The DDCB page needs a displacement and a second opcode.
                code.push(0x01);
                code.push(0x06);
            }
            code.extend_from_slice(&[0x00, 0x00]);
            m.load(0x1000, &code);
            m.start(0x1000);
            m.cpu.set_regs(Regs {
                sp: 0xf000,
                ..m.cpu.regs()
            });
            let t = m.cpu.step();
            assert!(t >= 4, "{lead:02x?} {opcode:02x} charged {t} T-states");
        }
    }
}

#[test]
fn every_row_the_index_transform_produces_stays_decodable() {
    for opcode in 0..=255u8 {
        let base = decode(opcode);
        if base.is_prefix() {
            continue;
        }
        for index in [Index::Ix, Index::Iy] {
            let out = index_substitute(base, index);
            assert_eq!(out.op, base.op, "the prefix never changes the operation");
            assert_eq!(out.cond, base.cond);
        }
    }
}

#[test]
fn state_round_trips_through_a_snapshot() {
    let m = Machine::new();
    m.load(0x1000, &[0x21, 0x34, 0x12, 0xdd, 0x21, 0x78, 0x56]);
    m.start(0x1000);
    m.cpu.step();
    m.cpu.step();
    m.cpu.set_iff(true, false);
    m.cpu.set_interrupt_mode(2).expect("mode 2 exists");
    m.cpu.set_int(true);
    m.cpu.set_interrupt_vector(0x40);

    let mut shape = MachineShape::new();
    shape.add_device("cpu", super::CLASS.name).expect("fresh");
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w
            .chunk("cpu", super::CLASS.name, super::CLASS.version)
            .expect("fresh");
        m.cpu.save(&mut chunk).expect("save");
    }
    let bytes = w.to_vec().expect("serialise");

    let restored = Z80::new(Config::NMOS);
    let reader = StateReader::new(&bytes).expect("parse");
    let chunk = reader
        .load(
            "cpu",
            super::CLASS.name,
            super::CLASS.version,
            &Migrations::new(),
        )
        .expect("the chunk is there");
    let mut cr = chunk.reader();
    restored.load(&mut cr).expect("load");
    cr.end()
        .expect("the loader read every field the saver wrote");

    assert_eq!(restored.regs(), m.cpu.regs());
    assert_eq!(restored.iff(), (true, false));
    assert_eq!(restored.interrupt_mode(), 2);
    assert!(restored.int_asserted());
    assert_eq!(restored.interrupt_vector(), 0x40);
    assert_eq!(restored.cycles(), m.cpu.cycles());

    // The hash the invariant actually asks for: save the restored core and
    // compare the bytes.
    let mut shape2 = MachineShape::new();
    shape2.add_device("cpu", super::CLASS.name).expect("fresh");
    let mut w2 = StateWriter::new(shape2);
    {
        let mut chunk = w2
            .chunk("cpu", super::CLASS.name, super::CLASS.version)
            .expect("fresh");
        restored.save(&mut chunk).expect("save");
    }
    assert_eq!(
        w2.to_vec().expect("serialise"),
        bytes,
        "a round trip must be a fixed point"
    );
}

#[test]
fn a_snapshot_naming_an_impossible_interrupt_mode_is_rejected() {
    let m = Machine::new();
    let mut shape = MachineShape::new();
    shape.add_device("cpu", super::CLASS.name).expect("fresh");
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w
            .chunk("cpu", super::CLASS.name, super::CLASS.version)
            .expect("fresh");
        m.cpu.save(&mut chunk).expect("save");
    }
    let bytes = w.to_vec().expect("serialise");
    let reader = StateReader::new(&bytes).expect("parse");
    let (_, _, data) = reader.load_raw("cpu").expect("the chunk is there");

    // Thirteen words, then I and R, then the two flip-flop bools, then the
    // mode. Breaking it in place is the cheapest way to prove `load`
    // validates rather than trusting the file.
    let index = 13 * 2 + 2 + 2;
    assert_eq!(data[index], 0, "the mode was zero before we broke it");
    let mut broken = data.to_vec();
    broken[index] = 7;

    let restored = Z80::new(Config::NMOS);
    let err = restored
        .load(&mut crate::core::state::ChunkReader::new(&broken))
        .expect_err("mode 7 does not exist");
    assert!(alloc::format!("{err}").contains("interrupt mode 7"));
}

#[test]
fn realize_does_nothing_outward_because_the_space_has_not_arrived_yet() {
    // The check that a core has an address space used to live in `realize`. It
    // cannot: the realizer runs `realize` for every device *before* it binds
    // any of them, so a core that refused here would refuse every machine. The
    // check is in `Instance::bind`, and the test for it is below.
    let cpu = Z80::new(Config::NMOS);
    let mut deferred = Deferred::new();
    let mut ctx = RealizeCtx::new("/cpu0", RequesterId::ANONYMOUS, &mut deferred);
    assert!(cpu.realize(&mut ctx).is_ok());
}

/// A `BuildOptions` and registry that know about this core and `ram`/`rom`.
fn machine_layer() -> (crate::core::Registry, crate::machine::BuildOptions) {
    let mut options = crate::machine::BuildOptions::new();
    options.classes.insert(super::schema());
    for schema in crate::machine::builtin::schemas() {
        options.classes.insert(schema);
    }
    super::bind(&mut options.bindings).expect("nothing else claims cpu.z80");
    crate::machine::builtin::bind(&mut options.bindings).expect("ram and rom");

    let mut registry = crate::core::Registry::new();
    crate::machine::builtin::register(&mut registry).expect("ram and rom");
    super::register(&mut registry).expect("nothing else claims cpu.z80");
    (registry, options)
}

#[test]
fn binding_a_core_with_no_address_space_is_a_machine_error() {
    let (registry, options) = machine_layer();
    let text = "machine \"m\" {\n  osc x = 3500000 Hz\n  space mem { width = 16 }\n  \
                object dram \"ram\" { size = 4K }\n  object cpu \"cpu.z80\" { clock = x }\n  \
                map mem 0 size 4K = dram\n}\n";
    let err = crate::machine::build("t.machine", text, &registry, &options)
        .expect_err("a core with no `space =` cannot fetch");
    let text = alloc::format!("{err}");
    assert!(text.contains("address space"), "{text}");
}

#[test]
fn an_iospace_that_names_nothing_is_a_machine_error() {
    let (registry, options) = machine_layer();
    let text = "machine \"m\" {\n  osc x = 3500000 Hz\n  space mem { width = 16 }\n  \
                object dram \"ram\" { size = 4K }\n  \
                object cpu \"cpu.z80\" { clock = x, space = mem, iospace = \"ports\" }\n  \
                map mem 0 size 4K = dram\n}\n";
    let err = crate::machine::build("t.machine", text, &registry, &options)
        .expect_err("there is no space called `ports`");
    let text = alloc::format!("{err}");
    assert!(text.contains("ports"), "{text}");
}

#[test]
fn the_pins_a_machine_file_may_name_are_exactly_these_three() {
    use crate::core::device::Device;
    let cpu = Z80::new(Config::NMOS);
    for port in ["int", "nmi", "reset"] {
        assert!(cpu.sink(port, &[]).is_some(), "`{port}` should be a pin");
    }
    for port in ["irq", "busrq", ""] {
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

    let m = Machine::new();
    m.load(0x0100, &[0x3e, 0x42]); // LD A,$42
    m.start(0x0100);
    m.cpu.step();
    assert_eq!(m.cpu.reg(Reg::Af) >> 8, 0x42);

    let src = WireId::new(1);
    let pin = m.cpu.sink("reset", &[src]).expect("a reset pin");
    let wire = Wire::builder()
        .source(src)
        .sink_weak(Arc::downgrade(&pin.sink), pin.line)
        .build();
    // The latch lives outside the execution lock, so it becomes execution
    // state on the step that consumes it — which is what keeps a device
    // asserting reset from inside an access this core issued out of the core's
    // own critical section.
    wire.set(src, Level::High);
    m.cpu.step();
    assert_eq!(m.cpu.reg(Reg::Pc), 0, "the reset sequence clears PC");
    assert_eq!(m.cpu.reg(Reg::I), 0);
}

#[test]
fn the_scheduler_budget_is_never_overshot_and_the_debt_is_paid_back() {
    // `LDIR` with a long count is far longer than a one-T-state budget, so this
    // is the case where a plain `run` reports more than it was handed — which
    // the scheduler rejects outright.
    let m = Machine::new();
    m.load(0x0100, &[0xed, 0xb0]); // LDIR
    m.start(0x0100);
    m.cpu.set_reg(Reg::Bc, 0x0400);
    m.cpu.set_reg(Reg::Hl, 0x2000);
    m.cpu.set_reg(Reg::De, 0x3000);

    let before = m.cpu.cycles();
    let mut total = 0u64;
    for _ in 0..64 {
        let used = m.cpu.run_budget(1);
        assert!(used <= 1, "a budget of one T-state reported {used}");
        total += used;
    }
    assert_eq!(total, 64, "every tick of every budget was granted and used");
    assert_eq!(
        m.cpu.cycles() - before,
        total + m.cpu.cycle_debt(),
        "T-states executed but not yet reported are exactly the debt"
    );
}

#[test]
fn an_interrupt_pin_wire_ors_its_sources() {
    use crate::core::wire::{Level, WireId, WireSink};

    let m = Machine::new();
    let a = WireId::new(1);
    let b = WireId::new(2);
    let pin = super::InterruptPin::new(m.cpu.clone(), Interrupt::Int, &[a, b]);
    assert_eq!(pin.which(), Interrupt::Int);

    pin.set_level(a, 0, Level::High);
    assert!(m.cpu.int_asserted());
    // One source deasserting must not drop a line the other is still holding —
    // which is the whole reason a sink tracks its sources.
    pin.set_level(b, 0, Level::High);
    pin.set_level(a, 0, Level::Low);
    assert!(m.cpu.int_asserted());
    pin.set_level(b, 0, Level::Low);
    assert!(!m.cpu.int_asserted());
}

#[test]
fn the_isa_description_covers_every_encoding() {
    let text = super::describe_isa();
    assert_eq!(text.lines().count(), 256);
    assert!(text.contains("LD A,n"));
    assert!(text.contains("JP (HL)"));
}

#[test]
fn properties_reach_the_configuration() {
    use crate::core::props::Props;

    let cpu = Z80::from_props(&Props::new().with("cmos", true)).expect("valid");
    assert_eq!(cpu.config().out_c_zero, 0xff);

    let cpu = Z80::from_props(&Props::new().with("floating-bus", 0x00u64)).expect("valid");
    assert_eq!(cpu.config().floating_bus, 0x00);

    assert!(
        Z80::from_props(&Props::new().with("clok", 1u64)).is_err(),
        "a typo'd property must not be swallowed"
    );
}

// ---------------------------------------------------------------------------
// The acknowledge cycle
// ---------------------------------------------------------------------------

/// A peripheral that answers the acknowledge cycle, the way a Z80 PIO or CTC
/// does: it drives `INT`, and when the CPU acknowledges it puts its programmed
/// vector on the data bus and counts the service.
#[derive(Debug)]
struct Peripheral {
    vector: u8,
    acknowledged: crate::core::sync::AtomicU32,
}

impl crate::core::wire::IntAck for Peripheral {
    fn acknowledge(&self) -> u32 {
        self.acknowledged
            .fetch_add(1, crate::core::sync::Ordering::Relaxed);
        u32::from(self.vector)
    }
}

#[test]
fn mode_two_takes_its_vector_from_the_device_that_answers_the_acknowledge() {
    use crate::core::device::Device;
    use crate::core::sync::{AtomicU32, Ordering};
    use crate::core::wire::IntAck;

    let m = Machine::new();
    // The interrupt vector table at $8000, with entry $80/$0e pointing at
    // $2000. In mode 2 the CPU forms the address from `I` and the byte the
    // device drove, with bit 0 forced clear on real silicon.
    m.load(0x800e, &[0x00, 0x20]);
    m.load(0x2000, &[0xed, 0x4d]); // RETI
    m.load(0x0100, &[0x00]); // NOP

    let device: Arc<Peripheral> = Arc::new(Peripheral {
        vector: 0x0e,
        acknowledged: AtomicU32::new(0),
    });
    let weak: alloc::sync::Weak<dyn IntAck> = Arc::downgrade(&device) as _;
    m.cpu.attach_int_ack("int", weak);

    // After `start`, which runs the reset sequence -- and a reset clears `I`.
    m.start(0x0100);
    m.cpu.set_reg(Reg::I, 0x80);
    m.cpu.set_interrupt_mode(2).expect("mode 2 exists");
    m.cpu.set_iff(true, true);
    m.cpu.set_int(true);
    m.cpu.step();

    assert_eq!(
        device.acknowledged.load(Ordering::Relaxed),
        1,
        "the CPU must run the acknowledge cycle, not read a latched byte"
    );
    assert_eq!(
        m.cpu.reg(Reg::Pc),
        0x2000,
        "the vector the device drove is what the CPU jumped through"
    );
}

#[test]
fn with_nothing_attached_the_latched_byte_still_answers() {
    // The common case: a Master System's VDP drives `INT` and nothing answers
    // the cycle, so what the CPU reads is whatever is on the bus. A machine
    // with one fixed source sets that once.
    let m = Machine::new();
    m.load(0x8022, &[0x00, 0x30]);
    m.load(0x0100, &[0x00]);
    m.start(0x0100);
    m.cpu.set_reg(Reg::I, 0x80);
    m.cpu.set_interrupt_mode(2).expect("mode 2 exists");
    m.cpu.set_iff(true, true);
    m.cpu.set_interrupt_vector(0x22);
    m.cpu.set_int(true);
    m.cpu.step();
    assert_eq!(m.cpu.reg(Reg::Pc), 0x3000);
}
