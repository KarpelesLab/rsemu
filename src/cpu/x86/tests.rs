//! Hand-written tests for the parts `SingleStepTests/8088` cannot reach.
//!
//! The corpus is the accuracy gate and it is far better at the instruction set
//! than anything written by hand: ten thousand random vectors per opcode beat
//! any number of chosen cases. What it deliberately does *not* exercise is
//! reset, `HLT`, `WAIT`, `LOCK`, the interrupt and trap flags, or anything
//! involving the `INTR` and NMI pins — its own notes say so. Those, plus the
//! snapshot round trip and the segmentation edge cases, are what lives here.

use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::{Deferred, Device, RealizeCtx, ResetKind};
use crate::core::space::{AddressSpace, RamStore, Region, RequesterId};
use crate::core::state::{ChunkReader, MachineShape, StateWriter};

use super::isa::{Arg, Class, Grp, Op, decode, resolve};
use super::{Config, I8086, Interrupt, Model, Reg, Regs, flags, linear};

/// A core with a megabyte of RAM and a 64 KiB I/O space, both readable back.
struct Machine {
    cpu: Arc<I8086>,
    ram: Arc<RamStore>,
    ports: Arc<RamStore>,
}

impl Machine {
    fn new(cfg: Config) -> Machine {
        let ram = Arc::new(RamStore::new(0x10_0000));
        let mem = AddressSpace::new("mem", 20);
        mem.topology()
            .map(Region::ram("ram", ram.clone()), 0)
            .expect("1 MiB fits in 20 bits");

        let ports = Arc::new(RamStore::new(0x1_0000));
        let io = AddressSpace::new("io", 16);
        io.topology()
            .map(Region::ram("ports", ports.clone()), 0)
            .expect("64 KiB fits in 16 bits");

        let cpu = Arc::new(I8086::new(cfg));
        cpu.attach_space(Arc::new(mem));
        cpu.attach_io_space(Arc::new(io));
        Machine { cpu, ram, ports }
    }

    /// Put code at `cs:ip` and point the core at it, skipping reset.
    fn load(&self, cs: u16, ip: u16, code: &[u8]) {
        for (i, byte) in code.iter().enumerate() {
            let addr = linear(cs, ip.wrapping_add(i as u16));
            self.ram.write_u8(u64::from(addr), *byte).unwrap();
        }
        let mut regs = self.cpu.regs();
        regs.cs = cs;
        regs.ip = ip;
        self.cpu.set_regs(regs);
        self.cpu.session.lock().state.reset_pending = false;
    }

    fn poke(&self, addr: u32, byte: u8) {
        self.ram.write_u8(u64::from(addr), byte).unwrap();
    }

    fn peek(&self, addr: u32) -> u8 {
        self.ram.read_u8(u64::from(addr)).unwrap()
    }

    fn regs(&self) -> Regs {
        self.cpu.regs()
    }

    fn set_regs(&self, f: impl FnOnce(&mut Regs)) {
        let mut regs = self.cpu.regs();
        f(&mut regs);
        self.cpu.set_regs(regs);
    }
}

fn machine() -> Machine {
    Machine::new(Config::I8088)
}

// ---------------------------------------------------------------------------
// Segmentation
// ---------------------------------------------------------------------------

#[test]
fn a_segmented_address_is_twenty_bits_and_wraps_at_one_megabyte() {
    assert_eq!(linear(0x0000, 0x0000), 0x0_0000);
    assert_eq!(linear(0x1000, 0x0010), 0x1_0010);
    assert_eq!(linear(0xf000, 0xfff0), 0xf_fff0);
    // The classic case: the last paragraph of memory plus an offset runs off
    // the top of the address space and comes back at zero. Software used it
    // deliberately, which is why the AT needed an A20 gate to keep it working.
    assert_eq!(linear(0xffff, 0x0010), 0x0_0000);
    assert_eq!(linear(0xffff, 0x0020), 0x0_0010);
    assert_eq!(linear(0xffff, 0xffff), 0x0_ffef);
}

#[test]
fn a_read_above_the_first_megabyte_wraps_to_the_bottom() {
    let m = machine();
    m.poke(0x0_0000, 0x5a);
    // mov al, [0x0010] with DS = 0xffff reaches physical zero.
    m.load(0x0000, 0x0100, &[0xa0, 0x10, 0x00]);
    m.set_regs(|r| r.ds = 0xffff);
    m.cpu.step();
    assert_eq!(m.regs().ax & 0xff, 0x5a);
}

#[test]
fn bp_based_addressing_defaults_to_the_stack_segment() {
    let m = machine();
    m.set_regs(|r| {
        r.ds = 0x1000;
        r.ss = 0x2000;
        r.bp = 0x0004;
        r.bx = 0x0004;
    });
    m.poke(linear(0x2000, 0x0004), 0x11);
    m.poke(linear(0x1000, 0x0004), 0x22);
    // mov al, [bp+0] — SS by default.
    m.load(0x0000, 0x0100, &[0x8a, 0x46, 0x00]);
    m.cpu.step();
    assert_eq!(m.regs().ax & 0xff, 0x11);
    // mov al, [bx] — DS by default.
    m.load(0x0000, 0x0200, &[0x8a, 0x07]);
    m.cpu.step();
    assert_eq!(m.regs().ax & 0xff, 0x22);
    // ds: mov al, [bp+0] — the override wins.
    m.load(0x0000, 0x0300, &[0x3e, 0x8a, 0x46, 0x00]);
    m.cpu.step();
    assert_eq!(m.regs().ax & 0xff, 0x22);
}

#[test]
fn the_direct_address_encoding_is_not_bp_relative() {
    // md=0 rm=6 is a 16-bit address in DS, not [BP] — the encoding [BP] would
    // have used, which is why an assembler emits [BP+0] instead.
    let m = machine();
    m.set_regs(|r| {
        r.ds = 0x1000;
        r.ss = 0x2000;
        r.bp = 0xbeef;
    });
    m.poke(linear(0x1000, 0x0034), 0x77);
    m.load(0x0000, 0x0100, &[0x8a, 0x06, 0x34, 0x00]);
    m.cpu.step();
    assert_eq!(m.regs().ax & 0xff, 0x77);
}

// ---------------------------------------------------------------------------
// Reset, halt, and the pins
// ---------------------------------------------------------------------------

#[test]
fn reset_starts_sixteen_bytes_below_the_top_of_memory() {
    let m = machine();
    m.cpu.step();
    let regs = m.regs();
    assert_eq!((regs.cs, regs.ip), (0xffff, 0x0000));
    assert_eq!(linear(regs.cs, regs.ip), 0xf_fff0);
    assert_eq!((regs.ds, regs.es, regs.ss), (0, 0, 0));
    assert_eq!(regs.flags, flags::RESERVED_SET);
    assert!(!m.cpu.reset_pending());
}

#[test]
fn a_reset_vector_jump_lands_where_it_says() {
    let m = machine();
    // The PC's own reset vector shape: jmpf 0xf000:0xe05b.
    for (i, byte) in [0xea, 0x5b, 0xe0, 0x00, 0xf0].into_iter().enumerate() {
        m.poke(0xf_fff0 + i as u32, byte);
    }
    m.cpu.step(); // reset
    m.cpu.step(); // the far jump
    let regs = m.regs();
    assert_eq!((regs.cs, regs.ip), (0xf000, 0xe05b));
}

#[test]
fn halt_stops_the_core_until_an_interrupt_arrives() {
    let m = machine();
    m.load(0x0000, 0x0100, &[0xf4]);
    m.cpu.step();
    assert!(m.cpu.is_halted());
    // A halted core charges nothing, so a scheduler has to notice rather than
    // spin.
    assert_eq!(m.cpu.step(), 0);

    // An NMI restarts it, whatever the interrupt flag says.
    m.poke(0x0008, 0x00);
    m.poke(0x0009, 0x40);
    m.poke(0x000a, 0x00);
    m.poke(0x000b, 0x00);
    m.cpu.pulse_nmi();
    assert!(m.cpu.step() > 0);
    assert!(!m.cpu.is_halted());
    let regs = m.regs();
    assert_eq!((regs.cs, regs.ip), (0x0000, 0x4000));
}

#[test]
fn an_interrupt_pushes_flags_then_cs_then_the_return_address() {
    let m = machine();
    m.load(0x1000, 0x0100, &[0x90]);
    m.set_regs(|r| {
        r.ss = 0x2000;
        r.sp = 0x0100;
        r.flags |= flags::IF | flags::CF;
    });
    // Vector 0x20 → 0x3000:0x1234.
    m.poke(0x80, 0x34);
    m.poke(0x81, 0x12);
    m.poke(0x82, 0x00);
    m.poke(0x83, 0x30);
    m.cpu.set_intr_vector(0x20);
    m.cpu.set_intr(true);
    m.cpu.step();

    let regs = m.regs();
    assert_eq!((regs.cs, regs.ip), (0x3000, 0x1234));
    assert_eq!(regs.sp, 0x00fa);
    // The saved flags still have IF set; the CPU clears it only after the
    // push, which is what makes IRET restore it.
    let word = |off: u16| {
        u16::from(m.peek(linear(0x2000, off))) | (u16::from(m.peek(linear(0x2000, off + 1))) << 8)
    };
    assert_eq!(word(0x00fe) & flags::IF, flags::IF);
    assert_eq!(word(0x00fc), 0x1000); // CS
    assert_eq!(word(0x00fa), 0x0100); // the return IP, not the handler's
    assert_eq!(regs.flags & (flags::IF | flags::TF), 0);
}

#[test]
fn an_interrupt_is_masked_by_the_interrupt_flag_but_an_nmi_is_not() {
    let m = machine();
    m.load(0x0000, 0x0100, &[0x90, 0x90]);
    m.set_regs(|r| {
        r.ss = 0x2000;
        r.sp = 0x0100;
        r.flags &= !flags::IF;
    });
    m.cpu.set_intr_vector(0x20);
    m.cpu.set_intr(true);
    m.cpu.step();
    assert_eq!(
        m.regs().ip,
        0x0101,
        "INTR must be ignored while IF is clear"
    );

    m.poke(0x0008, 0x00);
    m.poke(0x0009, 0x40);
    m.cpu.pulse_nmi();
    m.cpu.step();
    assert_eq!(m.regs().ip, 0x4000, "NMI is not maskable");
}

#[test]
fn writing_the_stack_segment_shadows_the_next_instruction() {
    let m = machine();
    // mov ss, ax ; mov sp, bx — the canonical stack switch. An interrupt
    // taken between the two would run the handler on a half-changed stack.
    m.load(0x0000, 0x0100, &[0x8e, 0xd0, 0x89, 0xdc]);
    m.set_regs(|r| {
        r.ax = 0x3000;
        r.bx = 0x0200;
        r.flags |= flags::IF;
    });
    m.cpu.set_intr_vector(0x20);
    m.poke(0x80, 0x00);
    m.poke(0x81, 0x50);

    m.cpu.step(); // mov ss, ax
    assert!(m.cpu.interrupt_shadow());
    m.cpu.set_intr(true); // ... and only now does the controller ask
    m.cpu.step(); // mov sp, bx runs anyway, because the shadow holds
    assert_eq!(m.regs().ss, 0x3000);
    assert_eq!(m.regs().sp, 0x0200);
    assert!(!m.cpu.interrupt_shadow());
    m.cpu.step(); // and only now is the interrupt taken
    assert_eq!(m.regs().ip, 0x5000);
}

#[test]
fn the_trap_flag_takes_a_type_one_interrupt_after_each_instruction() {
    let m = machine();
    m.load(0x0000, 0x0100, &[0x90]);
    m.set_regs(|r| {
        r.ss = 0x2000;
        r.sp = 0x0100;
        r.flags |= flags::TF;
    });
    m.poke(0x04, 0x00);
    m.poke(0x05, 0x60);
    m.cpu.step();
    let regs = m.regs();
    assert_eq!(regs.ip, 0x6000);
    // The handler runs with the trap off, or it would trap on its own first
    // instruction forever.
    assert_eq!(regs.flags & flags::TF, 0);
}

// ---------------------------------------------------------------------------
// The prefetch queue
// ---------------------------------------------------------------------------

#[test]
fn the_queue_depth_follows_the_part() {
    // The bus interface unit fills the queue before each instruction, so after
    // a one-byte `nop` exactly one slot is free again.
    let m = Machine::new(Config::I8088);
    m.load(0x0000, 0x0100, &[0x90; 8]);
    m.cpu.step();
    assert_eq!(m.cpu.prefetch_queue().len(), 3);
    assert!(m.cpu.set_prefetch_queue(&[0; 4]).is_ok());
    assert!(m.cpu.set_prefetch_queue(&[0; 5]).is_err());

    let m = Machine::new(Config::I8086);
    m.load(0x0000, 0x0100, &[0x90; 8]);
    m.cpu.step();
    assert_eq!(m.cpu.prefetch_queue().len(), 5);
    assert!(m.cpu.set_prefetch_queue(&[0; 6]).is_ok());
    assert!(m.cpu.set_prefetch_queue(&[0; 7]).is_err());
}

#[test]
fn a_control_transfer_flushes_the_queue() {
    let m = machine();
    // jmp +0 followed by a byte that must never be executed from the queue.
    m.load(0x0000, 0x0100, &[0xeb, 0x00, 0xf4]);
    m.cpu.step();
    assert_eq!(m.regs().ip, 0x0102);
    assert!(
        m.cpu.prefetch_queue().is_empty(),
        "the queue held bytes fetched before the jump"
    );
}

#[test]
fn an_installed_queue_is_executed_before_memory_is_read() {
    let m = machine();
    // Memory says `nop`, the queue says `inc ax`. The queue wins, because the
    // bus interface unit fetched before the byte was changed — which is how
    // self-modifying code within a few bytes of IP behaves on hardware.
    m.load(0x0000, 0x0100, &[0x90, 0x90]);
    m.cpu.set_prefetch_queue(&[0x40]).unwrap();
    m.cpu.step();
    assert_eq!(m.regs().ax, 1);
    assert_eq!(m.regs().ip, 0x0101);
}

// ---------------------------------------------------------------------------
// Instructions the corpus leaves out
// ---------------------------------------------------------------------------

#[test]
fn wait_and_lock_do_nothing_observable() {
    let m = machine();
    m.load(0x0000, 0x0100, &[0x9b, 0xf0, 0x40]);
    m.cpu.step(); // wait
    assert_eq!(m.regs().ip, 0x0101);
    m.cpu.step(); // lock inc ax — one instruction, prefix included
    assert_eq!(m.regs().ip, 0x0103);
    assert_eq!(m.regs().ax, 1);
}

#[test]
fn separate_address_spaces_mean_a_port_is_not_a_memory_address() {
    let m = machine();
    m.ports.write_u8(0x0060, 0xa5).unwrap();
    m.poke(0x0060, 0x5a);
    // in al, 0x60
    m.load(0x0000, 0x0100, &[0xe4, 0x60]);
    m.cpu.step();
    assert_eq!(m.regs().ax & 0xff, 0xa5, "IN must not read memory");

    // out 0x61, al with al = 0x12
    m.set_regs(|r| r.ax = 0x0012);
    m.load(0x0000, 0x0200, &[0xe6, 0x61]);
    m.cpu.step();
    assert_eq!(m.ports.read_u8(0x0061).unwrap(), 0x12);
    assert_eq!(m.peek(0x0061), 0x00, "OUT must not write memory");
}

#[test]
fn a_word_port_access_is_two_consecutive_ports() {
    let m = machine();
    m.ports.write_u8(0x0300, 0x34).unwrap();
    m.ports.write_u8(0x0301, 0x12).unwrap();
    m.set_regs(|r| r.dx = 0x0300);
    m.load(0x0000, 0x0100, &[0xed]); // in ax, dx
    m.cpu.step();
    assert_eq!(m.regs().ax, 0x1234);
}

#[test]
fn a_core_with_no_io_space_reads_ones() {
    // What an unterminated bus does, and what the hardware corpus records.
    let ram = Arc::new(RamStore::new(0x10_0000));
    let mem = AddressSpace::new("mem", 20);
    mem.topology()
        .map(Region::ram("ram", ram.clone()), 0)
        .unwrap();
    let cpu = I8086::new(Config::I8088);
    cpu.attach_space(Arc::new(mem));
    for (i, byte) in [0xe4u8, 0x60].into_iter().enumerate() {
        ram.write_u8(0x100 + i as u64, byte).unwrap();
    }
    cpu.set_regs(Regs {
        cs: 0,
        ip: 0x100,
        ..Regs::new()
    });
    cpu.session.lock().state.reset_pending = false;
    cpu.step();
    assert_eq!(cpu.regs().ax & 0xff, 0xff);
}

#[test]
fn string_moves_follow_the_direction_flag() {
    let m = machine();
    for i in 0..4u32 {
        m.poke(0x1_0000 + i, 0xa0 + i as u8);
    }
    m.set_regs(|r| {
        r.ds = 0x1000;
        r.es = 0x2000;
        r.si = 0;
        r.di = 0;
        r.cx = 4;
    });
    m.load(0x0000, 0x0100, &[0xf3, 0xa4]); // rep movsb
    m.cpu.step();
    assert_eq!(m.regs().cx, 0);
    assert_eq!(m.regs().si, 4);
    assert_eq!(m.regs().di, 4);
    for i in 0..4u32 {
        assert_eq!(m.peek(0x2_0000 + i), 0xa0 + i as u8);
    }

    // Backwards, and one short: REPNE stops on a match.
    m.set_regs(|r| {
        r.es = 0x2000;
        r.di = 3;
        r.cx = 4;
        r.ax = 0xa2;
        r.flags |= flags::DF;
    });
    m.load(0x0000, 0x0200, &[0xf2, 0xae]); // repne scasb
    m.cpu.step();
    assert_eq!(m.regs().cx, 2, "scan stops the moment it matches");
    assert_eq!(m.regs().di, 1);
}

#[test]
fn a_repeat_with_a_zero_count_does_nothing_at_all() {
    let m = machine();
    m.set_regs(|r| {
        r.cx = 0;
        r.si = 0x10;
        r.di = 0x20;
    });
    m.load(0x0000, 0x0100, &[0xf3, 0xa4]);
    m.cpu.step();
    let regs = m.regs();
    assert_eq!((regs.si, regs.di, regs.cx), (0x10, 0x20, 0));
}

#[test]
fn a_repeat_is_interruptible_between_iterations() {
    let m = machine();
    m.set_regs(|r| {
        r.ax = 0x3000;
        r.ds = 0x1000;
        r.es = 0x2000;
        r.cx = 100;
    });
    // Vector 2 (NMI) → 0x0000:0x7000.
    m.poke(0x08, 0x00);
    m.poke(0x09, 0x70);
    // `mov ss, ax` first, so its shadow suppresses the check that would
    // otherwise take the interrupt *before* the repeat rather than during it.
    m.load(0x0000, 0x0100, &[0x8e, 0xd0, 0xf3, 0xa4]);
    m.cpu.step();
    m.cpu.pulse_nmi();
    m.cpu.step();
    // The instruction backed itself out: IP points at the prefix again, so
    // the handler returns straight into the rest of the copy.
    assert_eq!(m.regs().ip, 0x0102);
    assert!(m.regs().cx < 100 && m.regs().cx > 0);
    m.cpu.step();
    assert_eq!(m.regs().ip, 0x7000);
}

#[test]
fn push_sp_stores_the_decremented_pointer() {
    // True of the 8086 and 8088 and of nothing later: the 286 pushes the value
    // SP had before the instruction.
    let m = machine();
    m.set_regs(|r| {
        r.ss = 0x2000;
        r.sp = 0x0100;
    });
    m.load(0x0000, 0x0100, &[0x54]);
    m.cpu.step();
    assert_eq!(m.regs().sp, 0x00fe);
    let pushed = u16::from(m.peek(linear(0x2000, 0x00fe)))
        | (u16::from(m.peek(linear(0x2000, 0x00ff))) << 8);
    assert_eq!(pushed, 0x00fe);
}

// ---------------------------------------------------------------------------
// The results the corpus taught us, locked in so `cargo test` protects them
// ---------------------------------------------------------------------------
//
// Each of these is a rule that was *measured* against `SingleStepTests/8088`
// rather than read out of a manual — in two cases the manual is wrong — and
// the corpus is optional. Without these the rules could be undone by a
// plausible-looking simplification and nothing offline would notice.

#[test]
fn the_decimal_adjust_threshold_moves_with_the_auxiliary_carry() {
    // `AL = 0x9a` is above 0x99, so the published algorithm corrects both
    // digits. The 8088 corrects only the low one when AF is set, because the
    // threshold it compares against is 0x9f then. 0x9a + 6 = 0xa0.
    let m = machine();
    m.load(0x0000, 0x0100, &[0x27]);
    m.set_regs(|r| {
        r.ax = 0x009a;
        r.flags = (r.flags | flags::AF) & !flags::CF;
    });
    m.cpu.step();
    assert_eq!(m.regs().ax & 0xff, 0xa0);
    assert_eq!(m.regs().flags & flags::CF, 0);

    // With AF clear the same AL takes both corrections: 0x9a + 0x66 = 0x00.
    m.load(0x0000, 0x0200, &[0x27]);
    m.set_regs(|r| {
        r.ax = 0x009a;
        r.flags &= !(flags::AF | flags::CF);
    });
    m.cpu.step();
    assert_eq!(m.regs().ax & 0xff, 0x00);
    assert_eq!(m.regs().flags & flags::CF, flags::CF);
}

#[test]
fn an_unadjusted_ascii_add_still_sets_sign_zero_and_parity() {
    // `AAA` performs an 8-bit `AL + 0` when no adjustment is needed, which is
    // why the officially undefined sign, zero and parity results are those of
    // the original AL rather than of the masked one.
    let m = machine();
    m.load(0x0000, 0x0100, &[0x37]);
    m.set_regs(|r| {
        r.ax = 0x0081; // AL = 0x81: low digit 1, so no adjustment
        r.flags &= !(flags::AF | flags::SF | flags::PF | flags::ZF);
    });
    m.cpu.step();
    let regs = m.regs();
    assert_eq!(regs.ax, 0x0001, "only the low digit survives");
    assert_eq!(
        regs.flags & flags::SF,
        flags::SF,
        "sign of 0x81, not of 0x01"
    );
    assert_eq!(regs.flags & (flags::CF | flags::AF), 0);
}

#[test]
fn a_shift_by_zero_still_writes_its_operand_back() {
    // Nothing changes and no flag moves, but the write happens on the bus —
    // which a memory-mapped device would see. Verified against the hardware
    // corpus's cycle traces.
    let m = machine();
    let log = Arc::new(BusLog::default());
    let mem = AddressSpace::new("mem", 20);
    mem.topology()
        .map(Region::ram("ram", m.ram.clone()), 0)
        .unwrap();
    mem.topology()
        .map_with_priority(Region::io("watch", 0x10, log.clone()), 0x2_0000, 1)
        .unwrap();
    m.cpu.attach_space(Arc::new(mem));

    m.ram.write_u8(0x0100, 0xd2).unwrap(); // rol byte [bx], cl
    m.ram.write_u8(0x0101, 0x07).unwrap();
    m.set_regs(|r| {
        r.cs = 0;
        r.ip = 0x100;
        r.ds = 0x2000;
        r.bx = 0;
        r.cx = 0; // CL = 0: no rotation at all
        r.flags |= flags::CF;
    });
    m.cpu.session.lock().state.reset_pending = false;
    log.clear();
    m.cpu.step();

    assert_eq!(
        log.entries(),
        alloc::vec![(0u64, false), (0u64, true)],
        "the operand is read and written back even with a zero count"
    );
    assert_eq!(m.regs().flags & flags::CF, flags::CF, "flags are untouched");
}

/// A region that records the accesses made to it, for the tests that care
/// about *which* bus cycles an instruction performs rather than only about the
/// state it leaves.
#[derive(Debug, Default)]
struct BusLog {
    cells: crate::core::sync::Mutex<BusLogState>,
}

/// The watched region's contents, and `(offset, is_write)` for every access.
#[derive(Debug, Default)]
struct BusLogState(alloc::vec::Vec<u8>, alloc::vec::Vec<(u64, bool)>);

impl BusLog {
    fn clear(&self) {
        let mut m = self.cells.lock();
        m.0.resize(0x10, 0);
        m.1.clear();
    }

    fn entries(&self) -> alloc::vec::Vec<(u64, bool)> {
        self.cells.lock().1.clone()
    }
}

impl crate::core::space::MemOps for BusLog {
    fn read(
        &self,
        offset: u64,
        dst: &mut [u8],
        attrs: crate::core::space::MemAttrs,
    ) -> crate::core::space::MemResult {
        let mut m = self.cells.lock();
        m.0.resize(0x10, 0);
        for (i, slot) in dst.iter_mut().enumerate() {
            let at = (offset as usize + i) & 0xf;
            *slot = m.0[at];
            if !attrs.debug {
                m.1.push((at as u64, false));
            }
        }
        Ok(())
    }

    fn write(
        &self,
        offset: u64,
        src: &[u8],
        attrs: crate::core::space::MemAttrs,
    ) -> crate::core::space::MemResult {
        let mut m = self.cells.lock();
        m.0.resize(0x10, 0);
        for (i, byte) in src.iter().enumerate() {
            let at = (offset as usize + i) & 0xf;
            m.0[at] = *byte;
            if !attrs.debug {
                m.1.push((at as u64, true));
            }
        }
        Ok(())
    }

    fn constraints(&self) -> crate::core::space::AccessConstraints {
        crate::core::space::AccessConstraints::ANY
    }
}

#[test]
fn a_multiply_takes_its_undefined_flags_from_the_high_half() {
    // Intel calls sign, zero, parity and the auxiliary carry undefined after
    // `MUL`. The hardware sets them from the product's high half, and the
    // corpus agrees on all 20 000 vectors.
    let m = machine();
    m.load(0x0000, 0x0100, &[0xf6, 0xe3]); // mul bl
    m.set_regs(|r| {
        r.ax = 0x0010;
        r.bx = 0x0010;
    });
    m.cpu.step();
    let regs = m.regs();
    assert_eq!(regs.ax, 0x0100);
    // AH is 1: not zero, not negative, odd parity.
    assert_eq!(regs.flags & flags::ZF, 0);
    assert_eq!(regs.flags & flags::SF, 0);
    assert_eq!(regs.flags & flags::PF, 0);
    assert_eq!(regs.flags & flags::AF, 0);
    assert_eq!(regs.flags & (flags::CF | flags::OF), flags::CF | flags::OF);
}

#[test]
fn a_divide_error_pushes_the_following_instruction() {
    // The 8088 pushes the address of the *next* instruction on a divide
    // error, not of the faulting one. Later parts changed this, and generic
    // x86 emulators habitually get it wrong.
    let m = machine();
    m.load(0x0000, 0x0100, &[0xf6, 0xf3, 0x90]); // div bl ; nop
    m.set_regs(|r| {
        r.ax = 0xffff;
        r.bx = 0x0001; // BL = 1: the quotient cannot fit in AL
        r.ss = 0x2000;
        r.sp = 0x0100;
    });
    m.poke(0x00, 0x00);
    m.poke(0x01, 0x04); // vector 0 → 0000:0400
    m.cpu.step();
    let regs = m.regs();
    assert_eq!((regs.cs, regs.ip), (0x0000, 0x0400));
    let pushed_ip = u16::from(m.peek(linear(0x2000, 0x00fa)))
        | (u16::from(m.peek(linear(0x2000, 0x00fb))) << 8);
    assert_eq!(pushed_ip, 0x0102, "the address after `div`, not of it");
}

#[test]
fn a_repeat_prefix_inverts_an_idiv_quotient() {
    // Undocumented, useless, and real: a `REP` in front of `IDIV` flips the
    // sign of the quotient. The corpus prepends one to a tenth of its IDIV
    // vectors precisely to catch a core that ignores it.
    let m = machine();
    m.load(0x0000, 0x0100, &[0xf3, 0xf6, 0xfb]); // rep idiv bl
    m.set_regs(|r| {
        r.ax = 0x0064; // 100
        r.bx = 0x000a; // BL = 10
    });
    m.cpu.step();
    assert_eq!(m.regs().ax & 0xff, 0xf6, "100 / 10 = 10, negated to -10");

    m.load(0x0000, 0x0200, &[0xf6, 0xfb]); // idiv bl, no prefix
    m.set_regs(|r| {
        r.ax = 0x0064;
        r.bx = 0x000a;
    });
    m.cpu.step();
    assert_eq!(m.regs().ax & 0xff, 0x0a);
}

#[test]
fn logical_operations_clear_the_auxiliary_carry() {
    // Documented as undefined; cleared on every corpus vector.
    let m = machine();
    m.load(0x0000, 0x0100, &[0x24, 0xff]); // and al, 0xff
    m.set_regs(|r| {
        r.ax = 0x0001;
        r.flags |= flags::AF | flags::CF | flags::OF;
    });
    m.cpu.step();
    let regs = m.regs();
    assert_eq!(regs.flags & (flags::AF | flags::CF | flags::OF), 0);
}

#[test]
fn a_left_shift_leaves_bit_four_of_its_result_in_the_auxiliary_carry() {
    // The microcode for `SHL` is an `ADD dst,dst`, so the auxiliary carry it
    // leaves is a real one — which is why it tracks bit 4 of the result.
    let m = machine();
    for (value, want) in [(0x08u16, true), (0x04u16, false)] {
        m.load(0x0000, 0x0100, &[0xd0, 0xe0]); // shl al, 1
        m.set_regs(|r| {
            r.ax = value;
            r.flags &= !flags::AF;
        });
        m.cpu.step();
        assert_eq!(
            m.regs().flags & flags::AF != 0,
            want,
            "shl of {value:#x} should leave AF = {want}"
        );
    }
}

// ---------------------------------------------------------------------------
// The register file
// ---------------------------------------------------------------------------

#[test]
fn byte_registers_are_the_halves_of_the_word_registers() {
    let mut regs = Regs::new();
    regs.ax = 0x1234;
    assert_eq!(regs.byte(0), 0x34); // al
    assert_eq!(regs.byte(4), 0x12); // ah
    regs.set_byte(4, 0xab);
    assert_eq!(regs.ax, 0xab34);
    regs.set_byte(0, 0xcd);
    assert_eq!(regs.ax, 0xabcd);
    // The order is AL CL DL BL AH CH DH BH, which is why AH is 4.
    regs.cx = 0x0000;
    regs.set_byte(5, 0xff);
    assert_eq!(regs.cx, 0xff00);
}

#[test]
fn the_hard_wired_flag_bits_cannot_be_written() {
    let m = machine();
    // mov ax, 0 ; push ax ; popf
    m.load(0x0000, 0x0100, &[0xb8, 0x00, 0x00, 0x50, 0x9d]);
    m.set_regs(|r| {
        r.ss = 0x2000;
        r.sp = 0x0100;
    });
    m.cpu.step();
    m.cpu.step();
    m.cpu.step();
    assert_eq!(m.regs().flags, flags::RESERVED_SET);
    assert_eq!(Regs::normalise_flags(0x0000), 0xf002);
    assert_eq!(Regs::normalise_flags(0xffff), 0xffd7);
}

#[test]
fn registers_are_reachable_by_name() {
    assert_eq!(Reg::from_name("ax"), Some(Reg::Ax));
    assert_eq!(Reg::from_name("flags"), Some(Reg::Flags));
    assert_eq!(Reg::from_name("eax"), None);
    for reg in Reg::ALL {
        assert_eq!(Reg::from_name(reg.name()), Some(*reg));
    }
    // The ModRM word order, which is not the alphabetical one.
    assert_eq!(Reg::from_word_index(3), Reg::Bx);
    assert_eq!(Reg::from_word_index(4), Reg::Sp);
}

// ---------------------------------------------------------------------------
// The device surface
// ---------------------------------------------------------------------------

#[test]
fn a_core_with_no_address_space_refuses_to_realize() {
    let cpu = I8086::new(Config::default());
    let mut deferred = Deferred::new();
    let mut ctx = RealizeCtx::new("/cpu0", RequesterId::ANONYMOUS, &mut deferred);
    let err = cpu.realize(&mut ctx).expect_err("no space attached");
    assert!(alloc::format!("{err}").contains("address space"));
}

#[test]
fn state_round_trips_through_a_snapshot() {
    let m = machine();
    m.load(0x1234, 0x5678, &[0x40, 0x41, 0x42]);
    m.set_regs(|r| {
        r.ax = 0x1111;
        r.bx = 0x2222;
        r.cx = 0x3333;
        r.dx = 0x4444;
        r.sp = 0x5555;
        r.bp = 0x6666;
        r.si = 0x7777;
        r.di = 0x8888;
        r.es = 0x9999;
        r.ss = 0xaaaa;
        r.ds = 0xbbbb;
        r.flags |= flags::CF | flags::DF;
    });
    m.cpu.step();
    m.cpu.set_intr_vector(0x42);
    m.cpu.set_intr(true);
    let before = m.regs();
    let queue_before = m.cpu.prefetch_queue();
    let cycles_before = m.cpu.cycles();

    let mut shape = MachineShape::new();
    shape.add_device("/cpu0", "cpu.i8086").unwrap();
    let mut writer = StateWriter::new(shape);
    {
        let mut chunk = writer.chunk("/cpu0", "cpu.i8086", 1).unwrap();
        m.cpu.save(&mut chunk).unwrap();
    }
    let bytes = writer.to_vec().unwrap();

    // Wreck the core, then put it back.
    m.cpu.reset(ResetKind::Cold);
    assert_ne!(m.cpu.regs(), before);

    let reader = crate::core::state::StateReader::new(&bytes).unwrap();
    let (_, _, data) = reader.load_raw("/cpu0").unwrap();
    let mut chunk = ChunkReader::new(data);
    m.cpu.load(&mut chunk).unwrap();
    chunk.end().unwrap();

    assert_eq!(m.cpu.regs(), before);
    assert_eq!(m.cpu.prefetch_queue(), queue_before);
    assert_eq!(m.cpu.cycles(), cycles_before);
    assert_eq!(m.cpu.intr_vector(), 0x42);
    assert!(m.cpu.intr_asserted());
}

#[test]
fn a_warm_reset_keeps_the_general_registers_and_a_cold_one_does_not() {
    let m = machine();
    m.set_regs(|r| r.ax = 0xbeef);
    m.cpu.reset(ResetKind::Warm);
    assert!(m.cpu.reset_pending());
    m.cpu.step();
    assert_eq!(m.regs().ax, 0xbeef);
    assert_eq!(m.regs().cs, 0xffff);

    m.cpu.reset(ResetKind::Cold);
    assert_eq!(m.cpu.regs().ax, 0);
    assert_eq!(m.cpu.cycles(), 0);
}

#[test]
fn the_model_property_picks_the_part() {
    use crate::core::props::Props;
    let cpu = I8086::from_props(&Props::new().with("model", "8086")).unwrap();
    assert_eq!(cpu.config().model, Model::I8086);

    // An unknown part is named in the error, with the set that is accepted.
    let err = I8086::from_props(&Props::new().with("model", "80386")).unwrap_err();
    let text = alloc::format!("{err}");
    assert!(text.contains("8086") && text.contains("8088"), "{text}");

    assert!(
        I8086::from_props(&Props::new().with("modle", "8088")).is_err(),
        "a typo'd property must not be ignored"
    );

    assert_eq!(
        I8086::from_props(&Props::new()).unwrap().config().model,
        Model::I8088
    );
}

#[test]
fn the_interrupt_pin_drives_the_core_through_a_wire() {
    use crate::core::wire::{Level, WireId, WireSink};
    let m = machine();
    let a = WireId(1);
    let b = WireId(2);
    let pin = super::InterruptPin::new(m.cpu.clone(), Interrupt::Intr, &[a, b]);
    assert_eq!(pin.which(), Interrupt::Intr);
    pin.set_level(a, 0, Level::High);
    assert!(m.cpu.intr_asserted());
    // Wire-OR: the line stays asserted while any source holds it.
    pin.set_level(b, 0, Level::High);
    pin.set_level(a, 0, Level::Low);
    assert!(m.cpu.intr_asserted());
    pin.set_level(b, 0, Level::Low);
    assert!(!m.cpu.intr_asserted());
}

// ---------------------------------------------------------------------------
// The table and the disassembler agree with the interpreter
// ---------------------------------------------------------------------------

#[test]
fn the_opcode_map_describes_every_byte() {
    let described = super::describe_isa();
    assert!(described.lines().count() > 256);
    assert!(described.contains("d6    *salc"));
    assert!(described.contains("ff/3  callf"));
}

#[test]
fn the_undocumented_encodings_execute_rather_than_fault() {
    // An 8086 has no invalid-opcode exception: every byte does something, and
    // software has depended on several of them.
    let m = machine();
    // salc with carry set.
    m.load(0x0000, 0x0100, &[0xf9, 0xd6]);
    m.cpu.step();
    m.cpu.step();
    assert_eq!(m.regs().ax & 0xff, 0xff);
    // setmo: the undocumented D0 /6.
    m.set_regs(|r| r.ax &= 0xff00);
    m.load(0x0000, 0x0200, &[0xd0, 0xf0]);
    m.cpu.step();
    assert_eq!(m.regs().ax & 0xff, 0xff);
    assert_eq!(resolve(decode(0xd0), 6).op, Op::SETMO);
    // The 60-6F aliases really are the conditional jumps.
    assert_eq!(decode(0x64).op, decode(0x74).op);
    assert_eq!(decode(0x64).class, Class::Alias);
}

#[test]
fn the_disassembler_and_the_interpreter_read_the_same_bytes() {
    let m = machine();
    let code = [0xb8u8, 0x34, 0x12, 0x03, 0x46, 0xfe, 0xeb, 0xfa];
    m.load(0x0000, 0x0100, &code);
    let listing = m.cpu.disassemble(0x0000, 0x0100, 3);
    let text: Vec<_> = listing
        .iter()
        .map(alloc::string::ToString::to_string)
        .collect();
    assert_eq!(text[0], "mov ax, 0x1234");
    assert_eq!(text[1], "add ax, [ss:bp-0x2]");
    assert_eq!(text[2], "jmp 0x102");
    // Executing them advances IP by exactly the lengths the listing claims.
    let mut ip = 0x0100u16;
    for entry in &listing[..2] {
        m.cpu.step();
        ip = ip.wrapping_add(u16::from(entry.len));
        assert_eq!(m.regs().ip, ip);
    }
}

#[test]
fn group_rows_and_primary_rows_share_one_description() {
    // The point of the single table: an operand form is written once.
    for opcode in [0x80u8, 0x81, 0x82, 0x83] {
        let primary = decode(opcode);
        assert_eq!(primary.group, Grp::Alu);
        for reg in 0..8 {
            let row = resolve(primary, reg);
            assert_eq!(row.dst, primary.dst);
            assert_eq!(row.src, primary.src);
        }
    }
    // The unary group is the exception, and says so by carrying its own.
    assert_eq!(resolve(decode(0xf6), 0).src, Arg::Ib);
    assert_eq!(resolve(decode(0xf6), 2).src, Arg::None);
}
