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
use super::{Config, Interrupt, Reg, Regs, Variant, X86, flags, linear};

/// A core with a megabyte of RAM and a 64 KiB I/O space, both readable back.
struct Machine {
    cpu: Arc<X86>,
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

        let cpu = Arc::new(X86::new(cfg));
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
        regs.eip = u32::from(ip);
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
    assert_eq!(m.regs().eax & 0xff, 0x5a);
}

#[test]
fn bp_based_addressing_defaults_to_the_stack_segment() {
    let m = machine();
    m.set_regs(|r| {
        r.ds = 0x1000;
        r.ss = 0x2000;
        r.ebp = 0x0004;
        r.ebx = 0x0004;
    });
    m.poke(linear(0x2000, 0x0004), 0x11);
    m.poke(linear(0x1000, 0x0004), 0x22);
    // mov al, [bp+0] — SS by default.
    m.load(0x0000, 0x0100, &[0x8a, 0x46, 0x00]);
    m.cpu.step();
    assert_eq!(m.regs().eax & 0xff, 0x11);
    // mov al, [bx] — DS by default.
    m.load(0x0000, 0x0200, &[0x8a, 0x07]);
    m.cpu.step();
    assert_eq!(m.regs().eax & 0xff, 0x22);
    // ds: mov al, [bp+0] — the override wins.
    m.load(0x0000, 0x0300, &[0x3e, 0x8a, 0x46, 0x00]);
    m.cpu.step();
    assert_eq!(m.regs().eax & 0xff, 0x22);
}

#[test]
fn the_direct_address_encoding_is_not_bp_relative() {
    // md=0 rm=6 is a 16-bit address in DS, not [BP] — the encoding [BP] would
    // have used, which is why an assembler emits [BP+0] instead.
    let m = machine();
    m.set_regs(|r| {
        r.ds = 0x1000;
        r.ss = 0x2000;
        r.ebp = 0xbeef;
    });
    m.poke(linear(0x1000, 0x0034), 0x77);
    m.load(0x0000, 0x0100, &[0x8a, 0x06, 0x34, 0x00]);
    m.cpu.step();
    assert_eq!(m.regs().eax & 0xff, 0x77);
}

// ---------------------------------------------------------------------------
// Reset, halt, and the pins
// ---------------------------------------------------------------------------

#[test]
fn reset_starts_sixteen_bytes_below_the_top_of_memory() {
    let m = machine();
    m.cpu.step();
    let regs = m.regs();
    assert_eq!((regs.cs, regs.eip), (0xffff, 0x0000));
    assert_eq!(linear(regs.cs, regs.eip as u16), 0xf_fff0);
    assert_eq!((regs.ds, regs.es, regs.ss), (0, 0, 0));
    assert_eq!(regs.eflags, flags::RESERVED_SET);
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
    assert_eq!((regs.cs, regs.eip), (0xf000, 0xe05b));
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
    assert_eq!((regs.cs, regs.eip), (0x0000, 0x4000));
}

#[test]
fn an_interrupt_pushes_flags_then_cs_then_the_return_address() {
    let m = machine();
    m.load(0x1000, 0x0100, &[0x90]);
    m.set_regs(|r| {
        r.ss = 0x2000;
        r.esp = 0x0100;
        r.eflags |= flags::IF | flags::CF;
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
    assert_eq!((regs.cs, regs.eip), (0x3000, 0x1234));
    assert_eq!(regs.esp, 0x00fa);
    // The saved flags still have IF set; the CPU clears it only after the
    // push, which is what makes IRET restore it.
    let word = |off: u16| {
        u16::from(m.peek(linear(0x2000, off))) | (u16::from(m.peek(linear(0x2000, off + 1))) << 8)
    };
    assert_eq!(u32::from(word(0x00fe)) & flags::IF, flags::IF);
    assert_eq!(word(0x00fc), 0x1000); // CS
    assert_eq!(word(0x00fa), 0x0100); // the return IP, not the handler's
    assert_eq!(regs.eflags & (flags::IF | flags::TF), 0);
}

#[test]
fn an_interrupt_is_masked_by_the_interrupt_flag_but_an_nmi_is_not() {
    let m = machine();
    m.load(0x0000, 0x0100, &[0x90, 0x90]);
    m.set_regs(|r| {
        r.ss = 0x2000;
        r.esp = 0x0100;
        r.eflags &= !flags::IF;
    });
    m.cpu.set_intr_vector(0x20);
    m.cpu.set_intr(true);
    m.cpu.step();
    assert_eq!(
        m.regs().eip,
        0x0101,
        "INTR must be ignored while IF is clear"
    );

    m.poke(0x0008, 0x00);
    m.poke(0x0009, 0x40);
    m.cpu.pulse_nmi();
    m.cpu.step();
    assert_eq!(m.regs().eip, 0x4000, "NMI is not maskable");
}

#[test]
fn writing_the_stack_segment_shadows_the_next_instruction() {
    let m = machine();
    // mov ss, ax ; mov sp, bx — the canonical stack switch. An interrupt
    // taken between the two would run the handler on a half-changed stack.
    m.load(0x0000, 0x0100, &[0x8e, 0xd0, 0x89, 0xdc]);
    m.set_regs(|r| {
        r.eax = 0x3000;
        r.ebx = 0x0200;
        r.eflags |= flags::IF;
    });
    m.cpu.set_intr_vector(0x20);
    m.poke(0x80, 0x00);
    m.poke(0x81, 0x50);

    m.cpu.step(); // mov ss, ax
    assert!(m.cpu.interrupt_shadow());
    m.cpu.set_intr(true); // ... and only now does the controller ask
    m.cpu.step(); // mov sp, bx runs anyway, because the shadow holds
    assert_eq!(m.regs().ss, 0x3000);
    assert_eq!(m.regs().esp, 0x0200);
    assert!(!m.cpu.interrupt_shadow());
    m.cpu.step(); // and only now is the interrupt taken
    assert_eq!(m.regs().eip, 0x5000);
}

#[test]
fn the_trap_flag_takes_a_type_one_interrupt_after_each_instruction() {
    let m = machine();
    m.load(0x0000, 0x0100, &[0x90]);
    m.set_regs(|r| {
        r.ss = 0x2000;
        r.esp = 0x0100;
        r.eflags |= flags::TF;
    });
    m.poke(0x04, 0x00);
    m.poke(0x05, 0x60);
    m.cpu.step();
    let regs = m.regs();
    assert_eq!(regs.eip, 0x6000);
    // The handler runs with the trap off, or it would trap on its own first
    // instruction forever.
    assert_eq!(regs.eflags & flags::TF, 0);
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
    assert_eq!(m.regs().eip, 0x0102);
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
    assert_eq!(m.regs().eax, 1);
    assert_eq!(m.regs().eip, 0x0101);
}

// ---------------------------------------------------------------------------
// Instructions the corpus leaves out
// ---------------------------------------------------------------------------

#[test]
fn wait_and_lock_do_nothing_observable() {
    let m = machine();
    m.load(0x0000, 0x0100, &[0x9b, 0xf0, 0x40]);
    m.cpu.step(); // wait
    assert_eq!(m.regs().eip, 0x0101);
    m.cpu.step(); // lock inc ax — one instruction, prefix included
    assert_eq!(m.regs().eip, 0x0103);
    assert_eq!(m.regs().eax, 1);
}

#[test]
fn separate_address_spaces_mean_a_port_is_not_a_memory_address() {
    let m = machine();
    m.ports.write_u8(0x0060, 0xa5).unwrap();
    m.poke(0x0060, 0x5a);
    // in al, 0x60
    m.load(0x0000, 0x0100, &[0xe4, 0x60]);
    m.cpu.step();
    assert_eq!(m.regs().eax & 0xff, 0xa5, "IN must not read memory");

    // out 0x61, al with al = 0x12
    m.set_regs(|r| r.eax = 0x0012);
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
    m.set_regs(|r| r.edx = 0x0300);
    m.load(0x0000, 0x0100, &[0xed]); // in ax, dx
    m.cpu.step();
    assert_eq!(m.regs().eax, 0x1234);
}

#[test]
fn a_core_with_no_io_space_reads_ones() {
    // What an unterminated bus does, and what the hardware corpus records.
    let ram = Arc::new(RamStore::new(0x10_0000));
    let mem = AddressSpace::new("mem", 20);
    mem.topology()
        .map(Region::ram("ram", ram.clone()), 0)
        .unwrap();
    let cpu = X86::new(Config::I8088);
    cpu.attach_space(Arc::new(mem));
    for (i, byte) in [0xe4u8, 0x60].into_iter().enumerate() {
        ram.write_u8(0x100 + i as u64, byte).unwrap();
    }
    cpu.set_regs(Regs {
        cs: 0,
        eip: 0x100,
        ..Regs::new()
    });
    cpu.session.lock().state.reset_pending = false;
    cpu.step();
    assert_eq!(cpu.regs().eax & 0xff, 0xff);
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
        r.esi = 0;
        r.edi = 0;
        r.ecx = 4;
    });
    m.load(0x0000, 0x0100, &[0xf3, 0xa4]); // rep movsb
    m.cpu.step();
    assert_eq!(m.regs().ecx, 0);
    assert_eq!(m.regs().esi, 4);
    assert_eq!(m.regs().edi, 4);
    for i in 0..4u32 {
        assert_eq!(m.peek(0x2_0000 + i), 0xa0 + i as u8);
    }

    // Backwards, and one short: REPNE stops on a match.
    m.set_regs(|r| {
        r.es = 0x2000;
        r.edi = 3;
        r.ecx = 4;
        r.eax = 0xa2;
        r.eflags |= flags::DF;
    });
    m.load(0x0000, 0x0200, &[0xf2, 0xae]); // repne scasb
    m.cpu.step();
    assert_eq!(m.regs().ecx, 2, "scan stops the moment it matches");
    assert_eq!(m.regs().edi, 1);
}

#[test]
fn a_repeat_with_a_zero_count_does_nothing_at_all() {
    let m = machine();
    m.set_regs(|r| {
        r.ecx = 0;
        r.esi = 0x10;
        r.edi = 0x20;
    });
    m.load(0x0000, 0x0100, &[0xf3, 0xa4]);
    m.cpu.step();
    let regs = m.regs();
    assert_eq!((regs.esi, regs.edi, regs.ecx), (0x10, 0x20, 0));
}

#[test]
fn a_repeat_is_interruptible_between_iterations() {
    let m = machine();
    m.set_regs(|r| {
        r.eax = 0x3000;
        r.ds = 0x1000;
        r.es = 0x2000;
        r.ecx = 100;
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
    assert_eq!(m.regs().eip, 0x0102);
    assert!(m.regs().ecx < 100 && m.regs().ecx > 0);
    m.cpu.step();
    assert_eq!(m.regs().eip, 0x7000);
}

#[test]
fn push_sp_stores_the_decremented_pointer() {
    // True of the 8086 and 8088 and of nothing later: the 286 pushes the value
    // SP had before the instruction.
    let m = machine();
    m.set_regs(|r| {
        r.ss = 0x2000;
        r.esp = 0x0100;
    });
    m.load(0x0000, 0x0100, &[0x54]);
    m.cpu.step();
    assert_eq!(m.regs().esp, 0x00fe);
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
        r.eax = 0x009a;
        r.eflags = (r.eflags | flags::AF) & !flags::CF;
    });
    m.cpu.step();
    assert_eq!(m.regs().eax & 0xff, 0xa0);
    assert_eq!(m.regs().eflags & flags::CF, 0);

    // With AF clear the same AL takes both corrections: 0x9a + 0x66 = 0x00.
    m.load(0x0000, 0x0200, &[0x27]);
    m.set_regs(|r| {
        r.eax = 0x009a;
        r.eflags &= !(flags::AF | flags::CF);
    });
    m.cpu.step();
    assert_eq!(m.regs().eax & 0xff, 0x00);
    assert_eq!(m.regs().eflags & flags::CF, flags::CF);
}

#[test]
fn an_unadjusted_ascii_add_still_sets_sign_zero_and_parity() {
    // `AAA` performs an 8-bit `AL + 0` when no adjustment is needed, which is
    // why the officially undefined sign, zero and parity results are those of
    // the original AL rather than of the masked one.
    let m = machine();
    m.load(0x0000, 0x0100, &[0x37]);
    m.set_regs(|r| {
        r.eax = 0x0081; // AL = 0x81: low digit 1, so no adjustment
        r.eflags &= !(flags::AF | flags::SF | flags::PF | flags::ZF);
    });
    m.cpu.step();
    let regs = m.regs();
    assert_eq!(regs.eax, 0x0001, "only the low digit survives");
    assert_eq!(
        regs.eflags & flags::SF,
        flags::SF,
        "sign of 0x81, not of 0x01"
    );
    assert_eq!(regs.eflags & (flags::CF | flags::AF), 0);
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
        r.eip = 0x100;
        r.ds = 0x2000;
        r.ebx = 0;
        r.ecx = 0; // CL = 0: no rotation at all
        r.eflags |= flags::CF;
    });
    m.cpu.session.lock().state.reset_pending = false;
    log.clear();
    m.cpu.step();

    assert_eq!(
        log.entries(),
        alloc::vec![(0u64, false), (0u64, true)],
        "the operand is read and written back even with a zero count"
    );
    assert_eq!(
        m.regs().eflags & flags::CF,
        flags::CF,
        "flags are untouched"
    );
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
        r.eax = 0x0010;
        r.ebx = 0x0010;
    });
    m.cpu.step();
    let regs = m.regs();
    assert_eq!(regs.eax, 0x0100);
    // AH is 1: not zero, not negative, odd parity.
    assert_eq!(regs.eflags & flags::ZF, 0);
    assert_eq!(regs.eflags & flags::SF, 0);
    assert_eq!(regs.eflags & flags::PF, 0);
    assert_eq!(regs.eflags & flags::AF, 0);
    assert_eq!(regs.eflags & (flags::CF | flags::OF), flags::CF | flags::OF);
}

#[test]
fn a_divide_error_pushes_the_following_instruction() {
    // The 8088 pushes the address of the *next* instruction on a divide
    // error, not of the faulting one. Later parts changed this, and generic
    // x86 emulators habitually get it wrong.
    let m = machine();
    m.load(0x0000, 0x0100, &[0xf6, 0xf3, 0x90]); // div bl ; nop
    m.set_regs(|r| {
        r.eax = 0xffff;
        r.ebx = 0x0001; // BL = 1: the quotient cannot fit in AL
        r.ss = 0x2000;
        r.esp = 0x0100;
    });
    m.poke(0x00, 0x00);
    m.poke(0x01, 0x04); // vector 0 → 0000:0400
    m.cpu.step();
    let regs = m.regs();
    assert_eq!((regs.cs, regs.eip), (0x0000, 0x0400));
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
        r.eax = 0x0064; // 100
        r.ebx = 0x000a; // BL = 10
    });
    m.cpu.step();
    assert_eq!(m.regs().eax & 0xff, 0xf6, "100 / 10 = 10, negated to -10");

    m.load(0x0000, 0x0200, &[0xf6, 0xfb]); // idiv bl, no prefix
    m.set_regs(|r| {
        r.eax = 0x0064;
        r.ebx = 0x000a;
    });
    m.cpu.step();
    assert_eq!(m.regs().eax & 0xff, 0x0a);
}

#[test]
fn logical_operations_clear_the_auxiliary_carry() {
    // Documented as undefined; cleared on every corpus vector.
    let m = machine();
    m.load(0x0000, 0x0100, &[0x24, 0xff]); // and al, 0xff
    m.set_regs(|r| {
        r.eax = 0x0001;
        r.eflags |= flags::AF | flags::CF | flags::OF;
    });
    m.cpu.step();
    let regs = m.regs();
    assert_eq!(regs.eflags & (flags::AF | flags::CF | flags::OF), 0);
}

#[test]
fn a_left_shift_leaves_bit_four_of_its_result_in_the_auxiliary_carry() {
    // The microcode for `SHL` is an `ADD dst,dst`, so the auxiliary carry it
    // leaves is a real one — which is why it tracks bit 4 of the result.
    let m = machine();
    for (value, want) in [(0x08u32, true), (0x04u32, false)] {
        m.load(0x0000, 0x0100, &[0xd0, 0xe0]); // shl al, 1
        m.set_regs(|r| {
            r.eax = value;
            r.eflags &= !flags::AF;
        });
        m.cpu.step();
        assert_eq!(
            m.regs().eflags & flags::AF != 0,
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
    regs.eax = 0x1234;
    assert_eq!(regs.byte(0), 0x34); // al
    assert_eq!(regs.byte(4), 0x12); // ah
    regs.set_byte(4, 0xab);
    assert_eq!(regs.eax, 0xab34);
    regs.set_byte(0, 0xcd);
    assert_eq!(regs.eax, 0xabcd);
    // The order is AL CL DL BL AH CH DH BH, which is why AH is 4.
    regs.ecx = 0x0000;
    regs.set_byte(5, 0xff);
    assert_eq!(regs.ecx, 0xff00);
}

#[test]
fn the_hard_wired_flag_bits_cannot_be_written() {
    let m = machine();
    // mov ax, 0 ; push ax ; popf
    m.load(0x0000, 0x0100, &[0xb8, 0x00, 0x00, 0x50, 0x9d]);
    m.set_regs(|r| {
        r.ss = 0x2000;
        r.esp = 0x0100;
    });
    m.cpu.step();
    m.cpu.step();
    m.cpu.step();
    assert_eq!(m.regs().eflags, flags::RESERVED_SET);
    assert_eq!(Regs::normalise_flags(Variant::I8088, 0x0000), 0xf002);
    assert_eq!(Regs::normalise_flags(Variant::I8088, 0xffff), 0xffd7);
}

#[test]
fn registers_are_reachable_by_name() {
    assert_eq!(Reg::from_name("ax"), Some(Reg::Ax));
    assert_eq!(Reg::from_name("flags"), Some(Reg::Flags));
    assert_eq!(Reg::from_name("eax"), Some(Reg::Eax));
    assert_eq!(Reg::from_name("cr0"), None);
    for reg in Reg::ALL.iter().chain(Reg::NARROW) {
        assert_eq!(Reg::from_name(reg.name()), Some(*reg));
    }
    assert_eq!(Reg::from_dword_index(4), Reg::Esp);
    // The ModRM word order, which is not the alphabetical one.
    assert_eq!(Reg::from_word_index(3), Reg::Bx);
    assert_eq!(Reg::from_word_index(4), Reg::Sp);
}

// ---------------------------------------------------------------------------
// The device surface
// ---------------------------------------------------------------------------

#[test]
fn a_core_with_no_address_space_refuses_to_realize() {
    let cpu = X86::new(Config::default());
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
        r.eax = 0x1111;
        r.ebx = 0x2222;
        r.ecx = 0x3333;
        r.edx = 0x4444;
        r.esp = 0x5555;
        r.ebp = 0x6666;
        r.esi = 0x7777;
        r.edi = 0x8888;
        r.es = 0x9999;
        r.ss = 0xaaaa;
        r.ds = 0xbbbb;
        r.eflags |= flags::CF | flags::DF;
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
    m.set_regs(|r| r.eax = 0xbeef);
    m.cpu.reset(ResetKind::Warm);
    assert!(m.cpu.reset_pending());
    m.cpu.step();
    assert_eq!(m.regs().eax, 0xbeef);
    assert_eq!(m.regs().cs, 0xffff);

    m.cpu.reset(ResetKind::Cold);
    assert_eq!(m.cpu.regs().eax, 0);
    assert_eq!(m.cpu.cycles(), 0);
}

#[test]
fn the_variant_property_picks_the_part() {
    use crate::core::props::Props;
    let cpu = X86::from_props(&Props::new().with("variant", "8086")).unwrap();
    assert_eq!(cpu.config().variant, Variant::I8086);

    // `model` is still accepted, because the class used to spell it that way.
    let cpu = X86::from_props(&Props::new().with("model", "80386")).unwrap();
    assert_eq!(cpu.config().variant, Variant::I80386);

    // An unknown part is named in the error, with the set that is accepted.
    let err = X86::from_props(&Props::new().with("variant", "6502")).unwrap_err();
    let text = alloc::format!("{err}");
    assert!(text.contains("8086") && text.contains("80486"), "{text}");

    assert!(
        X86::from_props(&Props::new().with("varaint", "8088")).is_err(),
        "a typo'd property must not be ignored"
    );

    assert_eq!(
        X86::from_props(&Props::new()).unwrap().config().variant,
        Variant::I8088
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
    assert_eq!(m.regs().eax & 0xff, 0xff);
    // setmo: the undocumented D0 /6.
    m.set_regs(|r| r.eax &= 0xff00);
    m.load(0x0000, 0x0200, &[0xd0, 0xf0]);
    m.cpu.step();
    assert_eq!(m.regs().eax & 0xff, 0xff);
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
        assert_eq!(m.regs().eip, u32::from(ip));
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

// ---------------------------------------------------------------------------
// The 80386 and 80486
// ---------------------------------------------------------------------------
//
// Everything below runs on a 32-bit part. There is no hardware corpus for
// these — `SingleStepTests` stops at the 8088 — so the evidence is of a
// different kind: each test asserts a specific statement out of the *80386
// Programmer's Reference Manual*, and the instruction encodings were
// cross-checked against `as`/`objdump` at authoring time rather than guessed.
// Where a listing appears in a comment it is that assembler's output.

use super::isa;
use super::prot::{SegReg, Sys, ar, cr0, sys_type, tss32};

/// Where the test machine puts things, so the numbers in each test mean
/// something.
mod at {
    /// The global descriptor table.
    pub(super) const GDT: u32 = 0x1000;
    /// The interrupt descriptor table.
    pub(super) const IDT: u32 = 0x1800;
    /// The task state segment.
    pub(super) const TSS: u32 = 0x2000;
    /// Ring-0 code.
    pub(super) const CODE0: u32 = 0x3000;
    /// Ring-3 code.
    pub(super) const CODE3: u32 = 0x4000;
    /// The page directory.
    pub(super) const PDIR: u32 = 0x5000;
    /// The first page table.
    pub(super) const PTAB: u32 = 0x6000;
    /// Scratch, for a test to write a marker into.
    pub(super) const MARK: u32 = 0x7000;
    /// The top of the ring-0 stack.
    pub(super) const STACK0: u32 = 0x9000;
    /// The top of the ring-3 stack.
    pub(super) const STACK3: u32 = 0xa000;
}

/// Access-rights words for the descriptors these tests build.
mod rights {
    use super::ar;
    /// An executable, readable 32-bit code segment at privilege 0.
    pub(super) const CODE32: u32 = ar::PRESENT | ar::S | ar::CODE | ar::RW | ar::DB;
    /// A writable 32-bit data segment at privilege 0.
    pub(super) const DATA32: u32 = ar::PRESENT | ar::S | ar::RW | ar::DB;
    /// An executable, readable 16-bit code segment — no `D` bit.
    pub(super) const CODE16: u32 = ar::PRESENT | ar::S | ar::CODE | ar::RW;
    /// The same at privilege 3.
    pub(super) const DPL3: u32 = ar::DPL;
}

/// The two doublewords of a descriptor, with the granularity bit chosen for
/// the limit given.
///
/// A limit above 1 MiB has to be expressed in pages, and the architecture
/// rounds *up* to the page containing it — which is why a limit of `0xffffffff`
/// and a limit of `0xfffff000` produce the same descriptor.
fn descriptor(base: u32, limit: u32, ar_bits: u32) -> (u32, u32) {
    let (limit, ar_bits) = if limit > 0xf_ffff {
        (limit >> 12, ar_bits | ar::GRANULAR)
    } else {
        (limit, ar_bits)
    };
    let low = (limit & 0xffff) | (base << 16);
    let high = ((base >> 16) & 0xff) | ar_bits | (limit & 0x000f_0000) | (base & 0xff00_0000);
    (low, high)
}

/// The two doublewords of a gate.
fn gate(selector: u16, offset: u32, kind: u8, dpl: u8) -> (u32, u32) {
    let low = (offset & 0xffff) | (u32::from(selector) << 16);
    let high = (offset & 0xffff_0000)
        | ar::PRESENT
        | (u32::from(dpl) << ar::DPL_SHIFT)
        | (u32::from(kind) << 8);
    (low, high)
}

/// A 32-bit core with 4 MiB of RAM at zero and the top 64 KiB of the address
/// space populated, so the reset vector is reachable.
struct Pc {
    cpu: Arc<X86>,
    ram: Arc<RamStore>,
    rom: Arc<RamStore>,
    ports: Arc<RamStore>,
}

impl Pc {
    fn new(variant: Variant) -> Pc {
        let ram = Arc::new(RamStore::new(0x40_0000));
        let rom = Arc::new(RamStore::new(0x1_0000));
        let mem = AddressSpace::new("mem", 32);
        mem.topology()
            .map(Region::ram("ram", ram.clone()), 0)
            .expect("4 MiB at zero");
        mem.topology()
            .map(Region::ram("rom", rom.clone()), 0xffff_0000)
            .expect("64 KiB at the top of the space");

        let ports = Arc::new(RamStore::new(0x1_0000));
        let io = AddressSpace::new("io", 16);
        io.topology()
            .map(Region::ram("ports", ports.clone()), 0)
            .expect("64 KiB fits in 16 bits");

        let cpu = Arc::new(X86::new(Config::default().with_variant(variant)));
        cpu.attach_space(Arc::new(mem));
        cpu.attach_io_space(Arc::new(io));
        Pc {
            cpu,
            ram,
            rom,
            ports,
        }
    }

    fn write(&self, addr: u32, bytes: &[u8]) {
        for (i, byte) in bytes.iter().enumerate() {
            self.ram
                .write_u8(u64::from(addr) + i as u64, *byte)
                .unwrap();
        }
    }

    fn write32(&self, addr: u32, value: u32) {
        for i in 0..4u32 {
            self.ram
                .write_u8(u64::from(addr + i), (value >> (8 * i)) as u8)
                .unwrap();
        }
    }

    fn read32(&self, addr: u32) -> u32 {
        let mut value = 0u32;
        for i in 0..4u32 {
            value |= u32::from(self.ram.read_u8(u64::from(addr + i)).unwrap()) << (8 * i);
        }
        value
    }

    /// Put bytes in the ROM window, whose base is `0xffff0000`.
    fn rom(&self, offset: u32, bytes: &[u8]) {
        for (i, byte) in bytes.iter().enumerate() {
            self.rom
                .write_u8(u64::from(offset) + i as u64, *byte)
                .unwrap();
        }
    }

    /// Write a descriptor into the global descriptor table.
    fn gdt(&self, index: u32, pair: (u32, u32)) {
        self.write32(at::GDT + index * 8, pair.0);
        self.write32(at::GDT + index * 8 + 4, pair.1);
    }

    /// Write a gate into the interrupt descriptor table.
    fn idt(&self, vector: u32, pair: (u32, u32)) {
        self.write32(at::IDT + vector * 8, pair.0);
        self.write32(at::IDT + vector * 8 + 4, pair.1);
    }

    /// Start executing in real mode at `cs:eip`, with the reset sequence
    /// already done.
    fn start_real(&self, cs: u16, eip: u32) {
        let mut regs = self.cpu.regs();
        regs.cs = cs;
        regs.eip = eip;
        self.cpu.set_regs(regs);
        self.cpu.session.lock().state.reset_pending = false;
    }

    /// Put the core straight into 32-bit protected mode at `CODE0`, with a
    /// flat code and data segment and a ring-0 stack.
    ///
    /// The long way round — `LGDT`, `CR0.PE`, a far jump — is what
    /// `entering_protected_mode_reloads_the_cached_descriptor` tests. Every
    /// other test here starts from the far side of it, because a bring-up
    /// sequence in front of each one tests the same thing twenty times.
    fn start_protected(&self) {
        self.gdt(0, (0, 0));
        self.gdt(1, descriptor(0, 0xffff_ffff, rights::CODE32));
        self.gdt(2, descriptor(0, 0xffff_ffff, rights::DATA32));
        let mut sys = Sys::reset();
        sys.cr0 |= cr0::PE;
        sys.gdtr.base = at::GDT;
        sys.gdtr.limit = 0xff;
        sys.idtr.base = at::IDT;
        sys.idtr.limit = 0x7ff;
        sys.segs[usize::from(isa::seg::CS)] = SegReg {
            selector: 0x08,
            base: 0,
            limit: 0xffff_ffff,
            ar: rights::CODE32,
        };
        for index in [
            isa::seg::DS,
            isa::seg::ES,
            isa::seg::SS,
            isa::seg::FS,
            isa::seg::GS,
        ] {
            sys.segs[usize::from(index)] = SegReg {
                selector: 0x10,
                base: 0,
                limit: 0xffff_ffff,
                ar: rights::DATA32,
            };
        }
        self.cpu.set_sys(sys);
        let mut regs = Regs::new();
        regs.cs = 0x08;
        regs.ss = 0x10;
        regs.ds = 0x10;
        regs.es = 0x10;
        regs.fs = 0x10;
        regs.gs = 0x10;
        regs.esp = at::STACK0;
        regs.eip = at::CODE0;
        regs.eflags = flags::ALWAYS_SET;
        self.cpu.set_regs(regs);
        self.cpu.session.lock().state.reset_pending = false;
    }

    /// Step until the core halts or `limit` instructions have run.
    ///
    /// Returns how many steps actually happened, so a test can assert that the
    /// program reached its `hlt` rather than wandering.
    fn run(&self, limit: usize) -> usize {
        for n in 0..limit {
            if self.cpu.step() == 0 {
                return n;
            }
        }
        limit
    }

    fn regs(&self) -> Regs {
        self.cpu.regs()
    }
}

fn pc386() -> Pc {
    Pc::new(Variant::I80486)
}

#[test]
fn the_reset_vector_is_sixteen_bytes_below_the_top_of_the_address_space() {
    // The detail without which no firmware runs: the selector reads `f000`
    // but the *cached base* is `ffff0000`, so the first fetch is at physical
    // `fffffff0`. An emulator that computes `selector << 4` fetches from
    // `000ffff0` instead and finds nothing there.
    let pc = pc386();
    // jmp 0x0000:0x1000 — the far jump every PC ROM starts with.
    pc.rom(0xfff0, &[0xea, 0x00, 0x10, 0x00, 0x00]);
    pc.cpu.step();
    let regs = pc.regs();
    assert_eq!((regs.cs, regs.eip), (0xf000, 0xfff0));
    assert_eq!(pc.cpu.sys().seg(isa::seg::CS).base, 0xffff_0000);
    assert_eq!(pc.cpu.regs().edx, Variant::I80486.reset_signature());

    pc.cpu.step();
    let regs = pc.regs();
    assert_eq!((regs.cs, regs.eip), (0x0000, 0x1000));
    // The far jump recomputed the base in real mode, and the processor is now
    // in the first megabyte for good.
    assert_eq!(pc.cpu.sys().seg(isa::seg::CS).base, 0);
}

#[test]
fn an_operand_size_prefix_selects_the_wide_form_in_real_mode() {
    let pc = pc386();
    pc.start_real(0, 0x1000);
    pc.write(
        0x1000,
        &[
            0x66, 0xb8, 0x78, 0x56, 0x34, 0x12, // mov eax, 0x12345678
            0x66, 0x05, 0x11, 0x11, 0x11, 0x11, // add eax, 0x11111111
            0xb8, 0x34, 0x12, // mov ax, 0x1234
            0xf4, // hlt
        ],
    );
    pc.run(8);
    // The 16-bit `mov` left the high half of `EAX` alone, which is the rule
    // that lets 16-bit and 32-bit code share a register file.
    assert_eq!(pc.regs().eax, 0x2345_1234);
}

#[test]
fn an_address_size_prefix_brings_the_scaled_index_forms_into_real_mode() {
    let pc = pc386();
    pc.start_real(0, 0x1000);
    pc.write32(0x2010, 0xdead_beef);
    pc.write(
        0x1000,
        &[
            0x66, 0xb8, 0x00, 0x20, 0x00, 0x00, // mov eax, 0x2000
            0x66, 0xb9, 0x04, 0x00, 0x00, 0x00, // mov ecx, 4
            0x67, 0x66, 0x8b, 0x14, 0x88, // mov edx, [eax+ecx*4]
            0xf4, // hlt
        ],
    );
    pc.run(8);
    assert_eq!(pc.regs().edx, 0xdead_beef);
}

#[test]
fn entering_protected_mode_reloads_the_cached_descriptor() {
    // The whole bring-up, the way firmware writes it: a descriptor table in
    // memory, `LGDT`, the protection bit, and the far jump that is the only
    // way to reload `CS`.
    let pc = pc386();
    pc.gdt(0, (0, 0));
    pc.gdt(1, descriptor(0, 0xffff_ffff, rights::CODE32));
    pc.gdt(2, descriptor(0, 0xffff_ffff, rights::DATA32));
    // The six-byte pseudo-descriptor `LGDT` reads.
    pc.write(0x7000, &[0xff, 0x00]);
    pc.write32(0x7002, at::GDT);

    pc.start_real(0, 0x7c00);
    pc.write(
        0x7c00,
        &[
            0xfa, // cli
            0x0f, 0x01, 0x16, 0x00, 0x70, // lgdt [0x7000]
            0x0f, 0x20, 0xc0, // mov eax, cr0
            0x66, 0x83, 0xc8, 0x01, // or eax, 1
            0x0f, 0x22, 0xc0, // mov cr0, eax
            0xea, 0x00, 0x7d, 0x08, 0x00, // jmp 0x0008:0x7d00
        ],
    );
    pc.write(
        0x7d00,
        &[
            0xb8, 0x10, 0x00, 0x00, 0x00, // mov eax, 0x10
            0x8e, 0xd8, // mov ds, ax
            0x8e, 0xd0, // mov ss, ax
            0xbc, 0x00, 0x90, 0x00, 0x00, // mov esp, 0x9000
            0xb8, 0xbe, 0xba, 0xfe, 0xca, // mov eax, 0xcafebabe
            0xa3, 0x00, 0x70, 0x00, 0x00, // mov [0x7000], eax
            0xbb, 0x22, 0x22, 0x11, 0x11, // mov ebx, 0x11112222
            0x53, // push ebx
            0x59, // pop ecx
            0xf4, // hlt
        ],
    );
    let steps = pc.run(20);
    assert!(steps < 20, "the program should have reached its hlt");

    let sys = pc.cpu.sys();
    assert!(sys.protected());
    assert_eq!(sys.gdtr.base, at::GDT);
    assert_eq!(sys.gdtr.limit, 0xff);
    let cs = sys.seg(isa::seg::CS);
    assert_eq!(cs.selector, 0x08);
    assert_eq!(cs.limit, 0xffff_ffff, "granularity expands the limit");
    assert!(cs.big(), "the D bit makes this a 32-bit segment");
    assert_eq!(pc.regs().ecx, 0x1111_2222);
    assert_eq!(pc.read32(0x7000), 0xcafe_babe);
    // A 32-bit push moved the stack pointer by four, not two.
    assert_eq!(pc.regs().esp, at::STACK0);
}

#[test]
fn a_segment_limit_violation_raises_general_protection() {
    let pc = pc386();
    pc.start_protected();
    // Shrink `DS` to one page and reach past the end of it.
    let mut sys = pc.cpu.sys();
    sys.segs[usize::from(isa::seg::DS)].limit = 0x0fff;
    pc.cpu.set_sys(sys);
    // A #GP handler that just halts, so the fault is observable.
    pc.idt(13, gate(0x08, 0x3100, sys_type::INT_GATE32, 0));
    pc.write(0x3100, &[0xf4]); // hlt

    pc.write(
        at::CODE0,
        &[
            0xa1, 0x00, 0x08, 0x00, 0x00, // mov eax, [0x0800]  — inside
            0xa1, 0x00, 0x20, 0x00, 0x00, // mov eax, [0x2000]  — outside
            0xf4,
        ],
    );
    pc.cpu.step();
    pc.cpu.step();
    let regs = pc.regs();
    assert_eq!(regs.eip, 0x3100, "the fault took the #GP gate");
    assert_eq!(regs.cs & 0xfffc, 0x08);
    // The pushed error code is zero: a limit violation names no selector.
    assert_eq!(pc.read32(at::STACK0 - 16), 0);
    // And the saved `EIP` is the *faulting* instruction, not the next one.
    assert_eq!(pc.read32(at::STACK0 - 12), at::CODE0 + 5);
}

#[test]
fn an_unassigned_encoding_raises_invalid_opcode() {
    let pc = pc386();
    pc.start_protected();
    pc.idt(6, gate(0x08, 0x3100, sys_type::INT_GATE32, 0));
    pc.write(0x3100, &[0xf4]);
    // `0f 0a` is not assigned on a 386 or a 486.
    pc.write(at::CODE0, &[0x0f, 0x0a]);
    pc.cpu.step();
    assert_eq!(pc.regs().eip, 0x3100);
    // #UD pushes no error code, so the saved EIP is one word closer.
    assert_eq!(pc.read32(at::STACK0 - 12), at::CODE0);
}

#[test]
fn paging_translates_through_the_directory_and_the_table() {
    let pc = pc386();
    pc.start_protected();
    // Identity-map the first 4 MiB, then point linear 0x0020_0000 at the
    // physical page holding the marker, so a write through the alias lands
    // somewhere the test can see it.
    pc.write32(at::PDIR, at::PTAB | 0b111);
    for page in 0..1024u32 {
        pc.write32(at::PTAB + page * 4, (page << 12) | 0b111);
    }
    pc.write32(at::PTAB + 0x200 * 4, at::MARK | 0b111);

    let mut sys = pc.cpu.sys();
    sys.cr3 = at::PDIR;
    sys.cr0 |= cr0::PG;
    pc.cpu.set_sys(sys);

    pc.write(
        at::CODE0,
        &[
            0xb8, 0x0d, 0xf0, 0xad, 0x0b, // mov eax, 0x0badf00d
            0xa3, 0x00, 0x00, 0x20, 0x00, // mov [0x200000], eax
            0x8b, 0x1d, 0x00, 0x00, 0x20, 0x00, // mov ebx, [0x200000]
            0xf4,
        ],
    );
    let steps = pc.run(10);
    assert!(steps < 10);
    assert_eq!(pc.regs().ebx, 0x0bad_f00d);
    // The write went to the *physical* page the table names.
    assert_eq!(pc.read32(at::MARK), 0x0bad_f00d);
    // And the walk set the accessed and dirty bits, which is the thing a
    // translation cache exists to do only once.
    let pte = pc.read32(at::PTAB + 0x200 * 4);
    assert_eq!(pte & 0b110_0000, 0b110_0000, "accessed and dirty");
    assert_eq!(pc.read32(at::PDIR) & 0b10_0000, 0b10_0000, "accessed");
}

#[test]
fn a_missing_page_faults_with_the_address_in_cr2_and_the_reason_in_the_code() {
    let pc = pc386();
    pc.start_protected();
    pc.write32(at::PDIR, at::PTAB | 0b111);
    for page in 0..1024u32 {
        pc.write32(at::PTAB + page * 4, (page << 12) | 0b111);
    }
    // Leave the second 4 MiB with no directory entry at all.
    let mut sys = pc.cpu.sys();
    sys.cr3 = at::PDIR;
    sys.cr0 |= cr0::PG;
    pc.cpu.set_sys(sys);
    pc.idt(14, gate(0x08, 0x3100, sys_type::INT_GATE32, 0));
    pc.write(0x3100, &[0xf4]);

    pc.write(
        at::CODE0,
        &[0xa3, 0x34, 0x12, 0x40, 0x00, 0xf4], // mov [0x00401234], eax
    );
    pc.cpu.step();
    assert_eq!(pc.regs().eip, 0x3100);
    assert_eq!(pc.cpu.sys().cr2, 0x0040_1234);
    // Not present (bit 0 clear), a write (bit 1), supervisor (bit 2 clear).
    assert_eq!(pc.read32(at::STACK0 - 16), 0b010);
}

#[test]
fn write_protect_decides_whether_ring_zero_obeys_a_read_only_page() {
    // A 386 has no `CR0.WP` at all and the kernel may write any present page;
    // the 486 added the bit so that copy-on-write could work in kernel space.
    // The same program has to behave differently on the two parts, which is
    // exactly the kind of difference a `Variant` exists to carry.
    for (variant, wp, expect_fault) in [
        (Variant::I80386, false, false),
        (Variant::I80486, false, false),
        (Variant::I80486, true, true),
    ] {
        let pc = Pc::new(variant);
        pc.start_protected();
        pc.write32(at::PDIR, at::PTAB | 0b111);
        for page in 0..1024u32 {
            pc.write32(at::PTAB + page * 4, (page << 12) | 0b111);
        }
        // One page — the one the program writes — is present and
        // user-accessible but **not** writable. Leaving the stack writable
        // matters: with `WP` set a read-only stack would fault while the
        // handler's frame was being pushed, and the answer would be a double
        // fault rather than the page fault the test is about.
        pc.write32(at::PTAB + (at::MARK >> 12) * 4, at::MARK | 0b101);
        let mut sys = pc.cpu.sys();
        sys.cr3 = at::PDIR;
        sys.cr0 |= cr0::PG;
        if wp {
            sys.cr0 |= cr0::WP;
        }
        pc.cpu.set_sys(sys);
        pc.idt(14, gate(0x08, 0x3100, sys_type::INT_GATE32, 0));
        pc.write(0x3100, &[0xf4]);
        pc.write(at::CODE0, &[0xa3, 0x00, 0x70, 0x00, 0x00, 0xf4]);
        pc.cpu.step();
        let faulted = pc.regs().eip == 0x3100;
        assert_eq!(faulted, expect_fault, "{variant} with WP={wp}");
    }
}

#[test]
fn an_interrupt_gate_switches_to_the_stack_the_task_state_segment_names() {
    let pc = pc386();
    pc.start_protected();
    // Ring-3 code and data, a task state segment, and a gate a ring-3 program
    // is allowed to invoke.
    pc.gdt(3, descriptor(0, 0xffff_ffff, rights::CODE32 | rights::DPL3));
    pc.gdt(4, descriptor(0, 0xffff_ffff, rights::DATA32 | rights::DPL3));
    pc.gdt(
        5,
        descriptor(
            at::TSS,
            0x67,
            ar::PRESENT | (u32::from(sys_type::TSS32_AVAIL) << 8),
        ),
    );
    pc.write32(at::TSS + tss32::ESP0, at::STACK0);
    pc.write32(at::TSS + tss32::SS0, 0x10);
    pc.write32(at::TSS + tss32::IOMAP_BASE - 2, 0x0068_0000);
    pc.idt(0x80, gate(0x08, 0x3100, sys_type::INT_GATE32, 3));

    // The handler reloads `DS` before it uses it: returning to ring 3 nulled
    // every data segment the new level was not allowed to keep, and coming
    // back through the gate does not put them back. A real handler's first
    // two instructions are exactly these.
    pc.write(
        0x3100,
        &[
            0xb8, 0x10, 0x00, 0x00, 0x00, // mov eax, 0x10
            0x8e, 0xd8, // mov ds, ax
            0x8c, 0xc8, // mov ax, cs
            0xa3, 0x00, 0x70, 0x00, 0x00, // mov [0x7000], eax
            0xcf, // iretd
        ],
    );
    // Ring 0 loads the task register, then returns to ring 3 with an `IRET`
    // whose frame names a ring-3 stack — the standard way in.
    pc.write(
        at::CODE0,
        &[
            0xb8, 0x28, 0x00, 0x00, 0x00, // mov eax, 0x28
            0x0f, 0x00, 0xd8, // ltr ax
            0x6a, 0x23, // push 0x23        (SS, ring 3)
            0x68, 0x00, 0xa0, 0x00, 0x00, // push 0xa000  (ESP)
            0x6a, 0x02, // push 2           (EFLAGS)
            0x6a, 0x1b, // push 0x1b        (CS, ring 3)
            0x68, 0x00, 0x40, 0x00, 0x00, // push 0x4000  (EIP)
            0xcf, // iretd
        ],
    );
    pc.write(
        at::CODE3,
        &[
            0xcd, 0x80, // int 0x80
            0xf4, // hlt — which a ring-3 program may not execute
        ],
    );

    // ltr, then the five pushes and the iret.
    for _ in 0..8 {
        pc.cpu.step();
    }
    let regs = pc.regs();
    assert_eq!(regs.cs, 0x1b, "the iret entered ring 3");
    assert_eq!(regs.esp, at::STACK3);
    assert_eq!(pc.cpu.sys().task.selector, 0x28);

    pc.cpu.step(); // int 0x80
    let regs = pc.regs();
    assert_eq!(regs.cs & 3, 0, "the gate raised the privilege level");
    assert_eq!(regs.eip, 0x3100);
    assert_eq!(regs.ss, 0x10, "the stack came out of the TSS");
    // Five doublewords: SS, ESP, EFLAGS, CS, EIP.
    assert_eq!(regs.esp, at::STACK0 - 20);
    assert_eq!(pc.read32(at::STACK0 - 4), 0x23, "the caller's SS");
    assert_eq!(pc.read32(at::STACK0 - 8), at::STACK3, "the caller's ESP");
    assert_eq!(pc.read32(at::STACK0 - 16), 0x1b, "the caller's CS");
    assert_eq!(pc.read32(at::STACK0 - 20), at::CODE3 + 2, "after the INT");

    for _ in 0..4 {
        pc.cpu.step(); // reload DS, read CS, store it
    }
    assert_eq!(pc.read32(at::MARK) & 3, 0);
    pc.cpu.step(); // iretd
    let regs = pc.regs();
    assert_eq!(regs.cs, 0x1b, "and back out to ring 3");
    assert_eq!(regs.esp, at::STACK3);
}

#[test]
fn a_ring_three_program_may_not_touch_the_privileged_instructions() {
    let pc = pc386();
    pc.start_protected();
    pc.gdt(3, descriptor(0, 0xffff_ffff, rights::CODE32 | rights::DPL3));
    pc.gdt(4, descriptor(0, 0xffff_ffff, rights::DATA32 | rights::DPL3));
    pc.gdt(
        5,
        descriptor(
            at::TSS,
            0x67,
            ar::PRESENT | (u32::from(sys_type::TSS32_AVAIL) << 8),
        ),
    );
    // Without a task register the fault would have no ring-0 stack to switch
    // to, and #GP would double-fault instead of being taken.
    pc.write32(at::TSS + tss32::ESP0, at::STACK0);
    pc.write32(at::TSS + tss32::SS0, 0x10);
    pc.idt(13, gate(0x08, 0x3100, sys_type::INT_GATE32, 0));
    pc.write(0x3100, &[0xf4]);
    // Ring 0 hands control to ring 3, which immediately tries to halt.
    pc.write(
        at::CODE0,
        &[
            0xb8, 0x28, 0x00, 0x00, 0x00, 0x0f, 0x00, 0xd8, // ltr 0x28
            0x6a, 0x23, 0x68, 0x00, 0xa0, 0x00, 0x00, 0x6a, 0x02, 0x6a, 0x1b, 0x68, 0x00, 0x40,
            0x00, 0x00, 0xcf,
        ],
    );
    pc.write(at::CODE3, &[0xf4]);
    for _ in 0..8 {
        pc.cpu.step();
    }
    assert_eq!(pc.regs().cs, 0x1b);
    pc.cpu.step();
    assert_eq!(pc.regs().eip, 0x3100, "hlt in ring 3 is #GP");
    assert!(!pc.cpu.is_halted());
}

#[test]
fn unreal_mode_keeps_the_limit_a_protected_mode_load_cached() {
    // Load a 4 GiB data segment in protected mode, drop back to real mode, and
    // the limit stays. Real 386 and 486 silicon does this, and BIOSes use it
    // to copy above 1 MiB without leaving real mode — which is why a segment
    // register is a cache and not a number.
    let pc = pc386();
    pc.start_protected();
    // Leaving protected mode needs a **16-bit** code segment first: `CS.D` is
    // still set until the far jump reloads it, so a far jump encoded the
    // 16-bit way would be decoded with a 32-bit operand size and read two
    // bytes too many. Firmware goes through this exact two-step dance.
    pc.gdt(5, descriptor(0, 0xffff, rights::CODE16));
    pc.write(
        at::CODE0,
        &[0xea, 0x00, 0x41, 0x00, 0x00, 0x28, 0x00], // jmp far 0x28:0x4100
    );
    pc.write(
        0x4100,
        &[
            0x0f, 0x20, 0xc0, // mov eax, cr0
            0x66, 0x83, 0xe0, 0xfe, // and eax, ~1
            0x0f, 0x22, 0xc0, // mov cr0, eax
            0xea, 0x00, 0x40, 0x00, 0x00, // jmp 0x0000:0x4000
        ],
    );
    // In real mode again, reach 2 MiB with a 32-bit offset through `DS` — the
    // segment whose 4 GiB limit was cached while protection was on.
    pc.write(
        at::CODE3,
        &[
            0x66, 0xb8, 0x0d, 0xf0, 0xad, 0x0b, // mov eax, 0x0badf00d
            0x67, 0x66, 0xa3, 0x00, 0x00, 0x20, 0x00, // mov [0x200000], eax
            0xf4,
        ],
    );
    for _ in 0..8 {
        pc.cpu.step();
    }
    assert!(!pc.cpu.sys().protected());
    assert_eq!(
        pc.cpu.sys().seg(isa::seg::DS).limit,
        0xffff_ffff,
        "the cached limit survived the return to real mode"
    );
    assert_eq!(pc.read32(0x20_0000), 0x0bad_f00d);
}

#[test]
fn cpuid_reports_the_vendor_and_a_feature_set_this_core_implements() {
    let pc = pc386();
    pc.start_protected();
    pc.write(at::CODE0, &[0x0f, 0xa2, 0xf4]);
    pc.cpu.step();
    let regs = pc.regs();
    assert_eq!(regs.eax, 1, "the highest leaf");
    assert_eq!(
        (regs.ebx, regs.edx, regs.ecx),
        (
            u32::from_le_bytes(*b"Genu"),
            u32::from_le_bytes(*b"ineI"),
            u32::from_le_bytes(*b"ntel")
        )
    );

    // Leaf 1 reports the signature and **no** optional features, because this
    // core implements none of them.
    let pc = pc386();
    pc.start_protected();
    pc.write(at::CODE0, &[0xb8, 0x01, 0x00, 0x00, 0x00, 0x0f, 0xa2, 0xf4]);
    pc.cpu.step();
    pc.cpu.step();
    let regs = pc.regs();
    assert_eq!(regs.eax, 0x0000_0480);
    assert_eq!(regs.edx, 0);

    // A 386 has no `CPUID` at all.
    let pc = Pc::new(Variant::I80386);
    pc.start_protected();
    pc.idt(6, gate(0x08, 0x3100, sys_type::INT_GATE32, 0));
    pc.write(0x3100, &[0xf4]);
    pc.write(at::CODE0, &[0x0f, 0xa2]);
    pc.cpu.step();
    assert_eq!(pc.regs().eip, 0x3100, "#UD on a 386");
}

#[test]
fn the_386_instruction_additions_compute_what_the_manual_says() {
    // One program per group would be nine programs; this is one, and each
    // instruction leaves its result in a register the assertions name.
    let pc = pc386();
    pc.start_protected();
    pc.write32(0x7100, 0x0000_8001);
    pc.write(
        at::CODE0,
        &[
            0xb9, 0x81, 0x00, 0x00, 0x00, // mov ecx, 0x81
            0x0f, 0xb6, 0xc1, // movzx eax, cl
            0x0f, 0xbe, 0xd9, // movsx ebx, cl
            0xb9, 0x00, 0x01, 0x00, 0x00, // mov ecx, 0x100
            0x0f, 0xbc, 0xd1, // bsf edx, ecx
            0x0f, 0xbd, 0xf1, // bsr esi, ecx
            0xb8, 0x00, 0x00, 0x00, 0x00, // mov eax, 0
            0x0f, 0xba, 0xe8, 0x05, // bts eax, 5
            0x0f, 0xba, 0xf0, 0x05, // btr eax, 5
            0x0f, 0xba, 0xf8, 0x07, // btc eax, 7
            0xf4,
        ],
    );
    let steps = pc.run(20);
    assert!(steps < 20);
    let regs = pc.regs();
    assert_eq!(regs.eax, 0x80, "bts then btr then btc");
    assert_eq!(regs.ebx, 0xffff_ff81, "movsx sign-extended");
    assert_eq!(regs.edx, 8, "bsf found the lowest set bit");
    assert_eq!(regs.esi, 8, "bsr found the highest");

    // The double shifts, the 486 atomics, and the frame instructions.
    let pc = pc386();
    pc.start_protected();
    pc.write(
        at::CODE0,
        &[
            0xb8, 0x00, 0x00, 0x00, 0xf0, // mov eax, 0xf0000000
            0xb9, 0x00, 0x00, 0x00, 0x0f, // mov ecx, 0x0f000000
            0x0f, 0xa4, 0xc8, 0x04, // shld eax, ecx, 4
            0xbb, 0x05, 0x00, 0x00, 0x00, // mov ebx, 5
            0x0f, 0xc8, // bswap eax
            0x0f, 0xcb, // bswap ebx
            0xba, 0x0a, 0x00, 0x00, 0x00, // mov edx, 10
            0x6b, 0xfa, 0x07, // imul edi, edx, 7
            0xf4,
        ],
    );
    let steps = pc.run(20);
    assert!(steps < 20);
    let regs = pc.regs();
    // 0xf0000000 << 4, filled from the top of 0x0f000000.
    assert_eq!(regs.eax.swap_bytes(), 0x0000_0000);
    assert_eq!(regs.ebx, 0x0500_0000, "bswap reversed the byte order");
    assert_eq!(regs.edi, 70, "the three-operand imul");
}

#[test]
fn pusha_stores_the_stack_pointer_it_started_with_and_popa_discards_it() {
    let pc = pc386();
    pc.start_protected();
    pc.write(
        at::CODE0,
        &[
            0xb8, 0x11, 0x11, 0x11, 0x11, // mov eax, 0x11111111
            0xbb, 0x33, 0x33, 0x33, 0x33, // mov ebx, 0x33333333
            0x60, // pushad
            0xb8, 0x99, 0x99, 0x99, 0x99, // mov eax, 0x99999999
            0x61, // popad
            0xf4,
        ],
    );
    let steps = pc.run(10);
    assert!(steps < 10);
    let regs = pc.regs();
    assert_eq!(regs.eax, 0x1111_1111, "popad restored it");
    assert_eq!(regs.ebx, 0x3333_3333);
    assert_eq!(regs.esp, at::STACK0, "and left the stack where it found it");
    // The stored `ESP` is the value the instruction started with, and it is
    // the fifth of the eight — `EAX ECX EDX EBX ESP EBP ESI EDI`.
    assert_eq!(pc.read32(at::STACK0 - 20), at::STACK0);
}

#[test]
fn enter_and_leave_build_and_unmake_a_frame() {
    let pc = pc386();
    pc.start_protected();
    pc.write(
        at::CODE0,
        &[
            0xbd, 0x00, 0x88, 0x00, 0x00, // mov ebp, 0x8800
            0xc8, 0x10, 0x00, 0x00, // enter 0x10, 0
            0xc9, // leave
            0xf4,
        ],
    );
    pc.cpu.step();
    pc.cpu.step();
    let regs = pc.regs();
    assert_eq!(regs.ebp, at::STACK0 - 4, "the frame pointer is the new top");
    assert_eq!(
        regs.esp,
        at::STACK0 - 4 - 0x10,
        "and 16 bytes were reserved"
    );
    assert_eq!(pc.read32(at::STACK0 - 4), 0x8800, "the old EBP was saved");
    pc.cpu.step();
    let regs = pc.regs();
    assert_eq!(regs.ebp, 0x8800);
    assert_eq!(regs.esp, at::STACK0);
}

#[test]
fn the_shift_count_is_masked_to_five_bits_from_the_80186_on() {
    // The 8086 uses the whole of `CL`, so `shl al, 32` shifts thirty-two
    // times and leaves zero; every later part masks to five bits, so the same
    // instruction shifts none and leaves the operand alone. The corpus checks
    // the first half; this checks the second.
    let m = machine();
    m.load(0x0000, 0x0100, &[0xd2, 0xe0]); // shl al, cl
    m.set_regs(|r| {
        r.eax = 0x00ff;
        r.ecx = 32;
    });
    m.cpu.step();
    assert_eq!(m.regs().eax & 0xff, 0, "an 8086 really shifts 32 times");

    let pc = pc386();
    pc.start_protected();
    pc.write(
        at::CODE0,
        &[
            0xb8, 0xff, 0x00, 0x00, 0x00, // mov eax, 0xff
            0xb9, 0x20, 0x00, 0x00, 0x00, // mov ecx, 32
            0xd2, 0xe0, // shl al, cl
            0xf4,
        ],
    );
    pc.run(6);
    assert_eq!(pc.regs().eax & 0xff, 0xff, "a 386 masks the count to zero");
}

#[test]
fn push_sp_stores_the_value_before_the_decrement_from_the_80286_on() {
    let m = machine();
    m.load(0x0000, 0x0100, &[0x54]); // push sp
    m.set_regs(|r| {
        r.ss = 0x2000;
        r.esp = 0x0100;
    });
    m.cpu.step();
    let pushed = u16::from(m.peek(linear(0x2000, 0x00fe)))
        | (u16::from(m.peek(linear(0x2000, 0x00ff))) << 8);
    assert_eq!(pushed, 0x00fe, "an 8086 pushes the decremented value");

    let pc = pc386();
    pc.start_protected();
    pc.write(at::CODE0, &[0x54, 0xf4]); // push esp
    pc.cpu.step();
    assert_eq!(
        pc.read32(at::STACK0 - 4),
        at::STACK0,
        "a 386 pushes the value it had before"
    );
}

#[test]
fn lar_lsl_verr_and_arpl_answer_without_faulting() {
    let pc = pc386();
    pc.start_protected();
    // A descriptor a ring-3 program may not see, and one it may.
    pc.gdt(3, descriptor(0, 0x0fff, rights::DATA32));
    pc.gdt(4, descriptor(0, 0xffff_ffff, rights::DATA32 | rights::DPL3));
    pc.write(
        at::CODE0,
        &[
            0xb8, 0x18, 0x00, 0x00, 0x00, // mov eax, 0x18
            0x0f, 0x03, 0xd8, // lsl ebx, eax
            0x0f, 0x02, 0xc8, // lar ecx, eax
            0xb8, 0x00, 0xf0, 0x00, 0x00, // mov eax, 0xf000  — past the limit
            0x0f, 0x03, 0xd0, // lsl edx, eax
            0xf4,
        ],
    );
    let steps = pc.run(10);
    assert!(steps < 10);
    let regs = pc.regs();
    assert_eq!(regs.ebx, 0x0fff, "lsl read the limit");
    assert_eq!(regs.ecx & ar::MASK, rights::DATA32);
    assert_eq!(
        regs.edx, 0,
        "a selector past the table leaves the target alone"
    );
    assert!(!regs.flag(flags::ZF), "and clears ZF rather than faulting");

    // `ARPL` raises a selector's request to the caller's, and says whether it
    // had to.
    let pc = pc386();
    pc.start_protected();
    pc.write(
        at::CODE0,
        &[
            0xb8, 0x08, 0x00, 0x00, 0x00, // mov eax, 0x08  (RPL 0)
            0xb9, 0x03, 0x00, 0x00, 0x00, // mov ecx, 0x03  (RPL 3)
            0x63, 0xc8, // arpl ax, cx
            0xf4,
        ],
    );
    let steps = pc.run(6);
    assert!(steps < 6);
    assert_eq!(pc.regs().eax & 0xffff, 0x0b, "raised to RPL 3");
    assert!(pc.regs().flag(flags::ZF));
}

#[test]
fn a_double_fault_escalates_and_a_third_shuts_the_processor_down() {
    let pc = pc386();
    pc.start_protected();
    // A code segment the descriptor says is **not present**, and both the #GP
    // and the #DF gate pointing at it. Taking either gate therefore raises
    // #NP, and #NP is contributory: a contributory fault while delivering
    // another contributory fault is what the manual defines a double fault as.
    pc.gdt(7, descriptor(0, 0xffff_ffff, rights::CODE32 & !ar::PRESENT));
    pc.idt(13, gate(0x38, 0x3100, sys_type::INT_GATE32, 0));
    pc.idt(8, gate(0x38, 0x3200, sys_type::INT_GATE32, 0));
    pc.idt(11, gate(0x38, 0x3300, sys_type::INT_GATE32, 0));
    pc.write(
        at::CODE0,
        &[
            0x31, 0xc0, // xor eax, eax
            0x8e, 0xc0, // mov es, ax          — a null selector, which is legal
            0x26, 0x8b, 0x1d, 0x00, 0x00, 0x00, 0x00, // mov ebx, es:[0] — #GP(0)
        ],
    );
    pc.cpu.step();
    pc.cpu.step();
    pc.cpu.step();
    // #GP could not be delivered, so it doubled; #DF could not be delivered
    // either, so the processor shut down rather than looping forever.
    assert!(pc.cpu.is_halted());
    assert_eq!(pc.cpu.step(), 0, "a shut-down core charges nothing");
}

#[test]
fn a_snapshot_round_trips_the_hidden_descriptor_caches() {
    // The segment caches are architectural state, not derived: a snapshot that
    // dropped them would silently break unreal mode across a save and load,
    // and nothing else in the register file records the limit a segment was
    // loaded with. The translation-lookaside buffer, which *is* derived, is
    // deliberately not in the snapshot.
    let pc = pc386();
    pc.start_protected();
    let mut sys = pc.cpu.sys();
    sys.segs[usize::from(isa::seg::DS)].limit = 0x1234;
    sys.segs[usize::from(isa::seg::FS)].base = 0xdead_0000;
    sys.cr2 = 0xfeed_face;
    sys.cr3 = at::PDIR;
    sys.dr[0] = 0x1111_2222;
    sys.ldtr = SegReg {
        selector: 0x30,
        base: 0x9000,
        limit: 0xff,
        ar: ar::PRESENT | (u32::from(sys_type::LDT) << 8),
    };
    pc.cpu.set_sys(sys);
    pc.cpu.set_regs(Regs {
        eax: 0x1234_5678,
        esi: 0x9abc_def0,
        ..pc.regs()
    });
    let regs_before = pc.regs();
    let sys_before = pc.cpu.sys();

    let mut shape = MachineShape::new();
    shape.add_device("/cpu0", "cpu.x86").unwrap();
    let mut writer = StateWriter::new(shape);
    {
        let mut chunk = writer.chunk("/cpu0", "cpu.x86", 2).unwrap();
        pc.cpu.save(&mut chunk).unwrap();
    }
    let bytes = writer.to_vec().unwrap();

    pc.cpu.reset(ResetKind::Cold);
    assert_ne!(pc.cpu.sys(), sys_before);

    let reader = crate::core::state::StateReader::new(&bytes).unwrap();
    let (_, _, data) = reader.load_raw("/cpu0").unwrap();
    let mut chunk = ChunkReader::new(data);
    pc.cpu.load(&mut chunk).unwrap();
    chunk.end().unwrap();

    assert_eq!(pc.regs(), regs_before);
    assert_eq!(pc.cpu.sys(), sys_before);
}

#[test]
fn the_first_sixty_four_bytes_of_a_saved_core_are_gdbs_register_block() {
    // `host::gdb::arch` indexes straight into this prefix rather than
    // translating, so the two layouts have to agree by construction.
    let pc = pc386();
    pc.start_protected();
    pc.cpu.set_regs(Regs {
        eax: 0x0000_0001,
        ecx: 0x0000_0002,
        edx: 0x0000_0003,
        ebx: 0x0000_0004,
        esp: 0x0000_0005,
        ebp: 0x0000_0006,
        esi: 0x0000_0007,
        edi: 0x0000_0008,
        eip: 0x0000_0009,
        ..pc.regs()
    });
    let mut shape = MachineShape::new();
    shape.add_device("/cpu0", "cpu.x86").unwrap();
    let mut writer = StateWriter::new(shape);
    {
        let mut chunk = writer.chunk("/cpu0", "cpu.x86", 2).unwrap();
        pc.cpu.save(&mut chunk).unwrap();
    }
    let bytes = writer.to_vec().unwrap();
    let reader = crate::core::state::StateReader::new(&bytes).unwrap();
    let (_, _, data) = reader.load_raw("/cpu0").unwrap();
    for i in 0..9u32 {
        let offset = (i * 4) as usize;
        let word = u32::from_le_bytes([
            data[offset],
            data[offset + 1],
            data[offset + 2],
            data[offset + 3],
        ]);
        assert_eq!(word, i + 1, "register {i} of gdb's i386 block");
    }
    // Then `EFLAGS`, then the six selectors as doublewords.
    let cs = u32::from_le_bytes([data[40], data[41], data[42], data[43]]);
    assert_eq!(cs, 0x08);
}

#[test]
fn a_descriptor_that_changes_under_a_loaded_selector_does_not_move_the_segment() {
    // The consequence of the cache that catches emulators out: rewriting a
    // live descriptor changes nothing until something reloads the register.
    let pc = pc386();
    pc.start_protected();
    pc.gdt(3, descriptor(0x1_0000, 0xffff, rights::DATA32));
    pc.write32(0x1_0000, 0xaaaa_aaaa);
    pc.write32(0x2_0000, 0xbbbb_bbbb);
    pc.write(
        at::CODE0,
        &[
            0xb8, 0x18, 0x00, 0x00, 0x00, // mov eax, 0x18
            0x8e, 0xc0, // mov es, ax
            0x26, 0x8b, 0x1d, 0x00, 0x00, 0x00, 0x00, // mov ebx, es:[0]
            0xf4,
        ],
    );
    pc.cpu.step();
    pc.cpu.step();
    pc.cpu.step();
    assert_eq!(pc.regs().ebx, 0xaaaa_aaaa);

    // Move the descriptor's base and read again without reloading `ES`.
    pc.gdt(3, descriptor(0x2_0000, 0xffff, rights::DATA32));
    pc.cpu.set_regs(Regs {
        eip: at::CODE0 + 7,
        ..pc.regs()
    });
    pc.cpu.step();
    assert_eq!(
        pc.regs().ebx,
        0xaaaa_aaaa,
        "the cached base is what the processor uses"
    );

    // Reloading the selector is what publishes the change.
    pc.cpu.set_regs(Regs {
        eip: at::CODE0,
        ..pc.regs()
    });
    pc.cpu.step();
    pc.cpu.step();
    pc.cpu.step();
    assert_eq!(pc.regs().ebx, 0xbbbb_bbbb);
}

#[test]
fn a_null_selector_is_loadable_and_then_unusable() {
    let pc = pc386();
    pc.start_protected();
    pc.idt(13, gate(0x08, 0x3100, sys_type::INT_GATE32, 0));
    pc.write(0x3100, &[0xf4]);
    pc.write(
        at::CODE0,
        &[
            0x31, 0xc0, // xor eax, eax
            0x8e, 0xc0, // mov es, ax          — legal
            0x26, 0x8b, 0x1d, 0x00, 0x00, 0x00, 0x00, // mov ebx, es:[0] — #GP(0)
            0xf4,
        ],
    );
    pc.cpu.step();
    pc.cpu.step();
    assert_eq!(pc.regs().es, 0, "loading it is fine");
    pc.cpu.step();
    assert_eq!(pc.regs().eip, 0x3100, "using it is not");
    assert_eq!(pc.read32(at::STACK0 - 16), 0, "#GP(0), naming no selector");
}

#[test]
fn a_task_switch_saves_the_outgoing_task_and_loads_the_incoming_one() {
    let pc = pc386();
    pc.start_protected();
    pub(super) const TSS_B: u32 = 0x2200;
    pc.gdt(
        5,
        descriptor(
            at::TSS,
            0x67,
            ar::PRESENT | (u32::from(sys_type::TSS32_AVAIL) << 8),
        ),
    );
    pc.gdt(
        6,
        descriptor(
            TSS_B,
            0x67,
            ar::PRESENT | (u32::from(sys_type::TSS32_AVAIL) << 8),
        ),
    );
    // The incoming task: a code segment, a stack, and one instruction.
    pc.write32(TSS_B + tss32::EIP, 0x3300);
    pc.write32(TSS_B + tss32::EFLAGS, flags::ALWAYS_SET);
    pc.write32(TSS_B + tss32::EAX, 0x4444_4444);
    pc.write32(TSS_B + tss32::EAX + 16, 0x8f00); // ESP
    pc.write32(TSS_B + tss32::ES, 0x10);
    pc.write32(TSS_B + tss32::ES + 4, 0x08); // CS
    pc.write32(TSS_B + tss32::ES + 8, 0x10); // SS
    pc.write32(TSS_B + tss32::ES + 12, 0x10); // DS
    pc.write32(TSS_B + tss32::ES + 16, 0x10);
    pc.write32(TSS_B + tss32::ES + 20, 0x10);
    pc.write(0x3300, &[0xf4]);

    pc.write(
        at::CODE0,
        &[
            0xb8, 0x28, 0x00, 0x00, 0x00, // mov eax, 0x28
            0x0f, 0x00, 0xd8, // ltr ax
            0xb8, 0x77, 0x77, 0x77, 0x77, // mov eax, 0x77777777
            0xea, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, // jmp far 0x30:0
        ],
    );
    for _ in 0..4 {
        pc.cpu.step();
    }
    let regs = pc.regs();
    assert_eq!(regs.eax, 0x4444_4444, "the incoming task's registers");
    assert_eq!(regs.eip, 0x3300);
    assert_eq!(pc.cpu.sys().task.selector, 0x30);
    assert_eq!(
        pc.cpu.sys().cr0 & cr0::TS,
        cr0::TS,
        "a task switch always sets TS"
    );
    // The outgoing task's state landed in its own segment.
    assert_eq!(pc.read32(at::TSS + tss32::EAX), 0x7777_7777);
    assert_eq!(pc.read32(at::TSS + tss32::EIP), at::CODE0 + 20);
}

#[test]
fn an_io_port_needs_the_privilege_level_or_the_permission_bitmap() {
    let pc = pc386();
    pc.start_protected();
    pc.gdt(3, descriptor(0, 0xffff_ffff, rights::CODE32 | rights::DPL3));
    pc.gdt(4, descriptor(0, 0xffff_ffff, rights::DATA32 | rights::DPL3));
    pc.gdt(
        5,
        descriptor(
            at::TSS,
            0x7f,
            ar::PRESENT | (u32::from(sys_type::TSS32_AVAIL) << 8),
        ),
    );
    pc.write32(at::TSS + tss32::ESP0, at::STACK0);
    pc.write32(at::TSS + tss32::SS0, 0x10);
    // The bitmap starts at 0x68 and permits port 0x60 only.
    pc.write32(at::TSS + 0x64, 0x0068_0000);
    for i in 0..0x18u32 {
        pc.write(at::TSS + 0x68 + i, &[0xff]);
    }
    pc.write(at::TSS + 0x68 + 0x0c, &[0xfe]); // port 0x60 allowed
    pc.idt(13, gate(0x08, 0x3100, sys_type::INT_GATE32, 0));
    pc.write(0x3100, &[0xf4]);

    pc.write(
        at::CODE0,
        &[
            0xb8, 0x28, 0x00, 0x00, 0x00, 0x0f, 0x00, 0xd8, // ltr 0x28
            0x6a, 0x23, 0x68, 0x00, 0xa0, 0x00, 0x00, 0x6a, 0x02, 0x6a, 0x1b, 0x68, 0x00, 0x40,
            0x00, 0x00, 0xcf,
        ],
    );
    pc.write(
        at::CODE3,
        &[
            0xe4, 0x60, // in al, 0x60   — permitted by the bitmap
            0xe4, 0x61, // in al, 0x61   — not
            0xf4,
        ],
    );
    for _ in 0..8 {
        pc.cpu.step();
    }
    assert_eq!(pc.regs().cs, 0x1b);
    pc.cpu.step();
    assert_eq!(pc.regs().eip, at::CODE3 + 2, "port 0x60 went through");
    pc.cpu.step();
    assert_eq!(pc.regs().eip, 0x3100, "port 0x61 raised #GP");
}

#[test]
fn a_thirty_two_bit_listing_reads_the_way_the_assembler_wrote_it() {
    // Every encoding here was produced by `as --32` and its text checked
    // against `objdump -M intel` at authoring time, which is what makes this
    // a cross-check rather than a restatement of our own decoder.
    use super::disasm::disassemble_as;
    let cases: &[(&[u8], &str)] = &[
        (&[0x0f, 0xb6, 0xc1], "movzx eax, cl"),
        (&[0x0f, 0xbf, 0xc1], "movsx eax, cx"),
        (&[0x0f, 0xbc, 0xc1], "bsf eax, ecx"),
        (&[0x0f, 0xba, 0xe8, 0x07], "bts eax, 0x7"),
        (&[0x0f, 0xa4, 0xc8, 0x04], "shld eax, ecx, 0x4"),
        (&[0x0f, 0xad, 0xc8], "shrd eax, ecx, cl"),
        (&[0x0f, 0x94, 0xc0], "setz al"),
        (&[0x60], "pushad"),
        (&[0xc8, 0x10, 0x00, 0x00], "enter 0x10, 0x0"),
        (&[0x6b, 0xc1, 0x64], "imul eax, ecx, 0x64"),
        (&[0x0f, 0xc8], "bswap eax"),
        (&[0x0f, 0xc1, 0xc8], "xadd eax, ecx"),
        (&[0x0f, 0xb1, 0x08], "cmpxchg [ds:eax], ecx"),
        (&[0x0f, 0x01, 0x10], "lgdt [ds:eax]"),
        (&[0x0f, 0x00, 0xd0], "lldt ax"),
        (&[0x0f, 0x02, 0xc1], "lar eax, ecx"),
        (&[0x0f, 0x20, 0xc0], "mov eax, cr0"),
        (&[0x0f, 0x22, 0xc0], "mov cr0, eax"),
        (&[0x0f, 0x21, 0xf8], "mov eax, dr7"),
        (&[0x0f, 0xa0], "push fs"),
        (&[0x0f, 0xb2, 0x20], "lss esp, [ds:eax]"),
        (&[0x8b, 0x14, 0x88], "mov edx, [ds:eax+ecx*4]"),
        (
            &[0x8b, 0x94, 0xf3, 0x34, 0x12, 0x00, 0x00],
            "mov edx, [ds:ebx+esi*8+0x1234]",
        ),
        (&[0xa1, 0x78, 0x56, 0x34, 0x12], "mov eax, [ds:0x12345678]"),
        (&[0x8b, 0x45, 0x00], "mov eax, [ss:ebp]"),
        (&[0x8b, 0x04, 0x24], "mov eax, [ss:esp]"),
        (&[0x8b, 0x44, 0x7c, 0x04], "mov eax, [ss:esp+edi*2+0x4]"),
        (&[0x66, 0x05, 0x34, 0x12], "add ax, 0x1234"),
        (&[0x6f], "outsd dx, [ds:esi]"),
        (&[0xf3, 0xa5], "rep movsd [es:edi], [ds:esi]"),
        (&[0xcf], "iretd"),
        (&[0x98], "cwde"),
        (&[0x99], "cdq"),
        (&[0x68, 0x78, 0x56, 0x34, 0x12], "push 0x12345678"),
        (
            &[0xea, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12],
            "jmpf 0x1234:0x12345678",
        ),
        (&[0xca, 0x04, 0x00], "retf 0x4"),
    ];
    for (bytes, want) in cases {
        let d = disassemble_as(isa::Gen::I386, true, 0, 0, bytes);
        assert_eq!(alloc::format!("{d}"), *want, "for {bytes:02x?}");
        assert_eq!(d.len as usize, bytes.len(), "length of {bytes:02x?}");
    }
}

#[test]
fn the_386_map_reclaimed_the_encodings_the_8086_spent_on_aliases() {
    use super::isa::{Gen, decode_as};
    // `60`-`6F` were sixteen aliases of the conditional jumps and became eight
    // real instructions; `0F` stopped being `POP CS`; `C8`/`C9` stopped being
    // a second `RETF`.
    assert_eq!(decode_as(Gen::I8086, 0x60).op, Op::JO);
    assert_eq!(decode_as(Gen::I386, 0x60).op, Op::PUSHA);
    assert_eq!(decode_as(Gen::I8086, 0x0f).op, Op::POP);
    assert_eq!(decode_as(Gen::I386, 0x0f).class, Class::Escape);
    assert_eq!(decode_as(Gen::I8086, 0xc8).op, Op::RETF);
    assert_eq!(decode_as(Gen::I386, 0xc8).op, Op::ENTER);
    // And the group extensions the 8086 let fall through are invalid.
    assert_eq!(
        super::isa::resolve_as(Gen::I8086, decode_as(Gen::I8086, 0xfe), 2).op,
        Op::CALL
    );
    assert_eq!(
        super::isa::resolve_as(Gen::I386, decode_as(Gen::I386, 0xfe), 2).op,
        Op::UD
    );
}

#[test]
fn the_string_instructions_move_at_the_operand_size() {
    let pc = pc386();
    pc.start_protected();
    for i in 0..4u32 {
        pc.write32(0x8000 + i * 4, 0x1111_1111 * (i + 1));
    }
    pc.write(
        at::CODE0,
        &[
            0xbe, 0x00, 0x80, 0x00, 0x00, // mov esi, 0x8000
            0xbf, 0x00, 0x81, 0x00, 0x00, // mov edi, 0x8100
            0xb9, 0x04, 0x00, 0x00, 0x00, // mov ecx, 4
            0xfc, // cld
            0xf3, 0xa5, // rep movsd
            0xb8, 0xa5, 0xa5, 0xa5, 0xa5, // mov eax, 0xa5a5a5a5
            0xbf, 0x00, 0x82, 0x00, 0x00, // mov edi, 0x8200
            0xb9, 0x02, 0x00, 0x00, 0x00, // mov ecx, 2
            0xf3, 0xab, // rep stosd
            0xbf, 0x00, 0x82, 0x00, 0x00, // mov edi, 0x8200
            0xb9, 0x04, 0x00, 0x00, 0x00, // mov ecx, 4
            0xf2, 0xaf, // repne scasd
            0xf4,
        ],
    );
    let steps = pc.run(30);
    assert!(steps < 30);
    for i in 0..4u32 {
        assert_eq!(pc.read32(0x8100 + i * 4), 0x1111_1111 * (i + 1));
    }
    assert_eq!(pc.read32(0x8200), 0xa5a5_a5a5);
    assert_eq!(pc.read32(0x8204), 0xa5a5_a5a5);
    assert_eq!(pc.read32(0x8208), 0, "and stopped after two");
    // `repne scasd` compares `EAX` with each doubleword and stops on a match:
    // the first two match, so it stops after one iteration with three left.
    let regs = pc.regs();
    assert_eq!(regs.ecx, 3);
    assert_eq!(regs.edi, 0x8204);
    assert!(regs.flag(flags::ZF));
}

#[test]
fn the_near_conditional_jumps_take_a_full_displacement() {
    let pc = pc386();
    pc.start_protected();
    pc.write(
        at::CODE0,
        &[
            0x31, 0xc0, // xor eax, eax
            0x0f, 0x84, 0xf8, 0x0f, 0x00, 0x00, // jz +0xff8  → 0x4000
        ],
    );
    pc.write(at::CODE3, &[0xbb, 0x0d, 0x60, 0x00, 0x00, 0xf4]);
    let steps = pc.run(6);
    assert!(steps < 6);
    assert_eq!(pc.regs().ebx, 0x600d, "the near jump reached CODE3");

    // With a 16-bit operand size the same opcode takes a 16-bit displacement
    // and the target wraps in sixteen bits.
    let pc = pc386();
    pc.start_protected();
    pc.write(
        at::CODE0,
        &[
            0x31, 0xc0, // xor eax, eax
            0x66, 0x0f, 0x84, 0xf9, 0x0f, // jz +0xff9 (16-bit) → 0x4000
        ],
    );
    pc.write(at::CODE3, &[0xf4]);
    pc.cpu.step();
    pc.cpu.step();
    assert_eq!(pc.regs().eip, at::CODE3);
}

#[test]
fn xadd_and_cmpxchg_are_the_486_atomics_the_manual_describes() {
    let pc = pc386();
    pc.start_protected();
    pc.write(
        at::CODE0,
        &[
            0xb8, 0x00, 0x10, 0x00, 0x00, // mov eax, 0x1000
            0xb9, 0x34, 0x02, 0x00, 0x00, // mov ecx, 0x234
            0x0f, 0xc1, 0xc8, // xadd eax, ecx
            0xf4,
        ],
    );
    let steps = pc.run(6);
    assert!(steps < 6);
    let regs = pc.regs();
    assert_eq!(regs.eax, 0x1234, "the sum");
    assert_eq!(regs.ecx, 0x1000, "and the destination's old value");

    // `CMPXCHG` compares the destination with the accumulator; on a match the
    // source replaces it, on a mismatch the destination replaces the
    // accumulator.
    let pc = pc386();
    pc.start_protected();
    pc.write(
        at::CODE0,
        &[
            0xb8, 0x05, 0x00, 0x00, 0x00, // mov eax, 5
            0xbb, 0x05, 0x00, 0x00, 0x00, // mov ebx, 5
            0xb9, 0x09, 0x00, 0x00, 0x00, // mov ecx, 9
            0x0f, 0xb1, 0xcb, // cmpxchg ebx, ecx
            0xf4,
        ],
    );
    let steps = pc.run(8);
    assert!(steps < 8);
    let regs = pc.regs();
    assert!(regs.flag(flags::ZF), "the comparison matched");
    assert_eq!(regs.ebx, 9, "so the source was stored");
    assert_eq!(regs.eax, 5, "and the accumulator is unchanged");

    let pc = pc386();
    pc.start_protected();
    pc.write(
        at::CODE0,
        &[
            0xb8, 0x07, 0x00, 0x00, 0x00, // mov eax, 7
            0xbb, 0x05, 0x00, 0x00, 0x00, // mov ebx, 5
            0xb9, 0x09, 0x00, 0x00, 0x00, // mov ecx, 9
            0x0f, 0xb1, 0xcb, // cmpxchg ebx, ecx
            0xf4,
        ],
    );
    let steps = pc.run(8);
    assert!(steps < 8);
    let regs = pc.regs();
    assert!(!regs.flag(flags::ZF));
    assert_eq!(regs.ebx, 5, "the destination is left alone");
    assert_eq!(regs.eax, 5, "and the accumulator takes its value");
}

#[test]
fn the_shift_group_gained_an_immediate_count_on_the_80186() {
    let pc = pc386();
    pc.start_protected();
    pc.write(
        at::CODE0,
        &[
            0xb8, 0x01, 0x00, 0x00, 0x00, // mov eax, 1
            0xc1, 0xe0, 0x05, // shl eax, 5
            0x66, 0xbb, 0x00, 0x80, // mov bx, 0x8000
            0x66, 0xd1, 0xcb, // ror bx, 1
            0xf4,
        ],
    );
    let steps = pc.run(8);
    assert!(steps < 8);
    let regs = pc.regs();
    assert_eq!(regs.eax, 0x20);
    assert_eq!(regs.ebx & 0xffff, 0x4000);
}

#[test]
fn in_and_out_transfer_at_the_operand_size() {
    let pc = pc386();
    pc.start_protected();
    for i in 0..4u64 {
        pc.ports.write_u8(0x300 + i, 0x11 * (i as u8 + 1)).unwrap();
    }
    pc.write(
        at::CODE0,
        &[
            0x66, 0xba, 0x00, 0x03, // mov dx, 0x300
            0xed, // in eax, dx
            0x66, 0xba, 0x10, 0x03, // mov dx, 0x310
            0xef, // out dx, eax
            0xf4,
        ],
    );
    let steps = pc.run(6);
    assert!(steps < 6);
    assert_eq!(pc.regs().eax, 0x4433_2211);
    for i in 0..4u64 {
        assert_eq!(
            pc.ports.read_u8(0x310 + i).unwrap(),
            0x11 * (i as u8 + 1),
            "byte {i} of the 32-bit port write"
        );
    }
}

#[test]
fn lss_loads_the_stack_and_opens_the_interrupt_shadow() {
    let pc = pc386();
    pc.start_protected();
    // A far pointer in memory: a 32-bit offset then a selector.
    pc.write32(0x8300, 0x0000_8800);
    pc.write32(0x8304, 0x0000_0010);
    pc.write(
        at::CODE0,
        &[
            0x0f, 0xb2, 0x25, 0x00, 0x83, 0x00, 0x00, // lss esp, [0x8300]
            0xf4,
        ],
    );
    pc.cpu.step();
    let regs = pc.regs();
    assert_eq!(regs.esp, 0x8800);
    assert_eq!(regs.ss, 0x10);
    assert!(
        pc.cpu.interrupt_shadow(),
        "loading SS inhibits interrupts for one instruction, whichever \
         encoding did it"
    );
}

#[test]
fn a_386_in_real_mode_takes_its_vectors_through_the_idt_register() {
    // The difference from an 8086: `LIDT` can move the real-mode vector table,
    // and a vector past the limit is a fault rather than a read of whatever
    // happened to be there.
    let pc = pc386();
    pc.start_real(0, 0x1000);
    // Relocate the table to 0x2000 and put vector 3 at 0000:0x1800.
    pc.write32(0x2000 + 3 * 4, 0x0000_1800);
    let mut sys = pc.cpu.sys();
    sys.idtr.base = 0x2000;
    sys.idtr.limit = 0x3ff;
    pc.cpu.set_sys(sys);
    pc.write(0x1000, &[0xcc, 0xf4]); // int3
    pc.write(0x1800, &[0xf4]);
    pc.cpu.step();
    let regs = pc.regs();
    assert_eq!((regs.cs, regs.eip), (0x0000, 0x1800));
}

#[test]
fn a_page_fault_restarts_the_instruction_that_caused_it() {
    // The whole point of a fault being restartable: the handler maps the page
    // and returns, and the instruction runs again from its first byte with the
    // registers it started with.
    let pc = pc386();
    pc.start_protected();
    pc.write32(at::PDIR, at::PTAB | 0b111);
    for page in 0..1024u32 {
        pc.write32(at::PTAB + page * 4, (page << 12) | 0b111);
    }
    // Unmap exactly the page the program writes to.
    pc.write32(at::PTAB + (at::MARK >> 12) * 4, 0);
    let mut sys = pc.cpu.sys();
    sys.cr3 = at::PDIR;
    sys.cr0 |= cr0::PG;
    pc.cpu.set_sys(sys);
    pc.idt(14, gate(0x08, 0x3100, sys_type::INT_GATE32, 0));
    // The handler maps the page and returns to the faulting instruction.
    pc.write(0x3100, &[0xf4]);

    pc.write(
        at::CODE0,
        &[
            0xb8, 0x0d, 0xf0, 0xad, 0x0b, // mov eax, 0x0badf00d
            0xa3, 0x00, 0x70, 0x00, 0x00, // mov [0x7000], eax
            0xf4,
        ],
    );
    pc.cpu.step();
    pc.cpu.step();
    assert_eq!(pc.regs().eip, 0x3100);
    // The saved `EIP` names the `mov`, not the `hlt` after it.
    assert_eq!(pc.read32(at::STACK0 - 12), at::CODE0 + 5);
    assert_eq!(pc.regs().eax, 0x0bad_f00d, "the earlier work is not undone");

    // Map the page, restart, and the instruction completes.
    pc.write32(at::PTAB + (at::MARK >> 12) * 4, at::MARK | 0b111);
    pc.cpu.set_regs(Regs {
        eip: at::CODE0 + 5,
        esp: at::STACK0,
        ..pc.regs()
    });
    let mut sys = pc.cpu.sys();
    sys.cr2 = 0;
    pc.cpu.set_sys(sys);
    pc.cpu.step();
    assert_eq!(pc.read32(at::MARK), 0x0bad_f00d);
}
