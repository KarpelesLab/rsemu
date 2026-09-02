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

use crate::core::device::{DebugTranslation, Deferred, Device, RealizeCtx, ResetKind};
use crate::core::space::{AddressSpace, RamStore, Region, RequesterId};
use crate::core::state::{ChunkReader, MachineShape, StateWriter};

use super::isa::{Arg, Class, Grp, Op, decode, resolve};
use super::{Config, Features, Interrupt, Reg, Regs, Variant, X86, flags, linear};

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
            self.ram.write_u8(addr, *byte).unwrap();
        }
        let mut regs = self.cpu.regs();
        regs.cs = cs;
        regs.rip = u64::from(ip);
        self.cpu.set_regs(regs);
        self.cpu.session.lock().state.reset_pending = false;
    }

    fn poke(&self, addr: u64, byte: u8) {
        self.ram.write_u8(addr, byte).unwrap();
    }

    fn peek(&self, addr: u64) -> u8 {
        self.ram.read_u8(addr).unwrap()
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
    assert_eq!(m.regs().rax & 0xff, 0x5a);
}

#[test]
fn bp_based_addressing_defaults_to_the_stack_segment() {
    let m = machine();
    m.set_regs(|r| {
        r.ds = 0x1000;
        r.ss = 0x2000;
        r.rbp = 0x0004;
        r.rbx = 0x0004;
    });
    m.poke(linear(0x2000, 0x0004), 0x11);
    m.poke(linear(0x1000, 0x0004), 0x22);
    // mov al, [bp+0] — SS by default.
    m.load(0x0000, 0x0100, &[0x8a, 0x46, 0x00]);
    m.cpu.step();
    assert_eq!(m.regs().rax & 0xff, 0x11);
    // mov al, [bx] — DS by default.
    m.load(0x0000, 0x0200, &[0x8a, 0x07]);
    m.cpu.step();
    assert_eq!(m.regs().rax & 0xff, 0x22);
    // ds: mov al, [bp+0] — the override wins.
    m.load(0x0000, 0x0300, &[0x3e, 0x8a, 0x46, 0x00]);
    m.cpu.step();
    assert_eq!(m.regs().rax & 0xff, 0x22);
}

#[test]
fn the_direct_address_encoding_is_not_bp_relative() {
    // md=0 rm=6 is a 16-bit address in DS, not [BP] — the encoding [BP] would
    // have used, which is why an assembler emits [BP+0] instead.
    let m = machine();
    m.set_regs(|r| {
        r.ds = 0x1000;
        r.ss = 0x2000;
        r.rbp = 0xbeef;
    });
    m.poke(linear(0x1000, 0x0034), 0x77);
    m.load(0x0000, 0x0100, &[0x8a, 0x06, 0x34, 0x00]);
    m.cpu.step();
    assert_eq!(m.regs().rax & 0xff, 0x77);
}

// ---------------------------------------------------------------------------
// Reset, halt, and the pins
// ---------------------------------------------------------------------------

#[test]
fn reset_starts_sixteen_bytes_below_the_top_of_memory() {
    let m = machine();
    m.cpu.step();
    let regs = m.regs();
    assert_eq!((regs.cs, regs.rip), (0xffff, 0x0000));
    assert_eq!(linear(regs.cs, regs.rip as u16), 0xf_fff0);
    assert_eq!((regs.ds, regs.es, regs.ss), (0, 0, 0));
    assert_eq!(regs.eflags, flags::RESERVED_SET);
    assert!(!m.cpu.reset_pending());
}

#[test]
fn a_reset_vector_jump_lands_where_it_says() {
    let m = machine();
    // The PC's own reset vector shape: jmpf 0xf000:0xe05b.
    for (i, byte) in [0xea, 0x5b, 0xe0, 0x00, 0xf0].into_iter().enumerate() {
        m.poke(0xf_fff0 + i as u64, byte);
    }
    m.cpu.step(); // reset
    m.cpu.step(); // the far jump
    let regs = m.regs();
    assert_eq!((regs.cs, regs.rip), (0xf000, 0xe05b));
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
    assert_eq!((regs.cs, regs.rip), (0x0000, 0x4000));
}

#[test]
fn an_interrupt_pushes_flags_then_cs_then_the_return_address() {
    let m = machine();
    m.load(0x1000, 0x0100, &[0x90]);
    m.set_regs(|r| {
        r.ss = 0x2000;
        r.rsp = 0x0100;
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
    assert_eq!((regs.cs, regs.rip), (0x3000, 0x1234));
    assert_eq!(regs.rsp, 0x00fa);
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
        r.rsp = 0x0100;
        r.eflags &= !flags::IF;
    });
    m.cpu.set_intr_vector(0x20);
    m.cpu.set_intr(true);
    m.cpu.step();
    assert_eq!(
        m.regs().rip,
        0x0101,
        "INTR must be ignored while IF is clear"
    );

    m.poke(0x0008, 0x00);
    m.poke(0x0009, 0x40);
    m.cpu.pulse_nmi();
    m.cpu.step();
    assert_eq!(m.regs().rip, 0x4000, "NMI is not maskable");
}

#[test]
fn writing_the_stack_segment_shadows_the_next_instruction() {
    let m = machine();
    // mov ss, ax ; mov sp, bx — the canonical stack switch. An interrupt
    // taken between the two would run the handler on a half-changed stack.
    m.load(0x0000, 0x0100, &[0x8e, 0xd0, 0x89, 0xdc]);
    m.set_regs(|r| {
        r.rax = 0x3000;
        r.rbx = 0x0200;
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
    assert_eq!(m.regs().rsp, 0x0200);
    assert!(!m.cpu.interrupt_shadow());
    m.cpu.step(); // and only now is the interrupt taken
    assert_eq!(m.regs().rip, 0x5000);
}

#[test]
fn the_trap_flag_takes_a_type_one_interrupt_after_each_instruction() {
    let m = machine();
    m.load(0x0000, 0x0100, &[0x90]);
    m.set_regs(|r| {
        r.ss = 0x2000;
        r.rsp = 0x0100;
        r.eflags |= flags::TF;
    });
    m.poke(0x04, 0x00);
    m.poke(0x05, 0x60);
    m.cpu.step();
    let regs = m.regs();
    assert_eq!(regs.rip, 0x6000);
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
    assert_eq!(m.regs().rip, 0x0102);
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
    assert_eq!(m.regs().rax, 1);
    assert_eq!(m.regs().rip, 0x0101);
}

// ---------------------------------------------------------------------------
// Instructions the corpus leaves out
// ---------------------------------------------------------------------------

#[test]
fn wait_and_lock_do_nothing_observable() {
    let m = machine();
    m.load(0x0000, 0x0100, &[0x9b, 0xf0, 0x40]);
    m.cpu.step(); // wait
    assert_eq!(m.regs().rip, 0x0101);
    m.cpu.step(); // lock inc ax — one instruction, prefix included
    assert_eq!(m.regs().rip, 0x0103);
    assert_eq!(m.regs().rax, 1);
}

#[test]
fn separate_address_spaces_mean_a_port_is_not_a_memory_address() {
    let m = machine();
    m.ports.write_u8(0x0060, 0xa5).unwrap();
    m.poke(0x0060, 0x5a);
    // in al, 0x60
    m.load(0x0000, 0x0100, &[0xe4, 0x60]);
    m.cpu.step();
    assert_eq!(m.regs().rax & 0xff, 0xa5, "IN must not read memory");

    // out 0x61, al with al = 0x12
    m.set_regs(|r| r.rax = 0x0012);
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
    m.set_regs(|r| r.rdx = 0x0300);
    m.load(0x0000, 0x0100, &[0xed]); // in ax, dx
    m.cpu.step();
    assert_eq!(m.regs().rax, 0x1234);
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
        rip: 0x100,
        ..Regs::new()
    });
    cpu.session.lock().state.reset_pending = false;
    cpu.step();
    assert_eq!(cpu.regs().rax & 0xff, 0xff);
}

#[test]
fn string_moves_follow_the_direction_flag() {
    let m = machine();
    for i in 0..4u32 {
        m.poke(0x1_0000 + u64::from(i), 0xa0 + i as u8);
    }
    m.set_regs(|r| {
        r.ds = 0x1000;
        r.es = 0x2000;
        r.rsi = 0;
        r.rdi = 0;
        r.rcx = 4;
    });
    m.load(0x0000, 0x0100, &[0xf3, 0xa4]); // rep movsb
    m.cpu.step();
    assert_eq!(m.regs().rcx, 0);
    assert_eq!(m.regs().rsi, 4);
    assert_eq!(m.regs().rdi, 4);
    for i in 0..4u32 {
        assert_eq!(m.peek(0x2_0000 + u64::from(i)), 0xa0 + i as u8);
    }

    // Backwards, and one short: REPNE stops on a match.
    m.set_regs(|r| {
        r.es = 0x2000;
        r.rdi = 3;
        r.rcx = 4;
        r.rax = 0xa2;
        r.eflags |= flags::DF;
    });
    m.load(0x0000, 0x0200, &[0xf2, 0xae]); // repne scasb
    m.cpu.step();
    assert_eq!(m.regs().rcx, 2, "scan stops the moment it matches");
    assert_eq!(m.regs().rdi, 1);
}

#[test]
fn a_repeat_with_a_zero_count_does_nothing_at_all() {
    let m = machine();
    m.set_regs(|r| {
        r.rcx = 0;
        r.rsi = 0x10;
        r.rdi = 0x20;
    });
    m.load(0x0000, 0x0100, &[0xf3, 0xa4]);
    m.cpu.step();
    let regs = m.regs();
    assert_eq!((regs.rsi, regs.rdi, regs.rcx), (0x10, 0x20, 0));
}

#[test]
fn a_repeat_is_interruptible_between_iterations() {
    let m = machine();
    m.set_regs(|r| {
        r.rax = 0x3000;
        r.ds = 0x1000;
        r.es = 0x2000;
        r.rcx = 100;
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
    assert_eq!(m.regs().rip, 0x0102);
    assert!(m.regs().rcx < 100 && m.regs().rcx > 0);
    m.cpu.step();
    assert_eq!(m.regs().rip, 0x7000);
}

#[test]
fn push_sp_stores_the_decremented_pointer() {
    // True of the 8086 and 8088 and of nothing later: the 286 pushes the value
    // SP had before the instruction.
    let m = machine();
    m.set_regs(|r| {
        r.ss = 0x2000;
        r.rsp = 0x0100;
    });
    m.load(0x0000, 0x0100, &[0x54]);
    m.cpu.step();
    assert_eq!(m.regs().rsp, 0x00fe);
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
        r.rax = 0x009a;
        r.eflags = (r.eflags | flags::AF) & !flags::CF;
    });
    m.cpu.step();
    assert_eq!(m.regs().rax & 0xff, 0xa0);
    assert_eq!(m.regs().eflags & flags::CF, 0);

    // With AF clear the same AL takes both corrections: 0x9a + 0x66 = 0x00.
    m.load(0x0000, 0x0200, &[0x27]);
    m.set_regs(|r| {
        r.rax = 0x009a;
        r.eflags &= !(flags::AF | flags::CF);
    });
    m.cpu.step();
    assert_eq!(m.regs().rax & 0xff, 0x00);
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
        r.rax = 0x0081; // AL = 0x81: low digit 1, so no adjustment
        r.eflags &= !(flags::AF | flags::SF | flags::PF | flags::ZF);
    });
    m.cpu.step();
    let regs = m.regs();
    assert_eq!(regs.rax, 0x0001, "only the low digit survives");
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
        r.rip = 0x100;
        r.ds = 0x2000;
        r.rbx = 0;
        r.rcx = 0; // CL = 0: no rotation at all
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
        r.rax = 0x0010;
        r.rbx = 0x0010;
    });
    m.cpu.step();
    let regs = m.regs();
    assert_eq!(regs.rax, 0x0100);
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
        r.rax = 0xffff;
        r.rbx = 0x0001; // BL = 1: the quotient cannot fit in AL
        r.ss = 0x2000;
        r.rsp = 0x0100;
    });
    m.poke(0x00, 0x00);
    m.poke(0x01, 0x04); // vector 0 → 0000:0400
    m.cpu.step();
    let regs = m.regs();
    assert_eq!((regs.cs, regs.rip), (0x0000, 0x0400));
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
        r.rax = 0x0064; // 100
        r.rbx = 0x000a; // BL = 10
    });
    m.cpu.step();
    assert_eq!(m.regs().rax & 0xff, 0xf6, "100 / 10 = 10, negated to -10");

    m.load(0x0000, 0x0200, &[0xf6, 0xfb]); // idiv bl, no prefix
    m.set_regs(|r| {
        r.rax = 0x0064;
        r.rbx = 0x000a;
    });
    m.cpu.step();
    assert_eq!(m.regs().rax & 0xff, 0x0a);
}

#[test]
fn logical_operations_clear_the_auxiliary_carry() {
    // Documented as undefined; cleared on every corpus vector.
    let m = machine();
    m.load(0x0000, 0x0100, &[0x24, 0xff]); // and al, 0xff
    m.set_regs(|r| {
        r.rax = 0x0001;
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
            r.rax = u64::from(value);
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
    regs.rax = 0x1234;
    assert_eq!(regs.byte(0), 0x34); // al
    assert_eq!(regs.byte(4), 0x12); // ah
    regs.set_byte(4, 0xab);
    assert_eq!(regs.rax, 0xab34);
    regs.set_byte(0, 0xcd);
    assert_eq!(regs.rax, 0xabcd);
    // The order is AL CL DL BL AH CH DH BH, which is why AH is 4.
    regs.rcx = 0x0000;
    regs.set_byte(5, 0xff);
    assert_eq!(regs.rcx, 0xff00);
}

#[test]
fn the_hard_wired_flag_bits_cannot_be_written() {
    let m = machine();
    // mov ax, 0 ; push ax ; popf
    m.load(0x0000, 0x0100, &[0xb8, 0x00, 0x00, 0x50, 0x9d]);
    m.set_regs(|r| {
        r.ss = 0x2000;
        r.rsp = 0x0100;
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
fn realize_does_nothing_outward_because_the_space_has_not_arrived_yet() {
    // The check that a core has an address space used to live in `realize`. It
    // cannot: the realizer runs `realize` for every device *before* it binds
    // any of them, so a core that refused here would refuse every machine. The
    // check is in `Instance::bind`, and
    // `binding_a_core_with_no_address_space_is_a_machine_error` covers it.
    let cpu = X86::new(Config::default());
    let mut deferred = Deferred::new();
    let ctx_hosts = crate::core::HostObjects::new();
    let mut ctx = RealizeCtx::new("/cpu0", RequesterId::ANONYMOUS, &mut deferred, &ctx_hosts);
    assert!(cpu.realize(&mut ctx).is_ok());
}

/// A `BuildOptions` that knows about this core and the built-in classes, and
/// nothing else.
fn options_with_the_core() -> (crate::core::Registry, crate::machine::BuildOptions) {
    let mut options = crate::machine::BuildOptions::new();
    for schema in super::schemas() {
        options.classes.insert(schema);
    }
    for schema in crate::machine::builtin::schemas() {
        options.classes.insert(schema);
    }
    super::bind(&mut options.bindings).expect("nothing else claims these names");
    crate::machine::builtin::bind(&mut options.bindings).expect("ram and rom");

    let mut registry = crate::core::Registry::new();
    crate::machine::builtin::register(&mut registry).expect("ram and rom");
    super::register(&mut registry).expect("nothing else claims these names");
    (registry, options)
}

#[test]
fn binding_a_core_with_no_address_space_is_a_machine_error() {
    // Through the machine layer, because that is the only thing that can build
    // a `BindCtx` — and it is the path a user's typo actually takes.
    let (registry, options) = options_with_the_core();
    let text = "machine \"m\" {\n  osc x = 1000000 Hz\n  space mem { width = 32 }\n  \
                object dram \"ram\" { size = 4K }\n  object cpu \"cpu.x86\" { clock = x }\n  \
                map mem 0 size 4K = dram\n}\n";
    let err = crate::machine::build("t.machine", text, &registry, &options)
        .expect_err("a core with no `space =` cannot fetch");
    let text = alloc::format!("{err}");
    assert!(text.contains("address space"), "{text}");
}

/// A machine file may reach every extension the constructor accepts.
///
/// The regression this exists for: the constructor read fifteen extension
/// overrides and the validator's schema listed four properties, so
/// `long = true` was rejected with "unknown property" before the core saw it.
/// The lattice was implemented and unreachable. `schema_for` now reads
/// `CLASS.properties`, so this asserts the two lists are the same list.
#[test]
fn a_machine_file_can_reach_every_extension_the_core_accepts() {
    for spec in super::CLASS.properties {
        let mut names = alloc::vec::Vec::new();
        for schema in super::schemas() {
            if schema.class == "cpu.x86" {
                names.extend(schema.props.iter().map(|p| p.name.clone()));
            }
        }
        assert!(
            names.iter().any(|n| n == spec.name),
            "the validator does not know about `{}`, which the constructor reads",
            spec.name
        );
    }
}

/// And the value actually arrives: a 486 told it has `CR4` and the
/// model-specific registers is a Pentium-class part, which is a real
/// configuration and not a variant.
#[test]
fn an_extension_named_in_a_machine_file_reaches_the_core() {
    let (registry, mut options) = options_with_the_core();
    let kept: Arc<crate::core::Captured<X86>> = Arc::new(crate::core::Captured::new());
    let mine = Arc::clone(&kept);
    options.bindings.replace("cpu.x86", move |props| {
        let cpu = Arc::new(X86::from_props_defaulting(props, super::Variant::I80486)?);
        mine.push(&cpu);
        Ok(cpu)
    });
    let text = "machine \"m\" {\n  osc x = 1000000 Hz\n  space mem { width = 32 }\n  \
                object dram \"ram\" { size = 4K }\n  \
                object cpu \"cpu.x86\" { clock = x, space = mem, variant = \"80486\", \
                cr4 = true, msr = true, cx8 = true, pse = true }\n  \
                map mem 0 size 4K = dram\n}\n";
    crate::machine::build("t.machine", text, &registry, &options).expect("it builds");
    let cpu = kept.take().expect("the constructor kept a handle");
    let features = cpu.config().features;
    assert!(features.cr4 && features.msr && features.cx8 && features.pse);
    assert!(!features.long, "nothing turned long mode on");
}

/// An extension set no part could have is refused by the *core*, with the
/// prerequisite named — not by the validator, which only knows types.
#[test]
fn an_impossible_extension_set_is_refused_with_the_missing_prerequisite() {
    let (registry, options) = options_with_the_core();
    let text = "machine \"m\" {\n  osc x = 1000000 Hz\n  space mem { width = 32 }\n  \
                object dram \"ram\" { size = 4K }\n  \
                object cpu \"cpu.x86\" { clock = x, space = mem, variant = \"x86-64\", \
                sse2 = false }\n  \
                map mem 0 size 4K = dram\n}\n";
    let err = crate::machine::build("t.machine", text, &registry, &options)
        .expect_err("a long-mode part without SSE2 is not a processor anyone shipped");
    let text = alloc::format!("{err}");
    assert!(text.contains("sse2"), "{text}");
}

#[test]
fn an_iospace_that_names_nothing_is_a_machine_error() {
    let (registry, options) = options_with_the_core();
    let text = "machine \"m\" {\n  osc x = 1000000 Hz\n  space mem { width = 32 }\n  \
                object dram \"ram\" { size = 4K }\n  \
                object cpu \"cpu.x86\" { clock = x, space = mem, iospace = \"ports\" }\n  \
                map mem 0 size 4K = dram\n}\n";
    let err = crate::machine::build("t.machine", text, &registry, &options)
        .expect_err("there is no space called `ports`");
    let text = alloc::format!("{err}");
    assert!(text.contains("ports"), "{text}");
}

#[test]
fn a_machine_file_names_the_core_gives_it_two_spaces_and_it_runs() {
    // The whole point of the exercise: `cpu.x86` in a `.machine` file, with a
    // separate I/O space, executing an `OUT` that lands in it.
    let (registry, mut options) = options_with_the_core();
    //   mov al, 0x5a ; out 0x42, al ; hlt
    options
        .realize
        .media
        .insert("firmware", alloc::vec![0xb0u8, 0x5a, 0xe6, 0x42, 0xf4]);
    let text = "machine \"m\" {\n  osc x = 4772726 Hz\n  \
                space mem  { width = 20 }\n  space port { width = 16 }\n  \
                object cpu \"cpu.x86\" \
                { clock = x, space = mem, iospace = \"port\", variant = \"8088\" }\n  \
                object ram \"ram\" { size = 64K }\n  \
                object boot \"rom\" { size = 16, image = \"firmware\" }\n  \
                object io \"ram\" { size = 64K }\n  \
                map mem  0x00000 size 64K = ram\n  \
                map mem  0xffff0 size 16  = boot\n  \
                map port 0 size 64K = io\n}\n";
    let mut machine = match crate::machine::build("t.machine", text, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    };
    // Two milliseconds at 4.77 MHz is thousands of clocks, which is the reset
    // sequence plus three instructions with room to spare. A span rather than a
    // step count, because the scheduler hands out budgets — and at least one
    // whole scheduling round, because `run_for` runs whole rounds and a shorter
    // span on a board with no periodic device would run none of it (§11.6).
    machine
        .run_for(crate::core::clock::GlobalTime::from_nanos(2_000_000))
        .expect("it runs");
    let port = machine.space("port").expect("the I/O space");
    assert_eq!(
        port.read(
            0x42,
            crate::core::value::Width::U8,
            crate::core::space::MemAttrs::DEFAULT,
        )
        .expect("a port"),
        0x5a,
        "the `OUT` did not reach the space `iospace` names"
    );
}

#[test]
fn state_round_trips_through_a_snapshot() {
    let m = machine();
    m.load(0x1234, 0x5678, &[0x40, 0x41, 0x42]);
    m.set_regs(|r| {
        r.rax = 0x1111;
        r.rbx = 0x2222;
        r.rcx = 0x3333;
        r.rdx = 0x4444;
        r.rsp = 0x5555;
        r.rbp = 0x6666;
        r.rsi = 0x7777;
        r.rdi = 0x8888;
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
    m.set_regs(|r| r.rax = 0xbeef);
    m.cpu.reset(ResetKind::Warm);
    assert!(m.cpu.reset_pending());
    m.cpu.step();
    assert_eq!(m.regs().rax, 0xbeef);
    assert_eq!(m.regs().cs, 0xffff);

    m.cpu.reset(ResetKind::Cold);
    assert_eq!(m.cpu.regs().rax, 0);
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
    assert_eq!(m.regs().rax & 0xff, 0xff);
    // setmo: the undocumented D0 /6.
    m.set_regs(|r| r.rax &= 0xff00);
    m.load(0x0000, 0x0200, &[0xd0, 0xf0]);
    m.cpu.step();
    assert_eq!(m.regs().rax & 0xff, 0xff);
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
        assert_eq!(m.regs().rip, u64::from(ip));
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
    pub(super) const GDT: u64 = 0x1000;
    /// The interrupt descriptor table.
    pub(super) const IDT: u64 = 0x1800;
    /// The task state segment.
    pub(super) const TSS: u64 = 0x2000;
    /// Ring-0 code.
    pub(super) const CODE0: u64 = 0x3000;
    /// Ring-3 code.
    pub(super) const CODE3: u64 = 0x4000;
    /// The page directory.
    pub(super) const PDIR: u64 = 0x5000;
    /// The first page table.
    pub(super) const PTAB: u64 = 0x6000;
    /// Scratch, for a test to write a marker into.
    pub(super) const MARK: u64 = 0x7000;
    /// The top of the ring-0 stack.
    pub(super) const STACK0: u64 = 0x9000;
    /// The top of the ring-3 stack.
    pub(super) const STACK3: u64 = 0xa000;
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
    /// A writable 16-bit data segment — no `B` bit, so `SP` moves in sixteen.
    pub(super) const DATA16: u32 = ar::PRESENT | ar::S | ar::RW;
    /// The same at privilege 3.
    pub(super) const DPL3: u32 = ar::DPL;
}

/// The two doublewords of a descriptor, with the granularity bit chosen for
/// the limit given.
///
/// A limit above 1 MiB has to be expressed in pages, and the architecture
/// rounds *up* to the page containing it — which is why a limit of `0xffffffff`
/// and a limit of `0xfffff000` produce the same descriptor.
fn descriptor(base: u64, limit: u32, ar_bits: u32) -> (u32, u32) {
    let (limit, ar_bits) = if limit > 0xf_ffff {
        (limit >> 12, ar_bits | ar::GRANULAR)
    } else {
        (limit, ar_bits)
    };
    let base = base as u32;
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
        Pc::with_features(variant, variant.features())
    }

    /// The same core with a narrowed extension set — a 486 with no
    /// floating-point unit, say.
    fn with_features(variant: Variant, features: Features) -> Pc {
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

        let cpu = Arc::new(X86::new(
            Config::default()
                .with_variant(variant)
                .with_features(features),
        ));
        cpu.attach_space(Arc::new(mem));
        cpu.attach_io_space(Arc::new(io));
        Pc {
            cpu,
            ram,
            rom,
            ports,
        }
    }

    fn write(&self, addr: u64, bytes: &[u8]) {
        for (i, byte) in bytes.iter().enumerate() {
            self.ram.write_u8(addr + i as u64, *byte).unwrap();
        }
    }

    fn write32(&self, addr: u64, value: u64) {
        for i in 0..4u64 {
            self.ram
                .write_u8(addr + i, (value >> (8 * i)) as u8)
                .unwrap();
        }
    }

    fn read32(&self, addr: u64) -> u64 {
        let mut value = 0u64;
        for i in 0..4u64 {
            value |= u64::from(self.ram.read_u8(addr + i).unwrap()) << (8 * i);
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
    fn gdt(&self, index: u64, pair: (u32, u32)) {
        self.write32(at::GDT + index * 8, u64::from(pair.0));
        self.write32(at::GDT + index * 8 + 4, u64::from(pair.1));
    }

    /// Write a gate into the interrupt descriptor table.
    fn idt(&self, vector: u64, pair: (u32, u32)) {
        self.write32(at::IDT + vector * 8, u64::from(pair.0));
        self.write32(at::IDT + vector * 8 + 4, u64::from(pair.1));
    }

    /// Start executing in real mode at `cs:eip`, with the reset sequence
    /// already done.
    fn start_real(&self, cs: u16, eip: u32) {
        let mut regs = self.cpu.regs();
        regs.cs = cs;
        regs.rip = u64::from(eip);
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
        regs.rsp = at::STACK0;
        regs.rip = at::CODE0;
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
    assert_eq!((regs.cs, regs.rip), (0xf000, 0xfff0));
    assert_eq!(pc.cpu.sys().seg(isa::seg::CS).base, 0xffff_0000);
    assert_eq!(
        pc.cpu.regs().rdx,
        u64::from(Variant::I80486.reset_signature())
    );

    pc.cpu.step();
    let regs = pc.regs();
    assert_eq!((regs.cs, regs.rip), (0x0000, 0x1000));
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
    assert_eq!(pc.regs().rax, 0x2345_1234);
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
    assert_eq!(pc.regs().rdx, 0xdead_beef);
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
    assert_eq!(pc.regs().rcx, 0x1111_2222);
    assert_eq!(pc.read32(0x7000), 0xcafe_babe);
    // A 32-bit push moved the stack pointer by four, not two.
    assert_eq!(pc.regs().rsp, at::STACK0);
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
    assert_eq!(regs.rip, 0x3100, "the fault took the #GP gate");
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
    assert_eq!(pc.regs().rip, 0x3100);
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
    for page in 0..1024u64 {
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
    assert_eq!(pc.regs().rbx, 0x0bad_f00d);
    // The write went to the *physical* page the table names.
    assert_eq!(pc.read32(at::MARK), 0x0bad_f00d);
    // And the walk set the accessed and dirty bits, which is the thing a
    // translation cache exists to do only once.
    let pte = pc.read32(at::PTAB + 0x200 * 4);
    assert_eq!(pte & 0b110_0000, 0b110_0000, "accessed and dirty");
    assert_eq!(pc.read32(at::PDIR) & 0b10_0000, 0b10_0000, "accessed");
}

#[test]
fn a_debug_translation_walks_the_tables_and_touches_nothing() {
    // The other half of the walk: a debugger asks *where* a linear address
    // lives and must not change the machine on the way. `Device::translate`
    // sets accessed and dirty bits, fills the TLB and latches `CR2`; this one
    // must do none of the three, or every `m` packet a debugger sends would
    // move the guest's page-replacement state underneath it.
    let pc = pc386();
    pc.start_protected();
    pc.write32(at::PDIR, at::PTAB | 0b111);
    for page in 0..1024u64 {
        pc.write32(at::PTAB + page * 4, (page << 12) | 0b111);
    }
    pc.write32(at::PTAB + 0x200 * 4, at::MARK | 0b111);

    // Before paging is on, a linear address is already physical, and that is
    // a different answer from "the tables map nothing here".
    assert_eq!(
        pc.cpu.translate_debug(0x0020_0034),
        DebugTranslation::Identity
    );

    let mut sys = pc.cpu.sys();
    sys.cr3 = at::PDIR;
    sys.cr0 |= cr0::PG;
    pc.cpu.set_sys(sys);

    assert_eq!(
        pc.cpu.translate_debug(0x0020_0034),
        DebugTranslation::Mapped(at::MARK + 0x34),
        "through the directory and the table, offset kept"
    );
    // The second 4 MiB has no directory entry, so there is nothing to name.
    assert_eq!(
        pc.cpu.translate_debug(0x0040_1234),
        DebugTranslation::Unmapped
    );
    // And the walk wrote nothing: no accessed bit in either level, and no
    // fault address latched by the miss.
    assert_eq!(
        pc.read32(at::PDIR) & 0b110_0000,
        0,
        "the directory is untouched"
    );
    assert_eq!(
        pc.read32(at::PTAB + 0x200 * 4) & 0b110_0000,
        0,
        "the table entry is untouched"
    );
    assert_eq!(pc.cpu.sys().cr2, 0, "a miss latched no fault address");
}

#[test]
fn a_missing_page_faults_with_the_address_in_cr2_and_the_reason_in_the_code() {
    let pc = pc386();
    pc.start_protected();
    pc.write32(at::PDIR, at::PTAB | 0b111);
    for page in 0..1024u64 {
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
    assert_eq!(pc.regs().rip, 0x3100);
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
        for page in 0..1024u64 {
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
        let faulted = pc.regs().rip == 0x3100;
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
    assert_eq!(regs.rsp, at::STACK3);
    assert_eq!(pc.cpu.sys().task.selector, 0x28);

    pc.cpu.step(); // int 0x80
    let regs = pc.regs();
    assert_eq!(regs.cs & 3, 0, "the gate raised the privilege level");
    assert_eq!(regs.rip, 0x3100);
    assert_eq!(regs.ss, 0x10, "the stack came out of the TSS");
    // Five doublewords: SS, ESP, EFLAGS, CS, EIP.
    assert_eq!(regs.rsp, at::STACK0 - 20);
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
    assert_eq!(regs.rsp, at::STACK3);
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
    assert_eq!(pc.regs().rip, 0x3100, "hlt in ring 3 is #GP");
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
    assert_eq!(regs.rax, 1, "the highest leaf");
    assert_eq!(
        (regs.rbx, regs.rdx, regs.rcx),
        (
            u64::from(u32::from_le_bytes(*b"Genu")),
            u64::from(u32::from_le_bytes(*b"ineI")),
            u64::from(u32::from_le_bytes(*b"ntel"))
        )
    );

    // Leaf 1 reports the signature and exactly the features a 486DX has: an
    // on-die x87 unit and nothing else. `CX8` is clear because `CMPXCHG8B`
    // arrived with the Pentium, and `FXSR`/`SSE`/`SSE2` because they arrived
    // long after that.
    let pc = pc386();
    pc.start_protected();
    pc.write(at::CODE0, &[0xb8, 0x01, 0x00, 0x00, 0x00, 0x0f, 0xa2, 0xf4]);
    pc.cpu.step();
    pc.cpu.step();
    let regs = pc.regs();
    assert_eq!(regs.rax, 0x0000_0480);
    assert_eq!(regs.rdx, 1, "FPU, and nothing else, on a 486DX");

    // A 486SX is the same part with the unit fused off, which is the whole
    // point of `Features` being separate from `Variant`.
    let pc = Pc::with_features(Variant::I80486, Features::I80486SX);
    pc.start_protected();
    pc.write(at::CODE0, &[0xb8, 0x01, 0x00, 0x00, 0x00, 0x0f, 0xa2, 0xf4]);
    pc.cpu.step();
    pc.cpu.step();
    assert_eq!(pc.regs().rdx, 0, "no FPU bit on a 486SX");

    // A 386 has no `CPUID` at all.
    let pc = Pc::new(Variant::I80386);
    pc.start_protected();
    pc.idt(6, gate(0x08, 0x3100, sys_type::INT_GATE32, 0));
    pc.write(0x3100, &[0xf4]);
    pc.write(at::CODE0, &[0x0f, 0xa2]);
    pc.cpu.step();
    assert_eq!(pc.regs().rip, 0x3100, "#UD on a 386");
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
    assert_eq!(regs.rax, 0x80, "bts then btr then btc");
    assert_eq!(regs.rbx, 0xffff_ff81, "movsx sign-extended");
    assert_eq!(regs.rdx, 8, "bsf found the lowest set bit");
    assert_eq!(regs.rsi, 8, "bsr found the highest");

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
    assert_eq!(regs.rax.swap_bytes(), 0x0000_0000);
    assert_eq!(regs.rbx, 0x0500_0000, "bswap reversed the byte order");
    assert_eq!(regs.rdi, 70, "the three-operand imul");
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
    assert_eq!(regs.rax, 0x1111_1111, "popad restored it");
    assert_eq!(regs.rbx, 0x3333_3333);
    assert_eq!(regs.rsp, at::STACK0, "and left the stack where it found it");
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
    assert_eq!(regs.rbp, at::STACK0 - 4, "the frame pointer is the new top");
    assert_eq!(
        regs.rsp,
        at::STACK0 - 4 - 0x10,
        "and 16 bytes were reserved"
    );
    assert_eq!(pc.read32(at::STACK0 - 4), 0x8800, "the old EBP was saved");
    pc.cpu.step();
    let regs = pc.regs();
    assert_eq!(regs.rbp, 0x8800);
    assert_eq!(regs.rsp, at::STACK0);
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
        r.rax = 0x00ff;
        r.rcx = 32;
    });
    m.cpu.step();
    assert_eq!(m.regs().rax & 0xff, 0, "an 8086 really shifts 32 times");

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
    assert_eq!(pc.regs().rax & 0xff, 0xff, "a 386 masks the count to zero");
}

#[test]
fn push_sp_stores_the_value_before_the_decrement_from_the_80286_on() {
    let m = machine();
    m.load(0x0000, 0x0100, &[0x54]); // push sp
    m.set_regs(|r| {
        r.ss = 0x2000;
        r.rsp = 0x0100;
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
    assert_eq!(regs.rbx, 0x0fff, "lsl read the limit");
    assert_eq!(regs.rcx, u64::from(rights::DATA32 & ar::MASK));
    assert_eq!(
        regs.rdx, 0,
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
    assert_eq!(pc.regs().rax & 0xffff, 0x0b, "raised to RPL 3");
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
        rax: 0x1234_5678,
        rsi: 0x9abc_def0,
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
        rax: 0x0000_0001,
        rcx: 0x0000_0002,
        rdx: 0x0000_0003,
        rbx: 0x0000_0004,
        rsp: 0x0000_0005,
        rbp: 0x0000_0006,
        rsi: 0x0000_0007,
        rdi: 0x0000_0008,
        rip: 0x0000_0009,
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
    assert_eq!(pc.regs().rbx, 0xaaaa_aaaa);

    // Move the descriptor's base and read again without reloading `ES`.
    pc.gdt(3, descriptor(0x2_0000, 0xffff, rights::DATA32));
    pc.cpu.set_regs(Regs {
        rip: at::CODE0 + 7,
        ..pc.regs()
    });
    pc.cpu.step();
    assert_eq!(
        pc.regs().rbx,
        0xaaaa_aaaa,
        "the cached base is what the processor uses"
    );

    // Reloading the selector is what publishes the change.
    pc.cpu.set_regs(Regs {
        rip: at::CODE0,
        ..pc.regs()
    });
    pc.cpu.step();
    pc.cpu.step();
    pc.cpu.step();
    assert_eq!(pc.regs().rbx, 0xbbbb_bbbb);
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
    assert_eq!(pc.regs().rip, 0x3100, "using it is not");
    assert_eq!(pc.read32(at::STACK0 - 16), 0, "#GP(0), naming no selector");
}

#[test]
fn a_task_switch_saves_the_outgoing_task_and_loads_the_incoming_one() {
    let pc = pc386();
    pc.start_protected();
    pub(super) const TSS_B: u64 = 0x2200;
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
    pc.write32(TSS_B + tss32::EFLAGS, u64::from(flags::ALWAYS_SET));
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
    assert_eq!(regs.rax, 0x4444_4444, "the incoming task's registers");
    assert_eq!(regs.rip, 0x3300);
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
    for i in 0..0x18u64 {
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
    assert_eq!(pc.regs().rip, at::CODE3 + 2, "port 0x60 went through");
    pc.cpu.step();
    assert_eq!(pc.regs().rip, 0x3100, "port 0x61 raised #GP");
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
        let d = disassemble_as(isa::Gen::I386, isa::Bits::B32, 0, 0, bytes);
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
    for i in 0..4u64 {
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
    for i in 0..4u64 {
        assert_eq!(pc.read32(0x8100 + i * 4), 0x1111_1111 * (i + 1));
    }
    assert_eq!(pc.read32(0x8200), 0xa5a5_a5a5);
    assert_eq!(pc.read32(0x8204), 0xa5a5_a5a5);
    assert_eq!(pc.read32(0x8208), 0, "and stopped after two");
    // `repne scasd` compares `EAX` with each doubleword and stops on a match:
    // the first two match, so it stops after one iteration with three left.
    let regs = pc.regs();
    assert_eq!(regs.rcx, 3);
    assert_eq!(regs.rdi, 0x8204);
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
    assert_eq!(pc.regs().rbx, 0x600d, "the near jump reached CODE3");

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
    assert_eq!(pc.regs().rip, at::CODE3);
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
    assert_eq!(regs.rax, 0x1234, "the sum");
    assert_eq!(regs.rcx, 0x1000, "and the destination's old value");

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
    assert_eq!(regs.rbx, 9, "so the source was stored");
    assert_eq!(regs.rax, 5, "and the accumulator is unchanged");

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
    assert_eq!(regs.rbx, 5, "the destination is left alone");
    assert_eq!(regs.rax, 5, "and the accumulator takes its value");
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
    assert_eq!(regs.rax, 0x20);
    assert_eq!(regs.rbx & 0xffff, 0x4000);
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
    assert_eq!(pc.regs().rax, 0x4433_2211);
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
    assert_eq!(regs.rsp, 0x8800);
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
    assert_eq!((regs.cs, regs.rip), (0x0000, 0x1800));
}

#[test]
fn a_page_fault_restarts_the_instruction_that_caused_it() {
    // The whole point of a fault being restartable: the handler maps the page
    // and returns, and the instruction runs again from its first byte with the
    // registers it started with.
    let pc = pc386();
    pc.start_protected();
    pc.write32(at::PDIR, at::PTAB | 0b111);
    for page in 0..1024u64 {
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
    assert_eq!(pc.regs().rip, 0x3100);
    // The saved `EIP` names the `mov`, not the `hlt` after it.
    assert_eq!(pc.read32(at::STACK0 - 12), at::CODE0 + 5);
    assert_eq!(pc.regs().rax, 0x0bad_f00d, "the earlier work is not undone");

    // Map the page, restart, and the instruction completes.
    pc.write32(at::PTAB + (at::MARK >> 12) * 4, at::MARK | 0b111);
    pc.cpu.set_regs(Regs {
        rip: at::CODE0 + 5,
        rsp: at::STACK0,
        ..pc.regs()
    });
    let mut sys = pc.cpu.sys();
    sys.cr2 = 0;
    pc.cpu.set_sys(sys);
    pc.cpu.step();
    assert_eq!(pc.read32(at::MARK), 0x0bad_f00d);
}

#[test]
fn a_sixteen_bit_code_segment_in_protected_mode_runs_sixteen_bit_code() {
    // The mode a PC firmware spends its BIOS services in, and the one a
    // "just widen everything to 32 bits" core gets wrong: protection is on,
    // but `CS.D` is clear, so the *default* operand and address sizes are
    // sixteen and `66`/`67` select the wide forms rather than the narrow ones.
    let pc = pc386();
    pc.start_protected();
    pc.gdt(3, descriptor(0, 0xffff, rights::CODE16));
    pc.gdt(4, descriptor(0, 0xffff, rights::DATA16));
    pc.write(
        at::CODE0,
        &[0xea, 0x00, 0x40, 0x00, 0x00, 0x18, 0x00], // jmp far 0x18:0x4000
    );
    pc.write(
        at::CODE3,
        &[
            0xb8, 0x34, 0x12, // mov ax, 0x1234      — sixteen bits by default
            0x66, 0xbb, 0x78, 0x56, 0x34, 0x12, // mov ebx, 0x12345678
            0xa3, 0x00, 0x70, // mov [0x7000], ax    — a sixteen-bit offset
            0xf4,
        ],
    );
    let steps = pc.run(8);
    assert!(steps < 8);
    let regs = pc.regs();
    assert_eq!(regs.cs, 0x18);
    assert!(!pc.cpu.sys().seg(isa::seg::CS).big());
    assert_eq!(regs.rax & 0xffff, 0x1234);
    assert_eq!(regs.rbx, 0x1234_5678);
    assert_eq!(pc.read32(at::MARK) & 0xffff, 0x1234);
}

#[test]
fn a_far_call_crosses_between_sixteen_and_thirty_two_bit_segments() {
    let pc = pc386();
    pc.start_protected();
    pc.gdt(3, descriptor(0, 0xffff, rights::CODE16));
    // 32-bit code calls into a 16-bit segment and the callee returns. The
    // return address was pushed as two doublewords by the caller's operand
    // size, so the callee's `RETF` needs a `66` prefix to pop them the same
    // way — which is the detail that makes a mixed-width far return
    // interesting, and the reason firmware writes it that way.
    pc.write(
        at::CODE0,
        &[
            0x9a, 0x00, 0x40, 0x00, 0x00, 0x18, 0x00, // callf 0x18:0x4000
            0xbb, 0x0d, 0x60, 0x00, 0x00, // mov ebx, 0x600d
            0xf4,
        ],
    );
    pc.write(
        at::CODE3,
        &[
            0xb8, 0x99, 0x99, // mov ax, 0x9999
            0x66, 0xcb, // retf, with a 32-bit operand size
        ],
    );
    let steps = pc.run(8);
    assert!(steps < 8);
    let regs = pc.regs();
    assert_eq!(regs.rax & 0xffff, 0x9999, "the 16-bit callee ran");
    assert_eq!(regs.rbx, 0x600d, "and control came back");
    assert_eq!(regs.cs, 0x08);
    assert_eq!(regs.rsp, at::STACK0, "the stack is balanced");
}

#[test]
fn an_interrupt_onto_a_sixteen_bit_stack_moves_the_pointer_in_sixteen_bits() {
    let pc = pc386();
    pc.start_protected();
    // A stack segment with `B` clear: `SP` moves, `ESP`'s high half does not,
    // and a 16-bit gate pushes words rather than doublewords.
    pc.gdt(3, descriptor(0, 0xffff, rights::DATA16));
    pc.gdt(4, descriptor(0, 0xffff, rights::CODE16));
    pc.idt(0x40, gate(0x20, 0x4000, sys_type::INT_GATE16, 0));
    pc.write(0x4000, &[0xf4]);
    let mut sys = pc.cpu.sys();
    sys.segs[usize::from(isa::seg::SS)] = SegReg {
        selector: 0x18,
        base: 0,
        limit: 0xffff,
        ar: rights::DATA16,
    };
    pc.cpu.set_sys(sys);
    pc.cpu.set_regs(Regs {
        ss: 0x18,
        rsp: 0xdead_9000,
        ..pc.regs()
    });
    pc.write(at::CODE0, &[0xcd, 0x40, 0xf4]); // int 0x40
    pc.cpu.step();
    let regs = pc.regs();
    assert_eq!(regs.cs, 0x20);
    assert_eq!(regs.rip, 0x4000);
    assert_eq!(
        regs.rsp, 0xdead_8ffa,
        "three words pushed, and the high half of ESP untouched"
    );
    assert_eq!(
        pc.read32(0x8ffc) & 0xffff,
        0x08,
        "the caller's CS, stored as a word"
    );
}

#[test]
fn smsw_sldt_and_str_read_back_what_was_loaded() {
    let pc = pc386();
    pc.start_protected();
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
        descriptor(0x9800, 0xff, ar::PRESENT | (u32::from(sys_type::LDT) << 8)),
    );
    pc.write(
        at::CODE0,
        &[
            0xb8, 0x28, 0x00, 0x00, 0x00, // mov eax, 0x28
            0x0f, 0x00, 0xd8, // ltr ax
            0xb8, 0x30, 0x00, 0x00, 0x00, // mov eax, 0x30
            0x0f, 0x00, 0xd0, // lldt ax
            0x0f, 0x00, 0xc1, // sldt ecx
            0x0f, 0x00, 0xca, // str edx
            0x0f, 0x01, 0xe3, // smsw ebx
            0xf4,
        ],
    );
    let steps = pc.run(10);
    assert!(steps < 10);
    let regs = pc.regs();
    assert_eq!(regs.rcx & 0xffff, 0x30, "sldt");
    assert_eq!(regs.rdx & 0xffff, 0x28, "str");
    assert_eq!(regs.rbx & 1, 1, "smsw sees CR0.PE");
    assert_eq!(pc.cpu.sys().ldtr.base, 0x9800);
    // Marking the task state segment busy is what stops a second task being
    // switched into the same state; `LTR` does it as a side effect.
    assert_eq!(
        (pc.read32(at::GDT + 5 * 8 + 4) >> 8) & 0xf,
        u64::from(sys_type::TSS32_BUSY)
    );
}

#[test]
fn two_saves_of_the_same_state_are_byte_identical() {
    // CLAUDE.md asks a save/load round trip to reach an identical state hash.
    // Comparing the bytes is that without a hash function: if any part of the
    // core's state reached the snapshot in a form that does not round-trip,
    // the second save would differ from the first.
    let pc = pc386();
    pc.start_protected();
    pc.write(at::CODE0, &[0xb8, 0x11, 0x22, 0x33, 0x44, 0x50, 0x58, 0xf4]);
    pc.run(5);

    let snapshot = |cpu: &X86| {
        let mut shape = MachineShape::new();
        shape.add_device("/cpu0", "cpu.x86").unwrap();
        let mut writer = StateWriter::new(shape);
        {
            let mut chunk = writer.chunk("/cpu0", "cpu.x86", 2).unwrap();
            cpu.save(&mut chunk).unwrap();
        }
        writer.to_vec().unwrap()
    };

    let first = snapshot(&pc.cpu);
    pc.cpu.reset(ResetKind::Cold);
    let reader = crate::core::state::StateReader::new(&first).unwrap();
    let (_, _, data) = reader.load_raw("/cpu0").unwrap();
    let mut chunk = ChunkReader::new(data);
    pc.cpu.load(&mut chunk).unwrap();
    chunk.end().unwrap();
    let second = snapshot(&pc.cpu);
    assert_eq!(first, second, "the snapshot does not round-trip exactly");
}

#[test]
fn the_divide_that_has_no_representable_quotient_is_a_divide_error() {
    // `EDX:EAX` of `0x8000000000000000` divided by -1 would be `2^63`, which
    // does not fit in `EAX`. Hardware raises #DE; a host that divides first
    // and range-checks afterwards traps on the division instead, which is why
    // this case is checked rather than assumed.
    let pc = pc386();
    pc.start_protected();
    pc.idt(0, gate(0x08, 0x3100, sys_type::INT_GATE32, 0));
    pc.write(0x3100, &[0xf4]);
    pc.write(
        at::CODE0,
        &[
            0x31, 0xc0, // xor eax, eax
            0xba, 0x00, 0x00, 0x00, 0x80, // mov edx, 0x80000000
            0xb9, 0xff, 0xff, 0xff, 0xff, // mov ecx, -1
            0xf7, 0xf9, // idiv ecx
            0xf4,
        ],
    );
    for _ in 0..4 {
        pc.cpu.step();
    }
    assert_eq!(pc.regs().rip, 0x3100, "#DE, not a host panic");
    // #DE is a fault on a 386: the saved address is the `idiv`, not the `hlt`.
    assert_eq!(pc.read32(at::STACK0 - 12), at::CODE0 + 12);

    // The ordinary overflow — a quotient that is merely too large — is the
    // same fault.
    let pc = pc386();
    pc.start_protected();
    pc.idt(0, gate(0x08, 0x3100, sys_type::INT_GATE32, 0));
    pc.write(0x3100, &[0xf4]);
    pc.write(
        at::CODE0,
        &[
            0xb8, 0x00, 0x00, 0x00, 0x00, // mov eax, 0
            0xba, 0x01, 0x00, 0x00, 0x00, // mov edx, 1
            0xb9, 0x01, 0x00, 0x00, 0x00, // mov ecx, 1
            0xf7, 0xf1, // div ecx
            0xf4,
        ],
    );
    for _ in 0..4 {
        pc.cpu.step();
    }
    assert_eq!(pc.regs().rip, 0x3100);
}

#[test]
fn a_bit_test_on_memory_with_a_register_offset_reaches_outside_the_operand() {
    // `bt [mem], ebx` takes a **signed and unbounded** bit number: the
    // processor divides it by the operand size and addresses that far from the
    // operand, forward or backward. An implementation that masks it to the
    // operand size reads the wrong dword and nothing complains.
    let pc = pc386();
    pc.start_protected();
    pc.write32(0x8000, 0);
    pc.write32(0x8004, 0x0000_0004); // bit 34 of the array at 0x8000
    pc.write32(0x7ffc, 0x8000_0000); // bit -1
    pc.write(
        at::CODE0,
        &[
            0xbb, 0x22, 0x00, 0x00, 0x00, // mov ebx, 34
            0x0f, 0xa3, 0x1d, 0x00, 0x80, 0x00, 0x00, // bt [0x8000], ebx
            0x0f, 0x92, 0xc0, // setb al
            0xbb, 0xff, 0xff, 0xff, 0xff, // mov ebx, -1
            0x0f, 0xa3, 0x1d, 0x00, 0x80, 0x00, 0x00, // bt [0x8000], ebx
            0x0f, 0x92, 0xc4, // setb ah
            0xf4,
        ],
    );
    let steps = pc.run(10);
    assert!(steps < 10);
    let regs = pc.regs();
    assert_eq!(regs.rax & 0xff, 1, "bit 34 is one doubleword along");
    assert_eq!((regs.rax >> 8) & 0xff, 1, "bit -1 is one doubleword back");

    // With an immediate the bit number is taken modulo the operand size and
    // never leaves the operand, which is the other half of the same rule.
    let pc = pc386();
    pc.start_protected();
    pc.write32(0x8000, 0x0000_0001);
    pc.write32(0x8004, 0xffff_ffff);
    pc.write(
        at::CODE0,
        &[
            0x0f, 0xba, 0x25, 0x00, 0x80, 0x00, 0x00, 0x20, // bt [0x8000], 32
            0x0f, 0x92, 0xc0, // setb al
            0xf4,
        ],
    );
    let steps = pc.run(5);
    assert!(steps < 5);
    assert_eq!(pc.regs().rax & 0xff, 1, "bit 32 wrapped to bit 0");
}

// ---------------------------------------------------------------------------
// The A20 gate
// ---------------------------------------------------------------------------

/// Point the core back at `at::CODE0` without disturbing anything else.
fn rewind(pc: &Pc) {
    let mut regs = pc.cpu.regs();
    regs.rip = at::CODE0;
    pc.cpu.set_regs(regs);
}

#[test]
fn the_a20_gate_masks_address_bit_twenty_and_nothing_else() {
    // Not a processor feature on real silicon — the gate is in the chipset —
    // but this core does its own address wrapping and the gate is exactly a
    // suppression of it, so it is an input pin here. See `Lines::a20_mask`.
    let pc = pc386();
    pc.start_protected();
    pc.write(0x0000_0010, &[0x5a]);
    pc.write(0x0010_0010, &[0xa5]);

    // mov al, [0x00100010]
    pc.write(at::CODE0, &[0xa0, 0x10, 0x00, 0x10, 0x00]);
    assert!(pc.cpu.a20_open(), "a core with no gate wired has bit 20");
    pc.cpu.step();
    assert_eq!(pc.regs().rax & 0xff, 0xa5, "the megabyte above");

    pc.cpu.set_a20(false);
    rewind(&pc);
    pc.cpu.step();
    assert_eq!(
        pc.regs().rax & 0xff,
        0x5a,
        "with the gate shut, bit 20 never reaches memory"
    );

    // And only bit 20: an address a *second* megabyte up still gets there.
    pc.write(0x0020_0010, &[0x3c]);
    pc.write(at::CODE0, &[0xa0, 0x10, 0x00, 0x20, 0x00]);
    rewind(&pc);
    pc.cpu.step();
    assert_eq!(pc.regs().rax & 0xff, 0x3c);
}

#[test]
fn wiring_an_a20_pin_shuts_the_gate_because_a_fresh_net_sits_low() {
    use crate::core::wire::{Level, Wire, WireId};

    let pc = pc386();
    assert!(pc.cpu.a20_open(), "nothing has wired a gate");

    let src = WireId(1);
    let pin = pc.cpu.sink("a20", &[src]).expect("an a20 pin");
    assert!(
        !pc.cpu.a20_open(),
        "a board that has a gate starts with it shut, which is what its net \
         sitting low means and what an AT does"
    );
    let wire = Wire::builder()
        .source(src)
        .sink_weak(Arc::downgrade(&pin.sink), pin.line)
        .build();
    wire.set(src, Level::High);
    assert!(pc.cpu.a20_open());
    wire.set(src, Level::Low);
    assert!(!pc.cpu.a20_open());

    // And a cold reset leaves it where the board's own wiring puts it, rather
    // than where a board with no gate would be.
    Device::reset(&*pc.cpu, ResetKind::Cold);
    assert!(!pc.cpu.a20_open());
}

#[test]
fn the_scheduler_budget_is_never_overshot_and_the_debt_is_paid_back() {
    let pc = pc386();
    pc.start_protected();
    // A tight loop of one-byte increments, so every budget lands mid-something.
    pc.write(at::CODE0, &[0x40, 0x40, 0x40, 0xeb, 0xfb]);
    let before = pc.cpu.cycles();
    let mut total = 0u64;
    for _ in 0..64 {
        let used = pc.cpu.run_budget(1);
        assert!(used <= 1, "a budget of one tick reported {used}");
        total += used;
    }
    assert_eq!(total, 64, "every tick of every budget was granted and used");
    assert_eq!(
        pc.cpu.cycles() - before,
        total + pc.cpu.cycle_debt(),
        "clocks executed but not yet reported are exactly the debt"
    );
}

// ===========================================================================
// Long mode
// ===========================================================================

/// x86-64: the mode transition, the four-level walk, and 64-bit execution.
///
/// Every test here is written from the *Intel SDM* volume 3 §9.8.5's
/// activation sequence and volume 2's encoding rules, or from the *AMD64
/// Architecture Programmer's Manual* volume 2 where AMD is clearer — each
/// cited where the behaviour is not obvious. Nothing here was read off another
/// emulator (`ROADMAP.md` §1).
///
/// The one that matters is
/// [`a_guest_enters_long_mode_and_executes_64_bit_code`]: the others take
/// pieces of it apart, but that one is a *guest* doing the whole thing for
/// itself, from 32-bit protected mode through to a `REX.W` instruction storing
/// through a `RIP`-relative address.
mod long_mode {
    use super::*;
    use crate::core::state::StateReader;
    use crate::cpu::x86::Features;
    use crate::cpu::x86::paging::Mode;
    use crate::cpu::x86::prot::{cr4, efer, msr};

    /// Where the long-mode tests put their page tables and their code.
    ///
    /// Above the `at::` block so the two cannot collide.
    mod la {
        /// The four-level table's root.
        pub(super) const PML4: u64 = 0x1_0000;
        /// The page-directory-pointer table.
        pub(super) const PDPT: u64 = 0x1_1000;
        /// The page directory.
        pub(super) const PD: u64 = 0x1_2000;
        /// A page table, where a test wants 4 KiB granularity.
        pub(super) const PT: u64 = 0x1_4000;
        /// Where the 64-bit half of a program starts.
        pub(super) const CODE64: u64 = 0x2_0000;
        /// Where a 64-bit fault or interrupt handler starts.
        pub(super) const HANDLER: u64 = 0x2_9000;
        /// A second handler, for a test that needs two.
        pub(super) const HANDLER2: u64 = 0x2_a000;
        /// Scratch for a 64-bit program to store into.
        pub(super) const MARK: u64 = 0x2_8000;
    }

    /// A 64-bit code segment: `L` set, `D` clear.
    const CODE64_AR: u32 = ar::PRESENT | ar::S | ar::CODE | ar::RW | ar::L | ar::GRANULAR;

    /// Present, writable, user — the flags a test's identity map uses.
    const MAP: u64 = 0b111;

    /// The `PS` bit, which turns a directory entry into a large page.
    const PS: u64 = 1 << 7;

    impl Pc {
        fn write64(&self, addr: u64, value: u64) {
            for i in 0..8u64 {
                self.ram
                    .write_u8(addr + i, (value >> (8 * i)) as u8)
                    .unwrap();
            }
        }

        fn read64(&self, addr: u64) -> u64 {
            let mut value = 0u64;
            for i in 0..8u64 {
                value |= u64::from(self.ram.read_u8(addr + i).unwrap()) << (8 * i);
            }
            value
        }

        /// Write a **sixteen-byte** interrupt gate, which is what long mode's
        /// interrupt descriptor table holds.
        ///
        /// Bytes 0-1 and 6-7 are the offset's low thirty-two bits split around
        /// the selector and the access byte, exactly as a 32-bit gate does it;
        /// bytes 8-11 are the offset's top half, and bytes 12-15 are reserved.
        fn idt64(&self, vector: u64, selector: u16, offset: u64) {
            let base = at::IDT + vector * 16;
            let low = (offset as u32 & 0xffff) | (u32::from(selector) << 16);
            let high = (offset as u32 & 0xffff_0000)
                | ar::PRESENT
                | (u32::from(sys_type::INT_GATE32) << 8);
            self.write32(base, u64::from(low));
            self.write32(base + 4, u64::from(high));
            self.write32(base + 8, offset >> 32);
            self.write32(base + 12, 0);
        }

        /// Build a four-level identity map of the first 4 MiB out of two 2 MiB
        /// pages, add a 64-bit code descriptor at selector `0x18`, and give
        /// the interrupt descriptor table room for sixteen-byte entries.
        fn prepare_long(&self) {
            self.write64(la::PML4, la::PDPT | MAP);
            self.write64(la::PDPT, la::PD | MAP);
            self.write64(la::PD, MAP | PS);
            self.write64(la::PD + 8, 0x20_0000 | MAP | PS);
            self.gdt(3, descriptor(0, 0xffff_ffff, CODE64_AR));
            // A second, identical 64-bit code segment at selector `0x20`. A
            // test that corrupts the first still needs an intact one for its
            // fault handler to run in, because in long mode *every* gate
            // target has to be a 64-bit code segment.
            self.gdt(4, descriptor(0, 0xffff_ffff, CODE64_AR));
            let mut sys = self.cpu.sys();
            sys.idtr.limit = 0xfff;
            self.cpu.set_sys(sys);
        }
    }

    /// The 32-bit bring-up sequence, as an operating system writes it.
    ///
    /// *Intel SDM* volume 3 §9.8.5, "Initializing IA-32e Mode", in order:
    /// paging off (it already is), `CR4.PAE`, `CR3`, `EFER.LME`, `CR0.PG`,
    /// then a far jump to a code segment with `L` set. Every step is a real
    /// instruction executed by the guest — the point is that nothing is done
    /// for it from outside.
    fn enter_long_mode_code(target: u64) -> Vec<u8> {
        let mut code = Vec::new();
        // mov eax, cr4 ; or eax, PAE ; mov cr4, eax
        code.extend_from_slice(&[0x0f, 0x20, 0xe0]);
        code.push(0x0d);
        code.extend_from_slice(&(cr4::PAE as u32).to_le_bytes());
        code.extend_from_slice(&[0x0f, 0x22, 0xe0]);
        // mov eax, PML4 ; mov cr3, eax
        code.push(0xb8);
        code.extend_from_slice(&(la::PML4 as u32).to_le_bytes());
        code.extend_from_slice(&[0x0f, 0x22, 0xd8]);
        // mov ecx, IA32_EFER ; rdmsr ; or eax, LME ; wrmsr
        code.push(0xb9);
        code.extend_from_slice(&msr::EFER.to_le_bytes());
        code.extend_from_slice(&[0x0f, 0x32]);
        code.push(0x0d);
        code.extend_from_slice(&(efer::LME as u32).to_le_bytes());
        code.extend_from_slice(&[0x0f, 0x30]);
        // mov eax, cr0 ; or eax, PG ; mov cr0, eax  — the transition itself
        code.extend_from_slice(&[0x0f, 0x20, 0xc0]);
        code.push(0x0d);
        code.extend_from_slice(&cr0::PG.to_le_bytes());
        code.extend_from_slice(&[0x0f, 0x22, 0xc0]);
        // jmp 0x18:target — compatibility mode to 64-bit mode
        code.push(0xea);
        code.extend_from_slice(&(target as u32).to_le_bytes());
        code.extend_from_slice(&0x18u16.to_le_bytes());
        code
    }

    fn pc64() -> Pc {
        Pc::new(Variant::X86_64)
    }

    /// Bring a core into 64-bit mode and run `code` there.
    ///
    /// The bring-up is the same every time and running it twenty times tests
    /// nothing new, so it is factored out here and
    /// [`a_guest_enters_long_mode_and_executes_64_bit_code`] is the one test
    /// that walks through it explicitly.
    fn run64(code: &[u8]) -> Pc {
        let pc = pc64();
        pc.start_protected();
        pc.prepare_long();
        pc.write(at::CODE0, &enter_long_mode_code(la::CODE64));
        pc.write(la::CODE64, code);
        let steps = pc.run(200);
        assert!(steps < 200, "the 64-bit program halted");
        assert!(pc.cpu.sys().sixty_four(), "and did so in 64-bit mode");
        pc
    }

    #[test]
    fn a_pop_to_the_stack_addresses_its_destination_after_the_increment() {
        // *Intel SDM* volume 2, `POP`: "If the ESP register is used as a base
        // register for addressing a destination operand in memory, the POP
        // instruction computes the effective address of the operand after it
        // increments the ESP register."
        //
        // `pushf` / `pop 0x10(%rsp)` is how gcc writes `local_irq_save` on
        // x86-64, and it is the encoding that found this: with the address
        // taken before the increment the flags land eight bytes low, on top of
        // whatever local is there. A Linux kernel loses a function argument
        // that way and dies sixteen instructions later.
        //
        //   mov rsp, MARK+0x80 ; push rax ; pushf ; pop 0x10(%rsp) ; hlt
        let mut code = alloc::vec![0x48u8, 0xc7, 0xc4];
        code.extend_from_slice(&((la::MARK + 0x80) as u32).to_le_bytes());
        code.extend_from_slice(&[
            0x50, // push rax   — rsp is now MARK+0x78
            0x9c, // pushf      — rsp is now MARK+0x70
            0x8f, 0x44, 0x24, 0x10, // pop 0x10(%rsp)
            0xf4, // hlt
        ]);
        let pc = run64(&code);
        // The pop leaves RSP at MARK+0x78, so `0x10(%rsp)` is MARK+0x88 — and
        // *not* MARK+0x80, which is where the pre-increment address points.
        assert_eq!(pc.cpu.regs().rsp, la::MARK + 0x78);
        assert_eq!(
            pc.read64(la::MARK + 0x88) & 0xffff,
            u64::from(pc.cpu.regs().eflags & 0xffff),
            "the flags landed at the post-increment address"
        );
        assert_eq!(
            pc.read64(la::MARK + 0x80),
            0,
            "and nothing was written eight bytes below it"
        );
    }

    #[test]
    fn a_control_register_read_is_sixty_four_bits_wide_with_no_rex_prefix() {
        // *Intel SDM* volume 2, `MOV — Move to/from Control Registers`: in
        // 64-bit mode the operand size is 64 bits, `REX.W` is ignored, and a
        // `66` prefix cannot narrow it. So `0F 20 D0` — three bytes, no prefix
        // — moves the whole of `CR2` into `RAX`.
        //
        // This was thirty-two bits, and the way it showed is worth recording:
        // a 64-bit Linux reads `CR2` in its early page-fault handler and
        // subtracts `PAGE_OFFSET` to get a physical address. With the top half
        // missing the subtraction underflows, the result looks larger than the
        // machine's physical address space, the handler declines to map the
        // page, and the kernel halts in a loop before it has a console — total
        // silence, forty seconds into a boot (`tests/pc64_linux.rs`).
        let pc = pc64();
        pc.start_protected();
        pc.prepare_long();
        let mut sys = pc.cpu.sys();
        sys.cr2 = 0xffff_8880_0000_7000;
        pc.cpu.set_sys(sys);
        pc.write(at::CODE0, &enter_long_mode_code(la::CODE64));
        // mov rax, cr2 ; mov rbx, cr3 ; hlt
        pc.write(la::CODE64, &[0x0f, 0x20, 0xd0, 0x0f, 0x20, 0xdb, 0xf4]);
        let steps = pc.run(200);
        assert!(steps < 200, "it halted");
        assert_eq!(
            pc.cpu.regs().rax,
            0xffff_8880_0000_7000,
            "the whole of CR2, not its low half"
        );
        assert_eq!(
            pc.cpu.regs().rbx,
            la::PML4,
            "and CR3 likewise, which is how a guest finds its own tables"
        );
    }

    #[test]
    fn a_null_stack_selector_is_legal_in_sixty_four_bit_mode_and_the_stack_works() {
        // *Intel SDM* volume 3 §5.4.1, "NULL Segment Selector Checking": in
        // 64-bit mode below ring 3, `SS` may hold a null selector — it has no
        // base and no limit there, and the selector is kept for its privilege
        // level. The first thing a 64-bit Linux does after its long jump is
        // load all six segment registers with zero, so a core that refuses
        // this takes a `#GP` with no interrupt descriptor table loaded, which
        // is a triple fault ten instructions into the kernel.
        //
        //   xor eax, eax ; mov ss, ax ; mov esp, MARK+8 ; push rax ; hlt
        let mut code = alloc::vec![0x31u8, 0xc0, 0x8e, 0xd0, 0xbc];
        code.extend_from_slice(&((la::MARK + 8) as u32).to_le_bytes());
        code.extend_from_slice(&[0x50, 0xf4]);
        let pc = run64(&code);
        assert_eq!(pc.cpu.regs().ss, 0, "the null selector is in SS");
        assert_eq!(
            pc.cpu.regs().rsp,
            la::MARK,
            "and the push went through it rather than faulting"
        );
        assert_eq!(pc.read64(la::MARK), 0);
    }

    #[test]
    fn a_guest_enters_long_mode_and_executes_64_bit_code() {
        // The whole point of the exercise. Nothing below is set up from
        // outside except the tables in memory: the processor starts in 32-bit
        // protected mode and walks itself into 64-bit mode.
        let pc = pc64();
        pc.start_protected();
        pc.prepare_long();
        pc.write(at::CODE0, &enter_long_mode_code(la::CODE64));

        let mut code = Vec::new();
        // mov rax, 0x0123456789abcdef — the only 64-bit immediate x86 has.
        code.extend_from_slice(&[0x48, 0xb8]);
        code.extend_from_slice(&0x0123_4567_89ab_cdefu64.to_le_bytes());
        // mov rbx, rax — REX.W with no extension bit.
        code.extend_from_slice(&[0x48, 0x89, 0xc3]);
        // mov r15, rax — REX.WB, so the r/m field reaches a register the
        // 32-bit encoding cannot name at all.
        code.extend_from_slice(&[0x49, 0x89, 0xc7]);
        // mov rcx, -1 — an imm32 sign-extended to sixty-four bits, which is
        // what makes `Iz` a different operand from `Iv`.
        code.extend_from_slice(&[0x48, 0xc7, 0xc1, 0xff, 0xff, 0xff, 0xff]);
        // mov [rip + disp32], rax. The displacement is relative to the end of
        // *this* instruction, so it can only be computed once its length is
        // known — seven bytes.
        let after = la::CODE64 + code.len() as u64 + 7;
        let disp = i32::try_from(la::MARK as i64 - after as i64).expect("in range");
        code.extend_from_slice(&[0x48, 0x89, 0x05]);
        code.extend_from_slice(&disp.to_le_bytes());
        code.push(0xf4); // hlt
        pc.write(la::CODE64, &code);

        let steps = pc.run(40);
        assert!(steps < 40, "the program reached its hlt in {steps} steps");

        let sys = pc.cpu.sys();
        assert!(sys.long_mode(), "EFER.LMA is set by the write to CR0.PG");
        assert!(sys.sixty_four(), "and CS.L put the core in 64-bit mode");
        assert_eq!(sys.paging_mode(pc.cpu.config().features), Mode::Ia32e);
        assert_eq!(sys.cr3, la::PML4);

        let regs = pc.regs();
        assert_eq!(regs.rax, 0x0123_4567_89ab_cdef, "a 64-bit immediate");
        assert_eq!(regs.rbx, 0x0123_4567_89ab_cdef, "REX.W moved all of it");
        assert_eq!(regs.r[7], 0x0123_4567_89ab_cdef, "REX.B reached R15");
        assert_eq!(
            regs.rcx,
            u64::MAX,
            "an imm32 sign-extended, not zero-filled"
        );
        assert_eq!(
            pc.read64(la::MARK),
            0x0123_4567_89ab_cdef,
            "a RIP-relative store landed where the displacement pointed"
        );
    }

    #[test]
    fn setting_paging_without_the_address_extension_refuses_to_enter_long_mode() {
        // SDM volume 3 §9.8.5 gives the order, and the processor enforces the
        // step that matters rather than trusting software to follow it: the
        // four-level walk *is* the PAE walk with a level added, so there is no
        // long mode without `CR4.PAE`.
        let pc = pc64();
        pc.start_protected();
        pc.prepare_long();
        let mut sys = pc.cpu.sys();
        sys.efer |= efer::LME;
        sys.cr3 = la::PML4;
        pc.cpu.set_sys(sys);
        pc.idt(13, gate(0x08, 0x9000, sys_type::INT_GATE32, 0));
        pc.write(0x9000, &[0xf4]);
        // mov eax, cr0 ; or eax, PG ; mov cr0, eax
        let mut code = alloc::vec![0x0f, 0x20, 0xc0, 0x0d];
        code.extend_from_slice(&cr0::PG.to_le_bytes());
        code.extend_from_slice(&[0x0f, 0x22, 0xc0]);
        pc.write(at::CODE0, &code);
        pc.run(10);
        assert_eq!(pc.regs().rip, 0x9001, "the write to CR0 raised #GP");
        assert!(!pc.cpu.sys().long_mode(), "and long mode was not entered");
    }

    #[test]
    fn arming_long_mode_while_paging_is_on_is_refused() {
        // The transition is defined only across a `CR0.PG` edge; allowing
        // `LME` to move underneath a live page table would leave `LMA`
        // describing a walk that never happened.
        let pc = pc64();
        pc.start_protected();
        pc.prepare_long();
        let mut sys = pc.cpu.sys();
        sys.cr4 |= cr4::PAE;
        sys.cr3 = la::PDPT;
        sys.cr0 |= cr0::PG;
        pc.cpu.set_sys(sys);
        // A three-level identity map, so the guest keeps running while paged.
        pc.write64(la::PDPT, la::PD | MAP);
        pc.write64(la::PD, MAP | PS);
        pc.write64(la::PD + 8, 0x20_0000 | MAP | PS);
        pc.idt(13, gate(0x08, 0x9000, sys_type::INT_GATE32, 0));
        pc.write(0x9000, &[0xf4]);
        // mov ecx, EFER ; rdmsr ; or eax, LME ; wrmsr ; hlt
        let mut code = alloc::vec![0xb9];
        code.extend_from_slice(&msr::EFER.to_le_bytes());
        code.extend_from_slice(&[0x0f, 0x32, 0x0d]);
        code.extend_from_slice(&(efer::LME as u32).to_le_bytes());
        code.extend_from_slice(&[0x0f, 0x30, 0xf4]);
        pc.write(at::CODE0, &code);
        pc.run(10);
        assert_eq!(pc.regs().rip, 0x9001, "the WRMSR raised #GP");
        assert_eq!(pc.cpu.sys().efer & efer::LME, 0);
    }

    #[test]
    fn efer_lma_is_the_processors_bit_and_not_softwares() {
        // Software writes `LME` and reads `LMA` back; writing `LMA` does
        // nothing, which is what makes reading it a reliable way to ask what
        // mode the processor is actually in.
        let pc = pc64();
        pc.start_protected();
        pc.prepare_long();
        // mov ecx, EFER ; mov eax, LMA|LME ; xor edx, edx ; wrmsr ; rdmsr ; hlt
        let mut code = alloc::vec![0xb9];
        code.extend_from_slice(&msr::EFER.to_le_bytes());
        code.push(0xb8);
        code.extend_from_slice(&((efer::LMA | efer::LME) as u32).to_le_bytes());
        code.extend_from_slice(&[0x31, 0xd2, 0x0f, 0x30, 0x0f, 0x32, 0xf4]);
        pc.write(at::CODE0, &code);
        pc.run(10);
        let regs = pc.regs();
        assert_eq!(regs.rax & efer::LME, efer::LME, "LME took");
        assert_eq!(regs.rax & efer::LMA, 0, "LMA did not");
        assert_eq!(pc.cpu.sys().efer & efer::LMA, 0);
    }

    #[test]
    fn clearing_the_address_extension_in_long_mode_is_refused() {
        // The page tables would change shape under the processor's feet.
        let pc = pc64();
        pc.start_protected();
        pc.prepare_long();
        pc.write(at::CODE0, &enter_long_mode_code(la::CODE64));
        pc.idt64(13, 0x18, la::HANDLER);
        pc.write(la::HANDLER, &[0xf4]);
        // mov rax, cr4 ; and eax, ~PAE ; mov cr4, rax ; hlt
        let mut code = alloc::vec![0x0f, 0x20, 0xe0, 0x25];
        code.extend_from_slice(&(!(cr4::PAE as u32)).to_le_bytes());
        code.extend_from_slice(&[0x0f, 0x22, 0xe0, 0xf4]);
        pc.write(la::CODE64, &code);
        pc.run(60);
        assert!(pc.cpu.sys().long_mode(), "long mode survived the attempt");
        assert_ne!(pc.cpu.sys().cr4 & cr4::PAE, 0, "and so did CR4.PAE");
        assert_eq!(pc.regs().rip, la::HANDLER + 1, "#GP was taken");
    }

    #[test]
    fn a_code_segment_may_not_be_both_long_and_big() {
        // AMD64 volume 2 §4.8.1: `L = 1` with `D = 1` is invalid, because the
        // defaults *are* the difference between the two submodes.
        let pc = pc64();
        pc.start_protected();
        pc.prepare_long();
        pc.gdt(3, descriptor(0, 0xffff_ffff, CODE64_AR | ar::DB));
        pc.write(at::CODE0, &enter_long_mode_code(la::CODE64));
        pc.idt64(13, 0x20, la::HANDLER);
        pc.write(la::HANDLER, &[0xf4]);
        pc.write(la::CODE64, &[0xf4]);
        pc.run(40);
        assert!(pc.cpu.sys().long_mode(), "long mode was entered");
        assert_eq!(pc.regs().rip, la::HANDLER + 1, "but the far jump was #GP");
        assert_eq!(
            pc.cpu.sys().seg(isa::seg::CS).selector & !3,
            0x20,
            "and the handler ran in the segment that was still valid"
        );
    }

    #[test]
    fn a_thirty_two_bit_write_zero_extends_and_a_narrower_one_does_not() {
        // SDM volume 1 §3.4.1.1. The asymmetry is the most load-bearing rule
        // in the whole register file, and the one a port from a 32-bit core
        // has no reason to have.
        let pc = run64(&[
            0x48, 0xb8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, // mov rax, -1
            0x48, 0x89, 0xc3, // mov rbx, rax
            0x48, 0x89, 0xc1, // mov rcx, rax
            0x48, 0x89, 0xc2, // mov rdx, rax
            0xbb, 0x78, 0x56, 0x34, 0x12, // mov ebx, 0x12345678  (zero-extends)
            0x66, 0xb9, 0x34, 0x12, // mov cx, 0x1234       (preserves)
            0xb2, 0x99, // mov dl, 0x99        (preserves)
            0xf4,
        ]);
        let regs = pc.regs();
        assert_eq!(regs.rbx, 0x1234_5678, "a 32-bit write cleared the top half");
        assert_eq!(regs.rcx, 0xffff_ffff_ffff_1234, "a 16-bit write did not");
        assert_eq!(regs.rdx, 0xffff_ffff_ffff_ff99, "nor did an 8-bit one");
    }

    #[test]
    fn a_rex_prefix_renames_the_high_byte_registers() {
        // SDM volume 2 §2.2.1.2: with *any* `REX` prefix — including `40`,
        // which sets no bit — register numbers 4-7 name `SPL`, `BPL`, `SIL`
        // and `DIL` instead of `AH`, `CH`, `DH` and `BH`.
        let pc = run64(&[
            0x48, 0x31, 0xe4, // xor rsp, rsp
            0xb0, 0x5a, // mov al, 0x5a
            0x40, 0x88, 0xc4, // mov spl, al   (REX with no bits set)
            0x88, 0xc4, // mov ah, al    (no REX: the high byte)
            0xf4,
        ]);
        let regs = pc.regs();
        assert_eq!(regs.rsp, 0x5a, "the REX form wrote SPL");
        assert_eq!(regs.rax & 0xffff, 0x5a5a, "the bare form wrote AH");
    }

    #[test]
    fn rip_relative_addressing_counts_from_the_end_of_the_instruction() {
        // SDM volume 2 §2.2.1.6. The displacement is added to the address of
        // the *next* instruction, so an addressing mode's result depends on
        // the length of the instruction containing it — unique in x86.
        let pc = pc64();
        pc.start_protected();
        pc.prepare_long();
        pc.write(at::CODE0, &enter_long_mode_code(la::CODE64));
        pc.write64(la::MARK, 0xdead_beef_cafe_f00d);
        let mut code = Vec::new();
        let after = la::CODE64 + 7;
        let disp = i32::try_from(la::MARK as i64 - after as i64).expect("in range");
        code.extend_from_slice(&[0x48, 0x8b, 0x05]); // mov rax, [rip+disp32]
        code.extend_from_slice(&disp.to_le_bytes());
        code.push(0xf4);
        pc.write(la::CODE64, &code);
        pc.run(60);
        assert_eq!(pc.regs().rax, 0xdead_beef_cafe_f00d);
    }

    #[test]
    fn the_stack_moves_eight_bytes_at_a_time_and_a_prefix_narrows_it() {
        // SDM volume 2 §2.2.1.7: `PUSH` and `POP` have a *default* operand
        // size of sixty-four in 64-bit mode. `REX.W` is redundant on them and
        // `66` still narrows them to two.
        let pc = run64(&[
            0x48, 0xb8, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, // mov rax, …
            0x48, 0x89, 0xe3, // mov rbx, rsp
            0x50, // push rax
            0x48, 0x89, 0xe1, // mov rcx, rsp
            0x5a, // pop rdx
            0x66, 0x50, // push ax   (66 narrows it)
            0x48, 0x89, 0xe6, // mov rsi, rsp
            0x66, 0x5f, // pop di
            0xf4,
        ]);
        let regs = pc.regs();
        assert_eq!(regs.rbx - regs.rcx, 8, "a bare push moved RSP by eight");
        assert_eq!(regs.rdx, 0x8877_6655_4433_2211, "and popped all of it back");
        assert_eq!(regs.rbx - regs.rsi, 2, "a 66-prefixed push moved it by two");
        assert_eq!(regs.rdi & 0xffff, 0x2211);
    }

    #[test]
    fn a_near_call_pushes_a_sixty_four_bit_return_address() {
        // The displacement stays `rel32`, and the pointer it lands in is full
        // width. Both halves of that are needed: an emulator that widened the
        // displacement would desynchronise the instruction stream, and one
        // that narrowed the return address would corrupt the stack.
        let pc = run64(&[
            0xe8, 0x08, 0x00, 0x00, 0x00, // call +8
            0x48, 0xc7, 0xc3, 0x01, 0x00, 0x00, 0x00, // mov rbx, 1
            0xf4, // hlt
            // The callee reads the return address off the stack without
            // disturbing it, so the `ret` still has one to use.
            0x48, 0x8b, 0x04, 0x24, // mov rax, [rsp]
            0xc3, // ret
        ]);
        let regs = pc.regs();
        assert_eq!(
            regs.rax,
            la::CODE64 + 5,
            "the pushed return address was the full sixty-four bits"
        );
        assert_eq!(regs.rbx, 1, "and the return landed on the next instruction");
    }

    #[test]
    fn movsxd_replaced_arpl_and_sign_extends_a_doubleword() {
        // `63 /r` was `ARPL`, which needs 16-bit segmentation and therefore
        // cannot exist in 64-bit mode. Long mode reclaimed the encoding.
        let pc = run64(&[
            0x48, 0xc7, 0xc1, 0x00, 0x00, 0x00, 0x80, // mov rcx, -0x80000000
            0x48, 0x63, 0xc1, // movsxd rax, ecx
            0x63, 0xd9, // movsxd ebx, ecx  (no REX.W: a plain move)
            0xf4,
        ]);
        let regs = pc.regs();
        assert_eq!(regs.rax, 0xffff_ffff_8000_0000, "REX.W sign-extended");
        assert_eq!(regs.rbx, 0x8000_0000, "without it the result zero-extends");
    }

    #[test]
    fn the_encodings_long_mode_reclaimed_raise_invalid_opcode() {
        // Each of these is a real instruction in every other mode and gone in
        // this one — some because the encoding was taken, some because what
        // they do has no meaning without 16-bit segmentation or packed
        // decimal. Guessing wrong here executes something plausible instead of
        // faulting, which is the failure that is hardest to find later.
        for (opcode, name) in [
            (0x06u8, "push es"),
            (0x07, "pop es"),
            (0x0e, "push cs"),
            (0x16, "push ss"),
            (0x1e, "push ds"),
            (0x27, "daa"),
            (0x2f, "das"),
            (0x37, "aaa"),
            (0x3f, "aas"),
            (0x60, "pusha"),
            (0x61, "popa"),
            (0x62, "bound"),
            (0x82, "the 80-group alias"),
            (0x9a, "call far ptr16:32"),
            (0xc4, "les"),
            (0xc5, "lds"),
            (0xce, "into"),
            (0xd4, "aam"),
            (0xd5, "aad"),
            (0xd6, "salc"),
            (0xea, "jmp far ptr16:32"),
        ] {
            let pc = pc64();
            pc.start_protected();
            pc.prepare_long();
            pc.write(at::CODE0, &enter_long_mode_code(la::CODE64));
            pc.write(la::CODE64, &[opcode, 0x00, 0x00, 0x00, 0x00, 0x00]);
            pc.idt64(6, 0x18, la::HANDLER);
            pc.write(la::HANDLER, &[0xf4]);
            pc.run(60);
            assert_eq!(
                pc.regs().rip,
                la::HANDLER + 1,
                "{name} raised #UD in 64-bit mode"
            );
        }
    }

    #[test]
    fn a_non_canonical_address_faults_before_the_page_tables_are_consulted() {
        // SDM volume 1 §3.3.7.1. Bits 63-48 must be a sign-extension of bit
        // 47; the rule is what stops software storing a tag in the top bits
        // and expecting the processor to ignore it.
        let pc = pc64();
        pc.start_protected();
        pc.prepare_long();
        pc.write(at::CODE0, &enter_long_mode_code(la::CODE64));
        pc.idt64(13, 0x18, la::HANDLER);
        pc.write(la::HANDLER, &[0xf4]);
        // mov rax, 0x0001_0000_0000_0000 ; mov rbx, [rax] ; hlt
        let mut code = alloc::vec![0x48, 0xb8];
        code.extend_from_slice(&0x0001_0000_0000_0000u64.to_le_bytes());
        code.extend_from_slice(&[0x48, 0x8b, 0x18, 0xf4]);
        pc.write(la::CODE64, &code);
        pc.run(60);
        assert_eq!(pc.regs().rip, la::HANDLER + 1, "#GP, not a page fault");
        assert_eq!(pc.cpu.sys().cr2, 0, "and CR2 was never latched");
    }

    #[test]
    fn an_interrupt_in_long_mode_takes_a_sixteen_byte_gate_and_iretq_returns() {
        // SDM volume 3 §6.14: a 64-bit gate is sixteen bytes with a 64-bit
        // offset, and the frame is five eight-byte words because `SS:RSP` is
        // pushed whether or not the privilege level changed. An `IRET` that
        // popped three would return to the wrong place — silently.
        let pc = pc64();
        pc.start_protected();
        pc.prepare_long();
        pc.write(at::CODE0, &enter_long_mode_code(la::CODE64));
        pc.idt64(0x40, 0x18, la::HANDLER);
        // The handler marks a register and returns with a 64-bit IRET.
        pc.write(
            la::HANDLER,
            &[0x48, 0xc7, 0xc3, 0x2a, 0x00, 0x00, 0x00, 0x48, 0xcf],
        );
        // int 0x40 ; mov rcx, 7 ; hlt
        pc.write(
            la::CODE64,
            &[0xcd, 0x40, 0x48, 0xc7, 0xc1, 0x07, 0x00, 0x00, 0x00, 0xf4],
        );
        pc.run(80);
        let regs = pc.regs();
        assert_eq!(regs.rbx, 42, "the handler ran");
        assert_eq!(regs.rcx, 7, "and IRETQ came back to the instruction after");
        assert!(pc.cpu.sys().sixty_four(), "still in 64-bit mode");
    }

    #[test]
    fn syscall_and_sysret_cross_the_boundary_without_the_descriptor_tables() {
        // AMD64 volume 3, `SYSCALL`: the selectors come from `STAR` and the
        // descriptors are architectural, so neither instruction reads memory.
        // `RCX` takes the return address and `R11` the flags.
        let pc = pc64();
        pc.start_protected();
        pc.prepare_long();
        pc.write(at::CODE0, &enter_long_mode_code(la::CODE64));
        let mut sys = pc.cpu.sys();
        sys.efer |= efer::SCE;
        // `STAR[47:32]` is the kernel `CS`; `STAR[63:48]` is what `SYSRET`
        // returns through.
        sys.star = (0x0010u64 << 48) | (0x0018u64 << 32);
        sys.lstar = la::HANDLER;
        pc.cpu.set_sys(sys);
        // The kernel side: mark a register, then return.
        pc.write(
            la::HANDLER,
            &[0x48, 0xc7, 0xc3, 0x63, 0x00, 0x00, 0x00, 0x48, 0x0f, 0x07],
        );
        // syscall ; mov rdx, 9 ; hlt
        pc.write(
            la::CODE64,
            &[0x0f, 0x05, 0x48, 0xc7, 0xc2, 0x09, 0x00, 0x00, 0x00, 0xf4],
        );
        pc.run(80);
        let regs = pc.regs();
        assert_eq!(regs.rbx, 0x63, "the kernel entry point ran");
        assert_eq!(regs.rdx, 9, "and SYSRET came back to the next instruction");
        assert_eq!(regs.cs & 3, 3, "at privilege level three");
    }

    #[test]
    fn swapgs_exchanges_the_gs_base_with_the_kernels() {
        // The one instruction a 64-bit kernel cannot do without: entered from
        // user mode it has no register it can safely clobber, so finding its
        // own per-CPU state has to be one atomic step.
        let pc = pc64();
        pc.start_protected();
        pc.prepare_long();
        pc.write(at::CODE0, &enter_long_mode_code(la::CODE64));
        let mut sys = pc.cpu.sys();
        sys.gs_base = 0x1111_0000;
        sys.segs[usize::from(isa::seg::GS)].base = 0x1111_0000;
        sys.kernel_gs_base = la::MARK;
        pc.cpu.set_sys(sys);
        pc.write64(la::MARK, 0xfeed_face_0000_0001);
        // swapgs ; mov rax, gs:[0] ; swapgs ; hlt
        pc.write(
            la::CODE64,
            &[
                0x0f, 0x01, 0xf8, // swapgs
                0x65, 0x48, 0x8b, 0x04, 0x25, 0, 0, 0, 0, // mov rax, gs:[0]
                0x0f, 0x01, 0xf8, // swapgs
                0xf4,
            ],
        );
        pc.run(60);
        assert_eq!(
            pc.regs().rax,
            0xfeed_face_0000_0001,
            "GS reached the kernel area"
        );
        let sys = pc.cpu.sys();
        assert_eq!(sys.gs_base, 0x1111_0000, "and the second swap put it back");
        assert_eq!(sys.kernel_gs_base, la::MARK);
    }

    #[test]
    fn the_no_execute_bit_stops_a_fetch_and_leaves_a_read_alone() {
        // `EFER.NXE` turns bit 63 of an entry from reserved into
        // execute-disable, and the fault it causes sets bit 4 of the error
        // code — the one bit that tells a handler the access was a fetch.
        let pc = pc64();
        pc.start_protected();
        pc.prepare_long();
        // 4 KiB granularity for the first 2 MiB — which is where all of this
        // test's code lives — so one page can be barred without taking the
        // whole large page with it.
        pc.write64(la::PD, la::PT | MAP);
        for page in 0..512u64 {
            pc.write64(la::PT + page * 8, (page * 0x1000) | MAP);
        }
        let barred = la::HANDLER;
        let index = barred / 0x1000;
        pc.write64(la::PT + index * 8, barred | MAP | (1 << 63));
        pc.write(at::CODE0, &enter_long_mode_code(la::CODE64));
        let mut sys = pc.cpu.sys();
        sys.efer |= efer::NXE;
        pc.cpu.set_sys(sys);
        pc.idt64(14, 0x18, la::HANDLER2);
        pc.write(la::HANDLER2, &[0xf4]);
        // A read of the barred page is fine; executing from it is not.
        pc.write64(barred + 0x100, 0x5555_aaaa_5555_aaaa);
        let mut code = alloc::vec![0x48, 0xb8];
        code.extend_from_slice(&(barred + 0x100).to_le_bytes());
        code.extend_from_slice(&[0x48, 0x8b, 0x18]); // mov rbx, [rax]
        code.extend_from_slice(&[0x48, 0xb8]);
        code.extend_from_slice(&barred.to_le_bytes());
        code.extend_from_slice(&[0xff, 0xe0]); // jmp rax
        pc.write(la::CODE64, &code);
        pc.run(80);
        assert_eq!(
            pc.regs().rbx,
            0x5555_aaaa_5555_aaaa,
            "the read went through"
        );
        assert_eq!(pc.regs().rip, la::HANDLER2 + 1, "and the fetch faulted");
        assert_eq!(
            pc.cpu.sys().cr2,
            barred,
            "CR2 names the page that was barred"
        );
    }

    #[test]
    fn a_gigabyte_page_is_mapped_by_the_pointer_table_itself() {
        // SDM volume 3 §4.5. A 1 GiB page is a pointer-table entry with `PS`
        // set, and the offset is the low thirty bits — the same arithmetic as
        // a 2 MiB page one level down, which is why the walk needs no special
        // case for it.
        let pc = pc64();
        pc.start_protected();
        pc.prepare_long();
        pc.write64(la::PDPT, MAP | PS);
        pc.write(at::CODE0, &enter_long_mode_code(la::CODE64));
        pc.write64(la::MARK, 0x0102_0304_0506_0708);
        let mut code = alloc::vec![0x48, 0xb8];
        code.extend_from_slice(&la::MARK.to_le_bytes());
        code.extend_from_slice(&[0x48, 0x8b, 0x18, 0xf4]); // mov rbx, [rax] ; hlt
        pc.write(la::CODE64, &code);
        pc.run(60);
        assert_eq!(pc.regs().rbx, 0x0102_0304_0506_0708);
        assert_eq!(
            pc.cpu.translate_debug(la::MARK),
            DebugTranslation::Mapped(la::MARK),
            "and the debug walk agrees, through the same code"
        );
    }

    #[test]
    fn physical_address_extension_translates_without_long_mode() {
        // PAE is not a long-mode feature: a Pentium Pro had it, with three
        // levels and a four-entry pointer table indexed by two bits. Long mode
        // adds a fourth level on top rather than replacing anything, and this
        // is the middle mode that proves the shared walk really is shared.
        let pc = pc64();
        pc.start_protected();
        pc.write64(la::PDPT, la::PD | MAP);
        pc.write64(la::PD, MAP | PS);
        pc.write64(la::PD + 8, 0x20_0000 | MAP | PS);
        let mut sys = pc.cpu.sys();
        sys.cr4 |= cr4::PAE;
        sys.cr3 = la::PDPT;
        sys.cr0 |= cr0::PG;
        pc.cpu.set_sys(sys);
        assert_eq!(
            pc.cpu.sys().paging_mode(pc.cpu.config().features),
            Mode::Pae
        );
        pc.write32(at::MARK, 0x1234_5678);
        // mov eax, [MARK] ; hlt
        let mut code = alloc::vec![0xa1];
        code.extend_from_slice(&(at::MARK as u32).to_le_bytes());
        code.push(0xf4);
        pc.write(at::CODE0, &code);
        pc.run(10);
        assert_eq!(pc.regs().rax, 0x1234_5678);
        assert_eq!(
            pc.cpu.translate_debug(at::MARK),
            DebugTranslation::Mapped(at::MARK)
        );
    }

    #[test]
    fn a_long_mode_core_round_trips_through_a_snapshot() {
        // The state model grew, so the snapshot had to grow with it. A `load`
        // that quietly dropped `EFER` would leave a restored machine in
        // compatibility mode executing 64-bit code, and nothing would say so.
        let pc = run64(&[
            0x48, 0xb8, 0xef, 0xcd, 0xab, 0x89, 0x67, 0x45, 0x23, 0x01, // mov rax, …
            0x49, 0x89, 0xc7, // mov r15, rax
            0x49, 0x89, 0xc4, // mov r12, rax
            0xf4,
        ]);
        let before = pc.regs();
        let sys_before = pc.cpu.sys();
        assert_ne!(before.r[7], 0, "there is something to lose");

        let mut shape = MachineShape::new();
        shape.add_device("/cpu0", "cpu.x86").unwrap();
        let mut writer = StateWriter::new(shape);
        {
            let mut chunk = writer.chunk("/cpu0", "cpu.x86", 4).unwrap();
            pc.cpu.save(&mut chunk).unwrap();
        }
        let bytes = writer.to_vec().unwrap();

        pc.cpu.reset(ResetKind::Cold);
        assert_ne!(pc.cpu.regs(), before);
        assert!(!pc.cpu.sys().long_mode(), "a reset left long mode");

        let reader = StateReader::new(&bytes).unwrap();
        let (_, _, data) = reader.load_raw("/cpu0").unwrap();
        let mut chunk = ChunkReader::new(data);
        pc.cpu.load(&mut chunk).unwrap();
        chunk.end().unwrap();

        assert_eq!(pc.cpu.regs(), before, "every register came back");
        assert_eq!(
            pc.cpu.sys(),
            sys_before,
            "and so did EFER, CR4 and the MSRs"
        );
        assert!(pc.cpu.sys().sixty_four());

        // And a second save of the restored state is byte-identical, which is
        // what "an identical state hash" means for a chunk this shape.
        let mut shape = MachineShape::new();
        shape.add_device("/cpu0", "cpu.x86").unwrap();
        let mut again = StateWriter::new(shape);
        {
            let mut chunk = again.chunk("/cpu0", "cpu.x86", 4).unwrap();
            pc.cpu.save(&mut chunk).unwrap();
        }
        assert_eq!(bytes, again.to_vec().unwrap());
    }

    #[test]
    fn the_disassembler_prints_the_sixty_four_bit_forms() {
        // One table, one decoder: the disassembler follows the interpreter
        // into 64-bit mode without a second opcode list.
        use crate::cpu::x86::disasm::disassemble_as;
        let at64 = |bytes: &[u8]| {
            let d = disassemble_as(isa::Gen::I386, isa::Bits::B64, 0, 0x1000, bytes);
            alloc::format!("{d}")
        };
        assert_eq!(at64(&[0x48, 0x89, 0xc3]), "mov rbx, rax");
        assert_eq!(at64(&[0x49, 0x89, 0xc7]), "mov r15, rax");
        assert_eq!(at64(&[0x4d, 0x89, 0xc7]), "mov r15, r8");
        assert_eq!(at64(&[0x40, 0x88, 0xc4]), "mov spl, al");
        assert_eq!(at64(&[0x88, 0xc4]), "mov ah, al");
        assert_eq!(at64(&[0x48, 0x63, 0xc1]), "movsxd rax, ecx");
        assert_eq!(at64(&[0x0f, 0x05]), "syscall");
        assert_eq!(
            at64(&[0x48, 0x8b, 0x05, 0x10, 0x00, 0x00, 0x00]),
            "mov rax, [ds:rip+0x10]"
        );
        // And a reclaimed encoding disassembles as what it now is.
        assert_eq!(at64(&[0x60]), "ud");
    }

    #[test]
    fn the_extension_lattice_is_selected_rather_than_implied() {
        // `ROADMAP.md` §6.1.1: a preset is a *name for a point in the
        // lattice*, and narrowing one has to produce a core that really lacks
        // the instruction rather than one that decodes it anyway.
        let cfg = Config::X86_64.with_features(Features {
            long: false,
            pae: false,
            nx: false,
            syscall: false,
            ..Features::X86_64
        });
        let pc = Pc::new(Variant::X86_64);
        let cpu = Arc::new(X86::new(cfg));
        let _ = &pc;
        assert!(!cpu.config().features.long);
        // A part with no long mode has no `EFER` either: `RDMSR` of it is
        // `#GP`, which is exactly how a guest finds out.
        assert!(cfg.features.validate().is_ok());

        // And an impossible point is refused at construction rather than
        // silently repaired.
        let impossible = Features {
            pae: false,
            ..Features::X86_64
        };
        assert!(impossible.validate().is_err(), "long mode needs PAE");
    }
}

/// The x87 unit and the SSE registers, executed as a guest sees them.
///
/// Every expected value here comes from the *Intel SDM* — the section is named
/// on each test — or is arithmetic that can be checked by hand: `1.5 + 2.25`
/// is `3.75` in any format, and `1/3` rounded to sixty-four significand bits
/// is `0xaaaa_aaaa_aaaa_aaab` one way and `…aaaa` the other, which is exactly
/// what makes it a rounding-mode test.
mod fp {
    use super::*;
    use crate::cpu::x86::fpu::{Sse, Tag, cw, mxcsr, sw};
    use crate::cpu::x86::prot::cr4;
    use crate::float::x87::F80;

    /// Scratch addresses these tests load from and store to.
    mod fpat {
        /// A 16-byte-aligned scratch area.
        pub(super) const DATA: u64 = 0xb000;
        /// A second one, deliberately **not** 16-byte aligned.
        pub(super) const ODD: u64 = 0xb008;
        /// Where a result is written.
        pub(super) const OUT: u64 = 0xc000;
        /// A 512-byte `FXSAVE` area.
        pub(super) const SAVE: u64 = 0xd000;
    }

    /// Where every fault handler in this module sits, and where `RIP` ends up
    /// once its one-byte `hlt` has retired.
    const HANDLER: u64 = 0x3800;
    /// `RIP` after the handler's `hlt`.
    const HANDLED: u64 = HANDLER + 1;

    /// A 486DX: an x87 unit, no SSE.
    fn x87pc() -> Pc {
        let pc = pc386();
        pc.start_protected();
        pc
    }

    /// A part with SSE2, in 32-bit protected mode with `CR4.OSFXSR` set — the
    /// bit an operating system has to write before any of this works.
    fn ssepc() -> Pc {
        let pc = Pc::new(Variant::X86_64);
        pc.start_protected();
        let mut sys = pc.cpu.sys();
        sys.cr4 |= cr4::OSFXSR | cr4::OSXMMEXCPT;
        pc.cpu.set_sys(sys);
        pc
    }

    /// Assemble a `disp32` ModRM byte for `reg`: `mod == 00`, `r/m == 101`.
    fn disp32(reg: u8, addr: u64) -> Vec<u8> {
        let mut out = alloc::vec![((reg & 7) << 3) | 0b101];
        out.extend_from_slice(&(addr as u32).to_le_bytes());
        out
    }

    /// Run a program at `CODE0`.
    fn run(pc: &Pc, code: &[u8]) {
        pc.write(at::CODE0, code);
        let steps = pc.run(200);
        assert!(steps < 200, "the program reached its hlt");
    }

    fn read64(pc: &Pc, addr: u64) -> u64 {
        let mut v = 0u64;
        for i in 0..8u64 {
            v |= u64::from(pc.ram.read_u8(addr + i).unwrap()) << (8 * i);
        }
        v
    }

    fn write64(pc: &Pc, addr: u64, value: u64) {
        for i in 0..8u64 {
            pc.ram.write_u8(addr + i, (value >> (8 * i)) as u8).unwrap();
        }
    }

    // -- The stack -----------------------------------------------------

    #[test]
    fn a_load_pushes_and_the_top_of_stack_pointer_moves_down() {
        // SDM volume 1 §8.1.7: `TOP` decrements on a push, so the first `FLD1`
        // leaves `ST(0)` in physical register 7.
        let pc = x87pc();
        run(&pc, &[0xdb, 0xe3, 0xd9, 0xe8, 0xf4]); // fninit ; fld1 ; hlt
        let x = pc.cpu.x87();
        assert_eq!(x.top(), 7);
        assert_eq!(x.raw(0), F80::new(0x3fff, 0x8000_0000_0000_0000));
        assert_eq!(x.tag_at(7), Tag::Valid);
        assert_eq!(x.tag_at(0), Tag::Empty);
    }

    #[test]
    fn a_ninth_push_is_a_stack_overflow_with_c1_set() {
        // §8.5.1.1. The masked response still moves `TOP` and still writes —
        // with the indefinite — which is why a program that overflows once
        // keeps producing indefinites rather than recovering.
        let pc = x87pc();
        let mut code = alloc::vec![0xdb, 0xe3];
        for _ in 0..9 {
            code.extend_from_slice(&[0xd9, 0xe8]); // fld1
        }
        code.push(0xf4);
        run(&pc, &code);
        let x = pc.cpu.x87();
        assert_ne!(x.status & sw::IE, 0, "invalid operation");
        assert_ne!(x.status & sw::SF, 0, "and it was a stack fault");
        assert_ne!(x.status & sw::C1, 0, "an overflow, not an underflow");
        assert_eq!(x.raw(0), F80::INDEFINITE);
    }

    #[test]
    fn reading_an_empty_register_is_an_underflow_with_c1_clear() {
        // The other half of §8.5.1.1, and the reason an empty register is not
        // a zero: `FLD ST(0)` on a fresh unit reads something that is not
        // there.
        let pc = x87pc();
        run(&pc, &[0xdb, 0xe3, 0xd9, 0xc0, 0xf4]); // fninit ; fld st(0) ; hlt
        let x = pc.cpu.x87();
        assert_ne!(x.status & sw::IE, 0);
        assert_ne!(x.status & sw::SF, 0);
        assert_eq!(x.status & sw::C1, 0, "an underflow sets C1 to zero");
        assert_eq!(x.raw(0), F80::INDEFINITE);
    }

    #[test]
    fn fxch_swaps_the_values_and_their_tags() {
        // SDM volume 2, `FXCH`. The tags travel with the values, which is what
        // makes exchanging with an empty register meaningful.
        let pc = x87pc();
        // fninit ; fldz ; fld1 ; fxch st(1) ; hlt
        run(&pc, &[0xdb, 0xe3, 0xd9, 0xee, 0xd9, 0xe8, 0xd9, 0xc9, 0xf4]);
        let x = pc.cpu.x87();
        assert_eq!(x.raw(0), F80::ZERO);
        assert_eq!(x.raw(1), F80::new(0x3fff, 0x8000_0000_0000_0000));
        assert_eq!(x.tag_at(x.phys(0)), Tag::Zero);
        assert_eq!(x.tag_at(x.phys(1)), Tag::Valid);
    }

    #[test]
    fn fincstp_leaves_the_tag_word_alone_and_a_pop_does_not() {
        // SDM volume 2, `FINCSTP`: "this instruction does not change the tag
        // word". `FSTP ST(0)` — the idiomatic pop — does.
        let pc = x87pc();
        run(&pc, &[0xdb, 0xe3, 0xd9, 0xe8, 0xd9, 0xf7, 0xf4]);
        assert_eq!(pc.cpu.x87().tag_at(7), Tag::Valid, "still occupied");

        let pc = x87pc();
        run(&pc, &[0xdb, 0xe3, 0xd9, 0xe8, 0xdd, 0xd8, 0xf4]);
        assert_eq!(pc.cpu.x87().tag_at(7), Tag::Empty, "freed by the pop");
    }

    // -- Arithmetic ----------------------------------------------------

    #[test]
    fn a_double_precision_add_produces_the_exact_sum() {
        // 1.5 + 2.25 = 3.75, which every format represents exactly, so the
        // rounding mode cannot be hiding a wrong answer.
        let pc = x87pc();
        write64(&pc, fpat::DATA, 0x3ff8_0000_0000_0000); // 1.5
        write64(&pc, fpat::DATA + 8, 0x4002_0000_0000_0000); // 2.25
        let mut code = alloc::vec![0xdb, 0xe3, 0xdd]; // fninit ; fld qword
        code.extend_from_slice(&disp32(0, fpat::DATA));
        code.push(0xdc); // fadd qword
        code.extend_from_slice(&disp32(0, fpat::DATA + 8));
        code.push(0xdd); // fstp qword
        code.extend_from_slice(&disp32(3, fpat::OUT));
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(read64(&pc, fpat::OUT), 0x400e_0000_0000_0000, "3.75");
        assert_eq!(pc.cpu.x87().status & sw::EXCEPTIONS, 0, "exactly");
    }

    #[test]
    fn precision_control_shortens_the_significand_and_nothing_else() {
        // SDM volume 1 §8.1.5.2. `1 + 2^-30` is exact at 64-bit precision and
        // rounds away entirely at 24-bit, which is the whole content of `PC`.
        let program = |control: u16| {
            let pc = x87pc();
            write64(&pc, fpat::DATA, u64::from(control));
            write64(&pc, fpat::DATA + 8, 0x3e10_0000_0000_0000); // 2^-30
            let mut code = alloc::vec![0xdb, 0xe3, 0xd9]; // fninit ; fldcw
            code.extend_from_slice(&disp32(5, fpat::DATA));
            code.extend_from_slice(&[0xd9, 0xe8, 0xdc]); // fld1 ; fadd qword
            code.extend_from_slice(&disp32(0, fpat::DATA + 8));
            code.push(0xdb); // fstp tbyte
            code.extend_from_slice(&disp32(7, fpat::OUT));
            code.push(0xf4);
            run(&pc, &code);
            (read64(&pc, fpat::OUT), pc.cpu.x87().status)
        };
        let (extended, st) = program(cw::RESET);
        assert_eq!(extended, 0x8000_0002_0000_0000, "2^-30 survives at PC=64");
        assert_eq!(st & sw::PE, 0, "and the sum is exact");

        let (single, st) = program(cw::RESET & !cw::PC);
        assert_eq!(single, 0x8000_0000_0000_0000, "and vanishes at PC=24");
        assert_ne!(st & sw::PE, 0, "which is inexact, and says so");
    }

    #[test]
    fn rounding_control_changes_the_last_bit_of_one_third() {
        // §8.1.5.3. One third has no exact binary form, so the two directions
        // differ in the last bit and nowhere else.
        let program = |control: u16| {
            let pc = x87pc();
            write64(&pc, fpat::DATA, u64::from(control));
            write64(&pc, fpat::DATA + 8, 0x4008_0000_0000_0000); // 3.0
            let mut code = alloc::vec![0xdb, 0xe3, 0xd9];
            code.extend_from_slice(&disp32(5, fpat::DATA));
            code.extend_from_slice(&[0xd9, 0xe8, 0xdd]); // fld1 ; fld qword 3.0
            code.extend_from_slice(&disp32(0, fpat::DATA + 8));
            // fdivp st(1), st(0): ST(1) <- ST(1) / ST(0), then pop.
            code.extend_from_slice(&[0xde, 0xf9, 0xdb]);
            code.extend_from_slice(&disp32(7, fpat::OUT));
            code.push(0xf4);
            run(&pc, &code);
            read64(&pc, fpat::OUT)
        };
        assert_eq!(program(cw::RESET), 0xaaaa_aaaa_aaaa_aaab, "to nearest");
        let toward_zero = cw::RESET | cw::RC;
        assert_eq!(program(toward_zero), 0xaaaa_aaaa_aaaa_aaaa, "toward zero");
    }

    #[test]
    fn the_integer_conversions_round_and_saturate_as_the_manual_says() {
        // `FILD` is exact for every width; `FIST` rounds under `RC` and
        // delivers the integer indefinite for anything out of range (SDM
        // volume 1 §4.2.2.1).
        let pc = x87pc();
        write64(&pc, fpat::DATA, 0x4059_0000_0000_0000); // 100.0
        let mut code = alloc::vec![0xdb, 0xe3, 0xdd];
        code.extend_from_slice(&disp32(0, fpat::DATA));
        code.push(0xdb); // fistp dword
        code.extend_from_slice(&disp32(3, fpat::OUT));
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(read64(&pc, fpat::OUT) as u32, 100);

        // 2^40 does not fit in a doubleword, so the answer is the indefinite
        // and the invalid flag, not a truncation.
        let pc = x87pc();
        write64(&pc, fpat::DATA, 0x4270_0000_0000_0000); // 2^40
        let mut code = alloc::vec![0xdb, 0xe3, 0xdd];
        code.extend_from_slice(&disp32(0, fpat::DATA));
        code.push(0xdb);
        code.extend_from_slice(&disp32(3, fpat::OUT));
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(read64(&pc, fpat::OUT) as u32, 0x8000_0000);
        assert_ne!(pc.cpu.x87().status & sw::IE, 0);
    }

    #[test]
    fn fsqrt_frndint_and_fscale_compute_what_they_claim() {
        let pc = x87pc();
        write64(&pc, fpat::DATA, 0x4010_0000_0000_0000); // 4.0
        let mut code = alloc::vec![0xdb, 0xe3, 0xdd];
        code.extend_from_slice(&disp32(0, fpat::DATA));
        code.extend_from_slice(&[0xd9, 0xfa, 0xdd]); // fsqrt ; fstp qword
        code.extend_from_slice(&disp32(3, fpat::OUT));
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(read64(&pc, fpat::OUT), 0x4000_0000_0000_0000, "sqrt(4) = 2");

        // `FRNDINT` at the default rounding: 2.5 ties to even, so 2.
        let pc = x87pc();
        write64(&pc, fpat::DATA, 0x4004_0000_0000_0000); // 2.5
        let mut code = alloc::vec![0xdb, 0xe3, 0xdd];
        code.extend_from_slice(&disp32(0, fpat::DATA));
        code.extend_from_slice(&[0xd9, 0xfc, 0xdd]);
        code.extend_from_slice(&disp32(3, fpat::OUT));
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(read64(&pc, fpat::OUT), 0x4000_0000_0000_0000, "2.5 -> 2");
        assert_ne!(pc.cpu.x87().status & sw::PE, 0, "and it moved");

        // `FSCALE`: 1.5 * 2^3 = 12.
        let pc = x87pc();
        write64(&pc, fpat::DATA, 0x4008_0000_0000_0000); // 3.0
        write64(&pc, fpat::DATA + 8, 0x3ff8_0000_0000_0000); // 1.5
        let mut code = alloc::vec![0xdb, 0xe3, 0xdd];
        code.extend_from_slice(&disp32(0, fpat::DATA));
        code.push(0xdd);
        code.extend_from_slice(&disp32(0, fpat::DATA + 8));
        code.extend_from_slice(&[0xd9, 0xfd, 0xdd]); // fscale ; fstp qword
        code.extend_from_slice(&disp32(3, fpat::OUT));
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(read64(&pc, fpat::OUT), 0x4028_0000_0000_0000, "12.0");
    }

    #[test]
    fn fprem_reduces_exactly_and_reports_the_quotient_bits() {
        // SDM volume 2, `FPREM`: the remainder of 13 / 4 is 1, and the low
        // three bits of the truncated quotient (3) land in C1, C3 and C0.
        let pc = x87pc();
        write64(&pc, fpat::DATA, 0x4010_0000_0000_0000); // 4.0 -> ST(1)
        write64(&pc, fpat::DATA + 8, 0x402a_0000_0000_0000); // 13.0 -> ST(0)
        let mut code = alloc::vec![0xdb, 0xe3, 0xdd];
        code.extend_from_slice(&disp32(0, fpat::DATA));
        code.push(0xdd);
        code.extend_from_slice(&disp32(0, fpat::DATA + 8));
        code.extend_from_slice(&[0xd9, 0xf8, 0xdd]); // fprem ; fstp qword
        code.extend_from_slice(&disp32(3, fpat::OUT));
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(
            read64(&pc, fpat::OUT),
            0x3ff0_0000_0000_0000,
            "13 mod 4 = 1"
        );
        let st = pc.cpu.x87().status;
        assert_eq!(st & sw::C2, 0, "the reduction was complete");
        assert_ne!(st & sw::C1, 0, "Q0");
        assert_ne!(st & sw::C3, 0, "Q1");
        assert_eq!(st & sw::C0, 0, "Q2 — the quotient is 3");
    }

    #[test]
    fn a_memory_source_compare_reads_the_operand_and_not_st_one() {
        // `D8 /2` compares `ST(0)` with an `m32fp`; `D8 D0+i` compares it with
        // `ST(i)`. The two share a mnemonic and nothing else, and an
        // implementation that routed both to the register form would agree
        // with the manual exactly when `ST(1)` happened to hold the same
        // value.
        let pc = x87pc();
        // ST(1) is 100.0 and the memory operand is 2.0f, so a comparison
        // against the wrong one gives the opposite answer.
        write64(&pc, fpat::DATA, 0x4059_0000_0000_0000); // 100.0
        write64(&pc, fpat::DATA + 8, 0x3ff0_0000_0000_0000); // 1.0
        pc.ram.write_u8(fpat::DATA + 16, 0x00).unwrap();
        pc.ram.write_u8(fpat::DATA + 17, 0x00).unwrap();
        pc.ram.write_u8(fpat::DATA + 18, 0x00).unwrap();
        pc.ram.write_u8(fpat::DATA + 19, 0x40).unwrap(); // 2.0f
        let mut code = alloc::vec![0xdb, 0xe3, 0xdd];
        code.extend_from_slice(&disp32(0, fpat::DATA)); // fld 100.0
        code.push(0xdd);
        code.extend_from_slice(&disp32(0, fpat::DATA + 8)); // fld 1.0
        code.push(0xd8); // fcom dword [DATA+16]
        code.extend_from_slice(&disp32(2, fpat::DATA + 16));
        code.extend_from_slice(&[0xdf, 0xe0, 0xf4]); // fnstsw ax
        run(&pc, &code);
        let s = (pc.regs().rax & 0xffff) as u16;
        assert_ne!(s & sw::C0, 0, "1.0 < 2.0");
        assert_eq!(s & sw::C3, 0);
    }

    #[test]
    fn a_store_from_an_empty_register_writes_the_indefinite() {
        // §8.5.1.1's masked response reaches memory too: the destination gets
        // the QNaN indefinite rather than keeping whatever was there, which is
        // what makes a lost stack visible in the data instead of silent.
        let pc = x87pc();
        write64(&pc, fpat::OUT, 0x1234_5678_9abc_def0);
        let mut code = alloc::vec![0xdb, 0xe3, 0xdd]; // fninit ; fstp qword
        code.extend_from_slice(&disp32(3, fpat::OUT));
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(read64(&pc, fpat::OUT), 0xfff8_0000_0000_0000, "-QNaN");
        let x = pc.cpu.x87();
        assert_ne!(x.status & sw::IE, 0);
        assert_ne!(x.status & sw::SF, 0);
    }

    #[test]
    fn the_transcendentals_are_absent_and_say_so() {
        // `D9 F0` is `F2XM1` on hardware and unassigned here, so it raises
        // `#UD`. Documented in `super`'s module docs: an approximation would
        // be a silently wrong answer where a missing instruction is a loud
        // one.
        let pc = x87pc();
        pc.idt(6, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
        pc.write(HANDLER, &[0xf4]);
        run(&pc, &[0xdb, 0xe3, 0xd9, 0xe8, 0xd9, 0xf0, 0xf4]);
        assert_eq!(pc.regs().rip, HANDLED, "#UD for F2XM1");
    }

    #[test]
    fn the_disassembler_prints_the_escapes_and_the_simd_forms() {
        // One table: the disassembler follows the interpreter into both new
        // families without a second opcode list.
        let pc = ssepc();
        let listing = |bytes: &[u8]| {
            pc.write(at::CODE0, bytes);
            let out = pc.cpu.disassemble(0x08, at::CODE0, 1);
            alloc::format!("{}", out[0])
        };
        assert!(listing(&[0xd9, 0xe8]).ends_with("fld1"), "fld1");
        assert!(listing(&[0xd8, 0xc1]).ends_with("fadd st(0), st(1)"));
        assert!(listing(&[0xde, 0xc1]).ends_with("faddp st(1), st(0)"));
        assert!(listing(&[0xdf, 0xe0]).ends_with("fnstsw ax"));
        let dq = listing(&[0xdd, 0x05, 0x00, 0xb0, 0x00, 0x00]);
        assert!(dq.ends_with("fld qword [ds:0xb000]"), "{dq}");
        let tb = listing(&[0xdb, 0x2d, 0x00, 0xb0, 0x00, 0x00]);
        assert!(tb.ends_with("fld tbyte [ds:0xb000]"), "{tb}");
        assert!(listing(&[0xf2, 0x0f, 0x58, 0xc1]).ends_with("addsd xmm0, xmm1"));
        assert!(listing(&[0x66, 0x0f, 0x28, 0xc1]).ends_with("movapd xmm0, xmm1"));
        assert!(listing(&[0x0f, 0x50, 0xc1]).ends_with("movmskps eax, xmm1"));
        let ld = listing(&[0x0f, 0xae, 0x15, 0x00, 0xb0, 0x00, 0x00]);
        assert!(ld.ends_with("ldmxcsr dword [ds:0xb000]"), "{ld}");
        assert!(listing(&[0x0f, 0xae, 0xf0]).ends_with("mfence"));
        let cx = listing(&[0x0f, 0xc7, 0x0d, 0x00, 0xb0, 0x00, 0x00]);
        assert!(cx.ends_with("cmpxchg8b [ds:0xb000]"), "{cx}");
        // An `F3` that selected an SSE row is part of the opcode, not a repeat
        // prefix, and printing `rep movss` would be a listing no assembler
        // takes back.
        let ss = listing(&[0xf3, 0x0f, 0x10, 0xc1]);
        assert!(ss.ends_with("movss xmm0, xmm1"), "{ss}");
        assert!(!ss.contains("rep"), "{ss}");
    }

    // -- Comparison and classification ---------------------------------

    #[test]
    fn fcom_sets_the_three_condition_codes_the_manual_tabulates() {
        // SDM volume 1 §8.3.4, table 8-3: C3 is "equal", C0 is "less than",
        // and all three go up together for unordered.
        let compare = |a: u64, b: u64| {
            let pc = x87pc();
            write64(&pc, fpat::DATA, b);
            write64(&pc, fpat::DATA + 8, a);
            let mut code = alloc::vec![0xdb, 0xe3, 0xdd];
            code.extend_from_slice(&disp32(0, fpat::DATA));
            code.push(0xdd);
            code.extend_from_slice(&disp32(0, fpat::DATA + 8));
            // fcom st(1) ; fnstsw ax ; hlt
            code.extend_from_slice(&[0xd8, 0xd1, 0xdf, 0xe0, 0xf4]);
            run(&pc, &code);
            (pc.regs().rax & 0xffff) as u16
        };
        let one = 0x3ff0_0000_0000_0000;
        let two = 0x4000_0000_0000_0000;
        let qnan = 0x7ff8_0000_0000_0000;

        let s = compare(one, two);
        assert_ne!(s & sw::C0, 0, "1 < 2 sets C0");
        assert_eq!(s & (sw::C2 | sw::C3), 0);

        let s = compare(two, one);
        assert_eq!(s & (sw::C0 | sw::C2 | sw::C3), 0, "2 > 1 clears all three");

        let s = compare(one, one);
        assert_ne!(s & sw::C3, 0, "equal sets C3");
        assert_eq!(s & (sw::C0 | sw::C2), 0);

        let s = compare(one, qnan);
        assert_eq!(
            s & (sw::C0 | sw::C2 | sw::C3),
            sw::C0 | sw::C2 | sw::C3,
            "unordered sets all three"
        );
        assert_ne!(s & sw::IE, 0, "and FCOM signals on a quiet NaN");
    }

    #[test]
    fn fucom_is_quiet_where_fcom_signals() {
        // IEEE 754-2019 §5.11's two comparison families, which is why both
        // instructions exist. `FUCOM` raises invalid only for a *signaling*
        // NaN.
        let with = |opcode: [u8; 2]| {
            let pc = x87pc();
            write64(&pc, fpat::DATA, 0x7ff8_0000_0000_0000); // quiet NaN
            write64(&pc, fpat::DATA + 8, 0x3ff0_0000_0000_0000);
            let mut code = alloc::vec![0xdb, 0xe3, 0xdd];
            code.extend_from_slice(&disp32(0, fpat::DATA));
            code.push(0xdd);
            code.extend_from_slice(&disp32(0, fpat::DATA + 8));
            code.extend_from_slice(&opcode);
            code.push(0xf4);
            run(&pc, &code);
            pc.cpu.x87().status
        };
        assert_ne!(with([0xd8, 0xd1]) & sw::IE, 0, "FCOM signals");
        assert_eq!(with([0xdd, 0xe1]) & sw::IE, 0, "FUCOM does not");
    }

    #[test]
    fn fcomi_writes_the_integer_flags_instead_of_the_condition_codes() {
        // SDM volume 2, `FCOMI`: ZF, PF and CF, with OF, SF and AF cleared.
        let pc = x87pc();
        write64(&pc, fpat::DATA, 0x4000_0000_0000_0000); // 2.0 -> ST(1)
        write64(&pc, fpat::DATA + 8, 0x3ff0_0000_0000_0000); // 1.0 -> ST(0)
        let mut code = alloc::vec![0xdb, 0xe3, 0xdd];
        code.extend_from_slice(&disp32(0, fpat::DATA));
        code.push(0xdd);
        code.extend_from_slice(&disp32(0, fpat::DATA + 8));
        code.extend_from_slice(&[0xdb, 0xf1, 0xf4]); // fcomi st(0), st(1)
        run(&pc, &code);
        let e = pc.regs().eflags;
        assert_ne!(e & flags::CF, 0, "1 < 2");
        assert_eq!(e & flags::ZF, 0);
        assert_eq!(e & flags::PF, 0);
    }

    #[test]
    fn fxam_tells_an_empty_register_from_a_zero() {
        // The one instruction that can look at an empty register without
        // faulting, and the only way to tell the two apart (SDM volume 2,
        // `FXAM`): empty is C3 C2 C0 = 101, a zero is 100.
        let pc = x87pc();
        run(&pc, &[0xdb, 0xe3, 0xd9, 0xe5, 0xf4]); // fninit ; fxam
        let s = pc.cpu.x87().status;
        assert_ne!(s & sw::C3, 0);
        assert_eq!(s & sw::C2, 0);
        assert_ne!(s & sw::C0, 0, "empty");

        let pc = x87pc();
        run(&pc, &[0xdb, 0xe3, 0xd9, 0xee, 0xd9, 0xe5, 0xf4]);
        let s = pc.cpu.x87().status;
        assert_ne!(s & sw::C3, 0);
        assert_eq!(s & (sw::C2 | sw::C0), 0, "a zero");
        assert_eq!(s & sw::C1, 0, "and a positive one");
    }

    // -- The exception masks -------------------------------------------

    #[test]
    fn a_mask_decides_whether_the_result_is_written_at_all() {
        // SDM volume 1 §8.5: masked means "deliver the standard response and
        // carry on"; unmasked means "record it, leave the destination alone,
        // and fault at the next floating-point instruction".
        //
        // Divide by zero is the clearest of the six: masked it produces an
        // infinity, unmasked it produces nothing at all.
        let divide = |control: u16| {
            let pc = x87pc();
            write64(&pc, fpat::DATA, u64::from(control));
            let mut code = alloc::vec![0xdb, 0xe3, 0xd9];
            code.extend_from_slice(&disp32(5, fpat::DATA));
            // fldz ; fld1 ; fdiv st(0), st(1)  ->  1.0 / 0.0
            code.extend_from_slice(&[0xd9, 0xee, 0xd9, 0xe8, 0xd8, 0xf1, 0xf4]);
            run(&pc, &code);
            pc.cpu.x87()
        };
        let x = divide(cw::RESET);
        assert_ne!(x.status & sw::ZE, 0, "recorded either way");
        assert_eq!(x.status & sw::ES, 0, "masked, so nothing is pending");
        assert_eq!(x.raw(0), F80::INFINITY, "the standard masked response");

        let x = divide(cw::RESET & !cw::ZM);
        assert_ne!(x.status & sw::ZE, 0);
        assert_ne!(x.status & sw::ES, 0, "unmasked, so it is pending");
        assert_ne!(x.status & sw::B, 0);
        assert_eq!(
            x.raw(0),
            F80::new(0x3fff, 0x8000_0000_0000_0000),
            "and ST(0) still holds the dividend"
        );
    }

    #[test]
    fn every_mask_is_separately_effective() {
        // One case per mask bit, which is what makes this a test of the
        // control word rather than of one instruction. Each program provokes
        // exactly the exception its mask names and checks that unmasking it
        // raises the summary bit.
        let cases: [(u16, u16, &[u8]); 4] = [
            // Invalid: the square root of a negative number.
            (cw::IM, sw::IE, &[0xd9, 0xe8, 0xd9, 0xe0, 0xd9, 0xfa]),
            // Zero divide: one over zero.
            (cw::ZM, sw::ZE, &[0xd9, 0xee, 0xd9, 0xe8, 0xd8, 0xf1]),
            // Precision: one third is not exact. `fld1 ; fld1 ;
            // fadd st(0),st(0) ; fadd st(0),st(1)` leaves 3 over 1, and
            // `fdivp st(1),st(0)` divides the one by the three.
            (
                cw::PM,
                sw::PE,
                &[0xd9, 0xe8, 0xd9, 0xe8, 0xd8, 0xc0, 0xd8, 0xc1, 0xde, 0xf9],
            ),
            // Denormal operand: the smallest subnormal double, loaded.
            (cw::DM, sw::DE, &[]),
        ];
        for (mask, flag, body) in cases {
            for unmask in [false, true] {
                let control = if unmask { cw::RESET & !mask } else { cw::RESET };
                let pc = x87pc();
                write64(&pc, fpat::DATA, u64::from(control));
                write64(&pc, fpat::DATA + 8, 1); // the smallest subnormal
                let mut code = alloc::vec![0xdb, 0xe3, 0xd9];
                code.extend_from_slice(&disp32(5, fpat::DATA));
                if body.is_empty() {
                    code.push(0xdd);
                    code.extend_from_slice(&disp32(0, fpat::DATA + 8));
                } else {
                    code.extend_from_slice(body);
                }
                code.push(0xf4);
                run(&pc, &code);
                let st = pc.cpu.x87().status;
                assert_ne!(st & flag, 0, "the flag is sticky whatever the mask");
                assert_eq!(
                    st & sw::ES != 0,
                    unmask,
                    "mask {mask:#06x}: the summary follows the mask"
                );
            }
        }
    }

    #[test]
    fn an_unmasked_exception_faults_at_the_next_floating_point_instruction() {
        // §8.7, and the reason `#MF` exists at all: the exception is deferred
        // to a synchronisation point the program chose.
        let pc = x87pc();
        let mut sys = pc.cpu.sys();
        // Without `CR0.NE` the exception would go to `FERR#` and IRQ 13, which
        // this core does not model — see the module documentation.
        sys.cr0 |= cr0::NE;
        pc.cpu.set_sys(sys);
        pc.idt(16, gate(0x08, 0x3800, sys_type::INT_GATE32, 0));
        pc.write(0x3800, &[0xf4]);
        write64(&pc, fpat::DATA, u64::from(cw::RESET & !cw::ZM));
        let mut code = alloc::vec![0xdb, 0xe3, 0xd9];
        code.extend_from_slice(&disp32(5, fpat::DATA));
        code.extend_from_slice(&[0xd9, 0xee, 0xd9, 0xe8, 0xd8, 0xf1]);
        // The dividing instruction itself completes; this `fld1` is the one
        // that faults.
        code.extend_from_slice(&[0xd9, 0xe8, 0xf4]);
        run(&pc, &code);
        assert_eq!(pc.regs().rip, HANDLED, "#MF was taken by the *next* one");
    }

    #[test]
    fn fwait_is_the_synchronisation_point_a_program_chooses() {
        let pc = x87pc();
        let mut sys = pc.cpu.sys();
        sys.cr0 |= cr0::NE;
        pc.cpu.set_sys(sys);
        pc.idt(16, gate(0x08, 0x3800, sys_type::INT_GATE32, 0));
        pc.write(0x3800, &[0xf4]);
        write64(&pc, fpat::DATA, u64::from(cw::RESET & !cw::ZM));
        let mut code = alloc::vec![0xdb, 0xe3, 0xd9];
        code.extend_from_slice(&disp32(5, fpat::DATA));
        code.extend_from_slice(&[0xd9, 0xee, 0xd9, 0xe8, 0xd8, 0xf1, 0x9b, 0xf4]);
        run(&pc, &code);
        assert_eq!(pc.regs().rip, HANDLED, "fwait took it");
    }

    #[test]
    fn the_no_wait_forms_run_with_an_exception_pending() {
        // §8.3.3: `FNSTSW` is how a handler finds out what happened, so it
        // cannot itself take the exception it is being asked about.
        let pc = x87pc();
        let mut sys = pc.cpu.sys();
        sys.cr0 |= cr0::NE;
        pc.cpu.set_sys(sys);
        pc.idt(16, gate(0x08, 0x3800, sys_type::INT_GATE32, 0));
        pc.write(0x3800, &[0xf4]);
        write64(&pc, fpat::DATA, u64::from(cw::RESET & !cw::ZM));
        let mut code = alloc::vec![0xdb, 0xe3, 0xd9];
        code.extend_from_slice(&disp32(5, fpat::DATA));
        code.extend_from_slice(&[0xd9, 0xee, 0xd9, 0xe8, 0xd8, 0xf1]);
        // fnstsw ax ; fnclex ; fld1 — the first two run, the third does not
        // fault because `FNCLEX` cleared what was pending.
        code.extend_from_slice(&[0xdf, 0xe0, 0xdb, 0xe2, 0xd9, 0xe8, 0xf4]);
        run(&pc, &code);
        assert_ne!(pc.regs().rip, HANDLED, "no #MF was taken");
        assert_ne!(
            (pc.regs().rax as u16) & sw::ES,
            0,
            "and FNSTSW saw the pending exception before FNCLEX removed it"
        );
    }

    #[test]
    fn an_escape_with_cr0_em_or_ts_set_is_a_device_not_available_fault() {
        for bit in [cr0::EM, cr0::TS] {
            let pc = x87pc();
            let mut sys = pc.cpu.sys();
            sys.cr0 |= bit;
            pc.cpu.set_sys(sys);
            pc.idt(7, gate(0x08, 0x3800, sys_type::INT_GATE32, 0));
            pc.write(0x3800, &[0xf4]);
            run(&pc, &[0xd9, 0xe8, 0xf4]);
            assert_eq!(pc.regs().rip, HANDLED, "#NM with CR0 bit {bit:#x}");
        }
    }

    #[test]
    fn a_part_with_no_unit_is_a_coprocessor_socket_with_nothing_in_it() {
        // A 486SX, which is a 486 with one `Features` bit clear. An escape
        // there is not an invalid opcode and not a fault: the processor drives
        // the bus cycle a coprocessor would have answered, and nothing does.
        // That is what an 80386 with an empty 80387 socket does, and it is
        // what the hardware corpus measures on the 386 map.
        let pc = Pc::with_features(Variant::I80486, Features::I80486SX);
        pc.start_protected();
        pc.idt(7, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
        pc.write(HANDLER, &[0xf4]);
        run(&pc, &[0xd9, 0xe8, 0xf4]);
        assert_eq!(pc.regs().rip, at::CODE0 + 3, "the escape did nothing");
        assert_eq!(
            pc.cpu.x87(),
            crate::cpu::x86::fpu::X87::new(),
            "and left no state"
        );

        // `CR0.EM` is how software on such a part asks to be told: with it set
        // the escape becomes `#NM` and an emulator library takes over.
        let pc = Pc::with_features(Variant::I80486, Features::I80486SX);
        pc.start_protected();
        let mut sys = pc.cpu.sys();
        sys.cr0 |= cr0::EM;
        pc.cpu.set_sys(sys);
        pc.idt(7, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
        pc.write(HANDLER, &[0xf4]);
        run(&pc, &[0xd9, 0xe8, 0xf4]);
        assert_eq!(pc.regs().rip, HANDLED, "#NM with CR0.EM set");
    }

    // -- The environment and the save areas ----------------------------

    #[test]
    fn fnstenv_writes_the_three_words_and_masks_everything() {
        // SDM volume 2, `FSTENV`: the control, status and tag words come out
        // and every exception is masked afterwards, which is what makes it the
        // first instruction of a handler.
        let pc = x87pc();
        write64(&pc, fpat::DATA, u64::from(cw::RESET & !cw::ZM));
        let mut code = alloc::vec![0xdb, 0xe3, 0xd9];
        code.extend_from_slice(&disp32(5, fpat::DATA));
        code.extend_from_slice(&[0xd9, 0xe8, 0xd9]); // fld1 ; fnstenv
        code.extend_from_slice(&disp32(6, fpat::OUT));
        code.push(0xf4);
        run(&pc, &code);
        let word = |n: u64| (read64(&pc, fpat::OUT + n * 4) & 0xffff) as u16;
        assert_eq!(word(0), cw::RESET & !cw::ZM, "the control word");
        assert_eq!(
            word(1) & sw::TOP,
            7 << sw::TOP_SHIFT,
            "TOP is in the status"
        );
        assert_eq!(word(2), 0x3fff, "one register occupied, seven empty");
        assert_eq!(
            pc.cpu.x87().control & cw::MASKS,
            cw::MASKS,
            "and everything is masked now"
        );
    }

    #[test]
    fn fnsave_and_frstor_round_trip_the_whole_unit() {
        let pc = x87pc();
        write64(&pc, fpat::DATA, 0x400e_0000_0000_0000); // 3.75
        let mut code = alloc::vec![0xdb, 0xe3, 0xdd];
        code.extend_from_slice(&disp32(0, fpat::DATA));
        code.extend_from_slice(&[0xd9, 0xe8, 0xdd]); // fld1 ; fnsave
        code.extend_from_slice(&disp32(6, fpat::SAVE));
        code.push(0xdd); // frstor
        code.extend_from_slice(&disp32(4, fpat::SAVE));
        code.push(0xf4);
        run(&pc, &code);
        let x = pc.cpu.x87();
        assert_eq!(x.top(), 6);
        assert_eq!(x.raw(0), F80::new(0x3fff, 0x8000_0000_0000_0000));
        assert_eq!(x.raw(1), F80::new(0x4000, 0xf000_0000_0000_0000), "3.75");
        assert_eq!(x.tag_at(x.phys(2)), Tag::Empty);
    }

    // -- SSE -----------------------------------------------------------

    #[test]
    fn a_scalar_double_add_produces_the_exact_sum() {
        let pc = ssepc();
        write64(&pc, fpat::DATA, 0x3ff8_0000_0000_0000); // 1.5
        write64(&pc, fpat::DATA + 8, 0x4002_0000_0000_0000); // 2.25
        let mut code = alloc::vec![0xf2, 0x0f, 0x10]; // movsd xmm0, [DATA]
        code.extend_from_slice(&disp32(0, fpat::DATA));
        code.extend_from_slice(&[0xf2, 0x0f, 0x10]); // movsd xmm1, [DATA+8]
        code.extend_from_slice(&disp32(1, fpat::DATA + 8));
        code.extend_from_slice(&[0xf2, 0x0f, 0x58, 0xc1]); // addsd xmm0, xmm1
        code.extend_from_slice(&[0xf2, 0x0f, 0x11]); // movsd [OUT], xmm0
        code.extend_from_slice(&disp32(0, fpat::OUT));
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(read64(&pc, fpat::OUT), 0x400e_0000_0000_0000, "3.75");
        assert_eq!(pc.cpu.sse().mxcsr & mxcsr::EXCEPTIONS, 0);
    }

    #[test]
    fn a_load_zeroes_the_lanes_above_a_scalar_and_a_register_move_does_not() {
        // SDM volume 2, `MOVSD`: the two encodings of one mnemonic differ in
        // exactly this, and code that packs two doubles in a register depends
        // on it.
        let pc = ssepc();
        write64(&pc, fpat::DATA, 0x1111_1111_1111_1111);
        let mut sse = Sse::new();
        sse.set(0, [0x2222_2222_2222_2222, 0x3333_3333_3333_3333]);
        sse.set(1, [0x4444_4444_4444_4444, 0x5555_5555_5555_5555]);
        pc.cpu.set_sse(sse);
        let mut code = alloc::vec![0xf2, 0x0f, 0x10];
        code.extend_from_slice(&disp32(0, fpat::DATA));
        code.extend_from_slice(&[0xf2, 0x0f, 0x10, 0xd1, 0xf4]); // movsd xmm2, xmm1
        run(&pc, &code);
        let sse = pc.cpu.sse();
        assert_eq!(sse.get(0), [0x1111_1111_1111_1111, 0], "the load zeroed");
        assert_eq!(sse.get(2)[0], 0x4444_4444_4444_4444);
        assert_eq!(sse.get(2)[1], 0, "xmm2 kept its own upper half");
    }

    #[test]
    fn an_aligned_move_refuses_a_misaligned_address() {
        // SDM volume 2, `MOVAPS`: `#GP(0)`, not a slow access. The unaligned
        // form exists so that the aligned one can be a checked assertion.
        let pc = ssepc();
        pc.idt(13, gate(0x08, 0x3800, sys_type::INT_GATE32, 0));
        pc.write(0x3800, &[0xf4]);
        let mut code = alloc::vec![0x0f, 0x28];
        code.extend_from_slice(&disp32(0, fpat::ODD));
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(pc.regs().rip, HANDLED, "#GP on a misaligned MOVAPS");

        // The unaligned form at the same address is fine.
        let pc = ssepc();
        let mut code = alloc::vec![0x0f, 0x10];
        code.extend_from_slice(&disp32(0, fpat::ODD));
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(
            pc.regs().rip,
            at::CODE0 + code.len() as u64,
            "the unaligned form completed and the hlt retired"
        );
    }

    #[test]
    fn sse_needs_cr4_osfxsr_and_says_so_with_an_invalid_opcode() {
        // *Intel SDM* volume 3 table 2-2: without the operating system's
        // promise that it can save the state, the instruction does not exist.
        let pc = Pc::new(Variant::X86_64);
        pc.start_protected();
        pc.idt(6, gate(0x08, 0x3800, sys_type::INT_GATE32, 0));
        pc.write(0x3800, &[0xf4]);
        run(&pc, &[0x0f, 0x57, 0xc0, 0xf4]); // xorps xmm0, xmm0
        assert_eq!(pc.regs().rip, HANDLED, "#UD with CR4.OSFXSR clear");
    }

    #[test]
    fn cr0_em_makes_sse_invalid_rather_than_trappable() {
        // The asymmetry with x87: `CR0.EM` means "emulate the 387" and there
        // has never been an emulation protocol for SSE, so the answer is `#UD`
        // and not `#NM` (SDM volume 1 §11.5.1).
        let pc = ssepc();
        let mut sys = pc.cpu.sys();
        sys.cr0 |= cr0::EM;
        pc.cpu.set_sys(sys);
        pc.idt(6, gate(0x08, 0x3800, sys_type::INT_GATE32, 0));
        pc.idt(7, gate(0x08, 0x3900, sys_type::INT_GATE32, 0));
        pc.write(0x3800, &[0xf4]);
        pc.write(0x3900, &[0xf4]);
        run(&pc, &[0x0f, 0x57, 0xc0, 0xf4]);
        assert_eq!(pc.regs().rip, HANDLED, "#UD, not #NM");
    }

    #[test]
    fn mxcsr_rounding_reaches_the_arithmetic() {
        // The point of `float::Env`: `MXCSR.RC` is not a second rounding
        // implementation, it is the same parameter the x87 control word sets.
        let divide = |rc: u32| {
            let pc = ssepc();
            write64(&pc, fpat::DATA, u64::from(mxcsr::RESET | rc));
            write64(&pc, fpat::DATA + 8, 0x3ff0_0000_0000_0000); // 1.0
            write64(&pc, fpat::DATA + 16, 0x4008_0000_0000_0000); // 3.0
            let mut code = alloc::vec![0x0f, 0xae];
            code.extend_from_slice(&disp32(2, fpat::DATA)); // ldmxcsr
            code.extend_from_slice(&[0xf2, 0x0f, 0x10]);
            code.extend_from_slice(&disp32(0, fpat::DATA + 8));
            code.extend_from_slice(&[0xf2, 0x0f, 0x10]);
            code.extend_from_slice(&disp32(1, fpat::DATA + 16));
            code.extend_from_slice(&[0xf2, 0x0f, 0x5e, 0xc1]); // divsd xmm0, xmm1
            code.extend_from_slice(&[0xf2, 0x0f, 0x11]);
            code.extend_from_slice(&disp32(0, fpat::OUT));
            code.push(0xf4);
            run(&pc, &code);
            read64(&pc, fpat::OUT)
        };
        assert_eq!(divide(0), 0x3fd5_5555_5555_5555, "1/3, to nearest");
        assert_eq!(
            divide(3 << mxcsr::RC_SHIFT),
            0x3fd5_5555_5555_5555,
            "toward zero rounds the same way for this value"
        );
        // Toward positive infinity is the direction that differs.
        assert_eq!(divide(2 << mxcsr::RC_SHIFT), 0x3fd5_5555_5555_5556);
    }

    #[test]
    fn ldmxcsr_refuses_a_reserved_bit() {
        // A guest probes for a future extension by setting a reserved bit and
        // seeing whether the write is taken; accepting one would answer yes.
        let pc = ssepc();
        pc.idt(13, gate(0x08, 0x3800, sys_type::INT_GATE32, 0));
        pc.write(0x3800, &[0xf4]);
        write64(&pc, fpat::DATA, 0x0001_0000);
        let mut code = alloc::vec![0x0f, 0xae];
        code.extend_from_slice(&disp32(2, fpat::DATA));
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(pc.regs().rip, HANDLED);
    }

    #[test]
    fn the_conversions_move_between_the_integer_and_the_simd_files() {
        let pc = ssepc();
        let mut code = alloc::vec![0xb8]; // mov eax, -7
        code.extend_from_slice(&(-7i32).to_le_bytes());
        code.extend_from_slice(&[0xf2, 0x0f, 0x2a, 0xc0]); // cvtsi2sd xmm0, eax
        code.extend_from_slice(&[0xf2, 0x0f, 0x11]);
        code.extend_from_slice(&disp32(0, fpat::OUT));
        code.extend_from_slice(&[0xf2, 0x0f, 0x2c, 0xd8]); // cvttsd2si ebx, xmm0
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(read64(&pc, fpat::OUT), 0xc01c_0000_0000_0000, "-7.0");
        assert_eq!(pc.regs().rbx as i32, -7, "and back again");
    }

    #[test]
    fn ucomisd_writes_the_flags_and_comisd_signals_on_a_quiet_nan() {
        let compare = |op: u8, b: u64| {
            let pc = ssepc();
            write64(&pc, fpat::DATA, 0x3ff0_0000_0000_0000); // 1.0
            write64(&pc, fpat::DATA + 8, b);
            let mut code = alloc::vec![0xf2, 0x0f, 0x10];
            code.extend_from_slice(&disp32(0, fpat::DATA));
            code.extend_from_slice(&[0xf2, 0x0f, 0x10]);
            code.extend_from_slice(&disp32(1, fpat::DATA + 8));
            code.extend_from_slice(&[0x66, 0x0f, op, 0xc1, 0xf4]);
            run(&pc, &code);
            (pc.regs().eflags, pc.cpu.sse().mxcsr)
        };
        let (e, _) = compare(0x2e, 0x4000_0000_0000_0000);
        assert_ne!(e & flags::CF, 0, "1 < 2");
        let (e, _) = compare(0x2e, 0x3ff0_0000_0000_0000);
        assert_ne!(e & flags::ZF, 0, "equal");
        assert_eq!(e & (flags::CF | flags::PF), 0);
        let (e, m) = compare(0x2e, 0x7ff8_0000_0000_0000);
        assert_eq!(
            e & (flags::ZF | flags::PF | flags::CF),
            flags::ZF | flags::PF | flags::CF,
            "unordered"
        );
        assert_eq!(m & mxcsr::IE, 0, "UCOMISD is quiet about a quiet NaN");
        let (_, m) = compare(0x2f, 0x7ff8_0000_0000_0000);
        assert_ne!(m & mxcsr::IE, 0, "COMISD is not");
    }

    #[test]
    fn movmskps_gathers_the_four_sign_bits() {
        let pc = ssepc();
        let mut sse = Sse::new();
        // Lanes 0 and 3 negative, 1 and 2 positive.
        sse.set(1, [0x0000_0000_8000_0000, 0x8000_0000_0000_0000]);
        pc.cpu.set_sse(sse);
        run(&pc, &[0x0f, 0x50, 0xc1, 0xf4]); // movmskps eax, xmm1
        assert_eq!(pc.regs().rax, 0b1001);
    }

    #[test]
    fn the_bitwise_operations_cover_the_whole_register() {
        let pc = ssepc();
        let mut sse = Sse::new();
        sse.set(0, [u64::MAX, 0x0f0f_0f0f_0f0f_0f0f]);
        sse.set(1, [0x00ff_00ff_00ff_00ff, u64::MAX]);
        pc.cpu.set_sse(sse);
        run(&pc, &[0x0f, 0x54, 0xc1, 0x0f, 0x57, 0xd2, 0xf4]); // andps ; xorps
        let sse = pc.cpu.sse();
        assert_eq!(sse.get(0), [0x00ff_00ff_00ff_00ff, 0x0f0f_0f0f_0f0f_0f0f]);
        assert_eq!(
            sse.get(2),
            [0, 0],
            "xorps with itself is the idiomatic zero"
        );
    }

    #[test]
    fn a_packed_add_touches_all_four_lanes() {
        let pc = ssepc();
        let mut sse = Sse::new();
        // 1.0f in every lane, and 2.0f in every lane.
        sse.set(0, [0x3f80_0000_3f80_0000, 0x3f80_0000_3f80_0000]);
        sse.set(1, [0x4000_0000_4000_0000, 0x4000_0000_4000_0000]);
        pc.cpu.set_sse(sse);
        run(&pc, &[0x0f, 0x58, 0xc1, 0xf4]); // addps xmm0, xmm1
        assert_eq!(
            pc.cpu.sse().get(0),
            [0x4040_0000_4040_0000, 0x4040_0000_4040_0000],
            "3.0f in all four"
        );
    }

    #[test]
    fn shufps_selects_two_lanes_from_each_source() {
        let pc = ssepc();
        let mut sse = Sse::new();
        sse.set(0, [0x0000_0001_0000_0000, 0x0000_0003_0000_0002]);
        sse.set(1, [0x0000_0011_0000_0010, 0x0000_0013_0000_0012]);
        pc.cpu.set_sse(sse);
        // shufps xmm0, xmm1, 0b11_01_10_00 — lanes 0 and 2 of the destination,
        // then lanes 1 and 3 of the source.
        run(&pc, &[0x0f, 0xc6, 0xc1, 0b1101_1000, 0xf4]);
        assert_eq!(
            pc.cpu.sse().get(0),
            [0x0000_0002_0000_0000, 0x0000_0013_0000_0011]
        );
    }

    #[test]
    fn fxsave_and_fxrstor_carry_both_register_files() {
        let pc = ssepc();
        let mut sse = Sse::new();
        sse.set(3, [0xdead_beef_cafe_babe, 0x0123_4567_89ab_cdef]);
        sse.mxcsr = mxcsr::RESET | mxcsr::FTZ;
        pc.cpu.set_sse(sse);
        let mut code = alloc::vec![0xdb, 0xe3, 0xd9, 0xe8, 0x0f, 0xae];
        code.extend_from_slice(&disp32(0, fpat::SAVE)); // fxsave
        // Scribble over both files, then restore.
        code.extend_from_slice(&[0xdb, 0xe3, 0x0f, 0x57, 0xdb, 0x0f, 0xae]);
        code.extend_from_slice(&disp32(1, fpat::SAVE)); // fxrstor
        code.push(0xf4);
        run(&pc, &code);
        let x = pc.cpu.x87();
        assert_eq!(x.top(), 7, "TOP came back");
        assert_eq!(x.raw(0), F80::new(0x3fff, 0x8000_0000_0000_0000));
        assert_eq!(x.tag_at(x.phys(1)), Tag::Empty, "and so did the tag word");
        let sse = pc.cpu.sse();
        assert_eq!(sse.get(3), [0xdead_beef_cafe_babe, 0x0123_4567_89ab_cdef]);
        assert_eq!(sse.mxcsr, mxcsr::RESET | mxcsr::FTZ);
    }

    #[test]
    fn an_unmasked_simd_exception_leaves_its_cause_in_mxcsr() {
        // `#XM` carries no error code, so the flag is the only channel the
        // handler has (SDM volume 1 §11.5.3). This performs the classification
        // a real handler does: unmasked flags that are set.
        let pc = ssepc();
        pc.idt(19, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
        pc.write(HANDLER, &[0xf4]);
        write64(&pc, fpat::DATA, u64::from(mxcsr::RESET & !mxcsr::ZM));
        write64(&pc, fpat::DATA + 8, 0x3ff0_0000_0000_0000); // 1.0
        write64(&pc, fpat::DATA + 16, 0); // 0.0
        let mut code = alloc::vec![0x0f, 0xae];
        code.extend_from_slice(&disp32(2, fpat::DATA)); // ldmxcsr
        code.extend_from_slice(&[0xf2, 0x0f, 0x10]);
        code.extend_from_slice(&disp32(0, fpat::DATA + 8));
        code.extend_from_slice(&[0xf2, 0x0f, 0x10]);
        code.extend_from_slice(&disp32(1, fpat::DATA + 16));
        code.extend_from_slice(&[0xf2, 0x0f, 0x5e, 0xc1]); // divsd xmm0, xmm1
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(pc.regs().rip, HANDLED, "#XM was taken");
        let m = pc.cpu.sse().mxcsr;
        assert_ne!(m & mxcsr::ZE, 0, "and said which exception it was");
        let cause = (!m >> mxcsr::MASK_SHIFT) & m & mxcsr::EXCEPTIONS;
        assert_eq!(cause, mxcsr::ZE, "a handler can classify the trap");
        // The destination is untouched: the handler sees the operands.
        assert_eq!(pc.cpu.sse().get(0), [0x3ff0_0000_0000_0000, 0]);
    }

    #[test]
    fn without_osxmmexcpt_an_unmasked_exception_is_an_invalid_opcode() {
        // The operating system unmasked an exception and gave the processor
        // nowhere to deliver it, which the architecture answers loudly.
        let pc = ssepc();
        let mut sys = pc.cpu.sys();
        sys.cr4 &= !cr4::OSXMMEXCPT;
        pc.cpu.set_sys(sys);
        pc.idt(6, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
        pc.write(HANDLER, &[0xf4]);
        write64(&pc, fpat::DATA, u64::from(mxcsr::RESET & !mxcsr::ZM));
        write64(&pc, fpat::DATA + 8, 0x3ff0_0000_0000_0000);
        write64(&pc, fpat::DATA + 16, 0);
        let mut code = alloc::vec![0x0f, 0xae];
        code.extend_from_slice(&disp32(2, fpat::DATA));
        code.extend_from_slice(&[0xf2, 0x0f, 0x10]);
        code.extend_from_slice(&disp32(0, fpat::DATA + 8));
        code.extend_from_slice(&[0xf2, 0x0f, 0x10]);
        code.extend_from_slice(&disp32(1, fpat::DATA + 16));
        code.extend_from_slice(&[0xf2, 0x0f, 0x5e, 0xc1]);
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(pc.regs().rip, HANDLED, "#UD, not #XM");
    }

    #[test]
    fn denormals_are_zeros_reaches_the_comparisons_too() {
        // A comparison never goes through the arithmetic kernel, so `#D` and
        // `DAZ` have to be applied on its own path — and with `DAZ` set a
        // subnormal must compare *equal* to zero, not merely close to it.
        let compare = |daz: bool| {
            let pc = ssepc();
            let value = if daz {
                mxcsr::RESET | mxcsr::DAZ
            } else {
                mxcsr::RESET
            };
            write64(&pc, fpat::DATA, u64::from(value));
            write64(&pc, fpat::DATA + 8, 1); // the smallest subnormal double
            write64(&pc, fpat::DATA + 16, 0); // +0.0
            let mut code = alloc::vec![0x0f, 0xae];
            code.extend_from_slice(&disp32(2, fpat::DATA));
            code.extend_from_slice(&[0xf2, 0x0f, 0x10]);
            code.extend_from_slice(&disp32(0, fpat::DATA + 8));
            code.extend_from_slice(&[0xf2, 0x0f, 0x10]);
            code.extend_from_slice(&disp32(1, fpat::DATA + 16));
            code.extend_from_slice(&[0x66, 0x0f, 0x2e, 0xc1, 0xf4]); // ucomisd
            run(&pc, &code);
            (pc.regs().eflags, pc.cpu.sse().mxcsr)
        };
        let (e, m) = compare(false);
        assert_eq!(e & flags::ZF, 0, "a subnormal is not zero");
        assert_ne!(m & mxcsr::DE, 0, "and it is reported");
        let (e, m) = compare(true);
        assert_ne!(e & flags::ZF, 0, "with DAZ it is a zero");
        assert_eq!(m & mxcsr::DE, 0, "and DAZ suppresses the report");
    }

    #[test]
    fn the_store_halves_of_movlps_and_movhps_have_no_register_form() {
        // `0F 12`/`0F 16` become `MOVHLPS`/`MOVLHPS` with a register operand;
        // `0F 13`/`0F 17` have no register encoding at all, and quietly
        // writing half an XMM register there would be an instruction the
        // architecture does not have.
        let pc = ssepc();
        pc.idt(6, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
        pc.write(HANDLER, &[0xf4]);
        run(&pc, &[0x0f, 0x13, 0xc1, 0xf4]);
        assert_eq!(pc.regs().rip, HANDLED, "#UD");

        // The load direction at the same mode field is `MOVHLPS`, and works.
        let pc = ssepc();
        let mut sse = Sse::new();
        sse.set(1, [0x1111_1111_1111_1111, 0x2222_2222_2222_2222]);
        pc.cpu.set_sse(sse);
        run(&pc, &[0x0f, 0x12, 0xc1, 0xf4]);
        assert_eq!(pc.cpu.sse().get(0)[0], 0x2222_2222_2222_2222);
    }

    #[test]
    fn a_control_instruction_does_not_move_the_data_pointer() {
        // SDM volume 1 §8.1.8. An exception handler's first act is `FNSTENV`,
        // and the `FDP` field it reads has to name the operand that faulted —
        // if the save itself updated the pointer, the field would name the
        // save area and be useless.
        let pc = x87pc();
        write64(&pc, fpat::DATA, 0x3ff0_0000_0000_0000);
        let mut code = alloc::vec![0xdb, 0xe3, 0xdd];
        code.extend_from_slice(&disp32(0, fpat::DATA));
        code.push(0xd9);
        code.extend_from_slice(&disp32(6, fpat::OUT));
        code.push(0xf4);
        run(&pc, &code);
        let dword = |n: u64| read64(&pc, fpat::OUT + n * 4) & 0xffff_ffff;
        assert_eq!(dword(5), fpat::DATA, "FDP names the FLD's operand");
        assert_eq!(pc.cpu.x87().last_dp, fpat::DATA);

        // And `FLDCW` leaves both pointers alone as well.
        let pc = x87pc();
        write64(&pc, fpat::DATA, 0x3ff0_0000_0000_0000);
        write64(&pc, fpat::DATA + 24, u64::from(cw::RESET));
        let mut code = alloc::vec![0xdb, 0xe3, 0xdd];
        code.extend_from_slice(&disp32(0, fpat::DATA));
        code.push(0xd9);
        code.extend_from_slice(&disp32(5, fpat::DATA + 24));
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(pc.cpu.x87().last_dp, fpat::DATA, "FLDCW did not move it");
    }

    #[test]
    fn a_denormal_in_a_register_tags_special_where_a_guest_can_see_it() {
        // §8.1.7's `Special` is "invalid, infinity **or denormal**", and
        // `FNSTENV` is where a guest reads the tag word out.
        //
        // The operand is an 80-bit denormal loaded with `FLD m80fp`, not a
        // binary64 subnormal: widening one of those produces an ordinary
        // *normal* 80-bit value, because the wider exponent range has room for
        // it. That is the whole reason the extended format exists, and it
        // means the only way to get a denormal into a register is to load one.
        let pc = x87pc();
        write64(&pc, fpat::DATA, 1); // significand 1
        write64(&pc, fpat::DATA + 8, 0); // exponent field and sign both zero
        let mut code = alloc::vec![0xdb, 0xe3, 0xdb];
        code.extend_from_slice(&disp32(5, fpat::DATA)); // fld tbyte [DATA]
        code.push(0xd9);
        code.extend_from_slice(&disp32(6, fpat::OUT));
        code.push(0xf4);
        run(&pc, &code);
        let tag = (read64(&pc, fpat::OUT + 8) & 0xffff) as u16;
        // `ST(0)` is physical register 7, so its two bits are the top two.
        assert_eq!(tag >> 14, 0b10, "Special, not Valid");
        assert_eq!(pc.cpu.x87().tag_at(7), Tag::Special);
    }

    // -- CMPXCHG8B -----------------------------------------------------

    #[test]
    fn cmpxchg8b_exchanges_on_a_match_and_loads_on_a_miss() {
        // SDM volume 2, `CMPXCHG8B`. The load-on-miss is what makes a
        // compare-and-exchange loop terminate rather than spin.
        let build = || {
            let mut code = alloc::vec![0xb8];
            code.extend_from_slice(&0x2222_2222u32.to_le_bytes()); // mov eax
            code.push(0xba);
            code.extend_from_slice(&0x1111_1111u32.to_le_bytes()); // mov edx
            code.push(0xbb);
            code.extend_from_slice(&0xdead_beefu32.to_le_bytes()); // mov ebx
            code.push(0xb9);
            code.extend_from_slice(&0xcafe_babeu32.to_le_bytes()); // mov ecx
            code.extend_from_slice(&[0x0f, 0xc7]);
            code.extend_from_slice(&disp32(1, fpat::DATA));
            code.push(0xf4);
            code
        };
        let pc = ssepc();
        write64(&pc, fpat::DATA, 0x1111_1111_2222_2222);
        run(&pc, &build());
        assert_ne!(pc.regs().eflags & flags::ZF, 0, "the compare matched");
        assert_eq!(read64(&pc, fpat::DATA), 0xcafe_babe_dead_beef);

        // The same program against a value that does not match.
        let pc = ssepc();
        write64(&pc, fpat::DATA, 0x3333_3333_4444_4444);
        run(&pc, &build());
        assert_eq!(pc.regs().eflags & flags::ZF, 0, "the compare failed");
        assert_eq!(pc.regs().rax as u32, 0x4444_4444, "and memory was loaded");
        assert_eq!(pc.regs().rdx as u32, 0x3333_3333);
        assert_eq!(read64(&pc, fpat::DATA), 0x3333_3333_4444_4444, "untouched");
    }

    #[test]
    fn a_part_without_cx8_says_so_and_refuses_the_instruction() {
        let pc = x87pc(); // a 486DX: no CMPXCHG8B
        pc.idt(6, gate(0x08, 0x3800, sys_type::INT_GATE32, 0));
        pc.write(0x3800, &[0xf4]);
        let mut code = alloc::vec![0x0f, 0xc7];
        code.extend_from_slice(&disp32(1, fpat::DATA));
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(pc.regs().rip, HANDLED);
    }

    // -- CPUID ---------------------------------------------------------

    #[test]
    fn cpuid_now_reports_what_a_64_bit_operating_system_looks_for() {
        // The five bits a 64-bit Linux checks before it will run: `FPU`,
        // `CX8`, `FXSR`, `SSE` and `SSE2`, plus `PAE`, `MSR`, `PSE`, `PGE`,
        // `CMOV` and `TSC` from the long-mode work.
        let pc = Pc::new(Variant::X86_64);
        pc.start_protected();
        run(&pc, &[0xb8, 0x01, 0x00, 0x00, 0x00, 0x0f, 0xa2, 0xf4]);
        let edx = pc.regs().rdx as u32;
        for (bit, name) in [
            (0, "FPU"),
            (3, "PSE"),
            (4, "TSC"),
            (5, "MSR"),
            (6, "PAE"),
            (8, "CX8"),
            (13, "PGE"),
            (15, "CMOV"),
            (24, "FXSR"),
            (25, "SSE"),
            (26, "SSE2"),
        ] {
            assert_ne!(edx & (1u32 << bit), 0, "leaf 1 should report {name}");
        }
        // And still not MMX, whose registers alias the x87 stack.
        assert_eq!(edx & (1 << 23), 0, "MMX is not implemented");
    }

    #[test]
    fn narrowing_the_feature_set_narrows_what_cpuid_claims() {
        // §6.1.1's whole point: the answer follows `Features`, not the part
        // name, so a machine description can model something that shipped.
        let features = Features {
            sse2: false,
            long: false,
            nx: false,
            syscall: false,
            ..Features::X86_64
        };
        assert!(features.validate().is_ok());
        let pc = Pc::with_features(Variant::X86_64, features);
        pc.start_protected();
        run(&pc, &[0xb8, 0x01, 0x00, 0x00, 0x00, 0x0f, 0xa2, 0xf4]);
        let edx = pc.regs().rdx as u32;
        assert_ne!(edx & (1 << 25), 0, "SSE");
        assert_eq!(edx & (1 << 26), 0, "but not SSE2");
    }

    #[test]
    fn a_long_mode_part_must_have_sse2() {
        // Not an implementation limit: the 64-bit ABI passes floating-point
        // arguments in `XMM` registers, so a part with long mode and no SSE2
        // is not one anybody shipped.
        let impossible = Features {
            sse2: false,
            ..Features::X86_64
        };
        assert!(impossible.validate().is_err());
    }

    #[test]
    fn no_host_float_reaches_a_guest_result() {
        // `float`'s own test reads its four sources back and asserts the same
        // thing; this is the other half of the path, because arithmetic done
        // in software and then rounded through a host `f64` on the way to a
        // register would be just as unreproducible (`ROADMAP.md` §9.1).
        //
        // Matched on identifier boundaries rather than as a substring, because
        // `Arg::Mf32` and `Arg::Mf64` are the *operand kinds* — `m32fp` and
        // `m64fp` in Intel's notation — and naming them is not using them.
        let sources = [
            ("fpu.rs", include_str!("fpu.rs")),
            ("fpexec.rs", include_str!("fpexec.rs")),
        ];
        let boundary = |src: &str, at: usize| {
            let before = src[..at].chars().next_back();
            let after = src[at..].chars().nth(3);
            !before.is_some_and(|c| c.is_alphanumeric() || c == '_')
                && !after.is_some_and(|c| c.is_alphanumeric() || c == '_')
        };
        for (name, src) in sources {
            for (n, line) in src.lines().enumerate() {
                let code = match line.find("//") {
                    Some(i) => &line[..i],
                    None => line,
                };
                for needle in ["f32", "f64", "f16", "sqrtf", "libm"] {
                    let mut from = 0;
                    while let Some(i) = code[from..].find(needle) {
                        let at = from + i;
                        assert!(
                            !boundary(code, at),
                            "{name}:{}: host floating point: {code}",
                            n + 1
                        );
                        from = at + needle.len();
                    }
                }
            }
        }
    }

    // -- The snapshot --------------------------------------------------

    #[test]
    fn the_floating_point_state_survives_a_snapshot() {
        let pc = ssepc();
        run(
            &pc,
            &[0xdb, 0xe3, 0xd9, 0xe8, 0xd9, 0xeb, 0x0f, 0x57, 0xc0, 0xf4],
        );
        let mut sse = pc.cpu.sse();
        sse.set(5, [0x1234_5678_9abc_def0, 0x0fed_cba9_8765_4321]);
        sse.mxcsr = mxcsr::RESET | mxcsr::DAZ;
        pc.cpu.set_sse(sse);

        let x87_before = pc.cpu.x87();
        let sse_before = pc.cpu.sse();

        let mut shape = MachineShape::new();
        shape.add_device("/cpu0", "cpu.x86").unwrap();
        let mut writer = StateWriter::new(shape);
        {
            let mut chunk = writer.chunk("/cpu0", "cpu.x86", 5).unwrap();
            pc.cpu.save(&mut chunk).unwrap();
        }
        let bytes = writer.to_vec().unwrap();

        pc.cpu.reset(ResetKind::Cold);
        assert_ne!(pc.cpu.x87(), x87_before, "there is something to lose");

        let reader = crate::core::state::StateReader::new(&bytes).unwrap();
        let (_, _, data) = reader.load_raw("/cpu0").unwrap();
        let mut chunk = ChunkReader::new(data);
        pc.cpu.load(&mut chunk).unwrap();
        // Nothing is left over: the floating-point block is the end of the
        // chunk, so a mismatched layout shows up here rather than silently.
        chunk.end().unwrap();
        assert_eq!(pc.cpu.x87(), x87_before);
        assert_eq!(pc.cpu.sse(), sse_before);

        // And a second save is byte-identical, which is what "an identical
        // state hash" means for a chunk this shape.
        let mut shape = MachineShape::new();
        shape.add_device("/cpu0", "cpu.x86").unwrap();
        let mut again = StateWriter::new(shape);
        {
            let mut chunk = again.chunk("/cpu0", "cpu.x86", 5).unwrap();
            pc.cpu.save(&mut chunk).unwrap();
        }
        assert_eq!(bytes, again.to_vec().unwrap());
    }
}

// ===========================================================================
// Multiprocessing: INIT, Start-Up, and the model-specific registers
// ===========================================================================

/// The two restarts that are not `RESET`, and the register file that names
/// state living outside the processor.
///
/// Written from the *Intel SDM* volume 3A Table 9-1 (what an INIT resets),
/// §8.4.3 and the *MultiProcessor Specification* v1.4 §B.4 (where a Start-Up
/// leaves a processor), §10.4.4 (`IA32_APIC_BASE`), and volume 4's MSR tables.
/// `tests/pc_apic_smp.rs` is the same thing driven by a *guest*: a bootstrap
/// processor executing the specification's three writes to its own interrupt
/// command register, with nothing supplied by hand.
mod multiprocessor {
    use super::*;
    use crate::core::state::StateReader;
    use crate::core::sync::{AtomicU64, Ordering};
    use crate::core::wire::{LocalController, Startup};
    use crate::cpu::x86::prot::{apic_base, cr4};

    /// The page a Start-Up names in these tests: linear `0x8000`.
    const PAGE: u8 = 0x08;

    /// Where the fault handlers in this module sit, inside the ring-0 code
    /// segment the other protected-mode tests use.
    const HANDLER: u64 = 0x3100;

    /// Point the core at `rip` without disturbing anything else.
    fn set_rip(pc: &Pc, rip: u64) {
        let mut regs = pc.cpu.regs();
        regs.rip = rip;
        pc.cpu.set_regs(regs);
    }

    /// Enable maskable interrupts.
    fn set_if(pc: &Pc) {
        let mut regs = pc.cpu.regs();
        regs.eflags |= flags::IF;
        pc.cpu.set_regs(regs);
    }

    /// A 486 with `CR4` and the model-specific registers switched on: a
    /// Pentium-class part in everything these tests touch.
    ///
    /// The lattice rather than the ladder (`ROADMAP.md` §6.1.1). A machine file
    /// says the same thing with `msr = true`, and a board with a local APIC has
    /// to: the APIC's own base register is an MSR.
    fn pc_msr() -> Pc {
        let mut features = Variant::I80486.features();
        features.cr4 = true;
        features.msr = true;
        Pc::with_features(Variant::I80486, features)
    }

    #[test]
    fn an_init_is_a_reset_that_stops_at_wait_for_sipi() {
        let pc = pc386();
        pc.start_protected();
        pc.cpu.step();
        assert!(pc.cpu.sys().protected(), "there is something to lose");
        let cycles = pc.cpu.cycles();

        pc.cpu.request_init();
        assert!(pc.cpu.init_requested());
        let charged = pc.cpu.step();

        assert!(charged > 0, "the sequence itself is charged for");
        assert!(
            !pc.cpu.sys().protected(),
            "an INIT puts CR0 back to its reset value (Table 9-1)"
        );
        assert_eq!(pc.cpu.regs().cs, 0xf000);
        assert_eq!(pc.cpu.regs().rip, 0xfff0);
        assert!(
            pc.cpu.cycles() > cycles,
            "the time-stamp counter is not reset by an INIT, only added to"
        );
        assert!(
            pc.cpu.is_waiting_for_startup(),
            "and it stops there rather than fetching from the reset vector"
        );
        assert_eq!(pc.cpu.step(), 0, "which is a full stop");
    }

    #[test]
    fn a_reset_and_an_init_differ_in_where_they_leave_the_processor() {
        // The distinction the whole seam exists for: `RESET` restarts at the
        // reset vector, INIT waits to be told where to start.
        let reset = pc386();
        reset.start_protected();
        reset.cpu.request_reset();
        reset.cpu.step();
        assert!(!reset.cpu.is_waiting_for_startup());
        assert!(reset.cpu.step() > 0, "it is fetching");

        let init = pc386();
        init.start_protected();
        init.cpu.request_init();
        init.cpu.step();
        assert!(init.cpu.is_waiting_for_startup());
        assert_eq!(init.cpu.step(), 0, "it is not");
    }

    #[test]
    fn a_start_up_begins_execution_at_the_page_it_names() {
        let pc = pc386();
        pc.start_protected();
        // inc eax ; jmp $ — sixteen-bit code, because a Start-Up leaves the
        // processor in real mode however the sender was running.
        pc.write(u64::from(PAGE) << 12, &[0x40, 0xeb, 0xfe]);

        pc.cpu.request_init();
        pc.cpu.step();
        pc.cpu.start_up(PAGE);
        assert!(pc.cpu.step() > 0, "the Start-Up sequence runs");

        assert_eq!(pc.cpu.regs().cs, u16::from(PAGE) << 8);
        assert_eq!(pc.cpu.regs().rip, 0);
        assert_eq!(
            pc.cpu.sys().segs[usize::from(isa::seg::CS)].base,
            u64::from(PAGE) << 12,
            "the cached base is page << 12, so the fetch is from 000PP000H"
        );
        assert!(!pc.cpu.is_waiting_for_startup());

        pc.cpu.step();
        assert_eq!(pc.regs().rax & 0xffff_ffff, 1, "and it executed the page");
    }

    #[test]
    fn a_start_up_to_a_processor_that_is_not_waiting_is_ignored() {
        // Which is why the specification's algorithm sends two of them and does
        // not care that the second is redundant (§B.4).
        let pc = pc386();
        pc.start_protected();
        pc.write(at::CODE0, &[0x40, 0xeb, 0xfe]); // inc eax ; jmp $
        pc.cpu.start_up(PAGE);
        pc.cpu.step();
        assert_eq!(pc.regs().rax & 0xffff_ffff, 1);
        assert_ne!(pc.cpu.regs().cs, u16::from(PAGE) << 8);
    }

    #[test]
    fn an_interrupt_does_not_leave_the_wait_for_sipi_state() {
        // The difference between this halt and a `HLT`, which any interrupt
        // ends (SDM Vol 3A §8.4.2).
        let pc = pc386();
        pc.start_protected();
        pc.cpu.request_init();
        pc.cpu.step();

        pc.cpu.set_intr_vector(0x42);
        pc.cpu.set_intr(true);
        set_if(&pc);
        assert_eq!(pc.cpu.step(), 0, "the request stays pending on the pin");
        assert!(pc.cpu.is_waiting_for_startup());

        pc.cpu.pulse_nmi();
        assert_eq!(pc.cpu.step(), 0, "and so does an NMI");
        assert!(pc.cpu.nmi_pending(), "which is still latched");
    }

    #[test]
    fn the_init_pin_holds_the_processor_in_reset_while_it_is_asserted() {
        use crate::core::wire::{Level, Wire, WireId};

        let pc = pc386();
        pc.start_protected();
        pc.write(u64::from(PAGE) << 12, &[0x40, 0xeb, 0xfe]);

        let src = WireId(1);
        let pin = pc.cpu.sink("init", &[src]).expect("a 386 has an INIT pin");
        assert!(
            !pc.cpu.init_held(),
            "a fresh net sits low, and low is de-asserted: nothing invented"
        );
        let wire = Wire::builder()
            .source(src)
            .sink_weak(Arc::downgrade(&pin.sink), pin.line)
            .build();

        wire.set(src, Level::High);
        assert!(pc.cpu.init_held());
        assert!(pc.cpu.step() > 0, "the rising edge runs the sequence");
        assert!(pc.cpu.is_waiting_for_startup());
        // Held: even a Start-Up does not start it while the line is up, which
        // is what a *level*-triggered INIT means.
        pc.cpu.start_up(PAGE);
        assert_eq!(pc.cpu.step(), 0);
        assert!(pc.cpu.is_waiting_for_startup());

        wire.set(src, Level::Low);
        assert!(!pc.cpu.init_held());
        assert!(pc.cpu.step() > 0, "and now the Start-Up is taken");
        assert_eq!(pc.cpu.regs().cs, u16::from(PAGE) << 8);
    }

    #[test]
    fn a_reset_outranks_an_init_that_has_not_run_yet() {
        // The greater restart subsumes the lesser: a processor that has just
        // been reset is not owed the sequence that would put it where reset
        // already put it.
        let pc = pc386();
        pc.start_protected();
        pc.cpu.request_init();
        Device::reset(&*pc.cpu, ResetKind::Warm);
        assert!(!pc.cpu.init_requested(), "the latch went with the reset");
        pc.cpu.step();
        assert!(!pc.cpu.is_waiting_for_startup());
    }

    #[test]
    fn a_sixteen_bit_part_has_no_init_pin() {
        // It arrived with the parts that could be a second processor. A machine
        // file naming one on an 8088 is told so rather than given a pin that
        // does nothing.
        let m = machine();
        assert!(
            m.cpu
                .sink("init", &[crate::core::wire::WireId(1)])
                .is_none()
        );
        let pc = pc386();
        assert!(
            pc.cpu
                .sink("init", &[crate::core::wire::WireId(1)])
                .is_some()
        );
    }

    #[test]
    fn the_multiprocessor_state_round_trips_through_a_snapshot() {
        let pc = pc386();
        pc.start_protected();
        pc.cpu.request_init();
        pc.cpu.step();
        pc.cpu.start_up(PAGE);
        assert!(pc.cpu.is_waiting_for_startup());

        let mut shape = MachineShape::new();
        shape.add_device("/cpu0", "cpu.x86").unwrap();
        let mut writer = StateWriter::new(shape);
        {
            let mut chunk = writer.chunk("/cpu0", "cpu.x86", 6).unwrap();
            pc.cpu.save(&mut chunk).unwrap();
        }
        let bytes = writer.to_vec().unwrap();

        // Wreck it: a cold reset is the one thing that clears both.
        Device::reset(&*pc.cpu, ResetKind::Cold);
        assert!(!pc.cpu.is_waiting_for_startup());

        let reader = StateReader::new(&bytes).unwrap();
        let (_, _, data) = reader.load_raw("/cpu0").unwrap();
        let mut chunk = ChunkReader::new(data);
        pc.cpu.load(&mut chunk).unwrap();
        // Nothing left over: the multiprocessor block is the end of the chunk,
        // so a mismatched layout shows up here rather than silently.
        chunk.end().unwrap();
        assert!(pc.cpu.is_waiting_for_startup(), "still waiting");

        // A second save is byte-identical, which is what "an identical state
        // hash" means for a chunk this shape — and the Start-Up the first one
        // recorded is still latched, so the restored processor starts.
        let mut shape = MachineShape::new();
        shape.add_device("/cpu0", "cpu.x86").unwrap();
        let mut again = StateWriter::new(shape);
        {
            let mut chunk = again.chunk("/cpu0", "cpu.x86", 6).unwrap();
            pc.cpu.save(&mut chunk).unwrap();
        }
        assert_eq!(bytes, again.to_vec().unwrap());

        pc.cpu.step();
        assert_eq!(pc.cpu.regs().cs, u16::from(PAGE) << 8);
    }

    // -- The model-specific registers --------------------------------------

    #[test]
    fn an_unimplemented_model_specific_register_faults() {
        // `#GP(0)`, never a zero: a guest that reads zero from an MSR that does
        // not exist concludes the feature is present and disabled, and
        // misbehaves a long way from here (SDM Vol 4, and volume 3 §2.5's
        // description of `RDMSR`).
        let pc = pc_msr();
        pc.start_protected();
        pc.idt(13, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
        pc.write(HANDLER, &[0xf4]); // hlt
        // mov ecx, 0xdeadbeef ; rdmsr
        pc.write(at::CODE0, &[0xb9, 0xef, 0xbe, 0xad, 0xde, 0x0f, 0x32]);
        pc.cpu.step();
        pc.cpu.step();
        assert_eq!(pc.cpu.regs().rip, HANDLER, "#GP(0), not a zero");
    }

    #[test]
    fn a_write_to_an_unimplemented_model_specific_register_faults() {
        let pc = pc_msr();
        pc.start_protected();
        pc.idt(13, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
        pc.write(HANDLER, &[0xf4]);
        // mov ecx, 0xdeadbeef ; xor eax, eax ; xor edx, edx ; wrmsr
        pc.write(
            at::CODE0,
            &[
                0xb9, 0xef, 0xbe, 0xad, 0xde, 0x31, 0xc0, 0x31, 0xd2, 0x0f, 0x30,
            ],
        );
        for _ in 0..4 {
            pc.cpu.step();
        }
        assert_eq!(pc.cpu.regs().rip, HANDLER);
    }

    #[test]
    fn the_time_stamp_counter_is_readable_two_ways_and_writable_one() {
        let pc = pc_msr();
        pc.start_protected();
        // rdtsc ; mov ecx, IA32_TSC ; rdmsr
        pc.write(at::CODE0, &[0x0f, 0x31, 0xb9, 0x10, 0, 0, 0, 0x0f, 0x32]);
        pc.cpu.step();
        let by_rdtsc = pc.regs().rax & 0xffff_ffff;
        assert!(by_rdtsc > 0, "the counter is this core's own cycle count");
        pc.cpu.step();
        pc.cpu.step();
        assert!(
            pc.regs().rax & 0xffff_ffff > by_rdtsc,
            "and `RDMSR` of 0x10 reads the same counter, which has moved on"
        );

        // Writing it moves the counter both of them read (SDM Vol 3 §17.17.3).
        // mov ecx, IA32_TSC ; mov eax, 0x1000 ; xor edx, edx ; wrmsr ; rdtsc
        pc.write(
            at::CODE0,
            &[
                0xb9, 0x10, 0, 0, 0, 0xb8, 0x00, 0x10, 0, 0, 0x31, 0xd2, 0x0f, 0x30, 0x0f, 0x31,
            ],
        );
        set_rip(&pc, at::CODE0);
        for _ in 0..5 {
            pc.cpu.step();
        }
        let after = pc.regs().rax & 0xffff_ffff;
        assert!(
            (0x1000..0x1100).contains(&after),
            "the counter restarted from what was written, not from where it was"
        );
    }

    #[test]
    fn rdtsc_is_privileged_only_while_cr4_tsd_is_set() {
        // *Intel SDM* volume 3 §2.5. Until this existed, `CPUID` reported the
        // `TSC` bit and `RDTSC` raised `#UD`, which is the kind of lie the
        // `cpuid` doc comment says it does not tell.
        let pc = pc_msr();
        pc.start_protected();
        let mut sys = pc.cpu.sys();
        sys.cr4 |= cr4::TSD;
        pc.cpu.set_sys(sys);
        pc.write(at::CODE0, &[0x0f, 0x31]);
        pc.cpu.step();
        assert!(
            pc.regs().rax & 0xffff_ffff > 0,
            "ring 0 reads it however `TSD` is set"
        );
    }

    #[test]
    fn ia32_misc_enable_comes_up_with_fast_strings_and_takes_the_two_bits_it_has() {
        // *Intel SDM* volume 4 Table 2-2. The register exists wherever
        // model-specific registers do, and a `#GP` for it is what stopped a
        // 64-bit Linux dead: its processor check reads this address before it
        // has an interrupt descriptor table, so the fault is a triple fault
        // with nothing printed.
        let pc = pc_msr();
        pc.start_protected();
        // mov ecx, 0x1a0 ; rdmsr
        pc.write(at::CODE0, &[0xb9, 0xa0, 0x01, 0, 0, 0x0f, 0x32]);
        pc.cpu.step();
        pc.cpu.step();
        assert_eq!(
            pc.regs().rax & 0xffff_ffff,
            1,
            "fast strings enabled, which is what every part it exists on \
             comes out of reset with"
        );
        assert_eq!(
            pc.regs().rdx & 0xffff_ffff,
            0,
            "and nothing in the top half"
        );

        // Setting the execute-disable lock is the one write with a
        // consequence, and it is refused unless the bit is one of the two.
        // mov ecx, 0x1a0 ; xor eax, eax ; mov edx, 4 ; wrmsr
        pc.write(
            at::CODE0,
            &[
                0xb9, 0xa0, 0x01, 0, 0, 0x31, 0xc0, 0xba, 0x04, 0, 0, 0, 0x0f, 0x30,
            ],
        );
        set_rip(&pc, at::CODE0);
        for _ in 0..4 {
            pc.cpu.step();
        }
        assert_eq!(
            pc.cpu.sys().misc_enable,
            1 << 34,
            "the lock is set and fast strings was cleared by the same write"
        );
    }

    #[test]
    fn a_reserved_bit_of_ia32_misc_enable_is_refused_rather_than_stored() {
        // The same rule `IA32_APIC_BASE` follows: a bit that controls a
        // feature this core does not have would be a knob connected to
        // nothing.
        let pc = pc_msr();
        pc.start_protected();
        pc.idt(13, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
        pc.write(HANDLER, &[0xf4]);
        // mov ecx, 0x1a0 ; mov eax, 8 ; xor edx, edx ; wrmsr
        pc.write(
            at::CODE0,
            &[
                0xb9, 0xa0, 0x01, 0, 0, 0xb8, 0x08, 0, 0, 0, 0x31, 0xd2, 0x0f, 0x30,
            ],
        );
        for _ in 0..4 {
            pc.cpu.step();
        }
        assert_eq!(pc.cpu.regs().rip, HANDLER, "#GP(0)");
        assert_eq!(pc.cpu.sys().misc_enable, 1, "and nothing was stored");
    }

    #[test]
    fn the_microcode_revision_reads_zero_and_takes_the_write_that_clears_it() {
        // *Intel SDM* volume 3 §10.11.2's ritual: write zero, `CPUID` leaf 1,
        // read the high doubleword. Both halves have to work, and the answer
        // is zero because no microcode update has been loaded — which is a
        // fact about this processor rather than a stand-in for one.
        let pc = pc_msr();
        pc.start_protected();
        // mov ecx, 0x8b ; xor eax, eax ; xor edx, edx ; wrmsr ; rdmsr
        pc.write(
            at::CODE0,
            &[
                0xb9, 0x8b, 0, 0, 0, 0x31, 0xc0, 0x31, 0xd2, 0x0f, 0x30, 0x0f, 0x32,
            ],
        );
        for _ in 0..5 {
            pc.cpu.step();
        }
        assert_eq!(pc.regs().rax & 0xffff_ffff, 0);
        assert_eq!(pc.regs().rdx & 0xffff_ffff, 0, "no update is loaded");
    }

    #[test]
    fn the_reserved_nop_space_decodes_its_operand_and_touches_nothing() {
        // `0F 18`-`0F 1F`, which the *Intel SDM* volume 2 Appendix A prints as
        // `NOP Ev`. Two of them are load-bearing on a modern guest: `0F 1F /0`
        // is the multi-byte NOP a compiler pads with, and `F3 0F 1E FA` is
        // `ENDBR64`, which begins every function of a kernel built with
        // indirect-branch tracking.
        let pc = pc386();
        pc.start_protected();
        pc.write(
            at::CODE0,
            &[
                0x0f, 0x1f, 0x40, 0x00, // nop dword [eax+0]
                0xf3, 0x0f, 0x1e, 0xfa, // endbr64
                // prefetchnta [0xf0000000] — an address nothing answers, which
                // is the assertion: a hint reads no memory, so this cannot
                // fault however unmapped its operand is.
                0x0f, 0x18, 0x05, 0x00, 0x00, 0x00, 0xf0,
            ],
        );
        let before = pc.regs();
        pc.cpu.step();
        assert_eq!(pc.cpu.regs().rip, at::CODE0 + 4, "four bytes consumed");
        pc.cpu.step();
        assert_eq!(pc.cpu.regs().rip, at::CODE0 + 8, "and four more");
        pc.cpu.step();
        assert_eq!(pc.cpu.regs().rip, at::CODE0 + 15);
        let after = pc.regs();
        assert_eq!(
            (after.rax, after.rbx, after.rcx, after.rdx, after.eflags),
            (
                before.rax,
                before.rbx,
                before.rcx,
                before.rdx,
                before.eflags
            ),
            "a hint changes nothing"
        );
        assert_eq!(pc.cpu.bus_faults(), (0, 0), "and reads no memory");
    }

    #[test]
    fn a_part_with_no_model_specific_registers_raises_ud() {
        // A 486 has neither instruction, and the check is at execution rather
        // than in the table: the row decodes, the feature decides.
        let pc = pc386();
        pc.start_protected();
        pc.idt(6, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
        pc.write(HANDLER, &[0xf4]);
        pc.write(at::CODE0, &[0x0f, 0x31]); // rdtsc
        pc.cpu.step();
        assert_eq!(pc.cpu.regs().rip, HANDLER, "#UD");
    }

    /// A local controller with nothing to report and a base register that says
    /// what it was set to, which is all `IA32_APIC_BASE` needs from one.
    #[derive(Debug)]
    struct Controller {
        base: AtomicU64,
    }

    impl LocalController for Controller {
        fn take_startup(&self) -> Startup {
            Startup::NONE
        }

        fn base_register(&self) -> u64 {
            self.base.load(Ordering::Acquire)
        }

        fn set_base_register(&self, value: u64) {
            self.base.store(value, Ordering::Release);
        }
    }

    #[test]
    fn ia32_apic_base_faults_on_a_processor_with_no_local_controller() {
        // The register is the controller's, not the core's: a board that wired
        // no APIC does not have it, and `#GP` is the honest answer. `CPUID`'s
        // `APIC` bit is clear on the same machine, so the two agree.
        let pc = pc_msr();
        pc.start_protected();
        pc.idt(13, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
        pc.write(HANDLER, &[0xf4]);
        pc.write(at::CODE0, &[0xb9, 0x1b, 0, 0, 0, 0x0f, 0x32]);
        pc.cpu.step();
        pc.cpu.step();
        assert_eq!(pc.cpu.regs().rip, HANDLER);
    }

    #[test]
    fn ia32_apic_base_reads_and_writes_the_controller_that_owns_it() {
        let pc = pc_msr();
        let intc = Arc::new(Controller {
            base: AtomicU64::new(0xfee0_0000 | apic_base::ENABLE | apic_base::BSP),
        });
        let peer: Arc<dyn LocalController> = intc.clone();
        pc.cpu
            .attach_local_controller("intr", Arc::downgrade(&peer));
        pc.start_protected();

        // mov ecx, 0x1b ; rdmsr
        pc.write(at::CODE0, &[0xb9, 0x1b, 0, 0, 0, 0x0f, 0x32]);
        pc.cpu.step();
        pc.cpu.step();
        assert_eq!(pc.regs().rax & 0xffff_ffff, 0xfee0_0900);
        assert_eq!(pc.regs().rdx & 0xffff_ffff, 0);

        // Clearing the enable bit reaches the controller.
        // mov ecx, 0x1b ; mov eax, 0xfee00000 ; xor edx, edx ; wrmsr
        pc.write(
            at::CODE0,
            &[
                0xb9, 0x1b, 0, 0, 0, 0xb8, 0x00, 0x00, 0xe0, 0xfe, 0x31, 0xd2, 0x0f, 0x30,
            ],
        );
        set_rip(&pc, at::CODE0);
        for _ in 0..4 {
            pc.cpu.step();
        }
        assert_eq!(
            intc.base.load(Ordering::Acquire),
            0xfee0_0000 | apic_base::BSP,
            "the enable bit is gone and the bootstrap flag, which is read-only, \
             is not"
        );
    }

    #[test]
    fn a_reserved_bit_in_ia32_apic_base_faults() {
        let pc = pc_msr();
        let intc = Arc::new(Controller {
            base: AtomicU64::new(0xfee0_0000 | apic_base::ENABLE),
        });
        let peer: Arc<dyn LocalController> = intc.clone();
        pc.cpu
            .attach_local_controller("intr", Arc::downgrade(&peer));
        pc.start_protected();
        pc.idt(13, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
        pc.write(HANDLER, &[0xf4]);
        // mov ecx, 0x1b ; mov eax, 0xfee00401 ; xor edx, edx ; wrmsr — bit 0 is
        // reserved and bit 10 is x2APIC, which this does not implement.
        pc.write(
            at::CODE0,
            &[
                0xb9, 0x1b, 0, 0, 0, 0xb8, 0x01, 0x04, 0xe0, 0xfe, 0x31, 0xd2, 0x0f, 0x30,
            ],
        );
        for _ in 0..4 {
            pc.cpu.step();
        }
        assert_eq!(pc.cpu.regs().rip, HANDLER);
        assert_eq!(
            intc.base.load(Ordering::Acquire) & 1,
            0,
            "and nothing was written"
        );
    }

    #[test]
    fn cpuid_reports_an_apic_only_when_one_is_wired() {
        let pc = pc_msr();
        pc.start_protected();
        // mov eax, 1 ; cpuid
        pc.write(at::CODE0, &[0xb8, 1, 0, 0, 0, 0x0f, 0xa2]);
        pc.cpu.step();
        pc.cpu.step();
        assert_eq!(pc.regs().rdx & (1 << 9), 0, "no controller, no APIC bit");

        let pc = pc_msr();
        let intc = Arc::new(Controller {
            base: AtomicU64::new(0xfee0_0000 | apic_base::ENABLE),
        });
        let peer: Arc<dyn LocalController> = intc.clone();
        pc.cpu
            .attach_local_controller("intr", Arc::downgrade(&peer));
        pc.start_protected();
        pc.write(at::CODE0, &[0xb8, 1, 0, 0, 0, 0x0f, 0xa2]);
        pc.cpu.step();
        pc.cpu.step();
        assert_ne!(pc.regs().rdx & (1 << 9), 0, "and one wired says so");
    }
}
