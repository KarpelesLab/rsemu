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
    assert_eq!(Regs::normalise_flags(Config::I8088, 0x0000), 0xf002);
    assert_eq!(Regs::normalise_flags(Config::I8088, 0xffff), 0xffd7);
}

#[test]
fn the_identification_flag_is_writable_on_a_part_that_has_cpuid() {
    // The probe every x86 startup path opens with, in the sixteen-bit real
    // mode a processor comes out of `RESET` and out of a Start-Up in: push the
    // flags, flip bit 21, pop them back, push again, and see whether it moved.
    // "The ability of a program to set or clear this flag is one way to
    // determine whether a processor supports the `CPUID` instruction" — *Intel
    // SDM* Vol 1 §3.4.3.3 — so a core that answers *no* here is telling a
    // guest it is a pre-1993 part whatever `CPUID` would have said.
    let m = Machine::new(Config::X86_64);
    m.load(
        0x0000,
        0x0100,
        &[
            0x66, 0x9c, // pushfd
            0x66, 0x58, // pop eax
            0x66, 0x35, 0x00, 0x00, 0x20, 0x00, // xor eax, 0x00200000
            0x66, 0x50, // push eax
            0x66, 0x9d, // popfd
            0x66, 0x9c, // pushfd
            0x66, 0x5b, // pop ebx
        ],
    );
    m.set_regs(|r| {
        r.ss = 0x2000;
        r.rsp = 0x0100;
    });
    for _ in 0..7 {
        m.cpu.step();
    }
    assert_eq!(
        m.regs().rbx as u32 & flags::ID,
        flags::ID,
        "the flag went in and did not come back out"
    );

    // And the part that has no `CPUID` to announce keeps none of it. The two
    // are the same fact: `Config::flag_mask`.
    let mut features = Config::I80486.features;
    features.extras_486 = false;
    assert_eq!(
        Regs::normalise_flags(Config::I80486.with_features(features), flags::ID) & flags::ID,
        0
    );
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

/// `level = "v2"` selects the x86-64-v2 preset from a machine file, and the
/// per-extension overrides still apply on top of it.
#[test]
fn a_machine_file_selects_x86_64_v2_by_level() {
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
                object cpu \"cpu.x86\" { clock = x, space = mem, variant = \"x86-64\", \
                level = \"v2\" }\n  \
                map mem 0 size 4K = dram\n}\n";
    crate::machine::build("t.machine", text, &registry, &options).expect("it builds");
    let cpu = kept.take().expect("the constructor kept a handle");
    assert_eq!(cpu.config().features, Features::X86_64_V2);
}

#[test]
fn the_level_property_is_a_preset_the_overrides_refine() {
    use crate::core::props::Props;
    let level = |extra: &[(&str, bool)], level: &str, variant: &str| {
        let mut props = Props::new().with("variant", variant).with("level", level);
        for &(name, on) in extra {
            props = props.with(name, on);
        }
        X86::from_props(&props).map(|cpu| cpu.config().features)
    };
    assert_eq!(level(&[], "v2", "x86-64").unwrap(), Features::X86_64_V2);
    assert_eq!(level(&[], "v1", "x86-64").unwrap(), Features::X86_64);
    // The variant alone is still the baseline: no existing machine changes.
    assert_eq!(Variant::X86_64.features(), Features::X86_64);
    // A leaf of the chain can go on its own...
    let v = level(&[("sse42", false)], "v2", "x86-64").unwrap();
    assert!(!v.sse42 && v.sse41);
    // ...but not a link: SSE4.1 needs SSSE3.
    assert!(level(&[("ssse3", false)], "v2", "x86-64").is_err());
    // A level is an x86-64 level, and only those three exist here.
    assert!(level(&[], "v2", "80486").is_err());
    assert!(level(&[], "v4", "x86-64").is_err());
    // v3 is v2 plus the AVX2-era set, and every link of its chain holds.
    let v3 = level(&[], "v3", "x86-64").unwrap();
    assert_eq!(v3, Features::X86_64_V3);
    assert!(v3.validate().is_ok());
    assert!(v3.avx2 && v3.fma && v3.bmi2 && v3.movbe && v3.lzcnt && v3.xsave);
    // FMA can go on its own; AVX cannot while AVX2 needs it.
    assert!(!level(&[("fma", false)], "v3", "x86-64").unwrap().fma);
    assert!(level(&[("avx", false)], "v3", "x86-64").is_err());
    // And v1 plus the right overrides is v2.
    let built = level(
        &[
            ("sse3", true),
            ("ssse3", true),
            ("sse41", true),
            ("sse42", true),
            ("popcnt", true),
            ("cx16", true),
            ("lahf_lm", true),
        ],
        "v1",
        "x86-64",
    )
    .unwrap();
    assert_eq!(built, Features::X86_64_V2);
    // An extension past v2 is a boolean on top of either level.
    assert!(level(&[("lzcnt", true)], "v1", "x86-64").unwrap().lzcnt);
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

/// A `cpu.x86` chunk from before XSAVE (v8) and from before AVX (v9)
/// migrates to exactly what this build writes for the same processor.
///
/// Both hops appended to the end of the chunk, so the older encodings are this
/// one's prefix — which is what makes the fixture honest rather than a second
/// reading of the same belief: v9 is the v10 bytes less the `YMM` upper halves,
/// v8 those less `XCR0`, and the steps must put back exactly what a processor
/// that never had the instructions would hold.
#[test]
fn an_older_chunk_migrates_to_what_this_build_writes() {
    use crate::core::state::StateReader;
    use alloc::borrow::Cow;
    let m = machine();
    m.set_regs(|r| {
        r.rax = 0x1234;
        r.rbx = 0x5678;
    });
    let mut shape = MachineShape::new();
    shape.add_device("/cpu0", "cpu.x86").unwrap();
    let mut writer = StateWriter::new(shape);
    {
        let mut chunk = writer
            .chunk("/cpu0", "cpu.x86", super::CLASS.version)
            .unwrap();
        m.cpu.save(&mut chunk).unwrap();
    }
    let bytes = writer.to_vec().unwrap();
    let reader = StateReader::new(&bytes).unwrap();
    let (_, version, v10) = reader.load_raw("/cpu0").unwrap();
    assert_eq!(version, 10);
    let v9 = &v10[..v10.len() - 16 * 16];
    let v8 = &v9[..v9.len() - 8];

    let table = crate::machine::default_migrations().unwrap();
    assert_eq!(table.steps_for("cpu.x86"), alloc::vec![8, 9]);
    for (from, old) in [(8, v8), (9, v9)] {
        let up = table
            .upgrade("cpu.x86", from, 10, Cow::Borrowed(old))
            .unwrap_or_else(|e| panic!("v{from} does not upgrade: {e}"));
        assert_eq!(&*up, v10, "v{from} migrates to this build's bytes");
    }
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
use super::paging;
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
fn the_window_between_setting_pe_and_the_far_jump_is_ring_zero() {
    // *Intel SDM* Vol 3A §9.9.1 puts the far jump that reloads `CS` **after**
    // the `MOV CR0` that sets `PE`. Between the two the processor is already
    // in protected mode with `CS` still holding the real-address-mode segment
    // it had — and the low two bits of a real-mode segment are part of an
    // address, not a privilege level. Real-address mode is privilege 0
    // (Vol 3A §20.1, which is why `ar::REAL_CODE` has no `DPL` bits), and so
    // is that window: the level lives in the cached descriptor, and nothing
    // has reloaded it yet.
    //
    // The instructions a memory manager runs in there are the reason this is
    // not academic. JemmEx, loaded by FreeDOS at `CS = 0x029e`, sets `PE` and
    // then executes `LLDT AX` with `AX` zero before it jumps — invalidating
    // the local table, which Vol 2A's `LLDT` entry says a null selector does
    // rather than faulting. A core that read the level out of the selector saw
    // ring 2 there, raised `#GP(0)` on that `LLDT`, and — the interrupt
    // descriptor table register still holding the real-mode vector table —
    // escalated to a double fault and then to shutdown.
    let pc = pc386();
    pc.gdt(0, (0, 0));
    pc.gdt(1, descriptor(0, 0xffff_ffff, rights::CODE32));
    pc.gdt(2, descriptor(0, 0xffff_ffff, rights::DATA32));

    // `CS = DS = 0x029e`, JemmEx's own, so the selector's low two bits are
    // `10` and a core that mistook them for a level would find ring 2.
    const SEG: u16 = 0x029e;
    let base = u64::from(SEG) << 4;
    // The six-byte pseudo-descriptor, at `DS:0x40`.
    pc.write(base + 0x40, &[0x17, 0x00]);
    pc.write32(base + 0x42, at::GDT);

    pc.start_real(SEG, 0);
    // `DS` alongside it, the way a `.COM`-style driver image is entered; in
    // real mode `set_regs` recomputes the cached bases from the selectors.
    let mut regs = pc.regs();
    regs.ds = SEG;
    regs.ss = SEG;
    regs.rsp = 0x100;
    pc.cpu.set_regs(regs);
    pc.write(
        base,
        &[
            0xfa, // cli
            0x0f, 0x01, 0x16, 0x40, 0x00, // lgdt [0x40]
            0x0f, 0x20, 0xc0, // mov eax, cr0
            0x0c, 0x01, // or al, 1          — PE, and nothing else
            0x0f, 0x22, 0xc0, // mov cr0, eax
            0x31, 0xc0, // xor ax, ax
            0x0f, 0x00, 0xd0, // lldt ax     — ring 0 only, and a null selector
            0x0f, 0x22, 0xd8, // mov cr3, eax — ring 0 only
            0xc7, 0x06, 0x50, 0x00, 0x34, 0x12, // mov word [0x50], 0x1234
            0xf4, // hlt
        ],
    );

    let steps = pc.run(20);
    assert!(steps < 20, "the sequence should have reached its hlt");
    let sys = pc.cpu.sys();
    assert!(sys.protected(), "`PE` is set");
    assert_eq!(
        pc.regs().rip,
        0x1d,
        "every instruction after `MOV CR0` ran: nothing faulted"
    );
    assert!(
        !sys.ldtr.present(),
        "`LLDT` with a null selector marks the local table invalid"
    );
    assert_eq!(sys.cr3, 0, "`MOV CR3` was allowed too");
    assert_eq!(
        pc.read32(base + 0x50) & 0xffff,
        0x1234,
        "the store after them landed"
    );
    // And the negative half: with the level read out of the selector instead,
    // the first of those three would have faulted into the real-mode vector
    // table and shut the processor down.
    assert!(!pc.cpu.is_halted() || steps < 20);
}

#[test]
fn a_conforming_code_segment_does_not_hand_its_caller_ring_zero() {
    // The level is kept in the `CS` descriptor cache, so the thing cached
    // there has to be the level and not the table's `DPL`. A **conforming**
    // code segment is where the two part company: a transfer to one does not
    // change privilege (*Intel SDM* Vol 3A §5.8.1.2), so a ring-3 program may
    // jump into a `DPL` 0 conforming segment and keeps running at ring 3.
    // Caching the descriptor's own `DPL` there would promote it.
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
    // Conforming, readable, and at privilege 0 — the combination that is
    // reachable from anywhere.
    pc.gdt(6, descriptor(0, 0xffff_ffff, rights::CODE32 | ar::DC));
    pc.write32(at::TSS + tss32::ESP0, at::STACK0);
    pc.write32(at::TSS + tss32::SS0, 0x10);
    pc.idt(13, gate(0x08, 0x3100, sys_type::INT_GATE32, 0));
    pc.write(0x3100, &[0xf4]);

    pc.write(
        at::CODE0,
        &[
            0xb8, 0x28, 0x00, 0x00, 0x00, 0x0f, 0x00, 0xd8, // ltr 0x28
            0x6a, 0x23, 0x68, 0x00, 0xa0, 0x00, 0x00, 0x6a, 0x02, 0x6a, 0x1b, 0x68, 0x00, 0x40,
            0x00, 0x00, 0xcf, // iretd into ring 3 at CODE3
        ],
    );
    // Ring 3 jumps into the conforming segment and tries to halt from inside
    // it. `RPL` 3 on the selector, because that is what ring 3 has to write.
    pc.write(
        at::CODE3,
        &[
            0xea, 0x00, 0x50, 0x00, 0x00, 0x33, 0x00, // jmp 0x0033:0x00005000
        ],
    );
    pc.write(0x5000, &[0xf4]); // hlt

    for _ in 0..8 {
        pc.cpu.step();
    }
    assert_eq!(pc.regs().cs, 0x1b, "the iret entered ring 3");
    pc.cpu.step(); // the far jump
    let regs = pc.regs();
    assert_eq!(regs.rip, 0x5000, "the jump was allowed: conforming");
    assert_eq!(
        regs.cs & 3,
        3,
        "and it did not change the privilege level (§5.8.1.2)"
    );
    assert_eq!(
        pc.cpu.sys().seg(isa::seg::CS).dpl(),
        3,
        "the cached descriptor holds the level, not the table's DPL"
    );
    pc.cpu.step(); // hlt, from ring 3
    assert_eq!(pc.regs().rip, 0x3100, "hlt at ring 3 is still #GP");
    assert!(!pc.cpu.is_halted());
}

/// Where a virtual-8086 task's own segments live in the 4 MiB these tests
/// have. Paragraph numbers, because that is what a task loads.
mod v86_at {
    /// The task's code, at linear `0xb000`.
    pub(super) const CODE: u16 = 0x0b00;
    /// Its data, at `0xc000`.
    pub(super) const DATA: u16 = 0x0c00;
    /// Its stack, at `0xd000`.
    pub(super) const STACK: u16 = 0x0d00;
    /// The monitor's handler for the task's `INT 30h`.
    pub(super) const RESUME: u32 = 0x3100;
    /// And for its `INT 31h`, which stops the test.
    pub(super) const STOP: u32 = 0x3200;
}

/// The ring-0 preamble every virtual-8086 test shares: load the task register
/// and `IRET` into a task with `flags` as its flags image.
fn enter_v86_code(flags: u32) -> Vec<u8> {
    let mut code: Vec<u8> = Vec::new();
    code.extend_from_slice(&[
        0xb8, 0x28, 0x00, 0x00, 0x00, // mov eax, 0x28
        0x0f, 0x00, 0xd8, // ltr ax
    ]);
    // The frame *Intel SDM* Vol 3A §20.2.1 prescribes, pushed from the top
    // down so that it reads EIP, CS, EFLAGS, ESP, SS, ES, DS, FS, GS upwards.
    for value in [
        0u32,                     // GS
        0,                        // FS
        u32::from(v86_at::DATA),  // DS
        u32::from(v86_at::DATA),  // ES
        u32::from(v86_at::STACK), // SS
        0x100,                    // ESP
        flags,                    // EFLAGS, with VM
        u32::from(v86_at::CODE),  // CS
        0,                        // EIP
    ] {
        code.push(0x68); // push imm32
        code.extend_from_slice(&value.to_le_bytes());
    }
    code.push(0xcf); // iretd
    code
}

#[test]
fn an_iret_with_vm_set_runs_an_8086_task_and_an_interrupt_brings_it_back() {
    // The whole round trip *Intel SDM* Vol 3A §20.2 describes, and the reason
    // it is here: JemmEx — the memory manager FreeDOS loads out of
    // `FDCONFIG.SYS` — builds exactly this frame, and a core without the mode
    // answered its `IRETD` with `#GP` because it read the 8086 segment value
    // in the frame as a selector and went looking for it in the descriptor
    // table.
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
    pc.write32(at::TSS + tss32::ESP0, at::STACK0);
    pc.write32(at::TSS + tss32::SS0, 0x10);
    // Both gates are at `DPL` 3, because the task is at privilege 3 and may
    // not invoke a gate above itself.
    pc.idt(0x30, gate(0x08, v86_at::RESUME, sys_type::INT_GATE32, 3));
    pc.idt(0x31, gate(0x08, v86_at::STOP, sys_type::INT_GATE32, 3));
    pc.write(u64::from(v86_at::RESUME), &[0xcf]); // iretd, back into the task
    pc.write(u64::from(v86_at::STOP), &[0xf4]); // hlt, in the monitor

    // `VM`, `IOPL` 3, and the bit that is always set. `IOPL` 3 is what a
    // monitor gives a task that is allowed to run `CLI`, `STI` and `INT n`
    // without a trap (§20.2.7).
    const TASK_FLAGS: u32 = flags::VM | flags::IOPL | flags::ALWAYS_SET;
    pc.write(at::CODE0, &enter_v86_code(TASK_FLAGS));

    let base = u64::from(v86_at::CODE) << 4;
    let data = u64::from(v86_at::DATA) << 4;
    pc.write(
        base,
        &[
            0xb8, 0x34, 0x12, // mov ax, 0x1234
            0xa3, 0x10, 0x00, // mov [0x10], ax      — through DS
            0x2e, 0xa3, 0x40, 0x00, // mov cs:[0x40], ax  — through CS
            0xcd, 0x30, // int 0x30            — out to the monitor
            0xb8, 0x78, 0x56, // mov ax, 0x5678
            0xa3, 0x12, 0x00, // mov [0x12], ax      — and back again
            0xcd, 0x31, // int 0x31            — stop
        ],
    );

    // `mov eax` / `ltr` / nine pushes / `iretd`.
    for _ in 0..12 {
        pc.cpu.step();
    }
    let regs = pc.regs();
    assert_ne!(regs.eflags & flags::VM, 0, "the IRET entered the task");
    assert_eq!(regs.cs, v86_at::CODE, "CS is a segment, not a selector");
    assert_eq!(regs.rip, 0, "at the task's first instruction");
    let sys = pc.cpu.sys();
    assert_eq!(sys.seg(isa::seg::CS).base, base, "shifted four bits");
    assert_eq!(sys.seg(isa::seg::CS).limit, 0xffff, "and 64 KiB long");
    assert_eq!(
        sys.seg(isa::seg::CS).dpl(),
        3,
        "a virtual-8086 task runs at privilege 3 (§20.2.4)"
    );
    assert_eq!(regs.rsp & 0xffff, 0x100, "ESP came off the frame");

    // The three instructions before the `INT`.
    for _ in 0..4 {
        pc.cpu.step();
    }
    assert_eq!(
        pc.read32(data + 0x10) & 0xffff,
        0x1234,
        "the store through DS landed at DS << 4"
    );
    assert_eq!(
        pc.read32(base + 0x40) & 0xffff,
        0x1234,
        "and the one through CS landed too: an 8086 task has no segment types \
         to check (§20.2.2)"
    );

    // And the `INT 30h` is now in the monitor, on the ring-0 stack, with the
    // nine-doubleword frame §20.3.1 describes under it.
    let regs = pc.regs();
    assert_eq!(regs.eflags & flags::VM, 0, "VM is cleared on the way out");
    assert_eq!(regs.cs, 0x08, "in the monitor's code segment");
    assert_eq!(regs.rip, u64::from(v86_at::RESUME));
    assert_eq!(regs.ss, 0x10, "on the stack the TSS names");
    assert_eq!(regs.rsp, at::STACK0 - 36, "nine doublewords of frame");
    assert_eq!(pc.read32(at::STACK0 - 4), 0, "GS");
    assert_eq!(pc.read32(at::STACK0 - 8), 0, "FS");
    assert_eq!(pc.read32(at::STACK0 - 12), u64::from(v86_at::DATA), "DS");
    assert_eq!(pc.read32(at::STACK0 - 16), u64::from(v86_at::DATA), "ES");
    assert_eq!(pc.read32(at::STACK0 - 20), u64::from(v86_at::STACK), "SS");
    assert_eq!(pc.read32(at::STACK0 - 24), 0x100, "ESP");
    assert_eq!(
        pc.read32(at::STACK0 - 28) as u32 & flags::VM,
        flags::VM,
        "the saved flags still have VM — it is how the IRET back in knows"
    );
    assert_eq!(pc.read32(at::STACK0 - 32), u64::from(v86_at::CODE), "CS");
    assert_eq!(pc.read32(at::STACK0 - 36), 12, "EIP, after the INT");
    assert_eq!(regs.ds, 0, "the task's data segments are not the handler's");
    assert_eq!(regs.es, 0);

    // The handler's `IRET` goes straight back in, and the task finishes.
    let steps = pc.run(20);
    assert!(steps < 20, "the task should have reached the monitor's hlt");
    assert!(pc.cpu.is_halted());
    assert_eq!(
        pc.read32(data + 0x10),
        0x5678_1234,
        "the second store proves the monitor's IRET resumed the task"
    );
}

#[test]
fn a_virtual_8086_task_below_iopl_three_traps_on_the_six_sensitive_instructions() {
    // *Intel SDM* Vol 3A §20.2.7: `CLI`, `STI`, `PUSHF`, `POPF`, `INT n` and
    // `IRET` are `IOPL`-sensitive in virtual-8086 mode, and below `IOPL` 3
    // each one is `#GP(0)` so that the monitor sees it. A `CLI` the task
    // believed had worked would leave it running with interrupts it thinks it
    // masked, which is the defect the mechanism exists to prevent.
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
    pc.write32(at::TSS + tss32::ESP0, at::STACK0);
    pc.write32(at::TSS + tss32::SS0, 0x10);
    pc.idt(13, gate(0x08, v86_at::STOP, sys_type::INT_GATE32, 0));
    pc.write(u64::from(v86_at::STOP), &[0xf4]); // hlt

    // The same entry, with `IOPL` 0 instead of 3.
    pc.write(at::CODE0, &enter_v86_code(flags::VM | flags::ALWAYS_SET));
    pc.write(u64::from(v86_at::CODE) << 4, &[0xfa]); // cli

    for _ in 0..12 {
        pc.cpu.step();
    }
    assert_ne!(pc.regs().eflags & flags::VM, 0, "the task is running");
    pc.cpu.step(); // cli
    let regs = pc.regs();
    assert_eq!(regs.eflags & flags::VM, 0, "the fault left the task");
    assert_eq!(
        regs.rip,
        u64::from(v86_at::STOP),
        "CLI below IOPL 3 is #GP, not a silent success"
    );
    assert_eq!(pc.read32(at::STACK0 - 40), 0, "#GP's error code is zero");
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

/// A part with the first 4 MiB identity-mapped, paging on, and a program that
/// reads the same far linear address `reads` times before halting.
///
/// Returns the built machine and the clocks the program cost, so a caller can
/// look at both the cycle column and the buffers afterwards.
fn paged_far_read(variant: Variant, data: u64, reads: usize) -> (Pc, u64) {
    let pc = Pc::new(variant);
    pc.start_protected();
    pc.write32(at::PDIR, at::PTAB | 0b111);
    for page in 0..1024u64 {
        pc.write32(at::PTAB + page * 4, (page << 12) | 0b111);
    }
    let mut sys = pc.cpu.sys();
    sys.cr3 = at::PDIR;
    sys.cr0 |= cr0::PG;
    pc.cpu.set_sys(sys);

    let mut code = Vec::new();
    for _ in 0..reads {
        // mov eax, [imm32] — six bytes of fetch and one four-byte load, all
        // of them through the translation buffers.
        code.push(0xa1);
        code.extend_from_slice(&(data as u32).to_le_bytes());
    }
    code.push(0xf4);
    pc.write(at::CODE0, &code);

    let before = pc.cpu.cycles();
    let steps = pc.run(reads + 4);
    assert_eq!(
        steps,
        reads + 1,
        "the program ran to its hlt and no further"
    );
    let cost = pc.cpu.cycles() - before;
    (pc, cost)
}

#[test]
fn a_far_data_operand_evicts_the_code_page_on_a_486_and_not_on_a_part_with_two_buffers() {
    // The guest-visible consequence of `paging::Buffers`, measured in the one
    // column that is hashed: clocks. A 386 and a 486 have **one** buffer
    // (*Intel 80386 Programmer's Reference Manual* §5.2; *Intel486 …
    // Programmer's Reference Manual* chapter 4), so an operand 128 KiB from
    // the code page lands in the code page's slot, throws it out, and makes
    // the next instruction fetch re-walk. Every part since the Pentium has
    // two, so the same program costs the same as one reading the address a
    // page along.
    const READS: usize = 16;
    let code_page = at::CODE0 >> 12;
    let collides = (code_page + paging::ITLB_ENTRIES as u64) << 12;
    let apart = (code_page + paging::ITLB_ENTRIES as u64 + 1) << 12;

    let (unified, unified_collides) = paged_far_read(Variant::I80486, collides, READS);
    let (_, unified_apart) = paged_far_read(Variant::I80486, apart, READS);
    assert!(
        unified_collides > unified_apart,
        "one buffer: {unified_collides} clocks colliding against {unified_apart} apart"
    );
    // And the mechanism rather than the symptom: the two translations cannot
    // both be held. The last thing the program did was fetch its `hlt`, so it
    // is the *data* translation that is gone by the end — which is the same
    // eviction seen from the other side.
    assert!(
        unified
            .cpu
            .session
            .lock()
            .state
            .tlb
            .get(collides, false)
            .is_none(),
        "one buffer: the fetch that followed the load threw the load out"
    );

    let (split, split_collides) = paged_far_read(Variant::X86_64, collides, READS);
    let (_, split_apart) = paged_far_read(Variant::X86_64, apart, READS);
    assert_eq!(
        split_collides, split_apart,
        "two buffers: a data operand cannot index the instruction array"
    );
    assert!(
        split
            .cpu
            .session
            .lock()
            .state
            .tlb
            .get(at::CODE0, true)
            .is_some(),
        "the code page's translation survived every data access"
    );
    assert!(
        split
            .cpu
            .session
            .lock()
            .state
            .tlb
            .get(collides, false)
            .is_some(),
        "and the data translation survived every fetch"
    );
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

// ---------------------------------------------------------------------------
// The bus lock
// ---------------------------------------------------------------------------
//
// `tests/x86_bus_lock.rs` is the guest-level instrument — two cores on two host
// threads, and a counter that used to come out short. These pin the decisions
// that instrument cannot see: *which* instructions assert `LOCK#`, that an
// ordinary one asserts nothing, and that a fault gives the bus back.

/// `LOCK` on a memory read-modify-write takes the space's bus lock, and an
/// ordinary store does not.
///
/// The second half is the cost claim: the lock is affordable precisely because
/// nothing but a locked access ever reaches it.
#[test]
fn a_lock_prefix_takes_the_bus_and_an_ordinary_store_does_not() {
    let pc = pc386();
    pc.start_protected();
    let space = pc.cpu.space().expect("attached");
    pc.write(
        at::CODE0,
        &[
            0xb8, 0x01, 0x00, 0x00, 0x00, // mov eax, 1
            0xa3, 0x00, 0x70, 0x00, 0x00, // mov [MARK], eax
            0xf0, 0x0f, 0xc1, 0x05, 0x00, 0x70, 0x00, 0x00, // lock xadd [MARK], eax
            0xf4,
        ],
    );
    pc.cpu.step();
    pc.cpu.step();
    assert_eq!(
        space.bus_lock().taken(),
        0,
        "a `mov` to memory is not a locked transaction"
    );
    pc.cpu.step();
    assert_eq!(space.bus_lock().taken(), 1, "and `lock xadd` is");
    assert!(
        !space.bus_lock().held(),
        "released when the instruction ended"
    );
    assert_eq!(pc.read32(at::MARK), 2, "the increment still lands");
    assert_eq!(pc.regs().rax, 1, "and the old value comes back");
}

/// `XCHG` with a memory operand is locked **implicitly** — prefix or not.
///
/// *Intel SDM* volume 2, `XCHG`: if a memory operand is referenced, the
/// processor's locking protocol is implemented automatically for the duration
/// of the exchange, whether or not a `LOCK` prefix is present. It is the only
/// instruction with that property, and a guest that spins on
/// `xchg [lock], eax` — which is what a hand-written spinlock does — depends
/// on it.
#[test]
fn xchg_with_memory_is_locked_without_a_prefix_and_between_registers_is_not() {
    let pc = pc386();
    pc.start_protected();
    let space = pc.cpu.space().expect("attached");
    pc.write32(at::MARK, 0x1234);
    pc.write(
        at::CODE0,
        &[
            0xb8, 0x99, 0x00, 0x00, 0x00, // mov eax, 0x99
            0x87, 0x05, 0x00, 0x70, 0x00, 0x00, // xchg [MARK], eax
            0x87, 0xd8, // xchg eax, ebx
            0xf4,
        ],
    );
    pc.cpu.step();
    pc.cpu.step();
    assert_eq!(space.bus_lock().taken(), 1, "the memory form asserts LOCK#");
    assert_eq!(pc.read32(at::MARK), 0x99);
    assert_eq!(pc.regs().rax, 0x1234);
    pc.cpu.step();
    assert_eq!(
        space.bus_lock().taken(),
        1,
        "the register form touches no bus and must not take one"
    );
}

/// A locked instruction that faults gives the bus back before the exception is
/// delivered.
///
/// Hardware deasserts `LOCK#` when the operation is aborted, and here it
/// matters more than for accuracy: the guard is dropped on the way out of
/// `Exec::instruction`, so if it were not, one `#GP` would wedge every other
/// master on the space forever. The fault is a segment-limit violation, which
/// is raised inside `execute` — after the bus was taken.
#[test]
fn a_locked_instruction_that_faults_releases_the_bus() {
    let pc = pc386();
    pc.start_protected();
    let space = pc.cpu.space().expect("attached");
    let mut sys = pc.cpu.sys();
    sys.segs[usize::from(isa::seg::DS)].limit = 0x100;
    pc.cpu.set_sys(sys);
    pc.write(
        at::CODE0,
        &[0xf0, 0x0f, 0xc1, 0x05, 0x00, 0x70, 0x00, 0x00], // lock xadd [MARK], eax
    );
    pc.cpu.step();
    assert_eq!(space.bus_lock().taken(), 1, "the bus was taken");
    assert!(!space.bus_lock().held(), "and given back through the fault");
    assert_eq!(pc.read32(at::MARK), 0, "the store never happened");
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

/// And the counter a guest reads carries none of that debt.
///
/// `State::cycles` is charged whole instructions, so a core that stopped
/// part-way through a budget has charged cycles its clock domain has not
/// handed out yet — and a guest reading them stands ahead of its own domain,
/// and so ahead of a processor beside it on the same crystal, which is what a
/// kernel's time-stamp synchronisation check measures (*Intel SDM* vol. 3B
/// §17.17.1). `X86::tsc` is the counter without it: after every budget, it has
/// advanced by exactly the ticks that were granted.
#[test]
fn the_counter_a_guest_reads_advances_by_the_budget_and_not_by_the_debt() {
    let pc = pc386();
    pc.start_protected();
    // Increments and a jump: three, four and seven clocks, so a budget of five
    // lands inside an instruction most rounds and the debt is rarely zero.
    pc.write(at::CODE0, &[0x40, 0x40, 0x40, 0xeb, 0xfb]);
    let before = pc.cpu.tsc();
    let mut granted = 0u64;
    let mut saw_debt = false;
    for _ in 0..64 {
        granted += pc.cpu.run_budget(5);
        saw_debt |= pc.cpu.cycle_debt() > 0;
        assert_eq!(
            pc.cpu.tsc(),
            before + granted,
            "the counter is the ticks this core's domain has handed it"
        );
        assert_eq!(
            pc.cpu.tsc() + pc.cpu.cycle_debt(),
            pc.cpu.cycles(),
            "and the difference from the charge count is exactly the debt"
        );
    }
    assert_eq!(granted, 320, "every tick of every budget was granted");
    assert!(
        saw_debt,
        "no budget ever ended inside an instruction, so this asserted nothing"
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
    use crate::core::wire::{LocalController, Startup};
    use crate::cpu::x86::Features;
    use crate::cpu::x86::paging::Mode;
    use crate::cpu::x86::prot::{apic_base, cr4, efer, msr};

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
        run64_on(pc64(), 0, code)
    }

    /// [`run64`] on a part with `features`, with `CR4.OSFXSR` set so the
    /// SIMD extensions are usable.
    fn run64_with(features: Features, code: &[u8]) -> Pc {
        let pc = Pc::with_features(Variant::X86_64, features);
        run64_on(pc, cr4::OSFXSR | cr4::OSXMMEXCPT, code)
    }

    pub(super) fn run64_on(pc: Pc, cr4_bits: u64, code: &[u8]) -> Pc {
        pc.start_protected();
        let mut sys = pc.cpu.sys();
        sys.cr4 |= cr4_bits;
        pc.cpu.set_sys(sys);
        pc.prepare_long();
        pc.write(at::CODE0, &enter_long_mode_code(la::CODE64));
        pc.write(la::CODE64, code);
        let steps = pc.run(200);
        assert!(steps < 200, "the 64-bit program halted");
        assert!(pc.cpu.sys().sixty_four(), "and did so in 64-bit mode");
        pc
    }

    /// The x86-64-v2 extensions on top of the baseline.
    fn v2_features() -> Features {
        Features {
            sse3: true,
            ssse3: true,
            sse41: true,
            ..Features::X86_64
        }
    }

    #[test]
    fn pinsrq_and_pextrq_are_the_rex_w_forms_and_narrow_writes_zero_extend() {
        let mut code = alloc::vec![0x48u8, 0xb8];
        code.extend_from_slice(&0x1122_3344_5566_7788u64.to_le_bytes()); // mov rax, imm64
        code.extend_from_slice(&[0x66, 0x48, 0x0f, 0x3a, 0x22, 0xc8, 0x01]); // pinsrq xmm1, rax, 1
        code.extend_from_slice(&[0x66, 0x48, 0x0f, 0x3a, 0x16, 0xcb, 0x01]); // pextrq rbx, xmm1, 1
        code.extend_from_slice(&[0x48, 0xc7, 0xc1, 0xff, 0xff, 0xff, 0xff]); // mov rcx, -1
        code.extend_from_slice(&[0x66, 0x0f, 0x3a, 0x16, 0xc9, 0x03]); // pextrd ecx, xmm1, 3
        code.extend_from_slice(&[0x48, 0xc7, 0xc2, 0xff, 0xff, 0xff, 0xff]); // mov rdx, -1
        code.extend_from_slice(&[0x66, 0x0f, 0x3a, 0x14, 0xca, 0x08]); // pextrb edx, xmm1, 8
        code.push(0xf4);
        let pc = run64_with(v2_features(), &code);
        let r = pc.regs();
        assert_eq!(pc.cpu.sse().get(1)[1], 0x1122_3344_5566_7788);
        assert_eq!(r.rbx, 0x1122_3344_5566_7788);
        assert_eq!(r.rcx, 0x1122_3344, "a 32-bit write clears the upper half");
        assert_eq!(r.rdx, 0x88, "so does PEXTRB's");
    }

    /// `[disp32]` with no base and no index, which in 64-bit mode needs a SIB
    /// byte because `mod == 00`, `r/m == 101` is `RIP`-relative there.
    fn abs32(reg: u8, addr: u64) -> Vec<u8> {
        let mut out = alloc::vec![((reg & 7) << 3) | 0b100, 0x25];
        out.extend_from_slice(&(addr as u32).to_le_bytes());
        out
    }

    #[test]
    fn crc32_with_rex_w_reads_a_quadword_and_writes_a_whole_register() {
        // mov rax, -1 ; crc32 rax, qword [MARK] ; crc32 rax, byte [MARK+8]
        // ; mov rbx, rax ; not eax — the CRC-32C check value of "123456789".
        // `F2` has to come before `REX`, which must be the last prefix.
        let pc = Pc::with_features(
            Variant::X86_64,
            Features {
                sse42: true,
                ..v2_features()
            },
        );
        pc.write(la::MARK, b"123456789");
        let mut code = alloc::vec![0x48, 0xc7, 0xc0, 0xff, 0xff, 0xff, 0xff];
        code.extend_from_slice(&[0xf2, 0x48, 0x0f, 0x38, 0xf1]);
        code.extend_from_slice(&abs32(0, la::MARK));
        code.extend_from_slice(&[0xf2, 0x48, 0x0f, 0x38, 0xf0]);
        code.extend_from_slice(&abs32(0, la::MARK + 8));
        code.extend_from_slice(&[0x48, 0x89, 0xc3, 0xf7, 0xd0, 0xf4]);
        let pc = run64_on(pc, cr4::OSFXSR, &code);
        let r = pc.regs();
        assert_eq!(r.rbx, u64::from(!0xe306_9283u32), "upper half cleared");
        assert_eq!(r.rax, 0xe306_9283);
    }

    /// Run `code` in 64-bit mode on a part with `features` and require that
    /// it raises `vector`, taken by a handler that halts.
    pub(super) fn faults64(features: Features, vector: u64, code: &[u8]) {
        let pc = Pc::with_features(Variant::X86_64, features);
        pc.idt64(vector, 0x18, la::HANDLER);
        pc.write(la::HANDLER, &[0xf4]);
        let mut all = code.to_vec();
        all.push(0xf4);
        let pc = run64_on(pc, 0, &all);
        assert_eq!(
            pc.regs().rip,
            la::HANDLER + 1,
            "{code:02x?} raises vector {vector}"
        );
    }

    /// `lock cmpxchg16b [addr]`.
    fn cmpxchg16b(addr: u64) -> Vec<u8> {
        let mut code = alloc::vec![0xf0, 0x48, 0x0f, 0xc7];
        code.extend_from_slice(&abs32(1, addr));
        code
    }

    /// Load `RDX:RAX` and `RCX:RBX` with `mov r64, imm64`.
    fn pairs(want: [u64; 2], new: [u64; 2]) -> Vec<u8> {
        let mut code = Vec::new();
        for (rex_op, value) in [
            (0xb8u8, want[0]),
            (0xba, want[1]),
            (0xbb, new[0]),
            (0xb9, new[1]),
        ] {
            code.extend_from_slice(&[0x48, rex_op]);
            code.extend_from_slice(&value.to_le_bytes());
        }
        code
    }

    #[test]
    fn cmpxchg16b_exchanges_sixteen_bytes_on_a_part_with_cx16() {
        let features = Features {
            cx16: true,
            ..Features::X86_64
        };
        // Equal: RCX:RBX is stored and ZF set.
        let pc = Pc::with_features(Variant::X86_64, features);
        pc.write64(la::MARK, 0x1111);
        pc.write64(la::MARK + 8, 0x2222);
        let mut code = pairs([0x1111, 0x2222], [0x3333, 0x4444]);
        code.extend_from_slice(&cmpxchg16b(la::MARK));
        code.push(0xf4);
        let pc = run64_on(pc, 0, &code);
        assert_eq!(pc.read64(la::MARK), 0x3333);
        assert_eq!(pc.read64(la::MARK + 8), 0x4444);
        assert_ne!(pc.regs().eflags & flags::ZF, 0);
        // Unequal: memory is loaded into RDX:RAX and ZF cleared.
        let pc = Pc::with_features(Variant::X86_64, features);
        pc.write64(la::MARK, 0x1111);
        pc.write64(la::MARK + 8, 0x2222);
        let mut code = pairs([0x1111, 0x9999], [0x3333, 0x4444]);
        code.extend_from_slice(&cmpxchg16b(la::MARK));
        code.push(0xf4);
        let pc = run64_on(pc, 0, &code);
        assert_eq!(pc.read64(la::MARK), 0x1111, "memory unchanged");
        let r = pc.regs();
        assert_eq!((r.rax, r.rdx), (0x1111, 0x2222));
        assert_eq!(r.eflags & flags::ZF, 0);
        // Sixteen-byte alignment is required, and says so with #GP(0).
        faults64(features, 13, &cmpxchg16b(la::MARK + 8));
    }

    #[test]
    fn cmpxchg16b_is_undefined_without_cx16_but_cmpxchg8b_is_not() {
        // The baseline: CX8 and no CX16, as the first long-mode parts.
        faults64(Features::X86_64, 6, &cmpxchg16b(la::MARK));
        let mut code = alloc::vec![0xf0, 0x0f, 0xc7];
        code.extend_from_slice(&abs32(1, la::MARK));
        code.push(0xf4);
        run64_on(pc64(), 0, &code);
        // CX16 cannot be had without long mode or without CX8.
        for impossible in [
            Features {
                cx16: true,
                ..Features::I80486
            },
            Features {
                cx16: true,
                cx8: false,
                ..Features::X86_64
            },
        ] {
            assert!(impossible.validate().is_err());
        }
    }

    #[test]
    fn lahf_and_sahf_need_lahf_lm_in_64_bit_mode_only() {
        // mov ah, 0xd5 ; sahf ; mov ah, 0 ; lahf: every status flag round
        // trips through AH.
        let code = [0xb4, 0xd5, 0x9e, 0xb4, 0x00, 0x9f, 0xf4];
        let features = Features {
            lahf_lm: true,
            ..Features::X86_64
        };
        let pc = run64_on(Pc::with_features(Variant::X86_64, features), 0, &code);
        assert_eq!(
            (pc.regs().rax >> 8) & 0xff,
            0xd7,
            "SF ZF AF PF CF and bit 1"
        );
        // The baseline has neither in 64-bit mode.
        faults64(Features::X86_64, 6, &[0x9f]);
        faults64(Features::X86_64, 6, &[0x9e]);
        // And everywhere else they need nothing: the 32-bit bring-up runs
        // them on the same baseline part.
        let pc = pc64();
        pc.start_protected();
        pc.write(at::CODE0, &code);
        pc.run(20);
        assert_eq!((pc.regs().rax >> 8) & 0xff, 0xd7);
        let impossible = Features {
            lahf_lm: true,
            ..Features::I80486
        };
        assert!(impossible.validate().is_err());
    }

    #[test]
    fn cpuid_reports_cx16_and_lahf_lm_from_the_features() {
        // mov eax, 1 ; cpuid ; mov esi, ecx ; mov eax, 0x80000001 ; cpuid
        let code = [
            0xb8, 0x01, 0x00, 0x00, 0x00, 0x0f, 0xa2, 0x89, 0xce, 0xb8, 0x01, 0x00, 0x00, 0x80,
            0x0f, 0xa2, 0xf4,
        ];
        let leaves = |features: Features| {
            let pc = run64_on(Pc::with_features(Variant::X86_64, features), 0, &code);
            (pc.regs().rsi, pc.regs().rcx)
        };
        let (ecx1, ecx8) = leaves(Features::X86_64);
        assert_eq!(ecx1 & (1 << 13), 0);
        assert_eq!(ecx8 & 1, 0);
        let (ecx1, ecx8) = leaves(Features {
            cx16: true,
            lahf_lm: true,
            ..Features::X86_64
        });
        assert_ne!(ecx1 & (1 << 13), 0, "CPUID.01H:ECX.CMPXCHG16B");
        assert_ne!(ecx8 & 1, 0, "CPUID.80000001H:ECX.LAHF-SAHF");
    }

    /// `CPUID` with `EAX = leaf` and `ECX = sub` on a part with `features`,
    /// in 64-bit mode: `(EAX, EBX, ECX, EDX)`.
    pub(super) fn cpuid64(features: Features, leaf: u32, sub: u32) -> (u32, u32, u32, u32) {
        let mut code = alloc::vec![0xb8];
        code.extend_from_slice(&leaf.to_le_bytes());
        code.push(0xb9);
        code.extend_from_slice(&sub.to_le_bytes());
        code.extend_from_slice(&[0x0f, 0xa2, 0xf4]);
        let pc = run64_on(Pc::with_features(Variant::X86_64, features), 0, &code);
        let r = pc.regs();
        (r.rax as u32, r.rbx as u32, r.rcx as u32, r.rdx as u32)
    }

    /// `level = "v3"` reports the System V ABI's v3 list and nothing past it:
    /// AVX, F16C, FMA, MOVBE, XSAVE in leaf 1; AVX2, BMI1 and BMI2 in leaf 7;
    /// `LZCNT` in `8000_0001`. `OSXSAVE` stays clear until the guest sets
    /// `CR4.OSXSAVE`, because it is the operating system's half of the bit.
    #[test]
    fn the_v3_preset_reports_the_v3_bits() {
        let v3 = Features::X86_64_V3;
        let max = cpuid64(v3, 0, 0).0;
        let ecx1 = cpuid64(v3, 1, 0).2;
        assert_eq!(max, 0xd, "XSAVE's leaf is the highest");
        for (bit, name) in [
            (0, "SSE3"),
            (9, "SSSE3"),
            (12, "FMA"),
            (13, "CX16"),
            (19, "SSE4.1"),
            (20, "SSE4.2"),
            (22, "MOVBE"),
            (23, "POPCNT"),
            (26, "XSAVE"),
            (28, "AVX"),
            (29, "F16C"),
        ] {
            assert_ne!(ecx1 & (1 << bit), 0, "CPUID.01H:ECX.{name}");
        }
        assert_eq!(
            ecx1 & (1 << 27),
            0,
            "OSXSAVE is CR4's, and CR4 has it clear"
        );
        let (_, ebx7, _, _) = cpuid64(v3, 7, 0);
        assert_eq!(ebx7, (1 << 3) | (1 << 5) | (1 << 8), "BMI1, AVX2, BMI2");
        let (_, _, ecx8, _) = cpuid64(v3, 0x8000_0001, 0);
        assert_eq!(ecx8 & 0b10_0001, 0b10_0001, "LAHF-SAHF and LZCNT");
    }

    #[test]
    fn leaf_7_is_reachable_exactly_when_something_needs_it() {
        use crate::cpu::x86::exec::structured_extended;
        // *Intel SDM* volume 2, `CPUID`, table 3-8, leaf 07H: sub-leaf 0's
        // `EAX` is the highest sub-leaf, and `EBX`/`ECX`/`EDX` the feature
        // bits, built from the lattice by one function the maximum asks too.
        let v2 = Features::X86_64_V2;
        let xsave = Features { xsave: true, ..v2 };
        let bmi = Features { bmi1: true, ..v2 };
        for features in [Features::X86_64, v2, xsave, bmi] {
            let (b, c, d) = structured_extended(features);
            let max = cpuid64(features, 0, 0).0;
            let expected = if features.xsave {
                0xd
            } else if (b, c, d) != (0, 0, 0) {
                7
            } else {
                1
            };
            assert_eq!(max, expected, "{features:?}");
            let leaf7 = if max >= 7 { (0, b, c, d) } else { (0, 0, 0, 0) };
            assert_eq!(cpuid64(features, 7, 0), leaf7);
            // Sub-leaf 1 is past the maximum sub-leaf and says nothing.
            assert_eq!(cpuid64(features, 7, 1), (0, 0, 0, 0));
        }
        // Neither the baseline nor v2 has anything in leaf 7, so both still
        // stop at leaf 1; BMI1 alone raises the maximum to 7.
        assert_eq!(cpuid64(v2, 0, 0).0, 1);
        assert_eq!(cpuid64(bmi, 0, 0).0, 7);
    }

    /// `mov rcx, src ; mov rax, PATTERN ; <insn> ; pushfq ; pop rdx ; hlt`
    /// on a part with `features`, in 64-bit mode: `(RAX, RFLAGS)`.
    fn count64(features: Features, insn: &[u8], src: u64) -> (u64, u64) {
        let mut code = alloc::vec![0x48, 0xb9];
        code.extend_from_slice(&src.to_le_bytes());
        code.extend_from_slice(&[0x48, 0xb8]);
        code.extend_from_slice(&PATTERN.to_le_bytes());
        code.extend_from_slice(insn);
        code.extend_from_slice(&[0x9c, 0x5a, 0xf4]);
        let pc = run64_on(Pc::with_features(Variant::X86_64, features), 0, &code);
        (pc.regs().rax, pc.regs().rdx)
    }

    /// What `RAX` holds before an instruction under test writes it.
    const PATTERN: u64 = 0x5555_5555_5555_5555;

    fn with_lzcnt() -> Features {
        Features {
            lzcnt: true,
            ..Features::X86_64
        }
    }

    #[test]
    fn lzcnt_counts_at_every_width_and_says_zero_in_cf() {
        // *Intel SDM* volume 2, `LZCNT`: a zero source gives the operand
        // width and `CF`; a set top bit gives zero and `ZF`.
        let cf_zf = |flags: u64| {
            (
                flags & u64::from(flags::CF) != 0,
                flags & u64::from(flags::ZF) != 0,
            )
        };
        // lzcnt ax, cx ; lzcnt eax, ecx ; lzcnt rax, rcx
        let forms: [(&[u8], u64, u64); 3] = [
            (&[0x66, 0xf3, 0x0f, 0xbd, 0xc1], 16, 0xffff),
            (&[0xf3, 0x0f, 0xbd, 0xc1], 32, 0xffff_ffff),
            (&[0xf3, 0x48, 0x0f, 0xbd, 0xc1], 64, u64::MAX),
        ];
        // A 16-bit write keeps the rest of `RAX`; a 32-bit one clears it.
        let merge = |width: u64, value: u64| match width {
            16 => (PATTERN & !0xffff) | value,
            _ => value,
        };
        for (insn, width, ones) in forms {
            // Bits above the operand are not part of the source.
            let above = !ones;
            let (rax, fl) = count64(with_lzcnt(), insn, above);
            assert_eq!(rax, merge(width, width), "{width}: zero is the width");
            assert_eq!(cf_zf(fl), (true, false), "{width}: CF for a zero source");
            let (rax, fl) = count64(with_lzcnt(), insn, ones);
            assert_eq!(rax, merge(width, 0), "{width}: all ones");
            assert_eq!(cf_zf(fl), (false, true), "{width}: ZF for a zero count");
            let (rax, fl) = count64(with_lzcnt(), insn, above | 1);
            assert_eq!(rax, merge(width, width - 1), "{width}: one");
            assert_eq!(cf_zf(fl), (false, false), "{width}");
        }
    }

    #[test]
    fn lzcnt_without_its_bit_is_bsr() {
        // *Intel SDM* volume 2, `LZCNT`: on a part without it the encoding
        // executes as `BSR`, the `F3` ignored.
        let lzcnt = [0xf3, 0x0f, 0xbd, 0xc1];
        let (rax, fl) = count64(Features::X86_64, &lzcnt, 0x10);
        assert_eq!(rax, 4, "the index of the top bit, not 27");
        assert_eq!(fl & u64::from(flags::ZF), 0);
        // And a zero source is BSR's: ZF, and the destination untouched.
        let (rax, fl) = count64(Features::X86_64, &lzcnt, 0);
        assert_eq!(rax, PATTERN);
        assert_ne!(fl & u64::from(flags::ZF), 0);
        assert_eq!(count64(with_lzcnt(), &lzcnt, 0x10).0, 27);
    }

    #[test]
    fn cpuid_reports_lzcnt_in_the_extended_leaf() {
        let ecx = |features| cpuid64(features, 0x8000_0001, 0).2;
        assert_eq!(ecx(Features::X86_64) & (1 << 5), 0);
        assert_eq!(ecx(Features::X86_64_V2) & (1 << 5), 0, "not a v2 bit");
        assert_ne!(ecx(with_lzcnt()) & (1 << 5), 0, "CPUID.80000001H:ECX.ABM");
        // The bit lives in a leaf only a long-mode part answers.
        let impossible = Features {
            lzcnt: true,
            ..Features::I80486
        };
        assert!(impossible.validate().is_err());
    }

    fn with_bmi1() -> Features {
        Features {
            bmi1: true,
            ..Features::X86_64
        }
    }

    #[test]
    fn tzcnt_counts_at_every_width_and_says_zero_in_cf() {
        // *Intel SDM* volume 2, `TZCNT`: a zero source gives the operand
        // width and `CF`; a set bottom bit gives zero and `ZF`.
        let cf_zf = |flags: u64| {
            (
                flags & u64::from(flags::CF) != 0,
                flags & u64::from(flags::ZF) != 0,
            )
        };
        // tzcnt ax, cx ; tzcnt eax, ecx ; tzcnt rax, rcx
        let forms: [(&[u8], u64, u64); 3] = [
            (&[0x66, 0xf3, 0x0f, 0xbc, 0xc1], 16, 0xffff),
            (&[0xf3, 0x0f, 0xbc, 0xc1], 32, 0xffff_ffff),
            (&[0xf3, 0x48, 0x0f, 0xbc, 0xc1], 64, u64::MAX),
        ];
        let merge = |width: u64, value: u64| match width {
            16 => (PATTERN & !0xffff) | value,
            _ => value,
        };
        for (insn, width, ones) in forms {
            // Bits above the operand are not part of the source.
            let above = !ones;
            let (rax, fl) = count64(with_bmi1(), insn, above);
            assert_eq!(rax, merge(width, width), "{width}: zero is the width");
            assert_eq!(cf_zf(fl), (true, false), "{width}: CF for a zero source");
            let (rax, fl) = count64(with_bmi1(), insn, ones);
            assert_eq!(rax, merge(width, 0), "{width}: all ones");
            assert_eq!(cf_zf(fl), (false, true), "{width}: ZF for a zero count");
            let top = (ones >> 1) + 1;
            let (rax, fl) = count64(with_bmi1(), insn, above | top);
            assert_eq!(rax, merge(width, width - 1), "{width}: the top bit");
            assert_eq!(cf_zf(fl), (false, false), "{width}");
        }
    }

    #[test]
    fn tzcnt_without_bmi1_is_bsf() {
        // *Intel SDM* volume 2, `TZCNT`: on a part without it the encoding
        // executes as `BSF`, the `F3` ignored — so the same answer for a
        // non-zero source, and `BSF`'s for a zero one.
        let tzcnt = [0xf3, 0x0f, 0xbc, 0xc1];
        let (rax, fl) = count64(Features::X86_64, &tzcnt, 0x10);
        assert_eq!(rax, 4);
        assert_eq!(fl & u64::from(flags::ZF), 0);
        let (rax, fl) = count64(Features::X86_64, &tzcnt, 0);
        assert_eq!(rax, PATTERN, "BSF leaves the destination");
        assert_ne!(fl & u64::from(flags::ZF), 0);
        let (rax, fl) = count64(with_bmi1(), &tzcnt, 0);
        assert_eq!(rax, 32);
        assert_eq!(fl & u64::from(flags::ZF), 0);
        // BMI2 alone is not enough: the bit is BMI1's.
        let bmi2 = Features {
            bmi2: true,
            ..Features::X86_64
        };
        assert_eq!(count64(bmi2, &tzcnt, 0).0, PATTERN);
    }

    fn with_movbe() -> Features {
        Features {
            movbe: true,
            ..Features::X86_64
        }
    }

    /// `bytes` followed by a ModRM and SIB naming `reg` and the absolute
    /// address `addr` — not `RIP`-relative, which `mod == 00`, `r/m == 101`
    /// is in 64-bit mode.
    fn abs(bytes: &[u8], reg: u8, addr: u64) -> Vec<u8> {
        let mut out = bytes.to_vec();
        out.extend_from_slice(&[(reg << 3) | 0b100, 0x25]);
        out.extend_from_slice(&(addr as u32).to_le_bytes());
        out
    }

    #[test]
    fn movbe_loads_and_stores_reversed_at_every_width() {
        // *Intel SDM* volume 2, `MOVBE`: `0F 38 F0` loads, `F1` stores, the
        // operand size picks 16, 32 or 64 bits.
        let mut code = alloc::vec![0x48, 0xc7, 0xc1, 0xff, 0xff, 0xff, 0xff]; // mov rcx, -1
        code.extend_from_slice(&[0x48, 0xc7, 0xc2, 0xff, 0xff, 0xff, 0xff]); // mov rdx, -1
        code.extend(abs(&[0x48, 0x0f, 0x38, 0xf0], 0, la::MARK)); // movbe rax, [MARK]
        code.extend(abs(&[0x0f, 0x38, 0xf0], 1, la::MARK)); // movbe ecx, [MARK]
        code.extend(abs(&[0x66, 0x0f, 0x38, 0xf0], 2, la::MARK)); // movbe dx, [MARK]
        code.extend_from_slice(&[0x48, 0xbb]); // mov rbx, imm64
        code.extend_from_slice(&0x1122_3344_5566_7788u64.to_le_bytes());
        code.extend(abs(&[0x48, 0x0f, 0x38, 0xf1], 3, la::MARK + 8)); // movbe [MARK+8], rbx
        code.extend(abs(&[0x0f, 0x38, 0xf1], 3, la::MARK + 16)); // movbe [MARK+16], ebx
        code.extend(abs(&[0x66, 0x0f, 0x38, 0xf1], 3, la::MARK + 24)); // movbe [MARK+24], bx
        code.push(0xf4);
        let pc = Pc::with_features(Variant::X86_64, with_movbe());
        pc.write64(la::MARK, 0x0102_0304_0506_0708);
        for at in [8, 16, 24] {
            pc.write64(la::MARK + at, u64::MAX);
        }
        let pc = run64_on(pc, 0, &code);
        let r = pc.regs();
        assert_eq!(r.rax, 0x0807_0605_0403_0201);
        assert_eq!(r.rcx, 0x0807_0605, "a 32-bit load zero-extends");
        assert_eq!(r.rdx, 0xffff_ffff_ffff_0807, "a 16-bit load merges");
        assert_eq!(pc.read64(la::MARK + 8), 0x8877_6655_4433_2211);
        assert_eq!(pc.read64(la::MARK + 16), 0xffff_ffff_8877_6655);
        assert_eq!(pc.read64(la::MARK + 24), 0xffff_ffff_ffff_8877);
    }

    #[test]
    fn movbe_has_no_register_form_and_needs_its_bit() {
        // movbe eax, ecx and movbe ecx, eax: `mod == 11` is `#UD`.
        faults64(with_movbe(), 6, &[0x0f, 0x38, 0xf0, 0xc1]);
        faults64(with_movbe(), 6, &[0x0f, 0x38, 0xf1, 0xc1]);
        // The memory form on a part without the feature.
        faults64(Features::X86_64, 6, &abs(&[0x0f, 0x38, 0xf0], 0, la::MARK));
        // `F2` in front is still `CRC32`, which v2 has: crc32 eax, ecx.
        let pc = run64_on(
            Pc::with_features(Variant::X86_64, Features::X86_64_V2),
            0,
            &[
                0x31, 0xc0, 0xb9, 0x01, 0, 0, 0, 0xf2, 0x0f, 0x38, 0xf1, 0xc1, 0xf4,
            ],
        );
        assert_ne!(pc.regs().rax, 0, "a CRC, not a byte swap");
    }

    #[test]
    fn cpuid_reports_movbe_in_leaf_1() {
        let ecx = |features| cpuid64(features, 1, 0).2;
        assert_eq!(ecx(Features::X86_64) & (1 << 22), 0);
        assert_eq!(ecx(Features::X86_64_V2) & (1 << 22), 0, "not a v2 bit");
        assert_ne!(ecx(with_movbe()) & (1 << 22), 0, "CPUID.01H:ECX.MOVBE");
        // A 32-bit part may have it — the first ones did — but not one
        // without `CPUID` to say so.
        let atomish = Features {
            movbe: true,
            ..Features::I80486
        };
        assert!(atomish.validate().is_ok());
        let impossible = Features {
            movbe: true,
            ..Features::NONE
        };
        assert!(impossible.validate().is_err());
    }

    #[test]
    fn movnti_with_rex_w_stores_all_eight_bytes() {
        // mov rax, imm64 ; movnti [MARK], rax ; movnti [MARK+8], eax ; hlt
        // — absolute addressing through a SIB byte, because `mod == 00`,
        // `r/m == 101` is `RIP`-relative here.
        let mut code = alloc::vec![0x48u8, 0xb8];
        code.extend_from_slice(&0x1122_3344_5566_7788u64.to_le_bytes());
        code.extend_from_slice(&[0x48, 0x0f, 0xc3, 0x04, 0x25]);
        code.extend_from_slice(&(la::MARK as u32).to_le_bytes());
        code.extend_from_slice(&[0x0f, 0xc3, 0x04, 0x25]);
        code.extend_from_slice(&((la::MARK + 8) as u32).to_le_bytes());
        code.push(0xf4);
        let pc = pc64();
        pc.write64(la::MARK + 8, u64::MAX);
        pc.start_protected();
        pc.prepare_long();
        pc.write(at::CODE0, &enter_long_mode_code(la::CODE64));
        pc.write(la::CODE64, &code);
        assert!(pc.run(200) < 200);
        assert_eq!(pc.read64(la::MARK), 0x1122_3344_5566_7788);
        assert_eq!(
            pc.read64(la::MARK + 8),
            0xffff_ffff_5566_7788,
            "without REX.W, four"
        );
    }

    #[test]
    fn movq_with_rex_w_moves_a_whole_general_register_to_and_from_mm() {
        // mov rax, imm64 ; movq mm0, rax ; movq [MARK], mm0 ;
        // movd mm1, eax ; movq rcx, mm1 ; mov [MARK+8], rcx ; hlt
        let mut code = alloc::vec![0x48u8, 0xb8];
        code.extend_from_slice(&0x1122_3344_5566_7788u64.to_le_bytes());
        code.extend_from_slice(&[0x48, 0x0f, 0x6e, 0xc0]);
        code.extend_from_slice(&[0x0f, 0x7f, 0x04, 0x25]);
        code.extend_from_slice(&(la::MARK as u32).to_le_bytes());
        code.extend_from_slice(&[0x0f, 0x6e, 0xc8, 0x48, 0x0f, 0x7e, 0xc9]);
        code.extend_from_slice(&[0x48, 0x89, 0x0c, 0x25]);
        code.extend_from_slice(&((la::MARK + 8) as u32).to_le_bytes());
        code.push(0xf4);
        let pc = pc64();
        pc.start_protected();
        pc.prepare_long();
        pc.write(at::CODE0, &enter_long_mode_code(la::CODE64));
        pc.write(la::CODE64, &code);
        assert!(pc.run(200) < 200);
        assert_eq!(pc.read64(la::MARK), 0x1122_3344_5566_7788);
        assert_eq!(
            pc.read64(la::MARK + 8),
            0x5566_7788,
            "movd zero-extends into the quadword"
        );
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

    // -- CR8, the task-priority register ---------------------------------

    /// Where these tests put the local APIC's register page.
    ///
    /// Inside the harness's RAM, so the "register page" is four bytes of
    /// memory a test can poke and read back. That is the whole assertion:
    /// `CR8` has no storage of its own, and every access to it has to land
    /// *there*.
    const APIC_PAGE: u64 = 0x30_0000;

    /// The task-priority register's address, at offset `0x80` of that page —
    /// *Intel SDM* volume 3A §11.4.4, table 11-1.
    const TPR: u64 = APIC_PAGE + 0x80;

    /// A local controller that does nothing but say where its registers are,
    /// which is all `CR8` asks of one: the transfer itself goes through the
    /// address space, exactly as a guest's own store to the page would.
    #[derive(Debug)]
    struct Apic {
        base: u64,
    }

    impl LocalController for Apic {
        fn take_startup(&self) -> Startup {
            Startup::NONE
        }

        fn base_register(&self) -> u64 {
            self.base
        }
    }

    /// Bring a core into 64-bit mode with a local APIC wired at `APIC_PAGE`
    /// and `tpr` already in its task-priority register, then run `code`.
    ///
    /// The controller comes back with the machine because the core holds only
    /// a weak reference to it — dropping it here would unwire the APIC halfway
    /// through the test.
    fn run64_with_apic(tpr: u64, code: &[u8]) -> (Pc, Arc<dyn LocalController>) {
        let pc = pc64();
        let intc: Arc<dyn LocalController> = Arc::new(Apic {
            base: APIC_PAGE | apic_base::ENABLE,
        });
        pc.cpu
            .attach_local_controller("intr", Arc::downgrade(&intc));
        pc.start_protected();
        pc.prepare_long();
        pc.write32(TPR, tpr);
        pc.write(at::CODE0, &enter_long_mode_code(la::CODE64));
        pc.write(la::CODE64, code);
        let steps = pc.run(200);
        assert!(steps < 200, "the 64-bit program halted");
        assert!(pc.cpu.sys().sixty_four(), "and did so in 64-bit mode");
        (pc, intc)
    }

    #[test]
    fn a_long_mode_interrupt_aligns_the_stack_frame_to_sixteen_bytes() {
        // *Intel SDM* volume 3A §6.14.2, "Stack Frame": in IA-32e mode `RSP` is
        // aligned **down** to a sixteen-byte boundary before the frame is
        // pushed, and the value pushed is the unaligned one — so `IRETQ` puts
        // the interrupted stack back exactly as it was and nothing outside the
        // handler can tell.
        //
        // Not cosmetic. `FXSAVE` raises `#GP(0)` on an operand that is not
        // sixteen-byte aligned, and EDK II's `CommonInterruptEntry` builds that
        // operand out of `RSP`: without this a UEFI firmware's first exception
        // handler faults on its own `FXSAVE`, faults again handling that, and
        // recurses until the stack walks off the identity map.
        let pc = pc64();
        pc.start_protected();
        pc.prepare_long();
        pc.idt64(0x40, 0x18, la::HANDLER);
        // mov rbx, rsp ; iretq
        pc.write(la::HANDLER, &[0x48, 0x89, 0xe3, 0x48, 0xcf]);
        pc.write(at::CODE0, &enter_long_mode_code(la::CODE64));
        // mov rsp, 0x27ff8 — deliberately eight bytes off a boundary, which is
        // where an odd number of pushes leaves any real stack.
        pc.write(
            la::CODE64,
            &[
                0x48, 0xc7, 0xc4, 0xf8, 0x7f, 0x02, 0x00, // mov rsp, 0x27ff8
                0xcd, 0x40, // int 0x40
                0x48, 0x89, 0xe1, // mov rcx, rsp
                0xf4,
            ],
        );
        let steps = pc.run(200);
        assert!(steps < 200, "the program halted");
        let regs = pc.regs();
        assert_eq!(
            (regs.rbx + 40) % 16,
            0,
            "the five-word frame starts on a sixteen-byte boundary, so the \
             handler's own stack is aligned the way the ABI it is written in \
             assumes"
        );
        assert_eq!(
            pc.read64(regs.rbx + 24),
            0x2_7ff8,
            "and the RSP the frame carries is the unaligned one the guest had"
        );
        assert_eq!(
            regs.rcx, 0x2_7ff8,
            "so IRETQ returns to the stack the interrupt found"
        );
    }

    #[test]
    fn cr8_reads_the_task_priority_class_out_of_the_local_apic() {
        // `44 0f 20 c0` — `REX.R` is what turns `CR0` into `CR8`, and these
        // are the four bytes EDK II's `CommonInterruptEntry` opens with: a
        // UEFI firmware fills an `EFI_SYSTEM_CONTEXT_X64`, and that structure
        // has `Cr8` in it. Until this existed the exception handler faulted on
        // its own second instruction and recursed until the stack left the
        // identity map (`docs/platforms/q35-uefi.md`).
        //
        // `0xb7` in the register is priority class `0xb` with sub-class `7`;
        // `CR8` is the class alone (*Intel SDM* volume 3A §11.8.6.1).
        let (pc, _intc) = run64_with_apic(0xb7, &[0x44, 0x0f, 0x20, 0xc0, 0xf4]);
        assert_eq!(pc.regs().rax, 0xb, "TPR[7:4], not the whole register");
    }

    #[test]
    fn a_write_to_cr8_lands_in_the_task_priority_register() {
        // mov eax, 7 ; mov cr8, rax ; hlt
        let (pc, _intc) = run64_with_apic(
            0xb7,
            &[0xb8, 0x07, 0x00, 0x00, 0x00, 0x44, 0x0f, 0x22, 0xc0, 0xf4],
        );
        assert_eq!(
            pc.read32(TPR),
            0x70,
            "the class shifts into TPR[7:4] and the sub-class goes to zero with \
             it, which is why the SDM tells software to use one route or the \
             other and not both"
        );
    }

    #[test]
    fn cr8_and_the_apics_own_register_are_one_piece_of_state() {
        // The alias, both ways, in one program — because a private four-bit
        // register in `Sys` would pass the two tests above and fail this one,
        // and a guest that set its priority through `CR8` and read it back
        // through the APIC's page would then see an interrupt-delivery bug
        // that was really a register bug.
        let code = &[
            // mov eax, 3 ; mov cr8, rax ; mov rax, cr8 ; mov ebx, eax
            0xb8, 0x03, 0x00, 0x00, 0x00, //
            0x44, 0x0f, 0x22, 0xc0, //
            0x44, 0x0f, 0x20, 0xc0, //
            0x89, 0xc3, //
            // mov dword [0x300080], 0x90 — an ordinary guest store to the
            // device's register page, which is the *other* route to the same
            // register.
            0xc7, 0x04, 0x25, 0x80, 0x00, 0x30, 0x00, 0x90, 0x00, 0x00, 0x00, //
            // mov rcx, cr8 ; hlt
            0x44, 0x0f, 0x20, 0xc1, //
            0xf4,
        ];
        let (pc, _intc) = run64_with_apic(0, code);
        let regs = pc.regs();
        assert_eq!(regs.rbx, 3, "what `CR8` wrote is what `CR8` reads");
        assert_eq!(
            regs.rcx, 9,
            "and a store to the APIC's own register moved `CR8` with it"
        );
        assert_eq!(pc.read32(TPR), 0x90, "there is one register, not two");
    }

    #[test]
    fn a_task_priority_class_above_fifteen_is_a_general_protection_fault() {
        // Bits 63:4 of `CR8` do not exist, and a write that names one is
        // `#GP(0)` rather than a silent truncation (*Intel SDM* volume 3A
        // §2.5). A guest that thought it was setting a class of sixteen finds
        // out.
        let pc = pc64();
        let intc: Arc<dyn LocalController> = Arc::new(Apic {
            base: APIC_PAGE | apic_base::ENABLE,
        });
        pc.cpu
            .attach_local_controller("intr", Arc::downgrade(&intc));
        pc.start_protected();
        pc.prepare_long();
        pc.write32(TPR, 0x20);
        pc.idt64(13, 0x18, la::HANDLER);
        // mov rbx, 42 ; hlt
        pc.write(
            la::HANDLER,
            &[0x48, 0xc7, 0xc3, 0x2a, 0x00, 0x00, 0x00, 0xf4],
        );
        pc.write(at::CODE0, &enter_long_mode_code(la::CODE64));
        // mov eax, 0x10 ; mov cr8, rax ; hlt
        pc.write(
            la::CODE64,
            &[0xb8, 0x10, 0x00, 0x00, 0x00, 0x44, 0x0f, 0x22, 0xc0, 0xf4],
        );
        pc.run(200);
        assert_eq!(pc.regs().rbx, 42, "#GP(0) was delivered");
        assert_eq!(
            pc.read32(TPR),
            0x20,
            "and the refused write never reached the device"
        );
    }

    #[test]
    fn a_core_with_no_local_controller_has_a_task_priority_of_zero() {
        // `CR8` is the APIC's register seen from the processor, so a board
        // that wired no APIC has no priority class — and nothing that could be
        // blocked by one, because its `INTR` comes from an 8259A whose
        // priorities the processor never sees. It reads zero and a legal write
        // has nowhere to go, which is a documented degenerate case rather than
        // a fault: the instruction is architecturally valid in 64-bit mode.
        //
        // Contrast `ia32_apic_base_faults_on_a_processor_with_no_local_controller`:
        // `IA32_APIC_BASE` is a register the *controller* owns, and a machine
        // without one does not have it at all.
        let pc = pc64();
        pc.start_protected();
        pc.prepare_long();
        pc.write(at::CODE0, &enter_long_mode_code(la::CODE64));
        // mov eax, 5 ; mov cr8, rax ; mov rbx, cr8 ; hlt
        let code = &[
            0xb8, 0x05, 0x00, 0x00, 0x00, //
            0x44, 0x0f, 0x22, 0xc0, //
            0x44, 0x0f, 0x20, 0xd8, //
            0xf4,
        ];
        pc.write(la::CODE64, code);
        let steps = pc.run(200);
        assert!(steps < 200, "no fault: the program reached its `hlt`");
        assert_eq!(pc.regs().rbx, 0, "and read a priority class of zero");
    }

    #[test]
    fn the_reserved_control_registers_are_still_invalid_opcodes() {
        // `CR8` is the only index long mode added. `CR1`, `CR5`-`CR7` and
        // `CR9`-`CR15` are reserved on every part there has ever been, and
        // widening the fix past what the SDM licenses is how an emulator grows
        // a fake instruction set.
        //
        // `44 0f 20 c8` is `mov rax, cr9`; without the prefix the same ModRM
        // byte is `mov rax, cr1`, and `e8` names `cr5`.
        for (bytes, name) in [
            (&[0x44u8, 0x0f, 0x20, 0xc8, 0xf4][..], "cr9"),
            (&[0x0fu8, 0x20, 0xc8, 0xf4][..], "cr1"),
            (&[0x0fu8, 0x20, 0xe8, 0xf4][..], "cr5"),
        ] {
            let pc = pc64();
            let intc: Arc<dyn LocalController> = Arc::new(Apic {
                base: APIC_PAGE | apic_base::ENABLE,
            });
            pc.cpu
                .attach_local_controller("intr", Arc::downgrade(&intc));
            pc.start_protected();
            pc.prepare_long();
            pc.idt64(6, 0x18, la::HANDLER);
            // mov rbx, 42 ; hlt
            pc.write(
                la::HANDLER,
                &[0x48, 0xc7, 0xc3, 0x2a, 0x00, 0x00, 0x00, 0xf4],
            );
            pc.write(at::CODE0, &enter_long_mode_code(la::CODE64));
            pc.write(la::CODE64, bytes);
            pc.run(200);
            assert_eq!(pc.regs().rbx, 42, "{name} is #UD");
        }
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

    /// Exits to the host (issues #35 and #36): the level-3 seam on this core,
    /// on every engine it has.
    ///
    /// Inside `long_mode` because half of these need its bring-up and its
    /// identity map; the flat 32-bit half uses `Pc::start_protected` like the
    /// rest of the file.
    mod host_exits {
        use super::*;
        use crate::core::exec::{Access as ExitAccess, Exit, ExitMask, ExitReason, ExitingCore};
        use crate::core::sched::Budget;
        use crate::cpu::x86::{Engine, FaultDetail, VectorSet};

        /// Every engine this build has. The interpreter is the oracle; the
        /// translating engines must stop on the same instruction with the same
        /// state, which is the property `Engine` promises.
        fn engines() -> Vec<Engine> {
            #[allow(unused_mut)]
            let mut all = alloc::vec![Engine::Interp];
            #[cfg(all(feature = "cpu-x86-lift", feature = "jit"))]
            all.extend([Engine::Jit, Engine::JitHost]);
            all
        }

        impl Pc {
            /// [`Pc::new`] on a chosen engine.
            fn on(variant: Variant, engine: Engine) -> Pc {
                let pc = Pc::new(variant);
                let cpu = X86::new(
                    Config::default()
                        .with_variant(variant)
                        .with_features(variant.features()),
                )
                .with_engine(engine);
                cpu.attach_space(pc.cpu.space().expect("a memory space"));
                cpu.attach_io_space(pc.cpu.io_space().expect("an I/O space"));
                Pc {
                    cpu: Arc::new(cpu),
                    ..pc
                }
            }

            /// Flat 32-bit protected mode at `at::CODE0`, running `code`.
            fn flat32(engine: Engine, code: &[u8]) -> Pc {
                let pc = Pc::on(Variant::X86_64, engine);
                pc.start_protected();
                pc.write(at::CODE0, code);
                pc
            }

            /// 64-bit mode reached by the guest's own bring-up, then `code` at
            /// `la::CODE64`. The bring-up raises nothing, so a mask armed
            /// before it changes nothing about it.
            fn long64(engine: Engine, code: &[u8]) -> Pc {
                let pc = Pc::on(Variant::X86_64, engine);
                pc.start_protected();
                pc.prepare_long();
                pc.write(at::CODE0, &enter_long_mode_code(la::CODE64));
                pc.write(la::CODE64, code);
                pc
            }

            fn exit(&self, budget: u64) -> Option<Exit> {
                self.cpu.run_to_exit(Budget::of(budget)).exit
            }
        }

        // ---- SYSCALL -----------------------------------------------------

        #[test]
        fn syscall_exits_to_the_host_with_rip_past_the_instruction() {
            for engine in engines() {
                // mov eax, 60 ; syscall ; hlt
                let pc = Pc::long64(engine, &[0xb8, 60, 0, 0, 0, 0x0f, 0x05, 0xf4]);
                pc.cpu
                    .set_exit_mask(ExitMask::NONE.with(ExitReason::SYSCALL));
                // A guest kernel's entry point, armed, so that a delivered
                // `SYSCALL` would have somewhere to go and visibly go there.
                let mut sys = pc.cpu.sys();
                sys.efer |= efer::SCE;
                sys.star = 0x0028u64 << 32;
                sys.lstar = la::HANDLER;
                pc.cpu.set_sys(sys);
                let exit = pc.exit(100_000).expect("an exit");
                assert_eq!(exit.reason, ExitReason::SYSCALL, "{engine:?}");
                assert_eq!(exit.pc, la::CODE64 + 5, "the syscall's own address");
                assert_eq!(exit.len, 2);
                assert_eq!(exit.resume_pc(), la::CODE64 + 7);
                let regs = pc.cpu.regs();
                assert_eq!(regs.rax, 60, "the ABI registers are the host's to read");
                assert_eq!(regs.rip, la::CODE64 + 7, "and the core is past it");
                // The bring-up left `ECX` naming `IA32_EFER` for its
                // `RDMSR`/`WRMSR` pair; a real `SYSCALL` would overwrite it with
                // the return address and `R11` with the flags.
                assert_eq!(regs.rcx, u64::from(msr::EFER), "RCX untouched");
                assert_eq!(regs.r[3], 0, "R11 untouched");
                assert_eq!((regs.cs, regs.ss), (0x18, 0x10), "no STAR selectors");
                let after = pc.cpu.sys();
                assert_eq!(after.seg(isa::seg::CS).selector, 0x18);
                assert!(after.sixty_four(), "still in the same code segment");

                // The host writes a result and resumes; the guest carries on.
                pc.cpu.set_reg(Reg::Rax, 7);
                assert!(pc.exit(1_000).is_none(), "the hlt is not an exit");
                assert!(pc.cpu.is_halted());
                assert_eq!(pc.cpu.reg(Reg::Rax), 7);
            }
        }

        #[test]
        fn sysenter_is_a_system_call_exit_and_otherwise_undefined() {
            for engine in engines() {
                // sysenter ; hlt
                let pc = Pc::flat32(engine, &[0x0f, 0x34, 0xf4]);
                pc.cpu.set_exit_mask(ExitMask::USER);
                let exit = pc.exit(1_000).expect("an exit");
                assert_eq!(exit.reason, ExitReason::SYSCALL, "{engine:?}");
                assert_eq!((exit.pc, exit.len), (at::CODE0, 2));
                assert_eq!(pc.cpu.regs().rip, at::CODE0 + 2);

                // With only faults asked for, it is the `#UD` of a part that
                // does not implement it.
                let pc = Pc::flat32(engine, &[0x0f, 0x34, 0xf4]);
                pc.cpu.set_exit_mask(ExitMask::NONE.with(ExitReason::FAULT));
                let exit = pc.exit(1_000).expect("an exit");
                assert_eq!(exit.reason, ExitReason::FAULT);
                assert_eq!(FaultDetail::from_detail(exit.detail).vector, 6);
            }
        }

        /// An `IDT` gate for `vector` at `handler`, which stores `0xAA` at
        /// `at::MARK` and halts.
        fn mark_handler(pc: &Pc, vector: u8, handler: u64) {
            pc.idt(
                u64::from(vector),
                gate(0x08, handler as u32, sys_type::INT_GATE32, 3),
            );
            // mov byte [MARK], 0xaa ; hlt
            let mut code = alloc::vec![0xc6, 0x05];
            code.extend_from_slice(&(at::MARK as u32).to_le_bytes());
            code.extend_from_slice(&[0xaa, 0xf4]);
            pc.write(handler, &code);
        }

        #[test]
        fn a_selected_int_vector_exits_and_an_unselected_one_takes_the_idt() {
            for engine in engines() {
                // mov eax, 4 ; int 0x80 ; int 0x81 ; hlt
                let code = [0xb8, 4, 0, 0, 0, 0xcd, 0x80, 0xcd, 0x81, 0xf4];
                let pc = Pc::flat32(engine, &code);
                mark_handler(&pc, 0x80, at::CODE0 + 0x100);
                mark_handler(&pc, 0x81, at::CODE0 + 0x200);
                pc.cpu.set_exit_mask(ExitMask::USER);
                pc.cpu
                    .set_syscall_vectors(VectorSet::NONE.with(0x80).with(0x21));
                assert_eq!(
                    pc.cpu.syscall_vectors(),
                    VectorSet::NONE.with(0x80).with(0x21)
                );
                let sp = pc.cpu.regs().rsp;

                let exit = pc.exit(1_000).expect("int 0x80 exits");
                assert_eq!(exit.reason, ExitReason::SYSCALL, "{engine:?}");
                assert_eq!((exit.pc, exit.len), (at::CODE0 + 5, 2));
                assert_eq!(pc.cpu.regs().rip, at::CODE0 + 7);
                assert_eq!(pc.cpu.regs().rsp, sp, "no frame was pushed");
                assert_eq!(pc.ram.read_u8(at::MARK).unwrap(), 0, "no handler ran");

                // `INT 0x81` is not in the set: through the guest's table.
                assert!(pc.exit(1_000).is_none());
                assert_eq!(pc.ram.read_u8(at::MARK).unwrap(), 0xaa, "the handler ran");
                assert_eq!(pc.cpu.regs().rsp, sp - 12, "on an interrupt frame");
            }
        }

        #[test]
        fn an_empty_mask_keeps_architectural_delivery() {
            for engine in engines() {
                // int 0x80 ; hlt — with the vector selected but no mask.
                let pc = Pc::flat32(engine, &[0xcd, 0x80, 0xf4]);
                mark_handler(&pc, 0x80, at::CODE0 + 0x100);
                pc.cpu.set_syscall_vectors(VectorSet::ALL);
                assert!(pc.exit(1_000).is_none(), "{engine:?}");
                assert_eq!(pc.ram.read_u8(at::MARK).unwrap(), 0xaa);
                assert_eq!(pc.cpu.exit_mask(), ExitMask::NONE);
            }
        }

        // ---- FAULT -------------------------------------------------------

        #[test]
        fn ud_exits_as_a_fault_at_the_instruction_when_masked_in() {
            for engine in engines() {
                // nop ; ud2
                let pc = Pc::flat32(engine, &[0x90, 0x0f, 0x0b]);
                pc.cpu.set_exit_mask(ExitMask::NONE.with(ExitReason::FAULT));
                let exit = pc.exit(1_000).expect("an exit");
                assert_eq!(exit.reason, ExitReason::FAULT, "{engine:?}");
                assert_eq!((exit.pc, exit.len), (at::CODE0 + 1, 2));
                assert_eq!(
                    FaultDetail::from_detail(exit.detail),
                    FaultDetail {
                        vector: 6,
                        error: None
                    }
                );
                assert_eq!(exit.access, ExitAccess::None);
                assert_eq!(pc.cpu.regs().rip, at::CODE0 + 1, "at the instruction");
                // Resuming re-executes it, so it faults again.
                assert_eq!(pc.exit(1_000).map(|e| e.pc), Some(at::CODE0 + 1));
            }
        }

        #[test]
        fn gp_exits_with_its_error_code_and_de_with_none() {
            for engine in engines() {
                // mov ax, 0x108 ; mov ds, ax — a selector past the GDT limit.
                let pc = Pc::flat32(engine, &[0x66, 0xb8, 0x08, 0x01, 0x8e, 0xd8]);
                pc.cpu.set_exit_mask(ExitMask::USER);
                let exit = pc.exit(1_000).expect("an exit");
                assert_eq!(exit.pc, at::CODE0 + 4, "{engine:?}");
                assert_eq!(
                    FaultDetail::from_detail(exit.detail),
                    FaultDetail {
                        vector: 13,
                        error: Some(0x108)
                    }
                );
                assert_eq!(pc.cpu.regs().ds, 0x10, "DS was not loaded");

                // mov ecx, 0 ; div ecx
                let pc = Pc::flat32(engine, &[0xb9, 0, 0, 0, 0, 0xf7, 0xf1]);
                pc.cpu.set_exit_mask(ExitMask::USER);
                let exit = pc.exit(1_000).expect("an exit");
                assert_eq!((exit.pc, exit.len), (at::CODE0 + 5, 2));
                assert_eq!(exit.detail, 0, "#DE, no error code");
            }
        }

        #[test]
        fn a_page_fault_exits_with_the_address_and_leaves_cr2_alone() {
            for engine in engines() {
                // mov rbx, 0x800000 ; mov rax, [rbx] ; mov [rbx], rax ; hlt
                let code = [
                    0x48, 0xc7, 0xc3, 0x00, 0x00, 0x80, 0x00, // mov rbx, 0x800000
                    0x48, 0x8b, 0x03, // mov rax, [rbx]
                    0x48, 0x89, 0x03, // mov [rbx], rax
                    0xf4,
                ];
                let pc = Pc::long64(engine, &code);
                pc.cpu.set_exit_mask(ExitMask::USER);
                let mut sys = pc.cpu.sys();
                sys.cr2 = 0x1234;
                pc.cpu.set_sys(sys);

                let exit = pc.exit(100_000).expect("a read fault");
                assert_eq!(exit.reason, ExitReason::FAULT, "{engine:?}");
                assert_eq!((exit.pc, exit.len), (la::CODE64 + 7, 3));
                assert_eq!(
                    FaultDetail::from_detail(exit.detail),
                    FaultDetail {
                        vector: 14,
                        error: Some(0)
                    }
                );
                assert_eq!(exit.address, 0x80_0000);
                assert_eq!(exit.access, ExitAccess::Read);
                assert_eq!(pc.cpu.sys().cr2, 0x1234, "CR2 is part of delivery");
                assert_eq!(pc.cpu.regs().rip, la::CODE64 + 7);

                // The host skips the read, as an emulating consumer would,
                // and the write faults as a write.
                pc.cpu.set_pc(exit.pc + u64::from(exit.len));
                let exit = pc.exit(100_000).expect("a write fault");
                assert_eq!((exit.pc, exit.address), (la::CODE64 + 10, 0x80_0000));
                assert_eq!(exit.access, ExitAccess::Write);
                assert_eq!(
                    FaultDetail::from_detail(exit.detail).error,
                    Some(paging::pf::WRITE)
                );
            }
        }

        #[test]
        fn a_fault_inside_a_translated_block_exits_where_the_interpreter_does() {
            // A loop hot enough to be translated, walking a pointer off the
            // end of the identity map: the fault happens *inside* a block, and
            // must leave the core exactly where the oracle leaves it.
            let mut seen = Vec::new();
            for engine in engines() {
                let code = [
                    0x48, 0xc7, 0xc3, 0x00, 0x00, 0x20, 0x00, // mov rbx, 0x200000
                    0x48, 0x8b, 0x03, // L: mov rax, [rbx]
                    0x48, 0x81, 0xc3, 0x00, 0x10, 0x00, 0x00, // add rbx, 0x1000
                    0xeb, 0xf4, // jmp L
                ];
                let pc = Pc::long64(engine, &code);
                pc.cpu.set_exit_mask(ExitMask::USER);
                let exit = pc.exit(10_000_000).expect("a fault");
                assert_eq!((exit.pc, exit.len), (la::CODE64 + 7, 3), "{engine:?}");
                assert_eq!((exit.address, exit.access), (0x40_0000, ExitAccess::Read));
                let regs = pc.cpu.regs();
                assert_eq!(regs.rbx, 0x40_0000);
                seen.push((exit, regs, pc.cpu.cycles()));
            }
            assert!(
                seen.windows(2).all(|w| w[0] == w[1]),
                "every engine agrees on the exit, the registers and the clock"
            );
        }

        #[test]
        fn a_fetch_fault_reports_an_execute_access() {
            for engine in engines() {
                // mov rax, 0x800000 ; jmp rax
                let code = [0x48, 0xc7, 0xc0, 0x00, 0x00, 0x80, 0x00, 0xff, 0xe0];
                let pc = Pc::long64(engine, &code);
                pc.cpu.set_exit_mask(ExitMask::USER);
                let exit = pc.exit(100_000).expect("a fetch fault");
                assert_eq!((exit.pc, exit.len), (0x80_0000, 0), "{engine:?}");
                assert_eq!(exit.access, ExitAccess::Execute);
                assert_eq!(exit.address, 0x80_0000);
            }
        }

        #[test]
        fn a_repeat_that_faults_keeps_the_iterations_it_completed() {
            // *Intel SDM* volume 2, `REP`: a fault restarts the instruction
            // with the count and the index reflecting what was done.
            for engine in engines() {
                // mov rdi, 0x3ffffc ; mov rcx, 8 ; mov al, 0x5a ; rep stosb
                let code = [
                    0x48, 0xc7, 0xc7, 0xfc, 0xff, 0x3f, 0x00, // mov rdi
                    0x48, 0xc7, 0xc1, 0x08, 0x00, 0x00, 0x00, // mov rcx, 8
                    0xb0, 0x5a, // mov al, 0x5a
                    0xf3, 0xaa, // rep stosb
                    0xf4,
                ];
                let pc = Pc::long64(engine, &code);
                pc.cpu.set_exit_mask(ExitMask::USER);
                let exit = pc.exit(100_000).expect("a fault");
                assert_eq!((exit.pc, exit.address), (la::CODE64 + 16, 0x40_0000));
                let regs = pc.cpu.regs();
                assert_eq!(regs.rcx, 4, "{engine:?}: four iterations done");
                assert_eq!(regs.rdi, 0x40_0000, "and the index where it stopped");
                assert_eq!(pc.ram.read_u8(0x3f_ffff).unwrap(), 0x5a);

                // And the same restart point when the fault is *delivered*:
                // the handler sees the completed iterations, and the frame
                // returns to the prefix.
                let pc = Pc::long64(engine, &code);
                pc.idt64(14, 0x20, la::HANDLER);
                pc.write(la::HANDLER, &[0xf4]);
                pc.run(400);
                let regs = pc.cpu.regs();
                assert_eq!(regs.rip, la::HANDLER + 1, "{engine:?}: in the handler");
                assert_eq!((regs.rcx, regs.rdi), (4, 0x40_0000));
                // Error code, then the return RIP.
                assert_eq!(pc.read64(regs.rsp + 8), la::CODE64 + 16);
            }
        }

        // ---- HALT --------------------------------------------------------

        #[test]
        fn hlt_is_a_halt_exit_and_resuming_runs_the_next_instruction() {
            for engine in engines() {
                // hlt ; mov eax, 5 ; hlt
                let pc = Pc::flat32(engine, &[0xf4, 0xb8, 5, 0, 0, 0, 0xf4]);
                pc.cpu.set_exit_mask(ExitMask::NONE.with(ExitReason::HALT));
                let exit = pc.exit(1_000).expect("an exit");
                assert_eq!(exit.reason, ExitReason::HALT, "{engine:?}");
                assert_eq!((exit.pc, exit.len), (at::CODE0 + 1, 0), "KVM's shape");
                assert!(!pc.cpu.is_halted(), "the halt is the host's");
                let exit = pc.exit(1_000).expect("the second hlt");
                assert_eq!(exit.pc, at::CODE0 + 7);
                assert_eq!(pc.cpu.reg(Reg::Eax), 5);

                // A core already halted when the mask arrives reports it too.
                let pc = Pc::flat32(engine, &[0xf4]);
                pc.run(10);
                assert!(pc.cpu.is_halted());
                let run = pc.cpu.run_to_exit(Budget::of(500));
                assert!(run.exit.is_none(), "no mask: an idle core, not an exit");
                assert!(run.consumed.ticks < 500, "returned early");
                pc.cpu.set_exit_mask(ExitMask::NONE.with(ExitReason::HALT));
                assert_eq!(pc.exit(500).map(|e| e.reason), Some(ExitReason::HALT));
                assert!(!pc.cpu.is_halted());
            }
        }

        // ---- BREAKPOINT --------------------------------------------------

        #[test]
        fn int3_and_icebp_stop_at_themselves() {
            for engine in engines() {
                // nop ; int3 ; icebp ; hlt
                let pc = Pc::flat32(engine, &[0x90, 0xcc, 0xf1, 0xf4]);
                pc.cpu
                    .set_exit_mask(ExitMask::NONE.with(ExitReason::BREAKPOINT));
                let exit = pc.exit(1_000).expect("int3");
                assert_eq!(exit.reason, ExitReason::BREAKPOINT, "{engine:?}");
                assert_eq!((exit.pc, exit.len), (at::CODE0 + 1, 1));
                assert_eq!(pc.cpu.pc(), at::CODE0 + 1, "at the instruction");
                // Stepping over is the host's: past it, the next one stops.
                pc.cpu.set_pc(exit.pc + u64::from(exit.len));
                let exit = pc.exit(1_000).expect("icebp");
                assert_eq!((exit.pc, exit.len), (at::CODE0 + 2, 1));
            }
        }

        #[test]
        fn a_host_breakpoint_stops_before_the_instruction_executes() {
            for engine in engines() {
                // nop ; nop ; mov eax, 1 ; hlt — breakpoint on the mov.
                let pc = Pc::flat32(engine, &[0x90, 0x90, 0xb8, 1, 0, 0, 0, 0xf4]);
                pc.cpu.add_breakpoint(at::CODE0 + 2);
                let exit = pc.exit(1_000).expect("a breakpoint");
                assert_eq!(exit.reason, ExitReason::BREAKPOINT, "{engine:?}");
                assert_eq!((exit.pc, exit.len), (at::CODE0 + 2, 0));
                assert_eq!(exit.address, at::CODE0 + 2);
                assert_eq!(pc.cpu.regs().rip, at::CODE0 + 2);
                assert_eq!(pc.cpu.reg(Reg::Eax), 0, "the mov has not run");

                // Resuming executes the instruction rather than re-trapping.
                assert!(pc.exit(1_000).is_none());
                assert_eq!(pc.cpu.reg(Reg::Eax), 1);
                assert!(pc.cpu.is_halted());
            }
        }

        #[test]
        fn a_breakpoint_range_catches_a_call_into_the_window() {
            // The thunk window: nothing is mapped there, and nothing needs to
            // be, because the core stops before it fetches.
            for engine in engines() {
                // call 0xfffe0010 ; mov ebx, eax ; hlt
                let target: u32 = 0xfffe_0010;
                let rel = target.wrapping_sub(at::CODE0 as u32 + 5);
                let mut code = alloc::vec![0xe8];
                code.extend_from_slice(&rel.to_le_bytes());
                code.extend_from_slice(&[0x89, 0xc3, 0xf4]);
                let pc = Pc::flat32(engine, &code);
                pc.cpu.add_breakpoint_range(0xfffe_0000..0xffff_0000);
                assert_eq!(
                    pc.cpu.breakpoints(),
                    alloc::vec![0xfffe_0000u64..0xffff_0000]
                );
                let exit = pc.exit(1_000).expect("a breakpoint");
                assert_eq!(exit.reason, ExitReason::BREAKPOINT, "{engine:?}");
                assert_eq!(exit.pc, u64::from(target));

                // The host runs its stub — returns 42 — and pops the return
                // address, as a `RET` would.
                let sp = pc.cpu.sp();
                let ret = pc.read32(sp);
                assert_eq!(ret, at::CODE0 + 5);
                pc.cpu.set_reg(Reg::Eax, 42);
                pc.cpu.set_sp(sp + 4);
                pc.cpu.set_pc(ret);
                assert!(pc.exit(1_000).is_none());
                assert_eq!(pc.cpu.reg(Reg::Ebx), 42);
            }
        }

        #[test]
        fn a_breakpoint_inside_a_hot_loop_stops_on_every_visit() {
            // The block-cache case: the loop runs translated first, and
            // arming a breakpoint in the *middle* of its block must still
            // stop on exactly that instruction every time round.
            for engine in engines() {
                // mov edx, 100 ; L: inc eax ; inc ebx ; inc ecx ; dec edx ;
                // jnz L ; hlt
                let code = [
                    0xba, 100, 0, 0, 0,    // mov edx, 100
                    0x40, // inc eax       (L = CODE0 + 5)
                    0x43, // inc ebx
                    0x41, // inc ecx       (CODE0 + 7)
                    0x4a, // dec edx
                    0x75, 0xfa, // jnz L
                    0xf4,
                ];
                let pc = Pc::flat32(engine, &code);
                // Partway, with nothing armed.
                assert!(pc.cpu.run_to_exit(Budget::of(400)).exit.is_none());
                let done = pc.cpu.reg(Reg::Ecx);
                assert!(done > 2 && done < 100, "{engine:?}: ran {done}");
                #[cfg(all(feature = "cpu-x86-lift", feature = "jit"))]
                if engine != Engine::Interp {
                    let stats = pc.cpu.jit_stats().expect("a translating engine");
                    assert!(stats.retired > 0, "{engine:?}: the loop ran as blocks");
                }

                pc.cpu.add_breakpoint(at::CODE0 + 7);
                let mut hits = 0;
                while let Some(exit) = pc.exit(10_000) {
                    assert_eq!(exit.pc, at::CODE0 + 7, "{engine:?}");
                    let regs = pc.cpu.regs();
                    assert_eq!(
                        regs.rbx,
                        regs.rcx + 1,
                        "{engine:?}: between inc ebx and inc ecx"
                    );
                    hits += 1;
                }
                assert_eq!(hits, 100 - done, "{engine:?}");
                assert_eq!(pc.cpu.reg(Reg::Ecx), 100);
            }
        }

        #[test]
        fn removing_a_breakpoint_disarms_it() {
            for engine in engines() {
                let pc = Pc::flat32(engine, &[0x90, 0x90, 0xb8, 1, 0, 0, 0, 0xf4]);
                pc.cpu.add_breakpoint(at::CODE0 + 2);
                pc.cpu.add_breakpoint(at::CODE0 + 1);
                assert!(pc.cpu.remove_breakpoint(at::CODE0 + 2));
                assert!(!pc.cpu.remove_breakpoint(at::CODE0 + 2), "already gone");
                assert!(!pc.cpu.remove_breakpoint_range(5..5));
                assert_eq!(pc.exit(1_000).map(|e| e.pc), Some(at::CODE0 + 1));
                pc.cpu.clear_breakpoints();
                assert!(pc.cpu.breakpoints().is_empty());
                assert!(pc.exit(1_000).is_none(), "{engine:?}");
                assert_eq!(pc.cpu.reg(Reg::Eax), 1);
            }
        }

        #[test]
        fn a_host_breakpoint_fires_without_a_mask_and_int3_stays_the_guests() {
            for engine in engines() {
                // int3 ; mov eax, 1 ; hlt — the guest owns vector 3.
                let pc = Pc::flat32(engine, &[0xcc, 0xb8, 1, 0, 0, 0, 0xf4]);
                mark_handler(&pc, 3, at::CODE0 + 0x100);
                pc.cpu.add_breakpoint(at::CODE0 + 0x100);
                let exit = pc.exit(1_000).expect("the host's breakpoint");
                assert_eq!(exit.pc, at::CODE0 + 0x100, "{engine:?}: in the handler");
                assert!(pc.exit(1_000).is_none());
                assert_eq!(pc.ram.read_u8(at::MARK).unwrap(), 0xaa);
            }
        }
    }

    /// The XSAVE feature set, as a guest drives it.
    ///
    /// Every expectation is from the *Intel SDM* volume 1 chapter 13 or the
    /// volume 2 instruction pages, named on each test.
    mod xsave {
        use super::*;
        use crate::cpu::x86::fpu::{Sse, X87, mxcsr};
        use crate::cpu::x86::prot::xcr0;

        /// `0x48`, `REX.W`.
        const REX_W: u8 = 0x48;
        /// A 64-byte-aligned area, and a second for comparisons.
        const AREA: u64 = 0xd000;
        const AREA2: u64 = 0xe000;
        /// Where vector `v`'s handler — a lone `hlt` — sits.
        const HANDLERS: u64 = 0x3800;

        /// An x86-64 part with the XSAVE feature set.
        fn features() -> Features {
            Features {
                xsave: true,
                ..Features::X86_64
            }
        }

        /// That part in flat 32-bit protected mode, with `CR4.OSFXSR` and
        /// `CR4.OSXSAVE` set as an operating system would, and a handler for
        /// each of `#UD`, `#NM` and `#GP` so a test can see which it took.
        fn xpc() -> Pc {
            xpc_with(features())
        }

        fn xpc_with(features: Features) -> Pc {
            let pc = Pc::with_features(Variant::X86_64, features);
            pc.start_protected();
            let mut sys = pc.cpu.sys();
            sys.cr4 |= cr4::OSFXSR | cr4::OSXMMEXCPT;
            if features.xsave {
                sys.cr4 |= cr4::OSXSAVE;
            }
            pc.cpu.set_sys(sys);
            for v in [6u64, 7, 13] {
                let at = HANDLERS + v * 16;
                pc.idt(v, gate(0x08, at as u32, sys_type::INT_GATE32, 0));
                pc.write(at, &[0xf4]);
            }
            pc
        }

        /// Run `code` (a `hlt` is appended) from `at::CODE0`, and say which
        /// exception it ended in, if any.
        fn exec(pc: &Pc, code: &[u8]) -> Option<u64> {
            let mut code = code.to_vec();
            code.push(0xf4);
            pc.write(at::CODE0, &code);
            let mut regs = pc.regs();
            regs.rip = at::CODE0;
            pc.cpu.set_regs(regs);
            pc.cpu.session.lock().state.halted = false;
            assert!(pc.run(200) < 200, "the program halted");
            let rip = pc.regs().rip;
            [6u64, 7, 13]
                .into_iter()
                .find(|v| rip == HANDLERS + v * 16 + 1)
        }

        /// `mov r32, imm32` for register number `r`.
        fn mov(r: u8, imm: u32) -> Vec<u8> {
            let mut out = alloc::vec![0xb8 + r];
            out.extend_from_slice(&imm.to_le_bytes());
            out
        }

        /// `EDX:EAX = mask` and `ECX = ecx`.
        fn args(mask: u64, ecx: u32) -> Vec<u8> {
            let mut out = mov(0, mask as u32);
            out.extend(mov(2, (mask >> 32) as u32));
            out.extend(mov(1, ecx));
            out
        }

        /// A two-byte-opcode instruction with a `disp32` memory operand —
        /// the 32-bit absolute form.
        fn mem32(op: [u8; 2], reg: u8, addr: u64) -> Vec<u8> {
            let mut out = alloc::vec![op[0], op[1], (reg << 3) | 0b101];
            out.extend_from_slice(&(addr as u32).to_le_bytes());
            out
        }

        /// The same in 64-bit mode, where `mod == 00, r/m == 101` is
        /// `RIP`-relative and absolute takes a SIB byte; `rex` prefixed if
        /// given.
        fn mem64(rex: Option<u8>, op: [u8; 2], reg: u8, addr: u64) -> Vec<u8> {
            let mut out = Vec::new();
            out.extend(rex);
            out.extend_from_slice(&[op[0], op[1], (reg << 3) | 0b100, 0x25]);
            out.extend_from_slice(&(addr as u32).to_le_bytes());
            out
        }

        const GRP15: [u8; 2] = [0x0f, 0xae];
        const GRP9: [u8; 2] = [0x0f, 0xc7];
        const XGETBV: [u8; 3] = [0x0f, 0x01, 0xd0];
        const XSETBV: [u8; 3] = [0x0f, 0x01, 0xd1];

        fn xsetbv(mask: u64) -> Vec<u8> {
            let mut out = args(mask, 0);
            out.extend_from_slice(&XSETBV);
            out
        }

        fn bytes(pc: &Pc, addr: u64, len: u64) -> Vec<u8> {
            (0..len)
                .map(|i| pc.ram.read_u8(addr + i).unwrap())
                .collect()
        }

        /// A state with something distinctive in every part of both legacy
        /// components: two x87 registers, all sixteen `XMM` registers, and a
        /// non-default `MXCSR`.
        fn distinctive(pc: &Pc) {
            let mut sse = Sse::new();
            for i in 0..16u8 {
                let n = u64::from(i) + 1;
                sse.set(i, [n * 0x0101_0101_0101_0101, !n]);
            }
            sse.mxcsr = mxcsr::RESET | mxcsr::FTZ;
            pc.cpu.set_sse(sse);
        }

        /// `fninit ; fld1 ; fldpi`.
        const X87_CODE: [u8; 6] = [0xdb, 0xe3, 0xd9, 0xe8, 0xd9, 0xeb];

        #[test]
        fn without_osxsave_every_instruction_is_an_invalid_opcode() {
            // *Intel SDM* volume 2, `XGETBV`/`XSETBV`/`XSAVE`: `#UD` if
            // `CR4.OSXSAVE[bit 18] = 0`.
            let pc = xpc();
            let mut sys = pc.cpu.sys();
            sys.cr4 &= !cr4::OSXSAVE;
            pc.cpu.set_sys(sys);
            let mut getbv = args(0, 0);
            getbv.extend_from_slice(&XGETBV);
            assert_eq!(exec(&pc, &getbv), Some(6));
            assert_eq!(exec(&pc, &xsetbv(3)), Some(6));
            assert_eq!(exec(&pc, &mem32(GRP15, 4, AREA)), Some(6));
            assert_eq!(exec(&pc, &mem32(GRP15, 5, AREA)), Some(6));
        }

        #[test]
        fn a_part_without_the_feature_answers_invalid_opcode_and_refuses_osxsave() {
            let pc = xpc_with(Features::X86_64);
            let mut getbv = args(0, 0);
            getbv.extend_from_slice(&XGETBV);
            assert_eq!(exec(&pc, &getbv), Some(6));
            assert_eq!(exec(&pc, &mem32(GRP15, 4, AREA)), Some(6));
            assert_eq!(exec(&pc, &mem32(GRP9, 4, AREA)), Some(6));
            // mov eax, cr4 ; or eax, OSXSAVE ; mov cr4, eax — a reserved bit
            // on this part, so `#GP(0)` (*Intel SDM* volume 3 §2.5).
            let mut code = alloc::vec![0x0f, 0x20, 0xe0, 0x0d];
            code.extend_from_slice(&(cr4::OSXSAVE as u32).to_le_bytes());
            code.extend_from_slice(&[0x0f, 0x22, 0xe0]);
            assert_eq!(exec(&pc, &code), Some(13));
            assert_eq!(pc.cpu.sys().cr4 & cr4::OSXSAVE, 0);
            // And the part with it takes the same write.
            let pc = xpc();
            let mut sys = pc.cpu.sys();
            sys.cr4 &= !cr4::OSXSAVE;
            pc.cpu.set_sys(sys);
            assert_eq!(exec(&pc, &code), None);
            assert_ne!(pc.cpu.sys().cr4 & cr4::OSXSAVE, 0);
        }

        #[test]
        fn xsetbv_writes_xcr0_and_xgetbv_reads_it_back() {
            let pc = xpc();
            assert_eq!(pc.cpu.sys().xcr0, xcr0::RESET, "x87 alone out of reset");
            let mut code = xsetbv(3);
            code.extend(args(u64::MAX, 0));
            code.extend_from_slice(&XGETBV);
            assert_eq!(exec(&pc, &code), None);
            let regs = pc.regs();
            assert_eq!((regs.rax, regs.rdx), (3, 0));
            assert_eq!(pc.cpu.sys().xcr0, xcr0::X87 | xcr0::SSE);
        }

        #[test]
        fn xsetbv_refuses_a_value_no_processor_could_hold() {
            // *Intel SDM* volume 2, `XSETBV`: `#GP(0)` for an unsupported
            // bit, for bit 0 clear, for bit 2 without bit 1, and for an `ECX`
            // other than zero — `XCR0` is the only one that is writable.
            let pc = xpc();
            for bad in [
                0,              // no x87
                xcr0::SSE,      // still no x87
                0b111,          // AVX, which this part does not have
                0b101,          // AVX without SSE
                1 << 32 | 0b11, // a bit in EDX
            ] {
                assert_eq!(exec(&pc, &xsetbv(bad)), Some(13), "{bad:#x}");
                assert_eq!(pc.cpu.sys().xcr0, xcr0::RESET);
            }
            let mut code = args(0b11, 1);
            code.extend_from_slice(&XSETBV);
            assert_eq!(exec(&pc, &code), Some(13), "XINUSE is read-only");
            let mut code = args(0, 2);
            code.extend_from_slice(&XGETBV);
            assert_eq!(exec(&pc, &code), Some(13), "and there is no XCR2");
        }

        #[test]
        fn xsetbv_is_ring_zero_only_and_xgetbv_is_not() {
            // The `a_ring_three_program_may_not_touch_the_privileged_
            // instructions` bring-up, then `xsetbv` in ring 3.
            let pc = xpc();
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
            pc.write(
                at::CODE0,
                &[
                    0xb8, 0x28, 0x00, 0x00, 0x00, 0x0f, 0x00, 0xd8, // ltr 0x28
                    0x6a, 0x23, 0x68, 0x00, 0xa0, 0x00, 0x00, 0x6a, 0x02, 0x6a, 0x1b, 0x68, 0x00,
                    0x40, 0x00, 0x00, 0xcf,
                ],
            );
            // Ring 3: xgetbv(0) is fine; xsetbv is #GP(0).
            let mut ring3 = args(0, 0);
            ring3.extend_from_slice(&XGETBV);
            ring3.extend_from_slice(&[0x89, 0xc3]); // mov ebx, eax
            ring3.extend(xsetbv(3));
            ring3.push(0xf4);
            pc.write(at::CODE3, &ring3);
            assert!(pc.run(200) < 200);
            assert_eq!(pc.regs().rip, HANDLERS + 13 * 16 + 1, "#GP");
            assert_eq!(pc.regs().rbx, 1, "XGETBV ran at CPL 3 and read XCR0");
            assert_eq!(pc.cpu.sys().xcr0, xcr0::RESET);
        }

        #[test]
        fn xgetbv_one_reports_which_components_are_not_in_their_init_state() {
            // *Intel SDM* volume 1 §13.6: `XGETBV` with `ECX = 1` returns
            // `XCR0 & XINUSE`. This core's `XINUSE` is "differs from the init
            // configuration", so it is exact.
            let pc = xpc();
            let xinuse = |pc: &Pc, prefix: &[u8]| {
                let mut code = prefix.to_vec();
                code.extend(args(0, 1));
                code.extend_from_slice(&XGETBV);
                assert_eq!(exec(pc, &code), None);
                pc.regs().rax
            };
            assert_eq!(xinuse(&pc, &xsetbv(3)), 0, "nothing used out of reset");
            assert_eq!(xinuse(&pc, &[0xd9, 0xe8]), 1, "fld1 uses the x87 unit");
            // pcmpeqd xmm5, xmm5 — all ones.
            assert_eq!(xinuse(&pc, &[0x66, 0x0f, 0x76, 0xed]), 3);
            // fninit leaves the registers' contents, so still in use; but
            // `MXCSR` alone does not count (§13.6).
            let mut x = X87::new();
            pc.cpu.set_x87(x);
            pc.cpu.set_sse(Sse {
                mxcsr: mxcsr::RESET | mxcsr::FTZ,
                ..Sse::new()
            });
            assert_eq!(xinuse(&pc, &[]), 0);
            x.control = 0x27f;
            pc.cpu.set_x87(x);
            assert_eq!(xinuse(&pc, &[]), 1, "a changed control word is use");
            // And only the enabled components are reported.
            assert_eq!(xinuse(&pc, &xsetbv(1)), 1);
        }

        #[test]
        fn cpuid_describes_the_feature_set_and_echoes_cr4() {
            // *Intel SDM* volume 2 `CPUID`: leaf 1 `ECX[26]` XSAVE and
            // `ECX[27]` OSXSAVE, the latter a copy of `CR4.OSXSAVE`; leaf
            // `0Dh` the components and sizes.
            let cpuid = |pc: &Pc, leaf: u32, sub: u32| {
                let mut code = mov(0, leaf);
                code.extend(mov(1, sub));
                code.extend_from_slice(&[0x0f, 0xa2]);
                assert_eq!(exec(pc, &code), None);
                let r = pc.regs();
                (r.rax as u32, r.rbx as u32, r.rcx as u32, r.rdx as u32)
            };
            let pc = xpc();
            assert_eq!(cpuid(&pc, 0, 0).0, 0xd, "leaf 0Dh is reachable");
            let ecx = cpuid(&pc, 1, 0).2;
            assert_eq!(ecx & (3 << 26), 3 << 26, "XSAVE and OSXSAVE");
            let mut sys = pc.cpu.sys();
            sys.cr4 &= !cr4::OSXSAVE;
            pc.cpu.set_sys(sys);
            assert_eq!(cpuid(&pc, 1, 0).2 & (3 << 26), 1 << 26, "OSXSAVE is live");
            sys.cr4 |= cr4::OSXSAVE;
            pc.cpu.set_sys(sys);

            // Sub-leaf 0: XCR0 may hold x87|SSE; the area for XCR0 = x87 alone
            // and for everything is the legacy region plus the header.
            assert_eq!(cpuid(&pc, 0xd, 0), (3, 576, 576, 0));
            // Sub-leaf 1: XSAVEOPT, XSAVEC and XGETBV(1), not XSAVES.
            assert_eq!(cpuid(&pc, 0xd, 1), (0b111, 576, 0, 0));
            // No extended component yet, and 0/1 describe the legacy region.
            assert_eq!(cpuid(&pc, 0xd, 2), (0, 0, 0, 0));
            assert_eq!(cpuid(&pc, 0xd, 63), (0, 0, 0, 0));
            // The leaves between say nothing, with leaf 2's architectural
            // `AL = 01h`.
            assert_eq!(cpuid(&pc, 2, 0), (1, 0, 0, 0));
            for leaf in [3, 4, 7, 0xb, 0xc] {
                assert_eq!(cpuid(&pc, leaf, 0), (0, 0, 0, 0), "leaf {leaf:#x}");
            }

            // A part without the feature still stops at leaf 1.
            let pc = xpc_with(Features::X86_64);
            assert_eq!(cpuid(&pc, 0, 0).0, 1);
            assert_eq!(cpuid(&pc, 1, 0).2 & (3 << 26), 0);
            assert_eq!(cpuid(&pc, 0xd, 0), (0, 0, 0, 0));
        }

        #[test]
        fn xsave_and_xrstor_carry_the_x87_and_sse_state_through_memory() {
            let pc = xpc();
            distinctive(&pc);
            let mut code = xsetbv(3);
            code.extend_from_slice(&X87_CODE);
            code.extend(args(u64::MAX, 0));
            code.extend(mem32(GRP15, 4, AREA)); // xsave
            assert_eq!(exec(&pc, &code), None);
            let x87 = pc.cpu.x87();
            let sse = pc.cpu.sse();
            assert_eq!(
                bytes(&pc, AREA + 512, 8),
                [3, 0, 0, 0, 0, 0, 0, 0],
                "XSTATE_BV: both in use"
            );
            // Wreck both, then restore.
            pc.cpu.set_x87(X87::new());
            pc.cpu.set_sse(Sse::new());
            let mut code = args(u64::MAX, 0);
            code.extend(mem32(GRP15, 5, AREA)); // xrstor
            assert_eq!(exec(&pc, &code), None);
            assert_eq!(pc.cpu.x87(), x87);
            let back = pc.cpu.sse();
            assert_eq!(back.mxcsr, sse.mxcsr);
            assert_eq!(back.xmm[..8], sse.xmm[..8]);
        }

        #[test]
        fn the_legacy_region_is_fxsaves_image_byte_for_byte() {
            // *Intel SDM* volume 1 §13.4.1: the legacy region "has the same
            // format as the FXSAVE area". Outside 64-bit mode the SSE
            // component is `XMM0`-`XMM7` (§13.5.2), so the image agrees up to
            // byte 288 and `XSAVE` leaves the rest of it alone where `FXSAVE`
            // writes all sixteen.
            let pc = xpc();
            distinctive(&pc);
            pc.write(AREA2, &[0xaa; 576]);
            let mut code = xsetbv(3);
            code.extend_from_slice(&X87_CODE);
            code.extend(mem32(GRP15, 0, AREA)); // fxsave
            code.extend(args(u64::MAX, 0));
            code.extend(mem32(GRP15, 4, AREA2)); // xsave
            assert_eq!(exec(&pc, &code), None);
            assert_eq!(bytes(&pc, AREA, 288), bytes(&pc, AREA2, 288));
            assert!(bytes(&pc, AREA2 + 288, 224).iter().all(|&b| b == 0xaa));
        }

        /// Bring the XSAVE part up into 64-bit mode and run `code` there.
        fn run64x(pc: &Pc, code: &[u8]) {
            pc.prepare_long();
            pc.write(at::CODE0, &enter_long_mode_code(la::CODE64));
            pc.write(la::CODE64, code);
            assert!(pc.run(300) < 300, "the 64-bit program halted");
            assert!(pc.cpu.sys().sixty_four(), "and did so in 64-bit mode");
        }

        #[test]
        fn in_64_bit_mode_both_forms_match_fxsave_and_rex_w_widens_the_pointers() {
            // `XSAVE64` is `XSAVE` with `REX.W`, and like `FXSAVE64` it writes
            // the x87 instruction and data pointers as eight bytes each
            // (*Intel SDM* volume 2, `XSAVE`). Sixteen `XMM` registers here.
            for rex in [None, Some(REX_W)] {
                let pc = xpc();
                distinctive(&pc);
                // Software zeroes the header before the first save into an
                // area; a stray bit there would make the restore below #GP.
                pc.write(AREA2, &[0xaa; 512]);
                pc.write(AREA2 + 512, &[0; 64]);
                let mut code = xsetbv(3);
                code.extend_from_slice(&X87_CODE);
                code.extend(mem64(rex, GRP15, 0, AREA));
                code.extend(args(u64::MAX, 0));
                code.extend(mem64(rex, GRP15, 4, AREA2));
                // Restore from the XSAVE image into a wrecked unit.
                code.extend_from_slice(&[0xdb, 0xe3, 0x0f, 0x57, 0xc0]); // fninit ; xorps xmm0
                code.extend(mem64(rex, GRP15, 5, AREA2));
                code.push(0xf4);
                let before_sse = pc.cpu.sse();
                run64x(&pc, &code);
                assert_eq!(bytes(&pc, AREA, 416), bytes(&pc, AREA2, 416), "{rex:?}");
                assert_eq!(pc.cpu.sse().xmm, before_sse.xmm, "all sixteen came back");
                assert_eq!(
                    pc.cpu.x87().raw(0),
                    crate::float::x87::F80::new(0x4000, 0xc90f_daa2_2168_c235)
                );
            }
        }

        #[test]
        fn a_clear_xstate_bv_bit_restores_the_component_to_its_init_state() {
            // *Intel SDM* volume 1 §13.8.1: with `RFBM[i] = 1` and
            // `XSTATE_BV[i] = 0`, `XRSTOR` initialises component `i` and does
            // not read it from the area. `MXCSR` is loaded from the area in
            // the standard form whatever `XSTATE_BV` says.
            let pc = xpc();
            distinctive(&pc);
            pc.write(AREA, &[0x55; 576]);
            pc.write(AREA + 512, &[0; 64]);
            pc.write32(AREA + 24, u64::from(mxcsr::RESET | mxcsr::DAZ));
            let mut code = xsetbv(3);
            code.extend_from_slice(&X87_CODE);
            code.extend(args(u64::MAX, 0));
            code.extend(mem32(GRP15, 5, AREA));
            assert_eq!(exec(&pc, &code), None);
            assert_eq!(pc.cpu.x87(), X87::new(), "x87 in its init configuration");
            let sse = pc.cpu.sse();
            assert!(sse.xmm[..8].iter().all(|r| *r == [0, 0]), "XMM0-7 zero");
            assert_eq!(sse.mxcsr, mxcsr::RESET | mxcsr::DAZ);
        }

        #[test]
        fn rfbm_is_xcr0_and_edx_eax_and_only_its_components_are_touched() {
            // *Intel SDM* volume 1 §13.7: XSAVE writes component `i` only if
            // `RFBM[i]`, and `XSTATE_BV` becomes
            // `(old & !RFBM) | (XINUSE & RFBM)`.
            let pc = xpc();
            distinctive(&pc);
            pc.write(AREA, &[0xaa; 512]);
            pc.write(AREA + 512, &[0; 64]);
            pc.write(AREA + 512, &[0x02]); // an old XSTATE_BV naming SSE
            let mut code = xsetbv(3);
            code.extend_from_slice(&X87_CODE);
            code.extend(args(1, 0)); // x87 only
            code.extend(mem32(GRP15, 4, AREA));
            assert_eq!(exec(&pc, &code), None);
            assert_ne!(bytes(&pc, AREA, 2), [0xaa, 0xaa], "FCW written");
            assert_eq!(bytes(&pc, AREA + 24, 8), [0xaa; 8], "MXCSR not");
            assert_eq!(bytes(&pc, AREA + 160, 128), [0xaa; 128], "XMM not");
            assert_eq!(bytes(&pc, AREA + 512, 1), [0x03], "old SSE bit kept");

            // `XCR0` masks too: with x87 alone enabled, asking for everything
            // gets x87 alone.
            let pc = xpc();
            distinctive(&pc);
            pc.write(AREA, &[0xaa; 576]);
            let mut code = X87_CODE.to_vec();
            code.extend(args(u64::MAX, 0));
            code.extend(mem32(GRP15, 4, AREA));
            assert_eq!(exec(&pc, &code), None);
            assert_eq!(bytes(&pc, AREA + 160, 16), [0xaa; 16]);
            assert_eq!(bytes(&pc, AREA + 512, 1), [0xab], "0xaa & !1 | 1");
        }

        #[test]
        fn a_misaligned_area_and_a_reserved_header_bit_are_general_protection() {
            let pc = xpc();
            let mut code = xsetbv(3);
            code.extend(args(u64::MAX, 0));
            code.extend(mem32(GRP15, 4, AREA + 16));
            assert_eq!(exec(&pc, &code), Some(13), "XSAVE: 64-byte alignment");
            let mut code = args(u64::MAX, 0);
            code.extend(mem32(GRP15, 5, AREA + 32));
            assert_eq!(exec(&pc, &code), Some(13), "XRSTOR too");

            // §13.8.1's standard-form checks: XSTATE_BV naming a component
            // XCR0 does not enable, or bytes 8-23 of the header not zero.
            distinctive(&pc);
            let before = (pc.cpu.x87(), pc.cpu.sse());
            for (at, value) in [(0u64, 0b100u64), (8, 1), (16, 1)] {
                pc.write(AREA, &[0; 576]);
                pc.write32(AREA + 24, u64::from(mxcsr::RESET));
                pc.write32(AREA + 512 + at, value);
                let mut code = args(u64::MAX, 0);
                code.extend(mem32(GRP15, 5, AREA));
                assert_eq!(exec(&pc, &code), Some(13), "header byte {at}");
                assert_eq!((pc.cpu.x87(), pc.cpu.sse()), before, "nothing restored");
            }
            // Bytes 24-63 are not checked in the standard form.
            pc.write(AREA, &[0; 576]);
            pc.write32(AREA + 24, u64::from(mxcsr::RESET));
            pc.write32(AREA + 512 + 40, 0xffff);
            let mut code = args(u64::MAX, 0);
            code.extend(mem32(GRP15, 5, AREA));
            assert_eq!(exec(&pc, &code), None);
            // A reserved MXCSR bit about to be loaded is #GP(0), as FXRSTOR.
            pc.write32(AREA + 24, 1 << 31);
            assert_eq!(exec(&pc, &code), Some(13));
        }

        #[test]
        fn a_set_task_switched_flag_makes_them_device_not_available() {
            // `#NM` with `CR0.TS` set, which is how a lazy context switch
            // catches the first save or restore.
            let pc = xpc();
            let mut sys = pc.cpu.sys();
            sys.cr0 |= cr0::TS;
            pc.cpu.set_sys(sys);
            let mut code = args(u64::MAX, 0);
            code.extend(mem32(GRP15, 4, AREA));
            assert_eq!(exec(&pc, &code), Some(7));
            // XGETBV is not a state instruction and does not care.
            let mut code = args(0, 0);
            code.extend_from_slice(&XGETBV);
            assert_eq!(exec(&pc, &code), None);
        }

        #[test]
        fn xsaveopt_saves_exactly_what_xsave_does() {
            let pc = xpc();
            distinctive(&pc);
            let mut code = xsetbv(3);
            code.extend_from_slice(&X87_CODE);
            code.extend(args(u64::MAX, 0));
            code.extend(mem32(GRP15, 4, AREA));
            code.extend(mem32(GRP15, 6, AREA2)); // xsaveopt
            assert_eq!(exec(&pc, &code), None);
            assert_eq!(bytes(&pc, AREA, 576), bytes(&pc, AREA2, 576));
        }

        #[test]
        fn xsavec_writes_the_compacted_header_and_skips_what_is_unused() {
            // *Intel SDM* volume 1 §13.10: `XCOMP_BV = RFBM | 1 << 63`,
            // `XSTATE_BV = RFBM & XINUSE` — with `MXCSR` off its default
            // counting as SSE in use — and only those components written.
            let pc = xpc();
            pc.write(AREA, &[0xaa; 512]);
            pc.write(AREA + 512, &[0; 64]);
            let mut code = xsetbv(3);
            code.extend(args(u64::MAX, 0));
            code.extend(mem32(GRP9, 4, AREA));
            assert_eq!(exec(&pc, &code), None);
            assert_eq!(bytes(&pc, AREA, 512), [0xaa; 512], "nothing in use");
            let header = bytes(&pc, AREA + 512, 16);
            assert_eq!(header[..8], [0; 8]);
            assert_eq!(header[8..], [3, 0, 0, 0, 0, 0, 0, 0x80]);

            // Now with state, and back through XRSTOR's compacted form.
            distinctive(&pc);
            let mut code = X87_CODE.to_vec();
            code.extend(args(u64::MAX, 0));
            code.extend(mem32(GRP9, 4, AREA));
            assert_eq!(exec(&pc, &code), None);
            assert_eq!(bytes(&pc, AREA + 512, 1), [3]);
            let (x87, sse) = (pc.cpu.x87(), pc.cpu.sse());
            pc.cpu.set_x87(X87::new());
            pc.cpu.set_sse(Sse::new());
            let mut code = args(u64::MAX, 0);
            code.extend(mem32(GRP15, 5, AREA));
            assert_eq!(exec(&pc, &code), None);
            assert_eq!(pc.cpu.x87(), x87);
            assert_eq!(pc.cpu.sse().mxcsr, sse.mxcsr);
            assert_eq!(pc.cpu.sse().xmm[..8], sse.xmm[..8]);

            // A compacted XSTATE_BV bit that XCOMP_BV does not name, or a
            // reserved header byte past 16, is #GP(0) (§13.8.2).
            pc.write(AREA + 520, &[1, 0, 0, 0, 0, 0, 0, 0x80]);
            assert_eq!(exec(&pc, &code), Some(13));
            pc.write(AREA + 520, &[3, 0, 0, 0, 0, 0, 0, 0x80]);
            pc.write(AREA + 560, &[1]);
            assert_eq!(exec(&pc, &code), Some(13));
            pc.write(AREA + 560, &[0]);
            // With XSTATE_BV clear the compacted form resets MXCSR too.
            pc.write(AREA + 512, &[0]);
            assert_eq!(exec(&pc, &code), None);
            assert_eq!(pc.cpu.sse().mxcsr, mxcsr::RESET);
            assert_eq!(pc.cpu.x87(), X87::new());
        }

        #[test]
        fn xcr0_survives_a_snapshot() {
            let pc = xpc();
            assert_eq!(exec(&pc, &xsetbv(3)), None);
            let sys_before = pc.cpu.sys();
            assert_eq!(sys_before.xcr0, 3);

            let mut shape = MachineShape::new();
            shape.add_device("/cpu0", "cpu.x86").unwrap();
            let mut writer = StateWriter::new(shape);
            {
                let mut chunk = writer
                    .chunk("/cpu0", "cpu.x86", super::super::super::CLASS.version)
                    .unwrap();
                pc.cpu.save(&mut chunk).unwrap();
            }
            let bytes = writer.to_vec().unwrap();
            pc.cpu.reset(ResetKind::Cold);
            assert_eq!(
                pc.cpu.sys().xcr0,
                xcr0::RESET,
                "a reset puts x87 alone back"
            );

            let reader = StateReader::new(&bytes).unwrap();
            let (_, _, data) = reader.load_raw("/cpu0").unwrap();
            let mut chunk = ChunkReader::new(data);
            pc.cpu.load(&mut chunk).unwrap();
            chunk.end().unwrap();
            assert_eq!(pc.cpu.sys(), sys_before);
        }

        // -- AVX: component 2, the upper halves of the `YMM` registers ----

        /// A part with AVX on top of the XSAVE feature set.
        fn avx_features() -> Features {
            Features {
                xsave: true,
                avx: true,
                ..Features::X86_64_V2
            }
        }

        /// Something distinctive in every `YMM` upper half.
        fn distinctive_upper(pc: &Pc) {
            let mut sse = pc.cpu.sse();
            for i in 0..16u8 {
                let n = u64::from(i) + 0x41;
                sse.set_hi(i, [n * 0x0001_0001_0001_0001, !n << 8]);
            }
            pc.cpu.set_sse(sse);
        }

        fn read_u64(pc: &Pc, addr: u64) -> u64 {
            u64::from_le_bytes(bytes(pc, addr, 8).try_into().unwrap())
        }

        #[test]
        fn the_avx_component_is_described_by_cpuid_and_enabled_by_xsetbv() {
            // *Intel SDM* volume 2 `CPUID` leaf `0Dh`: sub-leaf 2 is 256 bytes
            // at offset 576, not aligned when compacted; the area for x87,
            // SSE and AVX is 832 bytes. Volume 2 `XSETBV`: AVX without SSE is
            // `#GP(0)`.
            let cpuid = |pc: &Pc, leaf: u32, sub: u32| {
                let mut code = mov(0, leaf);
                code.extend(mov(1, sub));
                code.extend_from_slice(&[0x0f, 0xa2]);
                assert_eq!(exec(pc, &code), None);
                let r = pc.regs();
                (r.rax as u32, r.rbx as u32, r.rcx as u32, r.rdx as u32)
            };
            let pc = xpc_with(avx_features());
            assert_eq!(cpuid(&pc, 0xd, 0), (7, 576, 832, 0));
            assert_eq!(cpuid(&pc, 0xd, 2), (256, 576, 0, 0));
            assert_eq!(exec(&pc, &xsetbv(5)), Some(13), "AVX needs SSE");
            assert_eq!(exec(&pc, &xsetbv(7)), None);
            assert_eq!(pc.cpu.sys().xcr0, 7);
            assert_eq!(cpuid(&pc, 0xd, 0), (7, 832, 832, 0));
            assert_eq!(cpuid(&pc, 0xd, 1).1, 832, "compacted size, unaligned");
            // Without AVX the component does not exist, and XCR0 cannot name
            // it.
            let pc = xpc();
            assert_eq!(cpuid(&pc, 0xd, 2), (0, 0, 0, 0));
            assert_eq!(exec(&pc, &xsetbv(7)), Some(13));
        }

        #[test]
        fn xsave_and_xrstor_carry_the_ymm_upper_halves() {
            // *Intel SDM* volume 1 §13.5.3: `YMM0_H`-`YMM15_H` at 576 in the
            // standard form, sixteen bytes each — and outside 64-bit mode
            // only the first eight are state, so only 128 bytes move.
            let pc = xpc_with(avx_features());
            distinctive_upper(&pc);
            pc.write(AREA + 512, &[0; 64]);
            pc.write(AREA + 576, &[0xaa; 256]);
            let mut code = xsetbv(7);
            code.extend(args(u64::MAX, 0));
            code.extend(mem32(GRP15, 4, AREA)); // xsave
            assert_eq!(exec(&pc, &code), None);
            let sse = pc.cpu.sse();
            assert_eq!(read_u64(&pc, AREA + 512), 4, "XSTATE_BV: AVX in use");
            for i in 0..8u64 {
                let hi = sse.hi(i as u8);
                assert_eq!(read_u64(&pc, AREA + 576 + 16 * i), hi[0]);
                assert_eq!(read_u64(&pc, AREA + 584 + 16 * i), hi[1]);
            }
            assert_eq!(
                bytes(&pc, AREA + 704, 128),
                [0xaa; 128],
                "YMM8_H on: untouched"
            );

            // Wreck the upper halves and restore them: eight come back, and
            // the other eight are left as they are.
            let mut wrecked = sse;
            wrecked.ymm_hi = [[0x5a; 2]; 16];
            pc.cpu.set_sse(wrecked);
            let mut code = args(u64::MAX, 0);
            code.extend(mem32(GRP15, 5, AREA)); // xrstor
            assert_eq!(exec(&pc, &code), None);
            let back = pc.cpu.sse();
            assert_eq!(back.ymm_hi[..8], sse.ymm_hi[..8]);
            assert_eq!(back.ymm_hi[8..], [[0x5a; 2]; 8]);

            // A clear XSTATE_BV bit puts the component in its init state.
            pc.write(AREA + 512, &[3]);
            assert_eq!(exec(&pc, &code), None);
            assert_eq!(pc.cpu.sse().ymm_hi[..8], [[0; 2]; 8]);
            // And XRSTOR without AVX in RFBM leaves it alone.
            distinctive_upper(&pc);
            let before = pc.cpu.sse().ymm_hi;
            let mut code = args(3, 0);
            code.extend(mem32(GRP15, 5, AREA));
            assert_eq!(exec(&pc, &code), None);
            assert_eq!(pc.cpu.sse().ymm_hi, before);
        }

        #[test]
        fn xinuse_and_xsavec_see_the_avx_component() {
            // §13.6: in use when it differs from zero; §13.10: compacted, it
            // follows the legacy region directly, at 576.
            let pc = xpc_with(avx_features());
            let xinuse = |pc: &Pc| {
                let mut code = args(0, 1);
                code.extend_from_slice(&XGETBV);
                assert_eq!(exec(pc, &code), None);
                pc.regs().rax
            };
            assert_eq!(exec(&pc, &xsetbv(7)), None);
            assert_eq!(xinuse(&pc), 0);
            distinctive_upper(&pc);
            assert_eq!(xinuse(&pc), 4);
            pc.write(AREA + 512, &[0; 64]);
            let mut code = args(u64::MAX, 0);
            code.extend(mem32(GRP9, 4, AREA)); // xsavec
            assert_eq!(exec(&pc, &code), None);
            assert_eq!(read_u64(&pc, AREA + 512), 4, "XSTATE_BV");
            assert_eq!(read_u64(&pc, AREA + 520), 7 | 1 << 63, "XCOMP_BV");
            assert_eq!(read_u64(&pc, AREA + 576), pc.cpu.sse().hi(0)[0]);
        }

        #[test]
        fn in_64_bit_mode_all_sixteen_upper_halves_move() {
            let pc = xpc_with(avx_features());
            distinctive_upper(&pc);
            pc.write(AREA + 512, &[0; 64]);
            let mut code = xsetbv(7);
            code.extend(args(u64::MAX, 0));
            code.extend(mem64(None, GRP15, 4, AREA));
            code.push(0xf4);
            let sse = pc.cpu.sse();
            run64x(&pc, &code);
            for i in 0..16u64 {
                assert_eq!(read_u64(&pc, AREA + 576 + 16 * i), sse.hi(i as u8)[0]);
                assert_eq!(read_u64(&pc, AREA + 584 + 16 * i), sse.hi(i as u8)[1]);
            }
        }

        #[test]
        fn in_64_bit_mode_vex_reaches_sixteen_ymm_registers() {
            // `R̄`, `X̄`, `B̄` and `vvvv[3]` name registers 8-15, and
            // `VZEROUPPER` clears all sixteen upper halves (*Intel SDM*
            // volume 2 §2.3.5, `VZEROUPPER`).
            let pc = xpc_with(avx_features());
            let one = 0x3f80_0000_3f80_0000u64; // two singles of 1.0
            let two = 0x4000_0000_4000_0000u64; // two singles of 2.0
            let sum = 0x4040_0000_4040_0000u64; // two singles of 3.0
            let mut sse = pc.cpu.sse();
            sse.set_ymm(10, [[one; 2]; 2]);
            sse.set_ymm(11, [[two; 2]; 2]);
            sse.set_ymm(15, [[5, 6], [7, 8]]);
            pc.cpu.set_sse(sse);
            let mut code = xsetbv(7);
            code.extend_from_slice(&[0xc4, 0x41, 0x2c, 0x58, 0xcb]); // vaddps ymm9, ymm10, ymm11
            // vmovups [AREA], ymm9
            code.extend_from_slice(&[0xc4, 0x61, 0x7c, 0x11, 0x0c, 0x25]);
            code.extend_from_slice(&(AREA as u32).to_le_bytes());
            code.extend_from_slice(&[0xc5, 0xf8, 0x77]); // vzeroupper
            code.push(0xf4);
            run64x(&pc, &code);
            for i in 0..4 {
                assert_eq!(read_u64(&pc, AREA + 8 * i), sum, "quadword {i}");
            }
            let sse = pc.cpu.sse();
            assert_eq!(sse.ymm(9), [[sum; 2], [0; 2]]);
            assert_eq!(sse.ymm(15), [[5, 6], [0; 2]], "all sixteen cleared");
        }

        #[test]
        fn the_ymm_upper_halves_survive_a_snapshot() {
            let pc = xpc_with(avx_features());
            distinctive_upper(&pc);
            let before = pc.cpu.sse();
            let mut shape = MachineShape::new();
            shape.add_device("/cpu0", "cpu.x86").unwrap();
            let mut writer = StateWriter::new(shape);
            {
                let mut chunk = writer
                    .chunk("/cpu0", "cpu.x86", super::super::super::CLASS.version)
                    .unwrap();
                pc.cpu.save(&mut chunk).unwrap();
            }
            let bytes = writer.to_vec().unwrap();
            pc.cpu.reset(ResetKind::Cold);
            assert_eq!(pc.cpu.sse().ymm_hi, [[0; 2]; 16], "a reset clears them");
            let reader = StateReader::new(&bytes).unwrap();
            let (_, _, data) = reader.load_raw("/cpu0").unwrap();
            let mut chunk = ChunkReader::new(data);
            pc.cpu.load(&mut chunk).unwrap();
            chunk.end().unwrap();
            assert_eq!(pc.cpu.sse(), before);
        }

        #[test]
        fn the_disassembler_names_every_form() {
            let text = |bytes: &[u8]| {
                let mut b = bytes.to_vec();
                b.resize(16, 0);
                let d = crate::cpu::x86::disasm::disassemble_as(
                    isa::Gen::I386,
                    isa::Bits::B64,
                    0,
                    0,
                    &b,
                );
                alloc::format!("{d}")
            };
            assert_eq!(text(&XGETBV), "xgetbv");
            assert_eq!(text(&XSETBV), "xsetbv");
            assert!(text(&[0x0f, 0xae, 0x20]).starts_with("xsave "));
            assert!(text(&[0x48, 0x0f, 0xae, 0x28]).starts_with("xrstor64 "));
            assert!(text(&[0x0f, 0xae, 0x30]).starts_with("xsaveopt "));
            assert!(text(&[0x48, 0x0f, 0xc7, 0x20]).starts_with("xsavec64 "));
            // The rest of `0F 01 /2`'s register forms are not instructions.
            assert!(!text(&[0x0f, 0x01, 0xd2]).starts_with("lgdt"));
        }
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

    // -- The transcendentals, packed decimal and FISTTP ----------------------

    /// `fninit`, then `fldcw` from `fpat::DATA + 0x80` holding `control`.
    fn prologue(pc: &Pc, control: u16) -> Vec<u8> {
        write64(pc, fpat::DATA + 0x80, u64::from(control));
        let mut code = alloc::vec![0xdb, 0xe3, 0xd9];
        code.extend_from_slice(&disp32(5, fpat::DATA + 0x80));
        code
    }

    /// `fld qword [addr]` of a binary64 bit pattern, stored at `addr` first.
    fn fld64(pc: &Pc, code: &mut Vec<u8>, addr: u64, bits: u64) {
        write64(pc, addr, bits);
        code.push(0xdd);
        code.extend_from_slice(&disp32(0, addr));
    }

    /// `fstp tword [OUT]` and `hlt`, then the ten bytes as a value.
    fn store_and_read(pc: &Pc, mut code: Vec<u8>) -> F80 {
        code.push(0xdb);
        code.extend_from_slice(&disp32(7, fpat::OUT));
        code.push(0xf4);
        run(pc, &code);
        let mut b = [0u8; 10];
        for (i, byte) in b.iter_mut().enumerate() {
            *byte = pc.ram.read_u8(fpat::OUT + i as u64).unwrap();
        }
        F80::from_bytes(b)
    }

    #[test]
    fn fsin_of_zero_and_the_out_of_range_case() {
        // SDM volume 2, `FSIN`: sin(+0) is +0 exactly, and an operand of
        // magnitude 2^63 or more sets C2 and is left where it was.
        let pc = x87pc();
        let mut code = prologue(&pc, cw::RESET);
        code.extend_from_slice(&[0xd9, 0xee, 0xd9, 0xfe, 0xf4]); // fldz ; fsin
        run(&pc, &code);
        let x = pc.cpu.x87();
        assert_eq!(x.raw(0), F80::ZERO);
        assert_eq!(x.status & (sw::C2 | sw::EXCEPTIONS), 0);

        let pc = x87pc();
        let mut code = prologue(&pc, cw::RESET);
        fld64(&pc, &mut code, fpat::DATA, 0x43f0_0000_0000_0000); // 2^64
        code.extend_from_slice(&[0xd9, 0xfe]); // fsin
        let v = store_and_read(&pc, code);
        assert_eq!(v, F80::new(0x403f, 1 << 63), "2^64, untouched");
        let x = pc.cpu.x87();
        assert_ne!(x.status & sw::C2, 0, "C2: reduce the argument yourself");
        assert_eq!(x.status & sw::EXCEPTIONS, 0);

        // The same for FPTAN, which then pushes nothing.
        let pc = x87pc();
        let mut code = prologue(&pc, cw::RESET);
        fld64(&pc, &mut code, fpat::DATA, 0xc3e0_0000_0000_0000); // -2^63
        code.extend_from_slice(&[0xd9, 0xf2, 0xf4]); // fptan
        run(&pc, &code);
        let x = pc.cpu.x87();
        assert_ne!(x.status & sw::C2, 0);
        assert_eq!(x.top(), 7, "one value on the stack, not two");
    }

    #[test]
    fn fyl2x_f2xm1_fscale_compute_pow_2_10() {
        // MSVC's pow(x, y): y * log2(x) by FYL2X, split into integer and
        // fraction, 2^fraction by F2XM1 plus one, and the integer applied by
        // FSCALE. Both results are exact in binary64.
        let pow = |x: u64, y: u64| {
            let pc = x87pc();
            let mut code = prologue(&pc, cw::RESET);
            fld64(&pc, &mut code, fpat::DATA, y);
            fld64(&pc, &mut code, fpat::DATA + 8, x);
            code.extend_from_slice(&[
                0xd9, 0xf1, // fyl2x
                0xd9, 0xc0, // fld st(0)
                0xd9, 0xfc, // frndint
                0xdc, 0xe9, // fsub st(1), st(0)
                0xd9, 0xc9, // fxch st(1)
                0xd9, 0xf0, // f2xm1
                0xd9, 0xe8, // fld1
                0xde, 0xc1, // faddp st(1), st(0)
                0xd9, 0xfd, // fscale
                0xdd, 0xd9, // fstp st(1)
                0xdd, // fstp qword
            ]);
            code.extend_from_slice(&disp32(3, fpat::OUT));
            code.push(0xf4);
            run(&pc, &code);
            (read64(&pc, fpat::OUT), pc.cpu.x87())
        };
        // 2^10: log2 2 is exactly 1 and F2XM1(0) is exactly 0.
        let (bits, x) = pow(0x4000_0000_0000_0000, 0x4024_0000_0000_0000);
        assert_eq!(bits, 0x4090_0000_0000_0000, "1024.0");
        assert_eq!(x.status & sw::PE, 0, "and every step was exact");
        assert_eq!(x.top(), 0, "the stack is balanced");
        // 10^3, which goes through an irrational logarithm and still lands on
        // 1000 once rounded to binary64.
        let (bits, _) = pow(0x4024_0000_0000_0000, 0x4008_0000_0000_0000);
        assert_eq!(bits, 0x408f_4000_0000_0000, "1000.0");
    }

    #[test]
    fn fptan_pushes_one() {
        let pc = x87pc();
        let mut code = prologue(&pc, cw::RESET);
        code.extend_from_slice(&[0xd9, 0xe8, 0xd9, 0xf2, 0xf4]); // fld1 ; fptan
        run(&pc, &code);
        let x = pc.cpu.x87();
        assert_eq!(x.top(), 6, "two values");
        assert_eq!(x.raw(0), F80::new(0x3fff, 1 << 63), "1.0 on top");
        // tan 1, correctly rounded to nearest (mpmath, see
        // `float::elementary`'s tests), which is below the exact value.
        assert_eq!(x.raw(1), F80::new(0x3fff, 0xc759_22e5_f71d_2dc5));
        assert_eq!(x.status & (sw::C1 | sw::C2), 0, "rounded down, in range");
        assert_ne!(x.status & sw::PE, 0);
    }

    #[test]
    fn fsincos_leaves_the_cosine_on_top_and_c1_says_which_way_it_rounded() {
        let pc = x87pc();
        let mut code = prologue(&pc, cw::RESET);
        code.extend_from_slice(&[0xd9, 0xe8, 0xd9, 0xfb, 0xf4]); // fld1 ; fsincos
        run(&pc, &code);
        let x = pc.cpu.x87();
        assert_eq!(x.raw(0), F80::new(0x3ffe, 0x8a51_407d_a834_5c92), "cos 1");
        assert_eq!(x.raw(1), F80::new(0x3ffe, 0xd76a_a478_4867_7021), "sin 1");
        assert_ne!(x.status & sw::C1, 0, "cos 1 rounded up");
    }

    #[test]
    fn the_rounding_control_reaches_the_transcendentals_and_precision_control_does_not() {
        let sin_one = |control: u16| {
            let pc = x87pc();
            let mut code = prologue(&pc, control);
            code.extend_from_slice(&[0xd9, 0xe8, 0xd9, 0xfe]); // fld1 ; fsin
            let v = store_and_read(&pc, code);
            (v.sig, pc.cpu.x87().status & sw::C1 != 0)
        };
        let down = cw::RESET & !cw::RC | (1 << cw::RC_SHIFT);
        let up = cw::RESET & !cw::RC | (2 << cw::RC_SHIFT);
        assert_eq!(sin_one(cw::RESET), (0xd76a_a478_4867_7021, true));
        assert_eq!(sin_one(down), (0xd76a_a478_4867_7020, false));
        assert_eq!(sin_one(up), (0xd76a_a478_4867_7021, true));
        // SDM volume 1 §8.1.5.2: PC governs add, subtract, multiply, divide
        // and square root only.
        assert_eq!(sin_one(cw::RESET & !cw::PC), sin_one(cw::RESET));
    }

    #[test]
    fn fpatan_and_fyl2x_pop_and_report_their_special_cases() {
        // atan2(1, 1) = pi/4, rounded to nearest (up), with one pop.
        let pc = x87pc();
        let mut code = prologue(&pc, cw::RESET);
        code.extend_from_slice(&[0xd9, 0xe8, 0xd9, 0xe8, 0xd9, 0xf3, 0xf4]);
        run(&pc, &code);
        let x = pc.cpu.x87();
        assert_eq!(x.top(), 7);
        assert_eq!(x.raw(0), F80::new(0x3ffe, 0xc90f_daa2_2168_c235));
        assert_ne!(x.status & sw::C1, 0);

        // FYL2X of a negative argument is invalid, delivering the indefinite.
        let pc = x87pc();
        let mut code = prologue(&pc, cw::RESET);
        code.extend_from_slice(&[0xd9, 0xe8, 0xd9, 0xe8, 0xd9, 0xe0, 0xd9, 0xf1, 0xf4]);
        run(&pc, &code);
        let x = pc.cpu.x87();
        assert_eq!(x.raw(0), F80::INDEFINITE);
        assert_ne!(x.status & sw::IE, 0);
        assert_eq!(
            x.status & sw::SF,
            0,
            "an arithmetic invalid, not a stack fault"
        );

        // And of zero, a division by zero: 1 * log2(0) = -inf.
        let pc = x87pc();
        let mut code = prologue(&pc, cw::RESET);
        code.extend_from_slice(&[0xd9, 0xe8, 0xd9, 0xee, 0xd9, 0xf1, 0xf4]);
        run(&pc, &code);
        let x = pc.cpu.x87();
        assert_eq!(x.raw(0), F80::new(0xffff, 1 << 63), "-inf");
        assert_ne!(x.status & sw::ZE, 0);
    }

    #[test]
    fn the_transcendentals_fault_on_an_empty_or_a_full_stack() {
        // FSIN of an empty register: underflow, C1 clear, the indefinite.
        let pc = x87pc();
        let mut code = prologue(&pc, cw::RESET);
        code.extend_from_slice(&[0xd9, 0xfe, 0xf4]);
        run(&pc, &code);
        let x = pc.cpu.x87();
        assert_ne!(x.status & sw::SF, 0);
        assert_eq!(x.status & sw::C1, 0);
        assert_eq!(x.raw(0), F80::INDEFINITE);

        // FSINCOS with all eight full: overflow, C1 set, both results the
        // indefinite.
        let pc = x87pc();
        let mut code = prologue(&pc, cw::RESET);
        for _ in 0..8 {
            code.extend_from_slice(&[0xd9, 0xe8]);
        }
        code.extend_from_slice(&[0xd9, 0xfb, 0xf4]);
        run(&pc, &code);
        let x = pc.cpu.x87();
        assert_ne!(x.status & sw::SF, 0);
        assert_ne!(x.status & sw::C1, 0);
        assert_eq!(x.raw(0), F80::INDEFINITE);
        assert_eq!(x.raw(1), F80::INDEFINITE);

        // FPATAN with only one operand.
        let pc = x87pc();
        let mut code = prologue(&pc, cw::RESET);
        code.extend_from_slice(&[0xd9, 0xe8, 0xd9, 0xf3, 0xf4]);
        run(&pc, &code);
        let x = pc.cpu.x87();
        assert_ne!(x.status & sw::SF, 0);
        assert_eq!(x.top(), 0, "popped");
    }

    #[test]
    fn an_unmasked_precision_exception_leaves_the_operand_in_place() {
        let pc = x87pc();
        let mut code = prologue(&pc, cw::RESET & !cw::PM);
        code.extend_from_slice(&[0xd9, 0xe8, 0xd9, 0xfe, 0xf4]); // fld1 ; fsin
        run(&pc, &code);
        let x = pc.cpu.x87();
        assert_eq!(x.raw(0), F80::new(0x3fff, 1 << 63), "still 1.0");
        assert_ne!(x.status & sw::PE, 0);
        assert_ne!(x.status & sw::ES, 0, "pending for the next instruction");
    }

    /// Ten packed-decimal bytes at `addr`.
    fn write_bcd(pc: &Pc, addr: u64, bytes: [u8; 10]) {
        for (i, &b) in bytes.iter().enumerate() {
            pc.ram.write_u8(addr + i as u64, b).unwrap();
        }
    }

    fn read_bcd(pc: &Pc, addr: u64) -> [u8; 10] {
        let mut b = [0u8; 10];
        for (i, byte) in b.iter_mut().enumerate() {
            *byte = pc.ram.read_u8(addr + i as u64).unwrap();
        }
        b
    }

    #[test]
    fn fbld_fbstp_round_trip_and_fbstp_of_nan_stores_the_indefinite() {
        // -123456789012345678: digits low byte first, sign in byte 9.
        let bcd = [0x78, 0x56, 0x34, 0x12, 0x90, 0x78, 0x56, 0x34, 0x12, 0x80];
        let pc = x87pc();
        write_bcd(&pc, fpat::DATA, bcd);
        let mut code = prologue(&pc, cw::RESET);
        code.push(0xdf); // fbld
        code.extend_from_slice(&disp32(4, fpat::DATA));
        code.extend_from_slice(&[0xd9, 0xc0, 0xdf]); // fld st(0) ; fbstp
        code.extend_from_slice(&disp32(6, fpat::OUT));
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(read_bcd(&pc, fpat::OUT), bcd, "the digits come back");
        let x = pc.cpu.x87();
        assert_eq!(x.top(), 7, "fbstp popped its copy");
        let (want, _) =
            crate::float::x87::from_signed(-123_456_789_012_345_678, 64, crate::float::Env::X87);
        assert_eq!(x.raw(0), want, "and the value in between was exact");
        assert_eq!(x.status & sw::EXCEPTIONS, 0);

        // 2.5 rounds to nearest even, inexactly.
        let pc = x87pc();
        let mut code = prologue(&pc, cw::RESET);
        fld64(&pc, &mut code, fpat::DATA, 0x4004_0000_0000_0000);
        code.push(0xdf);
        code.extend_from_slice(&disp32(6, fpat::OUT));
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(read_bcd(&pc, fpat::OUT), [2, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        assert_ne!(pc.cpu.x87().status & sw::PE, 0);

        // A NaN, and 10^18, are invalid: the decimal indefinite.
        for bits in [0x7ff8_0000_0000_0000, 0x43ab_c16d_674e_c800] {
            let pc = x87pc();
            let mut code = prologue(&pc, cw::RESET);
            fld64(&pc, &mut code, fpat::DATA, bits);
            code.push(0xdf);
            code.extend_from_slice(&disp32(6, fpat::OUT));
            code.push(0xf4);
            run(&pc, &code);
            assert_eq!(
                read_bcd(&pc, fpat::OUT),
                crate::float::x87::BCD_INDEFINITE,
                "{bits:#x}"
            );
            assert_ne!(pc.cpu.x87().status & sw::IE, 0);
        }
    }

    /// A long-mode-class part with `Features::sse3` on, in protected mode
    /// with `CR4.OSFXSR` set.
    fn sse3pc() -> Pc {
        let pc = Pc::with_features(
            Variant::X86_64,
            Features {
                sse3: true,
                ..Features::X86_64
            },
        );
        pc.start_protected();
        let mut sys = pc.cpu.sys();
        sys.cr4 |= cr4::OSFXSR | cr4::OSXMMEXCPT;
        pc.cpu.set_sys(sys);
        pc
    }

    #[test]
    fn fisttp_truncates_regardless_of_rounding_control() {
        // -2.7 and 2.7 at all three widths, with RC set to round up: the
        // store truncates anyway, and pops.
        for (op, reg, width) in [(0xdfu8, 1u8, 2u64), (0xdb, 1, 4), (0xdd, 1, 8)] {
            for (bits, want) in [
                (0xc005_9999_9999_999au64, -2i64),
                (0x4005_9999_9999_999a, 2),
            ] {
                let pc = sse3pc();
                write64(&pc, fpat::OUT, 0x5555_5555_5555_5555);
                let up = cw::RESET & !cw::RC | (2 << cw::RC_SHIFT);
                let mut code = prologue(&pc, up);
                fld64(&pc, &mut code, fpat::DATA, bits);
                code.push(op);
                code.extend_from_slice(&disp32(reg, fpat::OUT));
                code.push(0xf4);
                run(&pc, &code);
                let mask = if width == 8 {
                    u64::MAX
                } else {
                    (1 << (width * 8)) - 1
                };
                let got = read64(&pc, fpat::OUT);
                assert_eq!(got & mask, want as u64 & mask, "{op:#x} of {bits:#x}");
                assert_eq!(got & !mask, 0x5555_5555_5555_5555 & !mask, "no wider");
                let x = pc.cpu.x87();
                assert_eq!(x.top(), 0, "popped");
                assert_ne!(x.status & sw::PE, 0);
            }
        }
        // Out of range is the integer indefinite, as for FISTP.
        let pc = sse3pc();
        let mut code = prologue(&pc, cw::RESET);
        fld64(&pc, &mut code, fpat::DATA, 0x40f0_0000_0000_0000); // 65536
        code.push(0xdf);
        code.extend_from_slice(&disp32(1, fpat::OUT));
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(read64(&pc, fpat::OUT) as u16, 0x8000);
        assert_ne!(pc.cpu.x87().status & sw::IE, 0);
    }

    #[test]
    fn fisttp_is_undefined_without_sse3_and_cpuid_reports_sse3_with_it() {
        // A 486 has no SSE3: `DB /1` raises #UD, and before `CR0.TS` or a
        // pending exception would have been looked at.
        let pc = x87pc();
        pc.idt(6, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
        pc.write(HANDLER, &[0xf4]);
        let mut code = alloc::vec![0xdb, 0xe3, 0xd9, 0xe8, 0xdb];
        code.extend_from_slice(&disp32(1, fpat::OUT));
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(pc.regs().rip, HANDLED, "#UD for FISTTP");

        // With the flag on, CPUID claims the extension, which is all here
        // but MONITOR/MWAIT — and those have their own bit, which stays
        // clear.
        let pc = sse3pc();
        run(&pc, &[0xb8, 0x01, 0x00, 0x00, 0x00, 0x0f, 0xa2, 0xf4]);
        assert_eq!(pc.regs().rcx & 1, 1, "CPUID.1:ECX.SSE3");
        assert_eq!(pc.regs().rcx & (1 << 3), 0, "CPUID.1:ECX.MONITOR");

        // An SSE3 flag needs SSE2 under it.
        let impossible = Features {
            sse3: true,
            ..Features::I80486
        };
        assert!(impossible.validate().is_err());
    }

    /// Store sixteen bytes as two little-endian quadwords.
    fn write128(pc: &Pc, addr: u64, v: [u64; 2]) {
        write64(pc, addr, v[0]);
        write64(pc, addr + 8, v[1]);
    }

    fn read128(pc: &Pc, addr: u64) -> [u64; 2] {
        [read64(pc, addr), read64(pc, addr + 8)]
    }

    /// `movdqu xmm<reg>, [addr]`.
    fn load_xmm(code: &mut Vec<u8>, reg: u8, addr: u64) {
        code.extend_from_slice(&[0xf3, 0x0f, 0x6f]);
        code.extend_from_slice(&disp32(reg, addr));
    }

    /// `movdqu [addr], xmm<reg>`.
    fn store_xmm(code: &mut Vec<u8>, reg: u8, addr: u64) {
        code.extend_from_slice(&[0xf3, 0x0f, 0x7f]);
        code.extend_from_slice(&disp32(reg, addr));
    }

    #[test]
    fn haddps_and_hsubpd_reach_memory_and_mxcsr() {
        // haddps xmm0, [DATA+16] with xmm0 = [1, 2, 3, 4] and the memory
        // [10, 20, 30, 40]: [3, 7, 30, 70] (*Intel SDM* volume 2, `HADDPS`).
        let singles = |l: [u32; 4]| {
            [
                u64::from(l[0]) | (u64::from(l[1]) << 32),
                u64::from(l[2]) | (u64::from(l[3]) << 32),
            ]
        };
        let pc = sse3pc();
        write128(
            &pc,
            fpat::DATA,
            singles([0x3f80_0000, 0x4000_0000, 0x4040_0000, 0x4080_0000]),
        );
        write128(
            &pc,
            fpat::DATA + 16,
            singles([0x4120_0000, 0x41a0_0000, 0x41f0_0000, 0x4220_0000]),
        );
        let mut code = Vec::new();
        load_xmm(&mut code, 0, fpat::DATA);
        code.extend_from_slice(&[0xf2, 0x0f, 0x7c]);
        code.extend_from_slice(&disp32(0, fpat::DATA + 16));
        store_xmm(&mut code, 0, fpat::OUT);
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(
            read128(&pc, fpat::OUT),
            singles([0x4040_0000, 0x40e0_0000, 0x41f0_0000, 0x428c_0000])
        );

        // hsubpd xmm0, xmm1 rounds under MXCSR: 1 - 2^-60 is 1 to nearest
        // and the double below 1 toward negative infinity.
        for (rc, want) in [(0u32, 0x3ff0_0000_0000_0000u64), (1, 0x3fef_ffff_ffff_ffff)] {
            let pc = sse3pc();
            write64(
                &pc,
                fpat::SAVE,
                u64::from(mxcsr::RESET | (rc << mxcsr::RC_SHIFT)),
            );
            write128(
                &pc,
                fpat::DATA,
                [0x3ff0_0000_0000_0000, 0x3c30_0000_0000_0000],
            );
            let mut code = alloc::vec![0x0f, 0xae];
            code.extend_from_slice(&disp32(2, fpat::SAVE)); // ldmxcsr
            load_xmm(&mut code, 0, fpat::DATA);
            code.extend_from_slice(&[0x66, 0x0f, 0x7d, 0xc0]); // hsubpd xmm0, xmm0
            store_xmm(&mut code, 0, fpat::OUT);
            code.push(0xf4);
            run(&pc, &code);
            assert_eq!(read128(&pc, fpat::OUT), [want, want], "RC {rc}");
            assert_ne!(pc.cpu.sse().mxcsr & mxcsr::PE, 0, "inexact");
        }
    }

    #[test]
    fn the_duplicating_loads_copy_the_lanes_the_manual_names() {
        let pc = sse3pc();
        write128(
            &pc,
            fpat::DATA,
            [0x2222_2222_1111_1111, 0x4444_4444_3333_3333],
        );
        let mut code = Vec::new();
        // movddup xmm1, [DATA] ; movsldup xmm2, [DATA] ; movshdup xmm3, [DATA]
        code.extend_from_slice(&[0xf2, 0x0f, 0x12]);
        code.extend_from_slice(&disp32(1, fpat::DATA));
        code.extend_from_slice(&[0xf3, 0x0f, 0x12]);
        code.extend_from_slice(&disp32(2, fpat::DATA));
        code.extend_from_slice(&[0xf3, 0x0f, 0x16]);
        code.extend_from_slice(&disp32(3, fpat::DATA));
        // The register forms are the same instructions — not MOVHLPS:
        // load xmm4, then movddup xmm5, xmm4.
        load_xmm(&mut code, 4, fpat::DATA);
        code.extend_from_slice(&[0xf2, 0x0f, 0x12, 0xec]);
        code.push(0xf4);
        run(&pc, &code);
        let sse = pc.cpu.sse();
        let lo = 0x2222_2222_1111_1111;
        assert_eq!(sse.get(1), [lo, lo]);
        assert_eq!(sse.get(2), [0x1111_1111_1111_1111, 0x3333_3333_3333_3333]);
        assert_eq!(sse.get(3), [0x2222_2222_2222_2222, 0x4444_4444_4444_4444]);
        assert_eq!(sse.get(5), [lo, lo]);
    }

    #[test]
    fn lddqu_loads_misaligned_memory_and_has_no_register_form() {
        let pc = sse3pc();
        write128(
            &pc,
            fpat::ODD,
            [0x0706_0504_0302_0100, 0x0f0e_0d0c_0b0a_0908],
        );
        let mut code = alloc::vec![0xf2, 0x0f, 0xf0];
        code.extend_from_slice(&disp32(6, fpat::ODD));
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(
            pc.cpu.sse().get(6),
            [0x0706_0504_0302_0100, 0x0f0e_0d0c_0b0a_0908]
        );

        let pc = sse3pc();
        pc.idt(6, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
        pc.write(HANDLER, &[0xf4]);
        run(&pc, &[0xf2, 0x0f, 0xf0, 0xc1, 0xf4]); // lddqu xmm0, xmm1
        assert_eq!(pc.regs().rip, HANDLED, "#UD for a register LDDQU");
    }

    #[test]
    fn sse3_is_undefined_on_a_part_without_it() {
        // The x86-64 baseline has SSE2 and not SSE3.
        for code in [
            &[0xf2u8, 0x0f, 0x7c, 0xc1, 0xf4][..], // haddps xmm0, xmm1
            &[0xf2, 0x0f, 0x12, 0xc1, 0xf4],       // movddup xmm0, xmm1
            &[0x66, 0x0f, 0xd0, 0xc1, 0xf4],       // addsubpd xmm0, xmm1
        ] {
            let pc = ssepc();
            pc.idt(6, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
            pc.write(HANDLER, &[0xf4]);
            run(&pc, code);
            assert_eq!(pc.regs().rip, HANDLED, "#UD for {code:02x?}");
        }
        // And CPUID says so.
        let pc = ssepc();
        run(&pc, &[0xb8, 0x01, 0x00, 0x00, 0x00, 0x0f, 0xa2, 0xf4]);
        assert_eq!(pc.regs().rcx & 1, 0);
    }

    #[test]
    fn the_disassembler_names_the_new_escapes() {
        let pc = sse3pc();
        let listing = |bytes: &[u8]| {
            pc.write(at::CODE0, bytes);
            let out = pc.cpu.disassemble(0x08, at::CODE0, 1);
            alloc::format!("{}", out[0])
        };
        for (bytes, name) in [
            (&[0xd9u8, 0xf0][..], "f2xm1"),
            (&[0xd9, 0xf1], "fyl2x"),
            (&[0xd9, 0xf2], "fptan"),
            (&[0xd9, 0xf3], "fpatan"),
            (&[0xd9, 0xf9], "fyl2xp1"),
            (&[0xd9, 0xfb], "fsincos"),
            (&[0xd9, 0xfe], "fsin"),
            (&[0xd9, 0xff], "fcos"),
        ] {
            assert!(listing(bytes).ends_with(name), "{name}");
        }
        let fbld = listing(&[0xdf, 0x25, 0x00, 0xb0, 0x00, 0x00]);
        assert!(fbld.ends_with("fbld tbyte [ds:0xb000]"), "{fbld}");
        let fisttp = listing(&[0xdb, 0x0d, 0x00, 0xb0, 0x00, 0x00]);
        assert!(fisttp.ends_with("fisttp dword [ds:0xb000]"), "{fisttp}");
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
            (23, "MMX"),
            (24, "FXSR"),
            (25, "SSE"),
            (26, "SSE2"),
        ] {
            assert_ne!(edx & (1u32 << bit), 0, "leaf 1 should report {name}");
        }
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
            ("mmx.rs", include_str!("mmx.rs")),
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

    // -- SSE2's packed-integer half --------------------------------------
    //
    // Every expected value below is arithmetic that can be checked by hand, or
    // a lane order taken from the instruction's page in *Intel SDM* volume 2.
    // What made these worth writing is the reason they exist at all: `CPUID`
    // leaf 1 has claimed `SSE2` since the feature bit landed, and until now
    // `66 0F 60`-`76` and `66 0F D4`-`FE` raised `#UD` — so a statically linked
    // glibc binary, whose startup runs `PUNPCKLDQ` inside the first hundred
    // instructions, took an invalid-opcode exception it had no reason to
    // expect.

    /// Load `xmm0` and `xmm1`, run one two-byte instruction, and read `xmm0`.
    fn packed2(prefix: u8, opcode: u8, a: [u64; 2], b: [u64; 2]) -> [u64; 2] {
        let pc = ssepc();
        let mut sse = Sse::new();
        sse.set(0, a);
        sse.set(1, b);
        pc.cpu.set_sse(sse);
        run(&pc, &[prefix, 0x0f, opcode, 0xc1, 0xf4]);
        pc.cpu.sse().get(0)
    }

    /// The same, with an immediate third operand.
    fn packed3(prefix: u8, opcode: u8, a: [u64; 2], b: [u64; 2], imm: u8) -> [u64; 2] {
        let pc = ssepc();
        let mut sse = Sse::new();
        sse.set(0, a);
        sse.set(1, b);
        pc.cpu.set_sse(sse);
        run(&pc, &[prefix, 0x0f, opcode, 0xc1, imm, 0xf4]);
        pc.cpu.sse().get(0)
    }

    #[test]
    fn punpckldq_is_the_instruction_a_static_glibc_start_up_runs() {
        // `66 0F 62`: the low doubleword of each operand, then the second, in
        // destination-then-source order. This is the encoding that stopped a
        // real guest, so it gets its own test rather than a row in a table.
        assert_eq!(
            packed2(
                0x66,
                0x62,
                [0x1111_1111_0000_0000, 0x3333_3333_2222_2222],
                [0xbbbb_bbbb_aaaa_aaaa, 0xdddd_dddd_cccc_cccc],
            ),
            [0xaaaa_aaaa_0000_0000, 0xbbbb_bbbb_1111_1111],
        );
    }

    #[test]
    fn the_packed_adds_and_subtracts_wrap_or_saturate_as_the_mnemonic_says() {
        let ones = [0x0101_0101_0101_0101u64; 2];
        // `PADDB` wraps: 0xff + 1 is 0x00.
        assert_eq!(
            packed2(0x66, 0xfc, [u64::MAX; 2], ones),
            [0; 2],
            "paddb wraps"
        );
        // `PADDUSB` clamps at 0xff instead.
        assert_eq!(
            packed2(0x66, 0xdc, [u64::MAX; 2], ones),
            [u64::MAX; 2],
            "paddusb saturates"
        );
        // `PADDSB` clamps at the *signed* limits: 0x7f + 1 is 0x7f.
        assert_eq!(
            packed2(0x66, 0xec, [0x7f7f_7f7f_7f7f_7f7f; 2], ones),
            [0x7f7f_7f7f_7f7f_7f7f; 2],
            "paddsb saturates"
        );
        // `PSUBUSB` clamps at zero: 0 - 1 is 0, not 0xff.
        assert_eq!(packed2(0x66, 0xd8, [0; 2], ones), [0; 2], "psubusb");
        assert_eq!(
            packed2(0x66, 0xf8, [0; 2], ones),
            [u64::MAX; 2],
            "psubb wraps"
        );
        // The wide lanes: 64-bit `PADDQ` carries across the whole quadword.
        assert_eq!(
            packed2(0x66, 0xd4, [u64::MAX, 0], [1, 1]),
            [0, 1],
            "paddq does not carry between lanes"
        );
    }

    #[test]
    fn the_packed_compares_write_a_lane_of_ones_or_a_lane_of_zeros() {
        // `PCMPEQB`, the instruction every SSE2 `strlen` is built on.
        assert_eq!(
            packed2(
                0x66,
                0x74,
                [0x0000_0000_0000_4142, 0],
                [0x0000_0000_0000_4142, 0]
            ),
            [0xffff_ffff_ffff_ffff, 0xffff_ffff_ffff_ffff],
            "every byte matched, including the fourteen zeros"
        );
        // `PCMPGTB` is *signed*: 0xff is -1 and is not greater than 0.
        assert_eq!(
            packed2(0x66, 0x64, [u64::MAX; 2], [0; 2]),
            [0; 2],
            "-1 is not greater than 0"
        );
        assert_eq!(
            packed2(0x66, 0x64, [0x0101_0101_0101_0101; 2], [0; 2]),
            [u64::MAX; 2],
            "but 1 is"
        );
    }

    #[test]
    fn pmovmskb_gathers_sixteen_sign_bits_and_pcmpeqb_feeds_it() {
        let pc = ssepc();
        let mut sse = Sse::new();
        // Bytes 0 and 15 have their top bit set; nothing else does.
        sse.set(1, [0x0000_0000_0000_0080, 0x8000_0000_0000_0000]);
        pc.cpu.set_sse(sse);
        run(&pc, &[0x66, 0x0f, 0xd7, 0xc1, 0xf4]); // pmovmskb eax, xmm1
        assert_eq!(pc.regs().rax, 0b1000_0000_0000_0001);
    }

    #[test]
    fn the_shift_groups_take_their_count_from_the_immediate_and_the_register() {
        // `66 0F 71 /2 ib` — group 12, extension 2: `PSRLW xmm1, 4`.
        let pc = ssepc();
        let mut sse = Sse::new();
        sse.set(1, [0xf000_0f00_00f0_000f; 2]);
        pc.cpu.set_sse(sse);
        run(&pc, &[0x66, 0x0f, 0x71, 0xd1, 0x04, 0xf4]);
        assert_eq!(pc.cpu.sse().get(1), [0x0f00_00f0_000f_0000; 2]);

        // And the register form, `66 0F D1`, which takes the whole low
        // quadword of the source as the count.
        assert_eq!(
            packed2(0x66, 0xd1, [0xf000_0f00_00f0_000f; 2], [4, 0]),
            [0x0f00_00f0_000f_0000; 2],
        );
        // A count past the lane width clears the lane rather than wrapping
        // into a shift of `count % 16`.
        assert_eq!(
            packed2(0x66, 0xd1, [u64::MAX; 2], [16, 0]),
            [0; 2],
            "a count of 16 clears a word lane"
        );
        assert_eq!(
            packed2(0x66, 0xd1, [u64::MAX; 2], [0x1_0000_0000, 0]),
            [0; 2],
            "and so does one that does not fit in five bits"
        );
        // An *arithmetic* shift saturates its count instead: the lane fills
        // with sign bits.
        assert_eq!(
            packed2(0x66, 0xe1, [0x8000_8000_8000_8000; 2], [99, 0]),
            [u64::MAX; 2],
            "psraw by 99 is psraw by 15"
        );
    }

    #[test]
    fn the_byte_shifts_move_the_whole_register_and_have_no_mmx_form() {
        // `66 0F 73 /3 ib` is `PSRLDQ xmm1, 1`; `/7` is `PSLLDQ`.
        let shift = |ext: u8, imm: u8| {
            let pc = ssepc();
            let mut sse = Sse::new();
            sse.set(1, [0x0807_0605_0403_0201, 0x100f_0e0d_0c0b_0a09]);
            pc.cpu.set_sse(sse);
            run(&pc, &[0x66, 0x0f, 0x73, 0xc1 | (ext << 3), imm, 0xf4]);
            pc.cpu.sse().get(1)
        };
        assert_eq!(
            shift(3, 1),
            [0x0908_0706_0504_0302, 0x0010_0f0e_0d0c_0b0a],
            "psrldq by one byte crosses the quadword boundary"
        );
        assert_eq!(
            shift(7, 1),
            [0x0706_0504_0302_0100, 0x0f0e_0d0c_0b0a_0908],
            "and so does pslldq"
        );
        assert_eq!(shift(3, 16), [0; 2], "a count of sixteen clears it");
        assert_eq!(shift(3, 255), [0; 2], "and so does any larger one");
    }

    #[test]
    fn the_shift_groups_have_no_memory_form_at_all() {
        // `mod != 11` names an address, and there is nothing at it to shift:
        // the operand *is* the register the `r/m` field selects. Hardware
        // raises `#UD`, and a decoder that resolved the group on the `reg`
        // field alone would happily execute this.
        let pc = ssepc();
        pc.idt(6, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
        pc.write(HANDLER, &[0xf4]);
        let mut code = alloc::vec![0x66, 0x0f, 0x71];
        code.extend_from_slice(&disp32(2, fpat::DATA));
        code.push(0x04);
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(pc.regs().rip, HANDLED);
    }

    #[test]
    fn the_packing_instructions_saturate_and_the_destination_supplies_the_low_half() {
        // `PACKSSWB`: eight words from the destination into bytes 0-7 and
        // eight from the source into bytes 8-15, each clamped to a signed
        // byte. 0x0100 is 256, which clamps to 127; 0xff00 is -256, to -128.
        assert_eq!(
            packed2(
                0x66,
                0x63,
                [0x0100_0100_0100_0100; 2],
                [0xff00_ff00_ff00_ff00; 2]
            ),
            [0x7f7f_7f7f_7f7f_7f7f, 0x8080_8080_8080_8080],
        );
        // `PACKUSWB` reads the same signed words and clamps them to *unsigned*
        // bytes, so the negative half becomes zero rather than 0x80.
        assert_eq!(
            packed2(
                0x66,
                0x67,
                [0x0100_0100_0100_0100; 2],
                [0xff00_ff00_ff00_ff00; 2]
            ),
            [0xffff_ffff_ffff_ffff, 0],
        );
    }

    #[test]
    fn the_multiplies_split_the_product_the_way_their_names_say() {
        // 0x4000 * 0x0004 = 0x1_0000: the low half is zero and the high half
        // is one, so the pair of instructions recovers the whole product.
        let a = [0x4000_4000_4000_4000u64; 2];
        let b = [0x0004_0004_0004_0004u64; 2];
        assert_eq!(packed2(0x66, 0xd5, a, b), [0; 2], "pmullw: the low halves");
        assert_eq!(
            packed2(0x66, 0xe5, a, b),
            [0x0001_0001_0001_0001; 2],
            "pmulhw: the high halves"
        );
        // `PMULHW` is signed and `PMULHUW` is not: -1 * -1 is 1, whose high
        // half is zero, while 0xffff * 0xffff is 0xfffe0001.
        assert_eq!(packed2(0x66, 0xe5, [u64::MAX; 2], [u64::MAX; 2]), [0; 2]);
        assert_eq!(
            packed2(0x66, 0xe4, [u64::MAX; 2], [u64::MAX; 2]),
            [0xfffe_fffe_fffe_fffe; 2],
        );
        // `PMULUDQ` reads only the even doublewords and widens.
        assert_eq!(
            packed2(
                0x66,
                0xf4,
                [0xdead_beef_0000_0003, 0xdead_beef_0000_0005],
                [0xdead_beef_0000_0007, 0xdead_beef_0000_000b],
            ),
            [21, 55],
        );
        // `PMADDWD`: two signed products summed into each doubleword.
        assert_eq!(
            packed2(
                0x66,
                0xf5,
                [0x0002_0003_0004_0005; 2],
                [0x0006_0007_0008_0009; 2]
            ),
            [0x0000_0021_0000_004d; 2],
            "5*9+4*8 = 77, and 3*7+2*6 = 33"
        );
    }

    #[test]
    fn psadbw_sums_eight_absolute_differences_into_each_halfs_low_word() {
        assert_eq!(
            packed2(0x66, 0xf6, [0; 2], [0x0101_0101_0101_0101; 2]),
            [8, 8],
        );
        // The order of the operands does not matter, because it is an
        // *absolute* difference.
        assert_eq!(
            packed2(0x66, 0xf6, [0x0101_0101_0101_0101; 2], [0; 2]),
            [8, 8],
        );
    }

    #[test]
    fn pshufd_permutes_four_doublewords_and_the_two_word_shuffles_copy_the_half_they_do_not_name() {
        let src = [0x1111_1111_0000_0000u64, 0x3333_3333_2222_2222];
        // 0b00_01_10_11: reverse the four doublewords.
        assert_eq!(
            packed3(0x66, 0x70, [0; 2], src, 0b00_01_10_11),
            [0x2222_2222_3333_3333, 0x0000_0000_1111_1111],
        );
        // `F2 0F 70` shuffles the low four words and copies the high quadword.
        let words = [0x3333_2222_1111_0000u64, 0x7777_6666_5555_4444];
        assert_eq!(
            packed3(0xf2, 0x70, [0; 2], words, 0b00_01_10_11),
            [0x0000_1111_2222_3333, 0x7777_6666_5555_4444],
        );
        // `F3 0F 70` does the other half, and copies the low quadword.
        assert_eq!(
            packed3(0xf3, 0x70, [0; 2], words, 0b00_01_10_11),
            [0x3333_2222_1111_0000, 0x4444_5555_6666_7777],
        );
    }

    #[test]
    fn pinsrw_and_pextrw_move_one_word_between_the_two_register_files() {
        let pc = ssepc();
        let mut sse = Sse::new();
        sse.set(0, [0; 2]);
        pc.cpu.set_sse(sse);
        // mov eax, 0xbeef ; pinsrw xmm0, eax, 5 ; pextrw ebx, xmm0, 5
        let mut code = alloc::vec![0xb8, 0xef, 0xbe, 0x00, 0x00];
        code.extend_from_slice(&[0x66, 0x0f, 0xc4, 0xc0, 0x05]);
        code.extend_from_slice(&[0x66, 0x0f, 0xc5, 0xd8, 0x05]);
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(pc.cpu.sse().get(0), [0, 0x0000_0000_beef_0000]);
        assert_eq!(pc.regs().rbx, 0xbeef, "and zero-extended on the way back");
    }

    #[test]
    fn the_interleaves_take_alternate_lanes_starting_with_the_destination() {
        let a = [0x0706_0504_0302_0100u64, 0x0f0e_0d0c_0b0a_0908];
        let b = [0x1716_1514_1312_1110u64, 0x1f1e_1d1c_1b1a_1918];
        // `PUNPCKLBW`: d0 s0 d1 s1 …
        assert_eq!(
            packed2(0x66, 0x60, a, b),
            [0x1303_1202_1101_1000, 0x1707_1606_1505_1404],
        );
        // `PUNPCKHBW` reads the high half of each.
        assert_eq!(
            packed2(0x66, 0x68, a, b),
            [0x1b0b_1a0a_1909_1808, 0x1f0f_1e0e_1d0d_1c0c],
        );
        // And the quadword forms, which are just a selection.
        assert_eq!(packed2(0x66, 0x6c, a, b), [a[0], b[0]]);
        assert_eq!(packed2(0x66, 0x6d, a, b), [a[1], b[1]]);
    }

    #[test]
    fn the_min_max_and_average_forms_are_the_four_sse2_has_and_not_six_more() {
        assert_eq!(
            packed2(
                0x66,
                0xda,
                [0x00ff_00ff_00ff_00ff; 2],
                [0x0f0f_0f0f_0f0f_0f0f; 2]
            ),
            [0x000f_000f_000f_000f; 2],
            "pminub is unsigned"
        );
        assert_eq!(
            packed2(0x66, 0xea, [u64::MAX; 2], [0; 2]),
            [u64::MAX; 2],
            "pminsw is signed, and -1 is less than 0"
        );
        // `PAVGB` rounds up: (0 + 1 + 1) / 2 is 1.
        assert_eq!(
            packed2(0x66, 0xe0, [0; 2], [0x0101_0101_0101_0101; 2]),
            [0x0101_0101_0101_0101; 2],
        );
        // And it does so at nine bits, so 0xff and 0xff average to 0xff
        // rather than overflowing to zero.
        assert_eq!(
            packed2(0x66, 0xe0, [u64::MAX; 2], [u64::MAX; 2]),
            [u64::MAX; 2]
        );
    }

    #[test]
    fn movntdq_is_an_aligned_store_and_has_no_register_form() {
        let pc = ssepc();
        let mut sse = Sse::new();
        sse.set(0, [0x0123_4567_89ab_cdef, 0xfedc_ba98_7654_3210]);
        pc.cpu.set_sse(sse);
        let mut code = alloc::vec![0x66, 0x0f, 0xe7];
        code.extend_from_slice(&disp32(0, fpat::DATA));
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(read64(&pc, fpat::DATA), 0x0123_4567_89ab_cdef);
        assert_eq!(read64(&pc, fpat::DATA + 8), 0xfedc_ba98_7654_3210);

        // Misaligned is `#GP(0)`, exactly as `MOVDQA` is: the aligned form
        // exists so that the check is a checked assertion.
        let pc = ssepc();
        pc.idt(13, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
        pc.write(HANDLER, &[0xf4]);
        let mut code = alloc::vec![0x66, 0x0f, 0xe7];
        code.extend_from_slice(&disp32(0, fpat::ODD));
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(pc.regs().rip, HANDLED);
    }

    #[test]
    fn a_part_without_sse2_refuses_the_packed_integer_half() {
        // `ROADMAP.md` §6.1.1: an instruction that is absent must trap as
        // undefined. `CPUID` on this part does not set the `SSE2` bit, and a
        // guest that probes by executing the encoding has to find that out.
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
        let mut sys = pc.cpu.sys();
        sys.cr4 |= cr4::OSFXSR | cr4::OSXMMEXCPT;
        pc.cpu.set_sys(sys);
        pc.idt(6, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
        pc.write(HANDLER, &[0xf4]);
        run(&pc, &[0x66, 0x0f, 0x62, 0xc1, 0xf4]); // punpckldq xmm0, xmm1
        assert_eq!(pc.regs().rip, HANDLED, "#UD, not an execution");
    }

    #[test]
    fn the_disassembler_names_every_packed_integer_encoding_it_decodes() {
        // The generator emits one description for decode, execution and
        // disassembly (CLAUDE.md, "CPU cores"), so this is the check that the
        // rows added to the table are reachable from all three.
        let pc = ssepc();
        let listing = |bytes: &[u8]| {
            pc.write(at::CODE0, bytes);
            let out = pc.cpu.disassemble(0x08, at::CODE0, 1);
            alloc::format!("{}", out[0])
        };
        assert!(listing(&[0x66, 0x0f, 0x62, 0xc1]).ends_with("punpckldq xmm0, xmm1"));
        assert!(listing(&[0x66, 0x0f, 0xfe, 0xc1]).ends_with("paddd xmm0, xmm1"));
        assert!(listing(&[0x66, 0x0f, 0x71, 0xd1, 0x04]).ends_with("psrlw xmm1, 0x4"));
        assert!(listing(&[0x66, 0x0f, 0x73, 0xf9, 0x01]).ends_with("pslldq xmm1, 0x1"));
        assert!(listing(&[0x66, 0x0f, 0xd7, 0xc1]).ends_with("pmovmskb eax, xmm1"));
        assert!(listing(&[0xf3, 0x0f, 0x70, 0xc1, 0x1b]).ends_with("pshufhw xmm0, xmm1, 0x1b"));
    }

    // -- The thirteen that CPUID claimed and the decoder lacked ----------
    //
    // Issue #39: the non-temporal stores, `MASKMOVDQU`, the six packed
    // integer/floating-point conversions and the four reciprocal
    // approximations. Expected values are hand-checkable arithmetic or the
    // special cases on each instruction's page in *Intel SDM* volume 2.

    /// Four doubleword lanes as a register value, lane 0 lowest.
    fn lanes(l: [u32; 4]) -> [u64; 2] {
        [
            u64::from(l[0]) | (u64::from(l[1]) << 32),
            u64::from(l[2]) | (u64::from(l[3]) << 32),
        ]
    }

    /// Load `xmm0`, `xmm1` and `MXCSR`, run `code`, and hand the core back.
    fn with_sse(a: [u64; 2], b: [u64; 2], mxcsr: u32, code: &[u8]) -> Pc {
        let pc = ssepc();
        let mut sse = Sse::new();
        sse.set(0, a);
        sse.set(1, b);
        sse.mxcsr = mxcsr;
        pc.cpu.set_sse(sse);
        run(&pc, code);
        pc
    }

    /// `op xmm0, xmm1` under a mandatory prefix (`0` for none): `xmm0` and
    /// the exception flags afterwards.
    fn convert(prefix: u8, opcode: u8, a: [u64; 2], b: [u64; 2], mxcsr: u32) -> ([u64; 2], u32) {
        let mut code = Vec::new();
        if prefix != 0 {
            code.push(prefix);
        }
        code.extend_from_slice(&[0x0f, opcode, 0xc1, 0xf4]);
        let pc = with_sse(a, b, mxcsr, &code);
        let sse = pc.cpu.sse();
        (sse.get(0), sse.mxcsr & mxcsr::EXCEPTIONS)
    }

    #[test]
    fn cvttps2dq_truncates_and_returns_the_indefinite_on_overflow() {
        // [1.9, -1.9, 3e9, NaN]: two truncations, then a value past
        // `i32::MAX` and a NaN, both of which are the integer indefinite and
        // both of which are invalid.
        let src = lanes([0x3ff3_3333, 0xbff3_3333, 0x4f32_d05e, 0x7fc0_0000]);
        let (out, flags) = convert(0xf3, 0x5b, [0; 2], src, mxcsr::RESET);
        assert_eq!(out, lanes([1, (-1i32) as u32, 0x8000_0000, 0x8000_0000]));
        assert_eq!(flags, mxcsr::IE | mxcsr::PE);
    }

    #[test]
    fn cvtps2dq_follows_the_mxcsr_rounding_mode() {
        // [1.5, 2.5, -1.5, -0.5] under each of the four `RC` settings.
        let src = lanes([0x3fc0_0000, 0x4020_0000, 0xbfc0_0000, 0xbf00_0000]);
        let at = |rc: u32| {
            convert(
                0x66,
                0x5b,
                [0; 2],
                src,
                mxcsr::RESET | (rc << mxcsr::RC_SHIFT),
            )
        };
        let i = |v: i32| v as u32;
        assert_eq!(at(0).0, lanes([2, 2, i(-2), 0]), "to nearest, ties to even");
        assert_eq!(at(1).0, lanes([1, 2, i(-2), i(-1)]), "down");
        assert_eq!(at(2).0, lanes([2, 3, i(-1), 0]), "up");
        assert_eq!(at(3).0, lanes([1, 2, i(-1), 0]), "toward zero");
        assert_eq!(at(0).1, mxcsr::PE, "inexact, and nothing else");
        // An exact conversion raises nothing.
        let (out, flags) = convert(0x66, 0x5b, [0; 2], lanes([0x4000_0000; 4]), mxcsr::RESET);
        assert_eq!(out, lanes([2; 4]));
        assert_eq!(flags, 0);
    }

    #[test]
    fn cvtdq2ps_rounds_what_a_single_cannot_hold() {
        // 2^24 + 1 is the first integer a single cannot represent, and it is
        // a tie that goes to the even neighbour; `i32::MIN` is exact.
        let src = lanes([1, (-2i32) as u32, 16_777_217, 0x8000_0000]);
        let (out, flags) = convert(0, 0x5b, [0; 2], src, mxcsr::RESET);
        assert_eq!(
            out,
            lanes([0x3f80_0000, 0xc000_0000, 0x4b80_0000, 0xcf00_0000])
        );
        assert_eq!(flags, mxcsr::PE);
    }

    #[test]
    fn the_packed_double_conversions_zero_or_widen_as_their_pages_say() {
        let junk = [0xdead_beef_dead_beef, 0xfeed_face_feed_face];
        // `CVTPD2DQ`: [2.5, -3.7] to nearest is [2, -4], and the upper
        // quadword of the destination is **zeroed**, not kept.
        let pd = [0x4004_0000_0000_0000, 0xc00d_9999_9999_999a];
        let (out, flags) = convert(0xf2, 0xe6, junk, pd, mxcsr::RESET);
        assert_eq!(out, lanes([2, (-4i32) as u32, 0, 0]));
        assert_eq!(flags, mxcsr::PE);
        // `CVTTPD2DQ` truncates the same pair to [2, -3] and zeroes the same
        // half; 1e10 does not fit and is the indefinite.
        let (out, _) = convert(0x66, 0xe6, junk, pd, mxcsr::RESET);
        assert_eq!(out, lanes([2, (-3i32) as u32, 0, 0]));
        let (out, flags) = convert(0x66, 0xe6, junk, [0x4202_a05f_2000_0000, 0], mxcsr::RESET);
        assert_eq!(out, lanes([0x8000_0000, 0, 0, 0]));
        assert_eq!(flags, mxcsr::IE);
        // `CVTDQ2PD` reads the low two doublewords and widens them exactly.
        let src = lanes([(-7i32) as u32, 3, 0x1111_1111, 0x2222_2222]);
        let (out, flags) = convert(0xf3, 0xe6, junk, src, mxcsr::RESET);
        assert_eq!(out, [0xc01c_0000_0000_0000, 0x4008_0000_0000_0000]);
        assert_eq!(flags, 0);
    }

    #[test]
    fn an_unmasked_invalid_conversion_faults_and_writes_nothing() {
        // The scalar conversions' `#XM` rule, lane-wise: one bad lane is
        // enough, and the destination keeps every lane it had.
        let before = [0x0123_4567_89ab_cdef, 0x0fed_cba9_8765_4321];
        let src = lanes([0x3f80_0000, 0x7fc0_0000, 0x3f80_0000, 0x3f80_0000]);
        let pc = ssepc();
        pc.idt(19, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
        pc.write(HANDLER, &[0xf4]);
        let mut sse = Sse::new();
        sse.set(0, before);
        sse.set(1, src);
        sse.mxcsr = mxcsr::RESET & !mxcsr::IM;
        pc.cpu.set_sse(sse);
        run(&pc, &[0xf3, 0x0f, 0x5b, 0xc1, 0xf4]); // cvttps2dq xmm0, xmm1
        assert_eq!(pc.regs().rip, HANDLED, "#XM");
        assert_eq!(pc.cpu.sse().get(0), before);
        assert_ne!(pc.cpu.sse().mxcsr & mxcsr::IE, 0);
    }

    #[test]
    fn a_packed_conversion_from_memory_must_be_aligned_except_cvtdq2pd() {
        // `CVTDQ2PD` reads a quadword and has no alignment requirement; the
        // 128-bit sources do (*Intel SDM* volume 2, each page's exceptions).
        let pc = ssepc();
        pc.idt(13, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
        pc.write(HANDLER, &[0xf4]);
        let mut code = alloc::vec![0x0f, 0x5b]; // cvtdq2ps xmm0, [ODD]
        code.extend_from_slice(&disp32(0, fpat::ODD));
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(pc.regs().rip, HANDLED, "#GP on a misaligned CVTDQ2PS");

        let pc = ssepc();
        write64(
            &pc,
            fpat::ODD,
            u64::from(5u32) | (u64::from((-1i32) as u32) << 32),
        );
        let mut code = alloc::vec![0xf3, 0x0f, 0xe6]; // cvtdq2pd xmm0, [ODD]
        code.extend_from_slice(&disp32(0, fpat::ODD));
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(pc.regs().rip, at::CODE0 + code.len() as u64);
        assert_eq!(
            pc.cpu.sse().get(0),
            [0x4014_0000_0000_0000, 0xbff0_0000_0000_0000]
        );
    }

    #[test]
    fn movntps_and_movntpd_are_aligned_stores_with_no_register_form() {
        for prefix in [None, Some(0x66u8)] {
            let value = [0x0123_4567_89ab_cdef, 0xfedc_ba98_7654_3210];
            let code_at = |addr: u64| {
                let mut code: Vec<u8> = prefix.into_iter().collect();
                code.extend_from_slice(&[0x0f, 0x2b]);
                code.extend_from_slice(&disp32(0, addr));
                code.push(0xf4);
                code
            };
            let pc = with_sse(value, [0; 2], mxcsr::RESET, &code_at(fpat::DATA));
            assert_eq!(read64(&pc, fpat::DATA), value[0]);
            assert_eq!(read64(&pc, fpat::DATA + 8), value[1]);

            // Misaligned: `#GP(0)`, as `MOVAPS` does.
            let pc = ssepc();
            pc.idt(13, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
            pc.write(HANDLER, &[0xf4]);
            run(&pc, &code_at(fpat::ODD));
            assert_eq!(pc.regs().rip, HANDLED, "{prefix:?}: #GP when misaligned");

            // `mod == 11` is not an encoding.
            let pc = ssepc();
            pc.idt(6, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
            pc.write(HANDLER, &[0xf4]);
            let mut code: Vec<u8> = prefix.into_iter().collect();
            code.extend_from_slice(&[0x0f, 0x2b, 0xc1, 0xf4]);
            run(&pc, &code);
            assert_eq!(
                pc.regs().rip,
                HANDLED,
                "{prefix:?}: #UD for a register form"
            );
        }
    }

    #[test]
    fn movnti_stores_a_general_register_exactly_as_mov_does() {
        let pc = ssepc();
        let mut code = alloc::vec![0xb8]; // mov eax, 0x12345678
        code.extend_from_slice(&0x1234_5678u32.to_le_bytes());
        code.extend_from_slice(&[0x0f, 0xc3]); // movnti [DATA], eax
        code.extend_from_slice(&disp32(0, fpat::DATA));
        code.push(0xf4);
        write64(&pc, fpat::DATA, u64::MAX);
        run(&pc, &code);
        assert_eq!(
            read64(&pc, fpat::DATA),
            0xffff_ffff_1234_5678,
            "four bytes and no more"
        );

        // Register form, and a mandatory prefix in front of the `NP`
        // encoding, are both `#UD`.
        for bytes in [
            &[0x0f, 0xc3, 0xc0, 0xf4][..],
            &[0x66, 0x0f, 0xc3, 0x05, 0, 0xb0, 0, 0, 0xf4],
        ] {
            let pc = ssepc();
            pc.idt(6, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
            pc.write(HANDLER, &[0xf4]);
            run(&pc, bytes);
            assert_eq!(pc.regs().rip, HANDLED, "{bytes:02x?}");
        }

        // It does not need `CR4.OSFXSR`: it touches no SIMD state.
        let pc = Pc::new(Variant::X86_64);
        pc.start_protected();
        run(&pc, &code);
        assert_eq!(read64(&pc, fpat::DATA) & 0xffff_ffff, 0x1234_5678);
    }

    #[test]
    fn maskmovdqu_writes_only_the_bytes_whose_mask_bit_is_set() {
        let data = [0x0706_0504_0302_0100, 0x0f0e_0d0c_0b0a_0908];
        // Bytes 0, 3, 8 and 15 selected; the other mask bytes have every bit
        // but the top one set, which must not count.
        let mask = [0x7f7f_7f7f_807f_7f80, 0x807f_7f7f_7f7f_7f80];
        let (lo, hi) = (0xaaaa_aaaa_03aa_aa00, 0x0faa_aaaa_aaaa_aa08);
        let setup = |code: &[u8], fs_base: u64| {
            let pc = ssepc();
            let mut sys = pc.cpu.sys();
            sys.segs[usize::from(isa::seg::FS)].base = fs_base;
            pc.cpu.set_sys(sys);
            for addr in [fpat::DATA, fpat::DATA + 0x1000] {
                write64(&pc, addr, 0xaaaa_aaaa_aaaa_aaaa);
                write64(&pc, addr + 8, 0xaaaa_aaaa_aaaa_aaaa);
            }
            let mut sse = Sse::new();
            sse.set(0, data);
            sse.set(1, mask);
            pc.cpu.set_sse(sse);
            run(&pc, code);
            pc
        };
        let mut mov_edi = alloc::vec![0xbf];
        mov_edi.extend_from_slice(&(fpat::DATA as u32).to_le_bytes());

        // maskmovdqu xmm0, xmm1 — to [ds:edi].
        let mut code = mov_edi.clone();
        code.extend_from_slice(&[0x66, 0x0f, 0xf7, 0xc1, 0xf4]);
        let pc = setup(&code, 0);
        assert_eq!(read64(&pc, fpat::DATA), lo);
        assert_eq!(read64(&pc, fpat::DATA + 8), hi);
        assert_eq!(read64(&pc, fpat::DATA + 0x1000), 0xaaaa_aaaa_aaaa_aaaa);

        // An `FS` override moves the store by `FS`'s base.
        let mut code = mov_edi.clone();
        code.extend_from_slice(&[0x64, 0x66, 0x0f, 0xf7, 0xc1, 0xf4]);
        let pc = setup(&code, 0x1000);
        assert_eq!(read64(&pc, fpat::DATA + 0x1000), lo);
        assert_eq!(read64(&pc, fpat::DATA + 0x1008), hi);
        assert_eq!(
            read64(&pc, fpat::DATA),
            0xaaaa_aaaa_aaaa_aaaa,
            "not at ds:edi"
        );

        // A `67` prefix makes the address `DI`, so the junk in `EDI`'s top
        // half is ignored rather than added.
        let mut code = alloc::vec![0xbf];
        code.extend_from_slice(&(0x7fff_0000 | fpat::DATA as u32).to_le_bytes());
        code.extend_from_slice(&[0x67, 0x66, 0x0f, 0xf7, 0xc1, 0xf4]);
        let pc = setup(&code, 0);
        assert_eq!(read64(&pc, fpat::DATA), lo);
        assert_eq!(read64(&pc, fpat::DATA + 8), hi);

        // Register form only.
        let pc = ssepc();
        pc.idt(6, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
        pc.write(HANDLER, &[0xf4]);
        let mut code = alloc::vec![0x66, 0x0f, 0xf7];
        code.extend_from_slice(&disp32(0, fpat::DATA));
        code.push(0xf4);
        run(&pc, &code);
        assert_eq!(pc.regs().rip, HANDLED, "#UD with a memory operand");
    }

    #[test]
    fn rcpps_and_rsqrtps_give_the_manuals_special_cases_and_raise_nothing() {
        let rcp = lanes([0x4000_0000, 0x4040_0000, 0x8000_0000, 0x7f80_0000]); // 2, 3, -0, +inf
        let (out, flags) = convert(0, 0x53, [0; 2], rcp, mxcsr::RESET);
        assert_eq!(out, lanes([0x3f00_0000, 0x3eaa_aaab, 0xff80_0000, 0]));
        assert_eq!(flags, 0);
        // A denormal is a zero of its sign, whatever `DAZ` says; a tiny
        // result is a zero, whatever `FTZ` says; a signalling NaN comes back
        // quiet and sets no `IE`.
        let rcp = lanes([0x0000_0001, 0x8000_0001, 0x7f00_0000, 0x7f80_0001]);
        let (out, flags) = convert(0, 0x53, [0; 2], rcp, mxcsr::RESET);
        assert_eq!(out, lanes([0x7f80_0000, 0xff80_0000, 0, 0x7fc0_0001]));
        assert_eq!(flags, 0);

        // 4, 2, -1, +inf
        let rsq = lanes([0x4080_0000, 0x4000_0000, 0xbf80_0000, 0x7f80_0000]);
        let (out, flags) = convert(0, 0x52, [0; 2], rsq, mxcsr::RESET);
        assert_eq!(out, lanes([0x3f00_0000, 0x3f35_04f3, 0xffc0_0000, 0]));
        assert_eq!(flags, 0);
        // +0, -0, a negative denormal, a signalling NaN.
        let rsq = lanes([0, 0x8000_0000, 0x8000_0001, 0x7f80_0001]);
        let (out, flags) = convert(0, 0x52, [0; 2], rsq, mxcsr::RESET);
        assert_eq!(
            out,
            lanes([0x7f80_0000, 0xff80_0000, 0xff80_0000, 0x7fc0_0001])
        );
        assert_eq!(flags, 0);
        // `RC` is ignored: round-up gives the same bits as to-nearest.
        let (up, _) = convert(
            0,
            0x53,
            [0; 2],
            lanes([0x4040_0000; 4]),
            mxcsr::RESET | (2 << mxcsr::RC_SHIFT),
        );
        assert_eq!(up, lanes([0x3eaa_aaab; 4]));
    }

    #[test]
    fn the_scalar_approximations_keep_the_upper_three_lanes() {
        let dst = lanes([0x1111_1111, 0x2222_2222, 0x3333_3333, 0x4444_4444]);
        let src = lanes([0x4080_0000, 0x5555_5555, 0x6666_6666, 0x7777_7777]);
        let (out, _) = convert(0xf3, 0x53, dst, src, mxcsr::RESET);
        assert_eq!(
            out,
            lanes([0x3e80_0000, 0x2222_2222, 0x3333_3333, 0x4444_4444]),
            "rcpss 4"
        );
        let (out, _) = convert(0xf3, 0x52, dst, src, mxcsr::RESET);
        assert_eq!(
            out,
            lanes([0x3f00_0000, 0x2222_2222, 0x3333_3333, 0x4444_4444]),
            "rsqrtss 4"
        );
    }

    /// Whether `y` is the correctly rounded value of `x^-power` (power 1 or
    /// 2 meaning `1/x` or `1/sqrt(x)`), by exact integer comparison of the
    /// two midpoints around `y` against the true value — which is then also
    /// inside the architecture's `1.5 × 2^-12` bound with room to spare.
    fn correctly_rounded(x: u32, y: u32, square: bool) -> bool {
        let parts = |v: u32| {
            let field = ((v >> 23) & 0xff) as i32;
            (u128::from(v & 0x7f_ffff | 0x80_0000), field - 150)
        };
        let (mx, ex) = parts(x);
        let (my, ey) = parts(y);
        // `mid · 2^(ey - 1)` for the two midpoints `2·my ∓ 1`; the condition
        // is `lo^p · x <= 1 <= hi^p · x`.
        let side = |mid: u128| {
            let (scaled, k) = if square {
                (mid * mid * mx, 2 * (ey - 1) + ex)
            } else {
                (mid * mx, (ey - 1) + ex)
            };
            // Compare `scaled · 2^k` with 1.
            if k >= 0 {
                core::cmp::Ordering::Greater
            } else {
                scaled.cmp(&(1u128 << (-k)))
            }
        };
        side(2 * my - 1) != core::cmp::Ordering::Greater
            && side(2 * my + 1) != core::cmp::Ordering::Less
    }

    #[test]
    fn the_approximations_are_correctly_rounded_across_the_normal_range() {
        use crate::cpu::x86::fpexec::{approx_reciprocal, approx_reciprocal_sqrt};
        // The check itself can say no: one unit in the last place either way
        // of 1/3 and of 1/sqrt(2) is refused.
        assert!(correctly_rounded(0x4040_0000, 0x3eaa_aaab, false));
        assert!(!correctly_rounded(0x4040_0000, 0x3eaa_aaaa, false));
        assert!(!correctly_rounded(0x4040_0000, 0x3eaa_aaac, false));
        assert!(correctly_rounded(0x4000_0000, 0x3f35_04f3, true));
        assert!(!correctly_rounded(0x4000_0000, 0x3f35_04f2, true));
        assert!(!correctly_rounded(0x4000_0000, 0x3f35_04f4, true));
        // A stride through every positive normal, plus the significands just
        // either side of a power of two where the rounding is most delicate.
        let mut inputs: Vec<u32> = (0x0080_0000u32..0x7f80_0000).step_by(4093).collect();
        for field in [1u32, 100, 126, 127, 128, 200, 252] {
            for frac in [0u32, 1, 2, 0x7f_fffe, 0x7f_ffff] {
                inputs.push((field << 23) | frac);
            }
        }
        for x in inputs {
            let y = approx_reciprocal_sqrt(x);
            assert!(
                correctly_rounded(x, y, true),
                "rsqrt {x:#010x} -> {y:#010x}"
            );
            let r = approx_reciprocal(x);
            // Above 2^126 the reciprocal is tiny and is flushed to zero.
            if x < 0x7e80_0000 {
                assert!(correctly_rounded(x, r, false), "rcp {x:#010x} -> {r:#010x}");
            } else if x > 0x7e80_0000 {
                assert_eq!(r, 0, "rcp {x:#010x} is tiny");
            }
        }
    }

    #[test]
    fn a_part_without_sse2_refuses_the_new_sse2_encodings_and_keeps_the_sse_ones() {
        let part = || {
            let features = Features {
                sse2: false,
                long: false,
                nx: false,
                syscall: false,
                ..Features::X86_64
            };
            let pc = Pc::with_features(Variant::X86_64, features);
            pc.start_protected();
            let mut sys = pc.cpu.sys();
            sys.cr4 |= cr4::OSFXSR | cr4::OSXMMEXCPT;
            pc.cpu.set_sys(sys);
            pc.idt(6, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
            pc.idt(13, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
            pc.write(HANDLER, &[0xf4]);
            pc
        };
        let sse2: [&[u8]; 9] = [
            &[0x0f, 0x5b, 0xc1],                      // cvtdq2ps
            &[0x66, 0x0f, 0x5b, 0xc1],                // cvtps2dq
            &[0xf3, 0x0f, 0x5b, 0xc1],                // cvttps2dq
            &[0xf2, 0x0f, 0xe6, 0xc1],                // cvtpd2dq
            &[0x66, 0x0f, 0xe6, 0xc1],                // cvttpd2dq
            &[0xf3, 0x0f, 0xe6, 0xc1],                // cvtdq2pd
            &[0x66, 0x0f, 0xf7, 0xc1],                // maskmovdqu
            &[0x66, 0x0f, 0x2b, 0x05, 0, 0xb0, 0, 0], // movntpd
            &[0x0f, 0xc3, 0x05, 0, 0xb0, 0, 0],       // movnti
        ];
        for bytes in sse2 {
            let pc = part();
            let mut code = bytes.to_vec();
            code.push(0xf4);
            run(&pc, &code);
            assert_eq!(pc.regs().rip, HANDLED, "{bytes:02x?} is #UD without SSE2");
        }
        // The SSE half is still there.
        let sse: [&[u8]; 5] = [
            &[0x0f, 0x53, 0xc1],                // rcpps
            &[0x0f, 0x52, 0xc1],                // rsqrtps
            &[0xf3, 0x0f, 0x53, 0xc1],          // rcpss
            &[0xf3, 0x0f, 0x52, 0xc1],          // rsqrtss
            &[0x0f, 0x2b, 0x05, 0, 0xb0, 0, 0], // movntps
        ];
        for bytes in sse {
            let pc = part();
            let mut code = bytes.to_vec();
            code.push(0xf4);
            run(&pc, &code);
            assert_eq!(
                pc.regs().rip,
                at::CODE0 + code.len() as u64,
                "{bytes:02x?} executes"
            );
        }
    }

    #[test]
    fn a_prefix_the_new_rows_do_not_name_is_an_invalid_opcode() {
        // `66 0F 52`/`53` (no double-precision approximation before
        // AVX-512), `F2 0F 5B`, and `F3`/`F2 0F 2B` would otherwise inherit
        // the no-prefix row.
        for bytes in [
            &[0x66, 0x0f, 0x52, 0xc1][..],
            &[0x66, 0x0f, 0x53, 0xc1],
            &[0xf2, 0x0f, 0x52, 0xc1],
            &[0xf2, 0x0f, 0x5b, 0xc1],
            &[0xf3, 0x0f, 0x2b, 0x05, 0, 0xb0, 0, 0],
            &[0xf2, 0x0f, 0x2b, 0x05, 0, 0xb0, 0, 0],
            &[0x0f, 0xe6, 0xc1],
            // `0F F7` with no prefix is `MASKMOVQ` now, an MMX instruction;
            // `0F D6` with none is still nothing.
            &[0x0f, 0xd6, 0xc1],
        ] {
            let pc = ssepc();
            pc.idt(6, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
            pc.write(HANDLER, &[0xf4]);
            let mut code = bytes.to_vec();
            code.push(0xf4);
            run(&pc, &code);
            assert_eq!(pc.regs().rip, HANDLED, "{bytes:02x?}");
        }
    }

    #[test]
    fn the_disassembler_names_the_thirteen() {
        let pc = ssepc();
        let listing = |bytes: &[u8]| {
            pc.write(at::CODE0, bytes);
            let out = pc.cpu.disassemble(0x08, at::CODE0, 1);
            alloc::format!("{}", out[0])
        };
        let cases: [(&[u8], &str); 15] = [
            (
                &[0x0f, 0x2b, 0x05, 0, 0xb0, 0, 0],
                "movntps xmmword [ds:0xb000], xmm0",
            ),
            (
                &[0x66, 0x0f, 0x2b, 0x0d, 0, 0xb0, 0, 0],
                "movntpd xmmword [ds:0xb000], xmm1",
            ),
            (
                &[0x0f, 0xc3, 0x05, 0, 0xb0, 0, 0],
                "movnti dword [ds:0xb000], eax",
            ),
            (&[0x66, 0x0f, 0xf7, 0xc1], "maskmovdqu xmm0, xmm1"),
            (&[0x0f, 0x5b, 0xc1], "cvtdq2ps xmm0, xmm1"),
            (&[0x66, 0x0f, 0x5b, 0xc1], "cvtps2dq xmm0, xmm1"),
            (&[0xf3, 0x0f, 0x5b, 0xc1], "cvttps2dq xmm0, xmm1"),
            (&[0xf2, 0x0f, 0xe6, 0xc1], "cvtpd2dq xmm0, xmm1"),
            (&[0x66, 0x0f, 0xe6, 0xc1], "cvttpd2dq xmm0, xmm1"),
            (&[0xf3, 0x0f, 0xe6, 0xc1], "cvtdq2pd xmm0, xmm1"),
            (&[0x0f, 0x53, 0xc1], "rcpps xmm0, xmm1"),
            (&[0x0f, 0x52, 0xc1], "rsqrtps xmm0, xmm1"),
            (&[0xf3, 0x0f, 0x53, 0xc1], "rcpss xmm0, xmm1"),
            (&[0xf3, 0x0f, 0x52, 0xc1], "rsqrtss xmm0, xmm1"),
            (
                &[0xf3, 0x0f, 0xe6, 0x05, 0, 0xb0, 0, 0],
                "cvtdq2pd xmm0, qword [ds:0xb000]",
            ),
        ];
        for (bytes, text) in cases {
            let got = listing(bytes);
            assert!(got.ends_with(text), "{bytes:02x?}: {got}");
            assert!(!got.contains("rep"), "{got}");
        }
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
    use crate::cpu::x86::prot::{apic_base, cr4, msr, mtrr};

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

    // -- The memory-type range registers ---------------------------------
    //
    // *Intel SDM* volume 3A §12.11 and volume 4 Table 2-2. What made these
    // worth having is written up on `prot::mtrr`: with `CPUID`'s `MTRR` bit
    // clear, Linux reports `MTRRs disabled (not available)` and `pmd_set_huge`
    // then refuses every 2 MiB mapping, so a 64-bit kernel loses its large
    // pages on a part that claims long mode.

    /// A part with the memory-type range registers: the `x86-64` preset, which
    /// is where they belong.
    fn pc_mtrr() -> Pc {
        let pc = Pc::new(Variant::X86_64);
        pc.start_protected();
        pc
    }

    /// `mov ecx, index ; mov eax, lo ; mov edx, hi ; wrmsr`, stepped exactly
    /// four times.
    ///
    /// Stepped rather than run to a `hlt`, because a halted core answers every
    /// later `step` with zero and a second program would then quietly not run
    /// at all — which is a way to write a passing test that measures nothing.
    fn wrmsr(pc: &Pc, index: u32, value: u64) {
        let mut code = alloc::vec![0xb9];
        code.extend_from_slice(&index.to_le_bytes());
        code.push(0xb8);
        code.extend_from_slice(&(value as u32).to_le_bytes());
        code.push(0xba);
        code.extend_from_slice(&((value >> 32) as u32).to_le_bytes());
        code.extend_from_slice(&[0x0f, 0x30]);
        pc.write(at::CODE0, &code);
        set_rip(pc, at::CODE0);
        for _ in 0..4 {
            pc.cpu.step();
        }
    }

    /// `mov ecx, index ; rdmsr`, stepped twice, with `EDX:EAX` reassembled.
    fn rdmsr(pc: &Pc, index: u32) -> u64 {
        let mut code = alloc::vec![0xb9];
        code.extend_from_slice(&index.to_le_bytes());
        code.extend_from_slice(&[0x0f, 0x32]);
        pc.write(at::CODE0, &code);
        set_rip(pc, at::CODE0);
        pc.cpu.step();
        pc.cpu.step();
        let r = pc.regs();
        u64::from(r.rax as u32) | (u64::from(r.rdx as u32) << 32)
    }

    #[test]
    fn every_long_mode_part_has_the_memory_type_range_registers() {
        // Not a preference: `Features::validate` refuses the combination,
        // because `CPUID` leaf 1 has set the `MTRR` bit on every part since
        // the Pentium Pro and a 64-bit kernel that finds it clear concludes
        // its firmware left the memory map undescribed.
        let impossible = Features {
            mtrr: false,
            ..Features::X86_64
        };
        assert!(impossible.validate().is_err(), "long mode needs MTRRs");
        // And a part with the ranges but no `RDMSR` to reach them is refused
        // for the reason that is not a part either.
        let impossible = Features {
            mtrr: true,
            ..Features::NONE
        };
        assert!(impossible.validate().is_err());
    }

    #[test]
    fn cpuid_reports_the_mtrr_bit_and_mtrrcap_says_how_many_there_are() {
        let pc = pc_mtrr();
        // mov eax, 1 ; cpuid
        pc.write(at::CODE0, &[0xb8, 1, 0, 0, 0, 0x0f, 0xa2, 0xf4]);
        pc.run(20);
        assert_ne!(pc.regs().rdx & (1 << 12), 0, "leaf 1 should report MTRR");

        let pc = pc_mtrr();
        let cap = rdmsr(&pc, msr::MTRRCAP);
        assert_eq!(cap & 0xff, 8, "eight variable ranges");
        assert_ne!(cap & mtrr::FIX, 0, "and the fixed ones");
        assert_ne!(cap & mtrr::WC, 0, "write-combining is an available type");
        assert_eq!(cap & (1 << 11), 0, "but there is no SMRR: no SMM here");
    }

    #[test]
    fn the_capability_register_is_read_only() {
        let pc = pc_mtrr();
        pc.idt(13, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
        pc.write(HANDLER, &[0xf4]);
        wrmsr(&pc, msr::MTRRCAP, 0);
        assert_eq!(pc.regs().rip, HANDLER, "#GP(0), not a stored value");
    }

    #[test]
    fn a_range_written_reads_back_and_survives_a_reset_of_nothing_else() {
        let pc = pc_mtrr();
        // The shape firmware writes: 0-2 GiB write-back, enabled.
        wrmsr(&pc, msr::MTRR_PHYSBASE0, 6);
        wrmsr(
            &pc,
            msr::MTRR_PHYSBASE0 + 1,
            0xff_8000_0000 | mtrr::MASK_VALID,
        );
        wrmsr(&pc, msr::MTRR_DEF_TYPE, mtrr::DEF_E | mtrr::DEF_FE);
        assert_eq!(rdmsr(&pc, msr::MTRR_PHYSBASE0), 6);
        assert_eq!(
            rdmsr(&pc, msr::MTRR_PHYSBASE0 + 1),
            0xff_8000_0000 | mtrr::MASK_VALID
        );
        assert_eq!(
            rdmsr(&pc, msr::MTRR_DEF_TYPE),
            mtrr::DEF_E | mtrr::DEF_FE,
            "and the enables"
        );
        // The seventh range is there too, which is what `VCNT` promised.
        wrmsr(&pc, msr::MTRR_PHYSBASE0 + 14, 0x1000 | 1);
        assert_eq!(rdmsr(&pc, msr::MTRR_PHYSBASE0 + 14), 0x1000 | 1);
        // And the register one past the last is not.
        pc.idt(13, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
        pc.write(HANDLER, &[0xf4]);
        rdmsr(&pc, msr::MTRR_PHYS_END);
        assert_eq!(pc.regs().rip, HANDLER, "eight ranges, not nine");
    }

    #[test]
    fn the_fixed_ranges_are_eleven_registers_at_three_runs_of_addresses() {
        let pc = pc_mtrr();
        pc.idt(13, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
        pc.write(HANDLER, &[0xf4]);
        // All write-back, which is what firmware leaves the first megabyte as
        // once it has finished shadowing itself.
        for index in [
            msr::MTRR_FIX64K,
            msr::MTRR_FIX16K,
            msr::MTRR_FIX16K + 1,
            msr::MTRR_FIX4K,
            msr::MTRR_FIX4K + 7,
        ] {
            wrmsr(&pc, index, 0x0606_0606_0606_0606);
            assert_eq!(rdmsr(&pc, index), 0x0606_0606_0606_0606, "{index:#x}");
        }
        // The gaps between the runs are not registers, and neither is the one
        // past the last 4 KiB range.
        for index in [
            msr::MTRR_FIX16K + 2,
            msr::MTRR_FIX4K - 1,
            msr::MTRR_FIX4K + 8,
        ] {
            rdmsr(&pc, index);
            assert_eq!(pc.regs().rip, HANDLER, "{index:#x} is not a register");
        }
    }

    #[test]
    fn a_reserved_bit_or_an_undefined_memory_type_is_refused() {
        // The rule `IA32_APIC_BASE` and `IA32_MISC_ENABLE` already follow: a
        // reserved bit accepted is a feature a guest thinks it turned on, and
        // an undefined type accepted is how software would fail to discover
        // which types a processor has.
        let pc = pc_mtrr();
        pc.idt(13, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
        pc.write(HANDLER, &[0xf4]);
        let faults = |value: u64, index: u32| {
            wrmsr(&pc, index, value);
            pc.regs().rip == HANDLER
        };
        assert!(faults(2, msr::MTRR_PHYSBASE0), "type 2 is reserved");
        assert!(faults(3, msr::MTRR_PHYSBASE0), "and so is type 3");
        assert!(faults(7, msr::MTRR_PHYSBASE0), "and everything above 6");
        assert!(faults(0x100, msr::MTRR_PHYSBASE0), "bits 11:8 are reserved");
        assert!(
            faults(0x0100_0000_0000, msr::MTRR_PHYSBASE0),
            "and so is every bit above the 40 CPUID reports"
        );
        assert!(
            faults(0x0100_0000_0000, msr::MTRR_PHYSBASE0 + 1),
            "the mask has the same width"
        );
        assert!(faults(1 << 12, msr::MTRR_DEF_TYPE), "def-type bit 12");
        assert!(faults(7, msr::MTRR_DEF_TYPE), "and its type field too");
        assert!(
            faults(0x0606_0606_0606_0602, msr::MTRR_FIX64K),
            "one bad byte in eight refuses the whole write"
        );
        assert_eq!(
            rdmsr(&pc, msr::MTRR_FIX64K),
            0,
            "and stores none of the other seven"
        );
    }

    #[test]
    fn a_part_without_the_feature_has_no_such_registers_at_all() {
        // `pc_msr` is a 486 with model-specific registers and no MTRRs, which
        // is a part that shipped. Reading one has to be `#GP(0)` rather than a
        // plausible zero, because that is how a guest tells "absent" from
        // "present and disabled".
        let pc = pc_msr();
        pc.start_protected();
        pc.idt(13, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
        pc.write(HANDLER, &[0xf4]);
        rdmsr(&pc, msr::MTRRCAP);
        assert_eq!(pc.regs().rip, HANDLER);
    }

    #[test]
    fn the_ranges_are_in_the_snapshot_because_a_guest_reads_back_what_it_wrote() {
        let pc = pc_mtrr();
        wrmsr(&pc, msr::MTRR_PHYSBASE0, 6);
        wrmsr(
            &pc,
            msr::MTRR_PHYSBASE0 + 1,
            0xff_8000_0000 | mtrr::MASK_VALID,
        );
        wrmsr(&pc, msr::MTRR_FIX4K + 3, 0x0505_0505_0505_0505);
        wrmsr(&pc, msr::MTRR_DEF_TYPE, mtrr::DEF_E);
        let before = pc.cpu.sys();

        let mut shape = MachineShape::new();
        shape.add_device("/cpu0", "cpu.x86").unwrap();
        let mut writer = StateWriter::new(shape);
        {
            let mut chunk = writer
                .chunk("/cpu0", "cpu.x86", crate::cpu::x86::CLASS.version)
                .unwrap();
            pc.cpu.save(&mut chunk).unwrap();
        }
        let bytes = writer.to_vec().unwrap();

        let restored = pc_mtrr();
        let reader = StateReader::new(&bytes).unwrap();
        let (_, _, data) = reader.load_raw("/cpu0").unwrap();
        let mut chunk = ChunkReader::new(data);
        restored.cpu.load(&mut chunk).unwrap();
        chunk.end().unwrap();
        let after = restored.cpu.sys();
        assert_eq!(after.mtrr_var, before.mtrr_var);
        assert_eq!(after.mtrr_fix, before.mtrr_fix);
        assert_eq!(after.mtrr_def_type, before.mtrr_def_type);
    }
}

/// MMX: the `MM` registers that alias the x87 ones, their side effects on the
/// x87 unit, and every encoding issue #37 asked for. Expected values are
/// hand-checkable lane arithmetic or the rule on the instruction's page in
/// *Intel SDM* volume 2; the aliasing is volume 1 §9.5.
mod mmx {
    use super::*;
    use crate::cpu::x86::fpu::{Sse, Tag, mxcsr, sw};
    use crate::cpu::x86::prot::{cr0, cr4};
    use crate::float::x87::F80;

    /// Scratch memory: two operands and a result.
    const A: u64 = 0xb000;
    const B: u64 = 0xb008;
    const OUT: u64 = 0xc000;
    /// A 512-byte save area, sixteen-byte aligned.
    const SAVE: u64 = 0xd000;
    /// Where a fault handler sits, and `RIP` once its `hlt` has retired.
    const HANDLER: u64 = 0x3800;
    const HANDLED: u64 = HANDLER + 1;

    /// An x86-64 part (MMX, SSE, SSE2) in 32-bit protected mode with
    /// `CR4.OSFXSR` set, so the cross-file forms are allowed too.
    fn mmxpc() -> Pc {
        let pc = Pc::new(Variant::X86_64);
        pc.start_protected();
        let mut sys = pc.cpu.sys();
        sys.cr4 |= cr4::OSFXSR | cr4::OSXMMEXCPT;
        pc.cpu.set_sys(sys);
        pc
    }

    /// `disp32` addressing for `reg`: `mod == 00`, `r/m == 101`.
    fn disp32(reg: u8, addr: u64) -> Vec<u8> {
        let mut out = alloc::vec![((reg & 7) << 3) | 0b101];
        out.extend_from_slice(&(addr as u32).to_le_bytes());
        out
    }

    /// `0F opcode` with a `disp32` memory operand.
    fn mem(opcode: u8, reg: u8, addr: u64) -> Vec<u8> {
        let mut out = alloc::vec![0x0f, opcode];
        out.extend_from_slice(&disp32(reg, addr));
        out
    }

    /// Run `code` followed by `hlt`, and require that the `hlt` is reached.
    fn run(pc: &Pc, code: &[u8]) {
        let mut all = code.to_vec();
        all.push(0xf4);
        pc.write(at::CODE0, &all);
        assert!(pc.run(200) < 200, "the program reached its hlt");
        assert_eq!(pc.regs().rip, at::CODE0 + all.len() as u64, "and no fault");
    }

    /// Run `code` and require that it raises `vector`, handled by a `hlt`.
    fn faults(pc: &Pc, vector: u64, code: &[u8]) {
        pc.idt(vector, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
        pc.write(HANDLER, &[0xf4]);
        let mut all = code.to_vec();
        all.push(0xf4);
        pc.write(at::CODE0, &all);
        assert!(pc.run(200) < 200);
        assert_eq!(pc.regs().rip, HANDLED, "{code:02x?} raises vector {vector}");
    }

    fn read64(pc: &Pc, addr: u64) -> u64 {
        (0..8u64).fold(0, |v, i| {
            v | (u64::from(pc.ram.read_u8(addr + i).unwrap()) << (8 * i))
        })
    }

    fn write64(pc: &Pc, addr: u64, value: u64) {
        for i in 0..8u64 {
            pc.ram.write_u8(addr + i, (value >> (8 * i)) as u8).unwrap();
        }
    }

    /// Load `MM` registers directly.
    fn set_mm(pc: &Pc, values: &[(u8, u64)]) {
        let mut x = pc.cpu.x87();
        for &(n, v) in values {
            x.set_mm(n, v);
        }
        pc.cpu.set_x87(x);
    }

    fn mm(pc: &Pc, n: u8) -> u64 {
        pc.cpu.x87().mm(n)
    }

    /// `op mm0, mm1` on two values: `mm0` afterwards.
    fn binary(opcode: u8, a: u64, b: u64) -> u64 {
        let pc = mmxpc();
        set_mm(&pc, &[(0, a), (1, b)]);
        run(&pc, &[0x0f, opcode, 0xc1]);
        mm(&pc, 0)
    }

    /// Four word lanes, lane 0 lowest.
    fn words(w: [u16; 4]) -> u64 {
        w.iter()
            .enumerate()
            .fold(0, |v, (i, &x)| v | (u64::from(x) << (16 * i)))
    }

    // -- The issue's tests ---------------------------------------------

    #[test]
    fn paddusb_saturates_each_byte() {
        // movq mm0, [A] ; movq mm1, [B] ; paddusb mm0, mm1 ; movq [OUT], mm0
        let pc = mmxpc();
        write64(
            &pc,
            A,
            u64::from_le_bytes([0xf0, 0x10, 0x80, 0x7f, 0, 0xff, 1, 0xfe]),
        );
        write64(
            &pc,
            B,
            u64::from_le_bytes([0x20, 0x10, 0x80, 1, 0, 1, 0xff, 1]),
        );
        let mut code = mem(0x6f, 0, A);
        code.extend(mem(0x6f, 1, B));
        code.extend([0x0f, 0xdc, 0xc1]);
        code.extend(mem(0x7f, 0, OUT));
        run(&pc, &code);
        assert_eq!(
            read64(&pc, OUT).to_le_bytes(),
            [0xff, 0x20, 0xff, 0x80, 0, 0xff, 0xff, 0xff]
        );
    }

    #[test]
    fn an_mmx_write_sets_top_zero_tags_valid_and_exponent_ffff() {
        // fninit ; fld1 ; mov eax, 0x12345678 ; movd mm3, eax
        let pc = mmxpc();
        run(
            &pc,
            &[
                0xdb, 0xe3, 0xd9, 0xe8, 0xb8, 0x78, 0x56, 0x34, 0x12, 0x0f, 0x6e, 0xd8,
            ],
        );
        let x = pc.cpu.x87();
        assert_eq!(x.top(), 0, "TOP was 7 after the push");
        assert_eq!(x.tag, 0, "every register valid, empty ones included");
        assert_eq!(x.regs[3], F80::new(0xffff, 0x1234_5678), "zero-extended");
        // The register the `FLD1` wrote is untouched: only a *write* to an
        // `MM` register changes its exponent.
        assert_eq!(x.regs[7], F80::new(0x3fff, 0x8000_0000_0000_0000));
        assert_eq!(x.mm(7), 0x8000_0000_0000_0000);
    }

    #[test]
    fn an_mmx_read_alone_still_makes_the_transition() {
        // fninit ; fld1 ; movd eax, mm7 — the significand of 1.0's low half.
        let pc = mmxpc();
        run(&pc, &[0xdb, 0xe3, 0xd9, 0xe8, 0x0f, 0x7e, 0xf8]);
        let x = pc.cpu.x87();
        assert_eq!((x.top(), x.tag), (0, 0));
        assert_eq!(x.regs[7].sign_exp, 0x3fff, "a read writes no exponent");
        assert_eq!(pc.regs().rax as u32, 0);
    }

    #[test]
    fn emms_empties_the_tag_word() {
        // movd mm0, eax ; emms
        let pc = mmxpc();
        run(&pc, &[0x0f, 0x6e, 0xc0, 0x0f, 0x77]);
        let x = pc.cpu.x87();
        assert_eq!(x.tag, 0xffff);
        assert_eq!(x.top(), 0, "EMMS leaves TOP alone");
        assert_eq!(x.tag_at(0), Tag::Empty);
    }

    #[test]
    fn x87_code_after_mmx_needs_emms_first() {
        // Without `EMMS` the stack is full and `FLD1` overflows; with it the
        // push lands in an empty register. That asymmetry is why the
        // instruction exists.
        let pc = mmxpc();
        run(&pc, &[0x0f, 0x6e, 0xc0, 0xd9, 0xe8]); // movd mm0, eax ; fld1
        let x = pc.cpu.x87();
        assert_ne!(x.status & sw::SF, 0, "a stack overflow");
        assert_ne!(x.status & sw::C1, 0);

        let pc = mmxpc();
        run(&pc, &[0x0f, 0x6e, 0xc0, 0x0f, 0x77, 0xd9, 0xe8]); // ... emms ; fld1
        let x = pc.cpu.x87();
        assert_eq!(x.status & sw::EXCEPTIONS, 0);
        assert_eq!(x.top(), 7);
        assert_eq!(x.raw(0), F80::new(0x3fff, 0x8000_0000_0000_0000));
    }

    #[test]
    fn mmx_is_ud_on_a_part_without_the_feature() {
        // A 486DX has an x87 unit and no MMX.
        let pc = Pc::new(Variant::I80486);
        pc.start_protected();
        faults(&pc, 6, &[0x0f, 0xfc, 0xc1]); // paddb mm0, mm1
        let pc = Pc::new(Variant::I80486);
        pc.start_protected();
        faults(&pc, 6, &[0x0f, 0x77]); // emms
        // And CPUID says so.
        let pc = Pc::new(Variant::I80486);
        pc.start_protected();
        run(&pc, &[0xb8, 0x01, 0x00, 0x00, 0x00, 0x0f, 0xa2]);
        assert_eq!(pc.regs().rdx & (1 << 23), 0);
    }

    #[test]
    fn the_sse_and_sse2_additions_need_their_own_features() {
        // A Pentium MMX's lattice point: MMX and an x87 unit, no SSE.
        let features = Features {
            sse: false,
            sse2: false,
            long: false,
            nx: false,
            syscall: false,
            ..Features::X86_64
        };
        assert!(features.validate().is_ok());
        let part = || {
            let pc = Pc::with_features(Variant::X86_64, features);
            pc.start_protected();
            pc
        };
        run(&part(), &[0x0f, 0xfc, 0xc1]); // paddb mm0, mm1
        faults(&part(), 6, &[0x0f, 0xe0, 0xc1]); // pavgb: SSE
        faults(&part(), 6, &[0x0f, 0x70, 0xc1, 0x1b]); // pshufw: SSE
        faults(&part(), 6, &[0x0f, 0xd4, 0xc1]); // paddq mm: SSE2
        faults(&part(), 6, &[0x0f, 0xf4, 0xc1]); // pmuludq mm: SSE2
    }

    #[test]
    fn the_feature_lattice_ties_mmx_to_the_x87_unit_and_sse_to_mmx() {
        let no_mmx = Features {
            mmx: false,
            ..Features::X86_64
        };
        assert!(no_mmx.validate().is_err(), "SSE without MMX");
        let no_fpu = Features {
            mmx: true,
            ..Features::NONE
        };
        assert!(no_fpu.validate().is_err(), "MMX without an x87 unit");
        assert!(
            Features {
                mmx: true,
                ..Features::I80486
            }
            .validate()
            .is_ok()
        );
    }

    #[test]
    fn cr0_em_makes_mmx_invalid_rather_than_trappable() {
        // Volume 2's MMX exception tables: `CR0.EM` is `#UD`, not the `#NM`
        // an x87 instruction would take — there is no emulation protocol.
        for code in [&[0x0f, 0xfc, 0xc1][..], &[0x0f, 0x77]] {
            let pc = mmxpc();
            let mut sys = pc.cpu.sys();
            sys.cr0 |= cr0::EM;
            pc.cpu.set_sys(sys);
            faults(&pc, 6, code);
        }
    }

    #[test]
    fn cr0_ts_makes_mmx_a_device_not_available_fault() {
        for code in [&[0x0f, 0xfc, 0xc1][..], &[0x0f, 0x77]] {
            let pc = mmxpc();
            let mut sys = pc.cpu.sys();
            sys.cr0 |= cr0::TS;
            pc.cpu.set_sys(sys);
            faults(&pc, 7, code);
            // And the x87 unit was left exactly as it was.
            assert_eq!(pc.cpu.x87().tag, 0xffff);
        }
    }

    #[test]
    fn a_pending_x87_exception_is_delivered_before_an_mmx_instruction() {
        for code in [&[0x0f, 0xfc, 0xc1][..], &[0x0f, 0x77]] {
            let pc = mmxpc();
            let mut sys = pc.cpu.sys();
            sys.cr0 |= cr0::NE;
            pc.cpu.set_sys(sys);
            let mut x = pc.cpu.x87();
            x.status |= sw::IE | sw::ES | sw::B;
            x.set_top(5);
            pc.cpu.set_x87(x);
            faults(&pc, 16, code);
            assert_eq!(pc.cpu.x87().top(), 5, "#MF comes before the transition");
        }
    }

    #[test]
    fn the_cross_file_forms_also_need_cr4_osfxsr() {
        let pc = Pc::new(Variant::X86_64);
        pc.start_protected();
        run(&pc, &[0x0f, 0xfc, 0xc1]); // paddb mm0, mm1: no XMM, no gate
        let pc = Pc::new(Variant::X86_64);
        pc.start_protected();
        faults(&pc, 6, &[0xf3, 0x0f, 0xd6, 0xc1]); // movq2dq xmm0, mm1
        let pc = Pc::new(Variant::X86_64);
        pc.start_protected();
        faults(&pc, 6, &[0x0f, 0x2a, 0xc1]); // cvtpi2ps xmm0, mm1
    }

    // -- Arithmetic ------------------------------------------------------

    #[test]
    fn pmaddwd_sums_signed_products_in_pairs() {
        // [1, 2 | 0x8000, 0x8000] . [4, 5 | 0x8000, 0x8000]: 1*4 + 2*5 = 14,
        // and the one pair that overflows a signed doubleword wraps to
        // 0x8000_0000, as the page says it does.
        let out = binary(
            0xf5,
            words([1, 2, 0x8000, 0x8000]),
            words([4, 5, 0x8000, 0x8000]),
        );
        assert_eq!(out, 14 | (0x8000_0000 << 32));
        // Negative products: -3 * 6 + 7 * -1.
        let out = binary(0xf5, words([0xfffd, 7, 0, 0]), words([6, 0xffff, 0, 0]));
        assert_eq!(out as u32, (-25i32) as u32);
    }

    #[test]
    fn psraw_by_a_register_count_of_sixteen_or_more_fills_with_the_sign() {
        let v = words([0x8000, 0x7fff, 0xffff, 0x0001]);
        assert_eq!(binary(0xe1, v, 16), words([0xffff, 0, 0xffff, 0]));
        // The count is the whole quadword, not its low bits.
        assert_eq!(binary(0xe1, v, 1 << 32), words([0xffff, 0, 0xffff, 0]));
        assert_eq!(binary(0xe1, v, 1), words([0xc000, 0x3fff, 0xffff, 0]));
        // The logical shifts clear instead.
        assert_eq!(binary(0xd1, v, 16), 0, "psrlw");
        assert_eq!(binary(0xf3, v, 64), 0, "psllq");
        assert_eq!(binary(0xd3, v, 4), v >> 4, "psrlq");
    }

    #[test]
    fn the_immediate_shifts_name_their_register_in_r_m() {
        // psraw mm1, 3 ; psrld mm2, 4 ; psllq mm3, 8
        let pc = mmxpc();
        set_mm(
            &pc,
            &[
                (1, words([0x8000, 0x0010, 0, 0])),
                (2, 0xf000_0000_0000_00f0),
                (3, 0xff),
            ],
        );
        run(
            &pc,
            &[
                0x0f, 0x71, 0xe1, 0x03, 0x0f, 0x72, 0xd2, 0x04, 0x0f, 0x73, 0xf3, 0x08,
            ],
        );
        assert_eq!(mm(&pc, 1), words([0xf000, 0x0002, 0, 0]));
        assert_eq!(mm(&pc, 2), 0x0f00_0000_0000_000f);
        assert_eq!(mm(&pc, 3), 0xff00);
    }

    #[test]
    fn the_byte_shifts_have_no_mmx_form() {
        // `0F 73 /3` and `/7` are `PSRLDQ`/`PSLLDQ` only under `66`.
        faults(&mmxpc(), 6, &[0x0f, 0x73, 0xd9, 0x01]);
        faults(&mmxpc(), 6, &[0x0f, 0x73, 0xf9, 0x01]);
    }

    #[test]
    fn packsswb_saturates_and_puts_the_source_above_the_destination() {
        let dst = words([0x0100, 0xff80, 0x0005, 0x7fff]);
        let src = words([0xffff, 0x0000, 0x8000, 0x0001]);
        assert_eq!(binary(0x63, dst, src), 0x0180_00ff_7f05_807f, "packsswb");
        assert_eq!(binary(0x67, dst, src), 0x0100_0000_ff05_00ff, "packuswb");
        // packssdw: [70000, -70000] and [1, -1].
        let out = binary(
            0x6b,
            u64::from(70000u32) | (u64::from((-70000i32) as u32) << 32),
            1 | (u64::from(u32::MAX) << 32),
        );
        assert_eq!(out, words([0x7fff, 0x8000, 1, 0xffff]));
    }

    #[test]
    fn the_unpacks_interleave_the_named_half() {
        let a = 0x8786_8584_8382_8180;
        let b = 0x9796_9594_9392_9190;
        assert_eq!(binary(0x60, a, b), 0x9383_9282_9181_9080, "punpcklbw");
        assert_eq!(binary(0x68, a, b), 0x9787_9686_9585_9484, "punpckhbw");
        assert_eq!(binary(0x61, a, b), 0x9392_8382_9190_8180, "punpcklwd");
        assert_eq!(binary(0x6a, a, b), 0x9796_9594_8786_8584, "punpckhdq");
    }

    #[test]
    fn a_low_unpack_reads_four_bytes_of_memory() {
        // punpcklbw mm0, [OUT - 4]: the four bytes before `OUT` are read and
        // the four at it are not — the operand is `m32`.
        let pc = mmxpc();
        write64(&pc, OUT - 4, 0xeeee_eeee_0403_0201);
        set_mm(&pc, &[(0, 0)]);
        run(&pc, &mem(0x60, 0, OUT - 4));
        assert_eq!(mm(&pc, 0), 0x0400_0300_0200_0100);
    }

    #[test]
    fn the_adds_and_compares_work_lane_by_lane() {
        assert_eq!(binary(0xfc, 0x01ff, 0x0101), 0x0200, "paddb wraps per byte");
        assert_eq!(binary(0xec, 0x7f, 0x01), 0x7f, "paddsb saturates");
        assert_eq!(binary(0xe8, 0x80, 0x01), 0x80, "psubsb saturates");
        assert_eq!(binary(0xd8, 0x05, 0x09), 0, "psubusb floors at zero");
        assert_eq!(
            binary(0xfe, 0xffff_ffff, 1),
            0,
            "paddd wraps per doubleword"
        );
        assert_eq!(binary(0xd4, u64::MAX, 2), 1, "paddq");
        assert_eq!(binary(0xfb, 0, 1), u64::MAX, "psubq");
        assert_eq!(
            binary(0x74, 0x1234, 0x5534),
            0xffff_ffff_ffff_00ff,
            "pcmpeqb"
        );
        assert_eq!(binary(0x65, words([1, 0xffff, 0, 0]), 0), 0xffff, "pcmpgtw");
        let w = |x: u16| words([x, 0, 0, 0]);
        assert_eq!(binary(0xd5, w(300), w(300)), 90000 & 0xffff, "pmullw");
        assert_eq!(binary(0xe5, w(0x8000), w(2)), 0xffff, "pmulhw");
        assert_eq!(binary(0xe4, w(0x8000), w(2)), 1, "pmulhuw");
        assert_eq!(
            binary(0xf4, 0xffff_ffff, 0xffff_ffff),
            0xffff_fffe_0000_0001,
            "pmuludq"
        );
        assert_eq!(binary(0xe0, 0x01, 0x02), 0x02, "pavgb rounds up");
        assert_eq!(binary(0xda, 0x0105, 0x0203), 0x0103, "pminub");
        assert_eq!(binary(0xee, 0xffff, 0x0001), 0x0001, "pmaxsw is signed");
        assert_eq!(binary(0xf6, 0xff, 0x0101_0101_0101_0100), 262, "psadbw");
    }

    #[test]
    fn the_logic_is_the_whole_quadword() {
        let a = 0xff00_ff00_f0f0_0f0f;
        let b = 0x0ff0_0ff0_ffff_0000;
        assert_eq!(binary(0xdb, a, b), a & b);
        assert_eq!(binary(0xdf, a, b), !a & b);
        assert_eq!(binary(0xeb, a, b), a | b);
        assert_eq!(binary(0xef, a, b), a ^ b);
    }

    #[test]
    fn pshufw_selects_words_by_two_bit_fields() {
        let pc = mmxpc();
        set_mm(&pc, &[(1, words([0x1111, 0x2222, 0x3333, 0x4444]))]);
        run(&pc, &[0x0f, 0x70, 0xc1, 0x1b]); // pshufw mm0, mm1, 0b00_01_10_11
        assert_eq!(mm(&pc, 0), words([0x4444, 0x3333, 0x2222, 0x1111]));
    }

    #[test]
    fn pmovmskb_pextrw_and_pinsrw_cross_to_the_general_registers() {
        let pc = mmxpc();
        set_mm(&pc, &[(1, 0x8001_7f80_00ff_8000)]);
        run(&pc, &[0x0f, 0xd7, 0xc1]); // pmovmskb eax, mm1
        assert_eq!(pc.regs().rax as u32, 0b1001_0110);

        let pc = mmxpc();
        set_mm(&pc, &[(1, words([1, 2, 0xbeef, 4])), (0, u64::MAX)]);
        let mut regs = pc.regs();
        regs.rax = 0xffff_1234;
        pc.cpu.set_regs(regs);
        // pextrw ecx, mm1, 2 ; pinsrw mm0, eax, 1
        run(&pc, &[0x0f, 0xc5, 0xc9, 0x02, 0x0f, 0xc4, 0xc0, 0x01]);
        assert_eq!(pc.regs().rcx as u32, 0xbeef, "zero-extended");
        assert_eq!(mm(&pc, 0), 0xffff_ffff_1234_ffff);
        // The register-only forms have no memory form.
        faults(&mmxpc(), 6, &mem(0xd7, 0, A));
    }

    #[test]
    fn the_moves_cover_every_form() {
        let pc = mmxpc();
        write64(&pc, A, 0x0123_4567_89ab_cdef);
        set_mm(&pc, &[(5, 0xffff_ffff_8765_4321)]);
        // movq mm2, [A] ; movq mm3, mm2 ; movd [OUT], mm5 ; movd ecx, mm2
        let mut code = mem(0x6f, 2, A);
        code.extend([0x0f, 0x6f, 0xda]);
        code.extend(mem(0x7e, 5, OUT));
        code.extend([0x0f, 0x7e, 0xd1]);
        // movd mm4, [A]: a doubleword, zero-extended.
        code.extend(mem(0x6e, 4, A));
        // movntq [OUT + 8], mm3
        code.extend(mem(0xe7, 3, OUT + 8));
        run(&pc, &code);
        assert_eq!(mm(&pc, 3), 0x0123_4567_89ab_cdef);
        assert_eq!(read64(&pc, OUT) as u32, 0x8765_4321);
        assert_eq!(pc.regs().rcx as u32, 0x89ab_cdef);
        assert_eq!(mm(&pc, 4), 0x89ab_cdef);
        assert_eq!(read64(&pc, OUT + 8), 0x0123_4567_89ab_cdef);
        // `MOVNTQ` has no register form.
        faults(&mmxpc(), 6, &[0x0f, 0xe7, 0xc1]);
    }

    #[test]
    fn maskmovq_stores_the_selected_bytes_to_ds_edi() {
        let pc = mmxpc();
        write64(&pc, OUT, 0xeeee_eeee_eeee_eeee);
        set_mm(
            &pc,
            &[(0, 0x8877_6655_4433_2211), (1, 0x8000_0080_0000_ff00)],
        );
        let mut regs = pc.regs();
        regs.rdi = OUT;
        pc.cpu.set_regs(regs);
        run(&pc, &[0x0f, 0xf7, 0xc1]); // maskmovq mm0, mm1
        assert_eq!(read64(&pc, OUT), 0x88ee_ee55_eeee_22ee);
    }

    #[test]
    fn movq2dq_and_movdq2q_cross_between_the_register_files() {
        let pc = mmxpc();
        let mut sse = Sse::new();
        sse.set(0, [u64::MAX, u64::MAX]);
        sse.set(1, [0x1111_2222_3333_4444, 0x5555_6666_7777_8888]);
        pc.cpu.set_sse(sse);
        set_mm(&pc, &[(1, 0xaaaa_bbbb_cccc_dddd)]);
        // movq2dq xmm0, mm1 ; movdq2q mm2, xmm1
        run(&pc, &[0xf3, 0x0f, 0xd6, 0xc1, 0xf2, 0x0f, 0xd6, 0xd1]);
        assert_eq!(
            pc.cpu.sse().get(0),
            [0xaaaa_bbbb_cccc_dddd, 0],
            "upper zeroed"
        );
        assert_eq!(mm(&pc, 2), 0x1111_2222_3333_4444);
        assert_eq!(pc.cpu.x87().regs[2].sign_exp, 0xffff);
    }

    // -- The conversions -------------------------------------------------

    /// Run `code` with `mm1` and `xmm1` loaded and `xmm0` holding junk.
    fn convert(code: &[u8], mm1: u64, xmm1: [u64; 2]) -> Pc {
        let pc = mmxpc();
        let mut sse = Sse::new();
        sse.set(0, [0xdead_beef_dead_beef, 0xfeed_face_feed_face]);
        sse.set(1, xmm1);
        pc.cpu.set_sse(sse);
        set_mm(&pc, &[(1, mm1)]);
        run(&pc, code);
        pc
    }

    /// Two doubleword lanes.
    fn pair(lo: i32, hi: i32) -> u64 {
        u64::from(lo as u32) | (u64::from(hi as u32) << 32)
    }

    #[test]
    fn cvtpi2ps_and_cvtpi2pd_widen_two_doublewords() {
        // cvtpi2ps xmm0, mm1: [3, -2] -> [3.0f, -2.0f] in the low quadword,
        // the upper two singles kept.
        let pc = convert(&[0x0f, 0x2a, 0xc1], pair(3, -2), [0; 2]);
        assert_eq!(
            pc.cpu.sse().get(0),
            [0xc000_0000_4040_0000, 0xfeed_face_feed_face]
        );
        // cvtpi2pd xmm0, mm1: the whole register.
        let pc = convert(&[0x66, 0x0f, 0x2a, 0xc1], pair(3, -2), [0; 2]);
        assert_eq!(
            pc.cpu.sse().get(0),
            [0x4008_0000_0000_0000, 0xc000_0000_0000_0000]
        );
        // 2^24 + 1 is inexact as a single, and says so.
        let pc = convert(&[0x0f, 0x2a, 0xc1], pair(16_777_217, 0), [0; 2]);
        assert_eq!(pc.cpu.sse().low(0) as u32, 0x4b80_0000);
        assert_eq!(pc.cpu.sse().mxcsr & mxcsr::EXCEPTIONS, mxcsr::PE);
    }

    #[test]
    fn cvtps2pi_rounds_and_cvttps2pi_truncates() {
        // [1.5, -2.5]: to nearest even is [2, -2]; truncated, [1, -2].
        let src = [0xc020_0000_3fc0_0000, 0];
        let pc = convert(&[0x0f, 0x2d, 0xc1], 0, src);
        assert_eq!(mm(&pc, 0), pair(2, -2));
        let pc = convert(&[0x0f, 0x2c, 0xc1], 0, src);
        assert_eq!(mm(&pc, 0), pair(1, -2));
        // A NaN is the integer indefinite, and invalid.
        let pc = convert(&[0x0f, 0x2c, 0xc1], 0, [0x7fc0_0000, 0]);
        assert_eq!(mm(&pc, 0), 0x8000_0000);
        assert_ne!(pc.cpu.sse().mxcsr & mxcsr::IE, 0);
    }

    #[test]
    fn cvtpd2pi_rounds_and_cvttpd2pi_truncates() {
        // [2.5, -3.7]: to nearest [2, -4]; truncated [2, -3].
        let src = [0x4004_0000_0000_0000, 0xc00d_9999_9999_999a];
        let pc = convert(&[0x66, 0x0f, 0x2d, 0xc1], 0, src);
        assert_eq!(mm(&pc, 0), pair(2, -4));
        let pc = convert(&[0x66, 0x0f, 0x2c, 0xc1], 0, src);
        assert_eq!(mm(&pc, 0), pair(2, -3));
        // Its memory operand is sixteen bytes and must be aligned.
        let mut code = alloc::vec![0x66];
        code.extend(mem(0x2d, 0, B));
        faults(&mmxpc(), 13, &code);
    }

    #[test]
    fn the_conversions_make_the_transition_only_when_they_name_an_mm_register() {
        // cvtpi2ps xmm0, mm1 after fninit ; fld1: TOP goes to zero.
        let pc = mmxpc();
        run(&pc, &[0xdb, 0xe3, 0xd9, 0xe8, 0x0f, 0x2a, 0xc1]);
        assert_eq!((pc.cpu.x87().top(), pc.cpu.x87().tag), (0, 0));
        // cvtpi2ps xmm0, [A]: no `MM` register, no transition.
        let pc = mmxpc();
        let mut code = alloc::vec![0xdb, 0xe3, 0xd9, 0xe8];
        code.extend(mem(0x2a, 0, A));
        run(&pc, &code);
        assert_eq!(pc.cpu.x87().top(), 7);
        // cvtps2pi mm0, xmm1 writes an `MM` register: it does.
        let pc = mmxpc();
        run(&pc, &[0xdb, 0xe3, 0xd9, 0xe8, 0x0f, 0x2d, 0xc1]);
        assert_eq!(pc.cpu.x87().top(), 0);
        assert_eq!(pc.cpu.x87().regs[0].sign_exp, 0xffff);
    }

    #[test]
    fn an_unmasked_conversion_exception_faults_and_leaves_mm_alone() {
        let pc = mmxpc();
        let mut sse = Sse::new();
        sse.set(1, [0x7fc0_0000, 0]);
        sse.mxcsr = mxcsr::RESET & !mxcsr::IM;
        pc.cpu.set_sse(sse);
        set_mm(&pc, &[(0, 0x1234)]);
        faults(&pc, 19, &[0x0f, 0x2c, 0xc1]); // cvttps2pi mm0, xmm1
        assert_eq!(mm(&pc, 0), 0x1234);
    }

    // -- The save formats carry the MMX state ------------------------------

    #[test]
    fn fxsave_holds_the_mm_registers_and_fxrstor_brings_them_back() {
        let pc = mmxpc();
        write64(&pc, A, 0x0123_4567_89ab_cdef);
        // movq mm2, [A] ; fxsave [SAVE]
        let mut code = mem(0x6f, 2, A);
        code.extend(mem(0xae, 0, SAVE));
        run(&pc, &code);
        // `ST(2)` is at 32 + 2 * 16, and with `TOP` zero it is `R2`: the
        // significand, then the all-ones exponent.
        assert_eq!(read64(&pc, SAVE + 64), 0x0123_4567_89ab_cdef);
        assert_eq!(read64(&pc, SAVE + 72) & 0xffff, 0xffff);
        assert_eq!(
            pc.ram.read_u8(SAVE + 4).unwrap(),
            0xff,
            "abridged tag: all full"
        );
        assert_eq!((read64(&pc, SAVE) >> 16) & u64::from(sw::TOP), 0, "TOP");

        // The image into a fresh core: fxrstor [SAVE] brings the register
        // back, and the tag word with it — full, and `Special` because of
        // the all-ones exponent.
        let fresh = mmxpc();
        for i in 0..512 {
            let byte = pc.ram.read_u8(SAVE + i).unwrap();
            fresh.ram.write_u8(SAVE + i, byte).unwrap();
        }
        run(&fresh, &mem(0xae, 1, SAVE));
        let x = fresh.cpu.x87();
        assert_eq!(x.mm(2), 0x0123_4567_89ab_cdef);
        assert_eq!(x.regs[2].sign_exp, 0xffff);
        assert_eq!((x.top(), x.tag_at(2)), (0, Tag::Special));
    }

    #[test]
    fn fsave_holds_the_mm_registers_too() {
        let pc = mmxpc();
        set_mm(&pc, &[(0, 0x1111), (1, 0xffff), (7, 0x7777)]);
        // pxor mm1, mm1 ; fnsave [SAVE] — straight after an MMX instruction,
        // with no `EMMS`, the way a task switch would catch it.
        let mut code = alloc::vec![0x0f, 0xef, 0xc9];
        code.push(0xdd);
        code.extend(disp32(6, SAVE));
        run(&pc, &code);
        // The 32-bit image: 28 bytes of environment, then ten per register.
        assert_eq!(read64(&pc, SAVE + 8) & 0xffff, 0, "tag word: all valid");
        assert_eq!(read64(&pc, SAVE + 28), 0x1111);
        assert_eq!(read64(&pc, SAVE + 28 + 8) & 0xffff, 0xffff);
        assert_eq!(read64(&pc, SAVE + 28 + 10), 0, "pxor wrote mm1");
        assert_eq!(read64(&pc, SAVE + 28 + 70), 0x7777);
    }

    #[test]
    fn the_mmx_state_survives_a_snapshot() {
        // The registers are the x87 unit's, so the core's own save/load
        // carries them; this is the round trip CLAUDE.md asks of a device.
        let pc = mmxpc();
        set_mm(&pc, &[(4, 0xfeed_beef_cafe_f00d)]);
        let before = pc.cpu.x87();
        let mut shape = MachineShape::new();
        shape.add_device("/cpu0", "cpu.x86").unwrap();
        let mut writer = StateWriter::new(shape);
        {
            let mut chunk = writer
                .chunk("/cpu0", "cpu.x86", crate::cpu::x86::CLASS.version)
                .unwrap();
            pc.cpu.save(&mut chunk).unwrap();
        }
        let bytes = writer.to_vec().unwrap();
        let restored = mmxpc();
        let reader = crate::core::state::StateReader::new(&bytes).unwrap();
        let (_, _, data) = reader.load_raw("/cpu0").unwrap();
        let mut chunk = ChunkReader::new(data);
        restored.cpu.load(&mut chunk).unwrap();
        chunk.end().unwrap();
        assert_eq!(restored.cpu.x87(), before);
        assert_eq!(restored.cpu.x87().mm(4), 0xfeed_beef_cafe_f00d);
    }

    // -- The disassembler -------------------------------------------------

    #[test]
    fn every_mmx_encoding_disassembles_with_mm_register_names() {
        let pc = mmxpc();
        let listing = |bytes: &[u8]| {
            pc.write(at::CODE0, bytes);
            let out = pc.cpu.disassemble(0x08, at::CODE0, 1);
            alloc::format!("{}", out[0])
        };
        for (op, name) in [
            (0x60, "punpcklbw"),
            (0x61, "punpcklwd"),
            (0x62, "punpckldq"),
            (0x63, "packsswb"),
            (0x64, "pcmpgtb"),
            (0x65, "pcmpgtw"),
            (0x66, "pcmpgtd"),
            (0x67, "packuswb"),
            (0x68, "punpckhbw"),
            (0x69, "punpckhwd"),
            (0x6a, "punpckhdq"),
            (0x6b, "packssdw"),
            (0x6f, "movq"),
            (0x74, "pcmpeqb"),
            (0x75, "pcmpeqw"),
            (0x76, "pcmpeqd"),
            (0xd1, "psrlw"),
            (0xd2, "psrld"),
            (0xd3, "psrlq"),
            (0xd4, "paddq"),
            (0xd5, "pmullw"),
            (0xd8, "psubusb"),
            (0xd9, "psubusw"),
            (0xda, "pminub"),
            (0xdb, "pand"),
            (0xdc, "paddusb"),
            (0xdd, "paddusw"),
            (0xde, "pmaxub"),
            (0xdf, "pandn"),
            (0xe0, "pavgb"),
            (0xe1, "psraw"),
            (0xe2, "psrad"),
            (0xe3, "pavgw"),
            (0xe4, "pmulhuw"),
            (0xe5, "pmulhw"),
            (0xe8, "psubsb"),
            (0xe9, "psubsw"),
            (0xea, "pminsw"),
            (0xeb, "por"),
            (0xec, "paddsb"),
            (0xed, "paddsw"),
            (0xee, "pmaxsw"),
            (0xef, "pxor"),
            (0xf1, "psllw"),
            (0xf2, "pslld"),
            (0xf3, "psllq"),
            (0xf4, "pmuludq"),
            (0xf5, "pmaddwd"),
            (0xf6, "psadbw"),
            (0xf7, "maskmovq"),
            (0xf8, "psubb"),
            (0xf9, "psubw"),
            (0xfa, "psubd"),
            (0xfb, "psubq"),
            (0xfc, "paddb"),
            (0xfd, "paddw"),
            (0xfe, "paddd"),
        ] {
            let text = listing(&[0x0f, op, 0xc1]);
            let want = alloc::format!("{name} mm0, mm1");
            assert!(text.ends_with(&want), "0f {op:02x}: {text}");
        }
        for (bytes, want) in [
            (&[0x0f, 0x77][..], "emms"),
            (&[0x0f, 0x6e, 0xc8], "movd mm1, eax"),
            (&[0x0f, 0x7e, 0xc8], "movd eax, mm1"),
            (&[0x0f, 0x7f, 0xc1], "movq mm1, mm0"),
            (&[0x0f, 0x70, 0xc1, 0x1b], "pshufw mm0, mm1, 0x1b"),
            (&[0x0f, 0x71, 0xd1, 0x04], "psrlw mm1, 0x4"),
            (&[0x0f, 0x71, 0xe1, 0x04], "psraw mm1, 0x4"),
            (&[0x0f, 0x71, 0xf1, 0x04], "psllw mm1, 0x4"),
            (&[0x0f, 0x72, 0xd1, 0x04], "psrld mm1, 0x4"),
            (&[0x0f, 0x72, 0xe1, 0x04], "psrad mm1, 0x4"),
            (&[0x0f, 0x72, 0xf1, 0x04], "pslld mm1, 0x4"),
            (&[0x0f, 0x73, 0xd1, 0x04], "psrlq mm1, 0x4"),
            (&[0x0f, 0x73, 0xf1, 0x04], "psllq mm1, 0x4"),
            (&[0x0f, 0xc4, 0xc0, 0x01], "pinsrw mm0, ax, 0x1"),
            (&[0x0f, 0xc5, 0xc1, 0x02], "pextrw eax, mm1, 0x2"),
            (&[0x0f, 0xd7, 0xc1], "pmovmskb eax, mm1"),
            (
                &[0x0f, 0xe7, 0x05, 0, 0xc0, 0, 0],
                "movntq qword [ds:0xc000], mm0",
            ),
            (
                &[0x0f, 0x6f, 0x05, 0, 0xb0, 0, 0],
                "movq mm0, qword [ds:0xb000]",
            ),
            (
                &[0x0f, 0x60, 0x05, 0, 0xb0, 0, 0],
                "punpcklbw mm0, dword [ds:0xb000]",
            ),
            (&[0xf3, 0x0f, 0xd6, 0xc1], "movq2dq xmm0, mm1"),
            (&[0xf2, 0x0f, 0xd6, 0xc1], "movdq2q mm0, xmm1"),
            (&[0x0f, 0x2a, 0xc1], "cvtpi2ps xmm0, mm1"),
            (&[0x0f, 0x2c, 0xc1], "cvttps2pi mm0, xmm1"),
            (&[0x0f, 0x2d, 0xc1], "cvtps2pi mm0, xmm1"),
            (&[0x66, 0x0f, 0x2a, 0xc1], "cvtpi2pd xmm0, mm1"),
            (&[0x66, 0x0f, 0x2c, 0xc1], "cvttpd2pi mm0, xmm1"),
            (&[0x66, 0x0f, 0x2d, 0xc1], "cvtpd2pi mm0, xmm1"),
        ] {
            let text = listing(bytes);
            assert!(text.ends_with(want), "{bytes:02x?}: {text}");
        }
    }

    #[test]
    fn rex_widens_movd_and_never_extends_an_mm_register() {
        use crate::cpu::x86::disasm::disassemble_as;
        use crate::cpu::x86::isa::{Bits, Gen};
        let text =
            |bytes: &[u8]| alloc::format!("{}", disassemble_as(Gen::I386, Bits::B64, 0, 0, bytes));
        assert!(text(&[0x48, 0x0f, 0x6e, 0xc0]).ends_with("movq mm0, rax"));
        assert!(text(&[0x48, 0x0f, 0x7e, 0xc1]).ends_with("movq rcx, mm0"));
        // `REX.R` and `REX.B` have nothing to extend: `mm9` does not exist.
        assert!(text(&[0x45, 0x0f, 0xfc, 0xc1]).ends_with("paddb mm0, mm1"));
        assert!(text(&[0x44, 0x0f, 0x6e, 0xc0]).ends_with("movd mm0, eax"));
        assert!(text(&[0x41, 0x0f, 0x6e, 0xc0]).ends_with("movd mm0, r8d"));
    }

    #[test]
    fn a_mandatory_prefix_the_maps_do_not_list_is_still_invalid() {
        // `F3`/`F2` in front of an MMX opcode is not the MMX instruction with
        // the prefix ignored, and `66 0F 77` is not `EMMS`.
        for bytes in [
            &[0xf3, 0x0f, 0xfc, 0xc1][..],
            &[0xf2, 0x0f, 0x6f, 0xc1],
            &[0x66, 0x0f, 0x77],
            &[0xf3, 0x0f, 0xf7, 0xc1],
        ] {
            faults(&mmxpc(), 6, bytes);
        }
        // And `66` still selects the XMM form, not the MMX one.
        let pc = mmxpc();
        run(&pc, &[0x66, 0x0f, 0xfc, 0xc1]);
        assert_eq!(pc.cpu.x87().tag, 0xffff, "paddb xmm made no transition");
    }
}

// ---------------------------------------------------------------------------
// x86-64-v2: SSSE3, SSE4.1, SSE4.2, POPCNT
// ---------------------------------------------------------------------------

mod v2 {
    use super::*;
    use crate::cpu::x86::prot::cr4;

    /// Scratch memory, sixteen-byte aligned.
    const DATA: u64 = 0xb000;
    /// Where a result is stored.
    const OUT: u64 = 0xc000;
    /// Where a fault handler sits, and `RIP` once its `hlt` has retired.
    const HANDLER: u64 = 0x3800;
    const HANDLED: u64 = HANDLER + 1;

    /// An x86-64 part with `features`, in 32-bit protected mode with
    /// `CR4.OSFXSR` and `OSXMMEXCPT` set.
    fn pc_with(features: Features) -> Pc {
        let pc = Pc::with_features(Variant::X86_64, features);
        pc.start_protected();
        let mut sys = pc.cpu.sys();
        sys.cr4 |= cr4::OSFXSR | cr4::OSXMMEXCPT;
        pc.cpu.set_sys(sys);
        pc
    }

    /// Everything this module tests, switched on.
    fn all() -> Features {
        Features {
            sse3: true,
            ssse3: true,
            sse41: true,
            sse42: true,
            popcnt: true,
            ..Features::X86_64
        }
    }

    fn v2pc() -> Pc {
        pc_with(all())
    }

    /// `disp32` addressing for `reg`: `mod == 00`, `r/m == 101`.
    fn disp32(reg: u8, addr: u64) -> Vec<u8> {
        let mut out = alloc::vec![((reg & 7) << 3) | 0b101];
        out.extend_from_slice(&(addr as u32).to_le_bytes());
        out
    }

    /// `bytes` followed by a `disp32` ModRM naming `reg` and `addr`.
    fn mem(bytes: &[u8], reg: u8, addr: u64) -> Vec<u8> {
        let mut out = bytes.to_vec();
        out.extend_from_slice(&disp32(reg, addr));
        out
    }

    /// Run `code` followed by `hlt`, and require that the `hlt` is reached.
    fn run(pc: &Pc, code: &[u8]) {
        let mut all = code.to_vec();
        all.push(0xf4);
        pc.write(at::CODE0, &all);
        assert!(pc.run(400) < 400, "the program reached its hlt");
        assert_eq!(pc.regs().rip, at::CODE0 + all.len() as u64, "and no fault");
    }

    /// Run `code` and require that it raises `vector`, handled by a `hlt`.
    fn faults(pc: &Pc, vector: u64, code: &[u8]) {
        pc.idt(vector, gate(0x08, HANDLER as u32, sys_type::INT_GATE32, 0));
        pc.write(HANDLER, &[0xf4]);
        let mut all = code.to_vec();
        all.push(0xf4);
        pc.write(at::CODE0, &all);
        assert!(pc.run(200) < 200);
        assert_eq!(pc.regs().rip, HANDLED, "{code:02x?} raises vector {vector}");
    }

    fn write64(pc: &Pc, addr: u64, value: u64) {
        for i in 0..8u64 {
            pc.ram.write_u8(addr + i, (value >> (8 * i)) as u8).unwrap();
        }
    }

    fn write128(pc: &Pc, addr: u64, v: [u64; 2]) {
        write64(pc, addr, v[0]);
        write64(pc, addr + 8, v[1]);
    }

    /// Load `XMM` registers directly.
    fn set_xmm(pc: &Pc, values: &[(u8, [u64; 2])]) {
        let mut sse = pc.cpu.sse();
        for &(n, v) in values {
            sse.set(n, v);
        }
        pc.cpu.set_sse(sse);
    }

    fn xmm(pc: &Pc, n: u8) -> [u64; 2] {
        pc.cpu.sse().get(n)
    }

    /// `CPUID.01H:ECX` on a part with `features`.
    fn leaf1_ecx(features: Features) -> u64 {
        let pc = pc_with(features);
        run(&pc, &[0xb8, 0x01, 0x00, 0x00, 0x00, 0x0f, 0xa2]);
        pc.regs().rcx & 0xffff_ffff
    }

    // -- SSSE3 -------------------------------------------------------------

    #[test]
    fn pshufb_reverses_a_register_from_memory() {
        // pshufb xmm0, [DATA] with the control 15, 14, ..., 0.
        let pc = v2pc();
        set_xmm(&pc, &[(0, [0x0706_0504_0302_0100, 0x0f0e_0d0c_0b0a_0908])]);
        write128(&pc, DATA, [0x0809_0a0b_0c0d_0e0f, 0x0001_0203_0405_0607]);
        run(&pc, &mem(&[0x66, 0x0f, 0x38, 0x00], 0, DATA));
        assert_eq!(xmm(&pc, 0), [0x0809_0a0b_0c0d_0e0f, 0x0001_0203_0405_0607]);
    }

    #[test]
    fn palignr_takes_its_shift_from_the_immediate() {
        // palignr xmm1, xmm2, 8: the source's high half, then the
        // destination's low half.
        let pc = v2pc();
        set_xmm(&pc, &[(1, [0x1111, 0x2222]), (2, [0x3333, 0x4444])]);
        run(&pc, &[0x66, 0x0f, 0x3a, 0x0f, 0xca, 0x08]);
        assert_eq!(xmm(&pc, 1), [0x4444, 0x1111]);
    }

    #[test]
    fn the_mm_forms_alias_the_x87_registers() {
        // phaddw mm0, mm1 on an MM register: the MMX transition happens
        // (TOP to 0, every tag valid) and the result's exponent field is all
        // ones, exactly as for the original MMX set.
        let pc = v2pc();
        let mut x = pc.cpu.x87();
        x.set_mm(0, 0x0004_0003_0002_0001);
        x.set_mm(1, 0x0040_0030_0020_0010);
        x.set_top(3);
        pc.cpu.set_x87(x);
        run(&pc, &[0x0f, 0x38, 0x01, 0xc1]);
        let x = pc.cpu.x87();
        assert_eq!(x.mm(0), 0x0070_0030_0007_0003);
        assert_eq!(x.top(), 0);
        assert_eq!(x.raw(0).sign_exp, 0xffff);
        // palignr mm2, [DATA], 3.
        let pc = v2pc();
        let mut x = pc.cpu.x87();
        x.set_mm(2, 0x1716_1514_1312_1110);
        pc.cpu.set_x87(x);
        write64(&pc, DATA, 0x0706_0504_0302_0100);
        let mut code = mem(&[0x0f, 0x3a, 0x0f], 2, DATA);
        code.push(3);
        run(&pc, &code);
        assert_eq!(pc.cpu.x87().mm(2), 0x1211_1007_0605_0403);
    }

    #[test]
    fn ssse3_is_undefined_without_it_in_both_register_files() {
        let without = Features {
            sse3: true,
            ..Features::X86_64
        };
        faults(&pc_with(without), 6, &[0x66, 0x0f, 0x38, 0x00, 0xc1]);
        faults(&pc_with(without), 6, &[0x0f, 0x38, 0x00, 0xc1]);
        faults(&pc_with(without), 6, &[0x0f, 0x3a, 0x0f, 0xc1, 0x01]);
        assert_eq!(leaf1_ecx(without) & (1 << 9), 0);
        assert_ne!(leaf1_ecx(all()) & (1 << 9), 0, "CPUID.01H:ECX.SSSE3");
        // And it needs SSE3 under it.
        let impossible = Features {
            ssse3: true,
            ..Features::X86_64
        };
        assert!(impossible.validate().is_err());
    }

    // -- SSE4.1 ------------------------------------------------------------

    fn set_regs(pc: &Pc, f: impl FnOnce(&mut Regs)) {
        let mut regs = pc.cpu.regs();
        f(&mut regs);
        pc.cpu.set_regs(regs);
    }

    fn read64(pc: &Pc, addr: u64) -> u64 {
        (0..8u64).fold(0, |v, i| {
            v | (u64::from(pc.ram.read_u8(addr + i).unwrap()) << (8 * i))
        })
    }

    #[test]
    fn pextrb_and_pinsrb_reach_a_whole_register_or_one_byte() {
        let pc = v2pc();
        set_xmm(&pc, &[(1, [0x0706_0504_0302_0100, 0x0f0e_0d0c_0b0a_0908])]);
        set_regs(&pc, |r| r.rax = 0xffff_ffff);
        write64(&pc, OUT, 0x5555_5555_5555_5555);
        let mut code = Vec::new();
        // pextrb eax, xmm1, 13: the register is written whole, zero-extended.
        code.extend_from_slice(&[0x66, 0x0f, 0x3a, 0x14, 0xc8, 13]);
        // pextrb [OUT], xmm1, 9: one byte, the rest left alone.
        code.extend_from_slice(&mem(&[0x66, 0x0f, 0x3a, 0x14], 1, OUT));
        code.push(9);
        // mov ecx, 0x1234_56aa ; pinsrb xmm1, ecx, 0: the low byte only.
        code.extend_from_slice(&[0xb9, 0xaa, 0x56, 0x34, 0x12]);
        code.extend_from_slice(&[0x66, 0x0f, 0x3a, 0x20, 0xc9, 0]);
        // pinsrb xmm1, [OUT+1], 15: a byte of memory.
        code.extend_from_slice(&mem(&[0x66, 0x0f, 0x3a, 0x20], 1, OUT + 1));
        code.push(15);
        // pextrw [OUT+8], xmm1, 7: SSE4.1's form, which can store.
        code.extend_from_slice(&mem(&[0x66, 0x0f, 0x3a, 0x15], 1, OUT + 8));
        code.push(7);
        run(&pc, &code);
        assert_eq!(pc.regs().rax, 0x0d);
        assert_eq!(read64(&pc, OUT), 0x5555_5555_5555_5509);
        assert_eq!(xmm(&pc, 1), [0x0706_0504_0302_01aa, 0x550e_0d0c_0b0a_0908]);
        assert_eq!(read64(&pc, OUT + 8), 0x550e, "two bytes and no more");
    }

    #[test]
    fn extractps_and_insertps_move_one_single() {
        let pc = v2pc();
        set_xmm(
            &pc,
            &[
                (0, [0x2222_2222_1111_1111, 0x4444_4444_3333_3333]),
                (1, [0xbbbb_bbbb_aaaa_aaaa, 0xdddd_dddd_cccc_cccc]),
            ],
        );
        write64(&pc, DATA, 0x9999_9999);
        let mut code = Vec::new();
        // extractps edx, xmm0, 2
        code.extend_from_slice(&[0x66, 0x0f, 0x3a, 0x17, 0xc2, 2]);
        // insertps xmm0, xmm1, 0b11_01_0001: lane 3 of xmm1 into lane 1,
        // then zero lane 0.
        code.extend_from_slice(&[0x66, 0x0f, 0x3a, 0x21, 0xc1, 0b1101_0001]);
        // insertps xmm1, [DATA], 0b11_10_0000: memory ignores imm[7:6].
        code.extend_from_slice(&mem(&[0x66, 0x0f, 0x3a, 0x21], 1, DATA));
        code.push(0b1110_0000);
        run(&pc, &code);
        assert_eq!(pc.regs().rdx, 0x3333_3333);
        assert_eq!(xmm(&pc, 0), [0xdddd_dddd_0000_0000, 0x4444_4444_3333_3333]);
        assert_eq!(xmm(&pc, 1), [0xbbbb_bbbb_aaaa_aaaa, 0xdddd_dddd_9999_9999]);
    }

    #[test]
    fn ptest_sets_zf_and_cf_and_clears_the_rest() {
        // (dst, src, ZF, CF): ZF when the AND is zero, CF when src AND NOT
        // dst is.
        for (dst, src, zf, cf) in [
            ([0xf0u64, 0], [0x0fu64, 0], true, false),
            ([0xff, 0], [0x0f, 0], false, true),
            ([0, 0], [0, 0], true, true),
            ([0, 1], [0, 3], false, false),
        ] {
            let pc = v2pc();
            set_xmm(&pc, &[(2, dst), (3, src)]);
            // Every arithmetic flag set first, so the clears are visible.
            set_regs(&pc, |r| r.eflags |= 0x8d5);
            run(&pc, &[0x66, 0x0f, 0x38, 0x17, 0xd3]); // ptest xmm2, xmm3
            let e = pc.regs().eflags;
            assert_eq!(e & flags::ZF != 0, zf, "{dst:x?} {src:x?}");
            assert_eq!(e & flags::CF != 0, cf, "{dst:x?} {src:x?}");
            assert_eq!(e & (flags::AF | flags::OF | flags::PF | flags::SF), 0);
        }
    }

    #[test]
    fn the_variable_blends_read_xmm0_without_naming_it() {
        let pc = v2pc();
        set_xmm(
            &pc,
            &[(0, [0x8000_0000_0000_0000, 0]), (1, [1, 2]), (2, [3, 4])],
        );
        run(&pc, &[0x66, 0x0f, 0x38, 0x15, 0xca]); // blendvpd xmm1, xmm2, xmm0
        assert_eq!(xmm(&pc, 1), [3, 2]);
    }

    #[test]
    fn pmovsxbq_reads_two_bytes_of_memory() {
        let pc = v2pc();
        write64(&pc, DATA, 0x7777_7777_7777_80ff);
        run(&pc, &mem(&[0x66, 0x0f, 0x38, 0x22], 4, DATA));
        assert_eq!(xmm(&pc, 4), [u64::MAX, 0xffff_ffff_ffff_ff80]);
        let pc = v2pc();
        write64(&pc, DATA, 0x7777_7777_7777_80ff);
        run(&pc, &mem(&[0x66, 0x0f, 0x38, 0x32], 4, DATA));
        assert_eq!(xmm(&pc, 4), [0xff, 0x80]);
    }

    #[test]
    fn roundps_follows_mxcsr_when_the_immediate_says_so() {
        // roundps xmm0, [DATA], 4 with MXCSR.RC = down: [2.5, -2.5] become
        // [2, -3].
        let pc = v2pc();
        let mut sse = pc.cpu.sse();
        sse.mxcsr |= 1 << 13;
        pc.cpu.set_sse(sse);
        write128(&pc, DATA, [0xc020_0000_4020_0000, 0]);
        let mut code = mem(&[0x66, 0x0f, 0x3a, 0x08], 0, DATA);
        code.push(4);
        run(&pc, &code);
        assert_eq!(xmm(&pc, 0), [0xc040_0000_4000_0000, 0]);
        assert_ne!(pc.cpu.sse().mxcsr & (1 << 5), 0, "PE");
    }

    #[test]
    fn movntdqa_is_an_aligned_load_with_no_register_form() {
        let pc = v2pc();
        write128(&pc, DATA, [1, 2]);
        run(&pc, &mem(&[0x66, 0x0f, 0x38, 0x2a], 5, DATA));
        assert_eq!(xmm(&pc, 5), [1, 2]);
        faults(&v2pc(), 13, &mem(&[0x66, 0x0f, 0x38, 0x2a], 5, DATA + 8));
        faults(&v2pc(), 6, &[0x66, 0x0f, 0x38, 0x2a, 0xc1]);
    }

    #[test]
    fn sse41_is_undefined_without_it() {
        let without = Features {
            sse3: true,
            ssse3: true,
            ..Features::X86_64
        };
        faults(&pc_with(without), 6, &[0x66, 0x0f, 0x38, 0x17, 0xc1]); // ptest
        faults(&pc_with(without), 6, &[0x66, 0x0f, 0x3a, 0x0e, 0xc1, 0]); // pblendw
        // The SSE4.1 PEXTRW needs SSE4.1; SSE2's does not.
        faults(&pc_with(without), 6, &[0x66, 0x0f, 0x3a, 0x15, 0xc0, 0]);
        run(&pc_with(without), &[0x66, 0x0f, 0xc5, 0xc0, 0]);
        assert_eq!(leaf1_ecx(without) & (1 << 19), 0);
        assert_ne!(leaf1_ecx(all()) & (1 << 19), 0, "CPUID.01H:ECX.SSE4_1");
        let impossible = Features {
            sse3: true,
            sse41: true,
            ..Features::X86_64
        };
        assert!(impossible.validate().is_err());
    }

    // -- SSE4.2 ------------------------------------------------------------

    /// `F2 0F 38 F1 /r`, or `F0` for a byte source, at `addr`, into `EAX`.
    fn crc32_mem(code: &mut Vec<u8>, prefix66: bool, opcode: u8, addr: u64) {
        if prefix66 {
            code.push(0x66);
        }
        code.extend_from_slice(&mem(&[0xf2, 0x0f, 0x38, opcode], 0, addr));
    }

    #[test]
    fn crc32_accumulates_castagnoli_in_every_width() {
        // The CRC-32C check value: "123456789" with an all-ones start and a
        // final inversion is 0xE3069283. Fed as two doublewords and a byte,
        // and again as a word, a doubleword, a word and a byte.
        for pieces in [
            &[(false, 0xf1u8, 0u64), (false, 0xf1, 4), (false, 0xf0, 8)][..],
            &[
                (true, 0xf1, 0),
                (false, 0xf1, 2),
                (true, 0xf1, 6),
                (false, 0xf0, 8),
            ],
        ] {
            let pc = v2pc();
            pc.write(DATA, b"123456789");
            let mut code = alloc::vec![0xb8, 0xff, 0xff, 0xff, 0xff]; // mov eax, -1
            for &(p66, opcode, at) in pieces {
                crc32_mem(&mut code, p66, opcode, DATA + at);
            }
            code.extend_from_slice(&[0xf7, 0xd0]); // not eax
            // The flags are not CRC32's business: set them all first.
            set_regs(&pc, |r| r.eflags |= 0x8d5);
            run(&pc, &code);
            assert_eq!(pc.regs().rax, 0xe306_9283, "{pieces:x?}");
            assert_eq!(pc.regs().eflags & 0x8d5, 0x8d5, "flags untouched");
        }
    }

    #[test]
    fn crc32_needs_only_its_cpuid_bit() {
        // It touches no SIMD state, so CR4.OSFXSR clear and CR0.EM set are
        // nothing to it.
        let pc = Pc::with_features(Variant::X86_64, all());
        pc.start_protected();
        let mut sys = pc.cpu.sys();
        sys.cr0 |= cr0::EM | cr0::TS;
        pc.cpu.set_sys(sys);
        run(&pc, &[0xf2, 0x0f, 0x38, 0xf0, 0xc1]); // crc32 eax, cl
        // And without the bit it is #UD.
        let without = Features {
            sse42: false,
            ..all()
        };
        faults(&pc_with(without), 6, &[0xf2, 0x0f, 0x38, 0xf0, 0xc1]);
    }

    #[test]
    fn pcmpistri_and_pcmpestrm_report_through_ecx_xmm0_and_the_flags() {
        // pcmpistri xmm1, xmm2, 0: the first vowel of "Hello, world!".
        let pc = v2pc();
        let text = |s: &[u8]| {
            let mut b = [0u8; 16];
            b[..s.len()].copy_from_slice(s);
            let mut lo = [0u8; 8];
            let mut hi = [0u8; 8];
            lo.copy_from_slice(&b[..8]);
            hi.copy_from_slice(&b[8..]);
            [u64::from_le_bytes(lo), u64::from_le_bytes(hi)]
        };
        set_xmm(&pc, &[(1, text(b"aeiou")), (2, text(b"Hello, world!"))]);
        set_regs(&pc, |r| r.eflags |= flags::AF | flags::PF);
        run(&pc, &[0x66, 0x0f, 0x3a, 0x63, 0xca, 0x00]);
        let r = pc.regs();
        assert_eq!(r.rcx, 1);
        let f = r.eflags;
        assert_ne!(f & flags::CF, 0, "a match");
        assert_ne!(f & flags::ZF, 0, "the second string ends in the register");
        assert_ne!(f & flags::SF, 0, "so does the first");
        assert_eq!(f & (flags::OF | flags::AF | flags::PF), 0);
        // imm[6]: the most significant match instead, the 'o' of "world".
        let pc = v2pc();
        set_xmm(&pc, &[(1, text(b"aeiou")), (2, text(b"Hello, world!"))]);
        run(&pc, &[0x66, 0x0f, 0x3a, 0x63, 0xca, 0x40]);
        assert_eq!(pc.regs().rcx, 8);

        // pcmpestrm xmm1, xmm2, 0x0c with EAX = -2 (a length of two: the
        // absolute value) and EDX = 16: "lo" as a substring, the mask in
        // XMM0, zero-extended.
        let pc = v2pc();
        set_xmm(
            &pc,
            &[
                (0, [!0, !0]),
                (1, text(b"lo")),
                (2, text(b"hello, worl lol")),
            ],
        );
        set_regs(&pc, |r| {
            r.rax = 0xffff_fffe;
            r.rdx = 16;
        });
        run(&pc, &[0x66, 0x0f, 0x3a, 0x60, 0xca, 0x0c]);
        // "hello, worl lol" is fifteen bytes and then a zero that EDX says
        // is part of the string: "lo" at 3 and at 12, and the 'l' at 14 is
        // followed by that zero, not by an 'o'.
        assert_eq!(xmm(&pc, 0), [(1 << 3) | (1 << 12), 0]);
        let f = pc.regs().eflags;
        assert_eq!(f & flags::ZF, 0, "EDX is not below sixteen");
        assert_ne!(f & flags::SF, 0, "|EAX| is");
    }

    #[test]
    fn pcmpgtq_is_a_signed_quadword_compare() {
        let pc = v2pc();
        set_xmm(&pc, &[(1, [1, u64::MAX]), (2, [0, 0])]);
        run(&pc, &[0x66, 0x0f, 0x38, 0x37, 0xca]);
        assert_eq!(xmm(&pc, 1), [u64::MAX, 0], "1 > 0 and -1 < 0");
    }

    #[test]
    fn sse42_is_undefined_without_it_and_cpuid_says_so() {
        let without = Features {
            sse42: false,
            ..all()
        };
        faults(&pc_with(without), 6, &[0x66, 0x0f, 0x38, 0x37, 0xca]);
        faults(&pc_with(without), 6, &[0x66, 0x0f, 0x3a, 0x63, 0xca, 0x00]);
        assert_eq!(leaf1_ecx(without) & (1 << 20), 0);
        assert_ne!(leaf1_ecx(all()) & (1 << 20), 0, "CPUID.01H:ECX.SSE4_2");
        let impossible = Features {
            sse42: true,
            ..Features::X86_64
        };
        assert!(impossible.validate().is_err());
    }

    // -- POPCNT ------------------------------------------------------------

    #[test]
    fn popcnt_counts_and_clears_every_flag_but_zf() {
        let pc = v2pc();
        set_regs(&pc, |r| {
            r.rcx = 0xf0f0_0001;
            r.rbx = 0x1234_ffff;
            r.eflags |= 0x8d5;
        });
        pc.write(DATA, &[0x07, 0x80]);
        let mut code = alloc::vec![0xf3, 0x0f, 0xb8, 0xc1]; // popcnt eax, ecx
        // popcnt bx, [DATA]: a `66` beside the `F3` is an operand size.
        code.extend_from_slice(&mem(&[0x66, 0xf3, 0x0f, 0xb8], 3, DATA));
        run(&pc, &code);
        let r = pc.regs();
        assert_eq!(r.rax, 9);
        assert_eq!(r.rbx, 0x1234_0004, "a 16-bit write");
        assert_eq!(r.eflags & 0x8d5, 0, "OF SF ZF AF PF CF all clear");
        // A zero source sets ZF and nothing else.
        let pc = v2pc();
        set_regs(&pc, |r| r.eflags |= 0x8d5);
        run(&pc, &[0x31, 0xc9, 0xf3, 0x0f, 0xb8, 0xc1]); // xor ecx, ecx ; popcnt
        assert_eq!(pc.regs().rax, 0);
        assert_eq!(pc.regs().eflags & 0x8d5, flags::ZF);
    }

    // -- The preset ----------------------------------------------------------

    #[test]
    fn the_v2_preset_reports_exactly_its_bits_and_the_baseline_none() {
        // SSE3, SSSE3, CX16, SSE4.1, SSE4.2, POPCNT — and not MONITOR (3).
        assert_eq!(
            leaf1_ecx(Features::X86_64_V2),
            1 | (1 << 9) | (1 << 13) | (1 << 19) | (1 << 20) | (1 << 23)
        );
        assert_eq!(leaf1_ecx(Features::X86_64), 0);
        assert!(Features::X86_64_V2.validate().is_ok());
    }

    #[test]
    fn each_v2_extension_is_undefined_on_a_part_without_it() {
        // One representative instruction per bit, on the v2 part less that
        // bit and everything built on it; the same bytes run on v2 itself.
        let v2 = Features::X86_64_V2;
        let cases: [(&str, Features, &[u8]); 6] = [
            (
                "sse3: haddps xmm0, xmm1",
                Features {
                    sse3: false,
                    ssse3: false,
                    sse41: false,
                    sse42: false,
                    ..v2
                },
                &[0xf2, 0x0f, 0x7c, 0xc1],
            ),
            (
                "ssse3: pshufb xmm0, xmm1",
                Features {
                    ssse3: false,
                    sse41: false,
                    sse42: false,
                    ..v2
                },
                &[0x66, 0x0f, 0x38, 0x00, 0xc1],
            ),
            (
                "sse41: ptest xmm0, xmm1",
                Features {
                    sse41: false,
                    sse42: false,
                    ..v2
                },
                &[0x66, 0x0f, 0x38, 0x17, 0xc1],
            ),
            (
                "sse42: pcmpgtq xmm0, xmm1",
                Features { sse42: false, ..v2 },
                &[0x66, 0x0f, 0x38, 0x37, 0xc1],
            ),
            (
                "sse42: crc32 eax, ecx",
                Features { sse42: false, ..v2 },
                &[0xf2, 0x0f, 0x38, 0xf1, 0xc1],
            ),
            (
                "popcnt: popcnt eax, ecx",
                Features {
                    popcnt: false,
                    ..v2
                },
                &[0xf3, 0x0f, 0xb8, 0xc1],
            ),
        ];
        for (name, without, code) in cases {
            assert!(without.validate().is_ok(), "{name}: a real part");
            let pc = pc_with(without);
            faults(&pc, 6, code);
            run(&pc_with(v2), code);
        }
    }

    #[test]
    fn popcnt_is_undefined_without_its_bit_and_cpuid_reports_it() {
        let without = Features {
            popcnt: false,
            ..all()
        };
        faults(&pc_with(without), 6, &[0xf3, 0x0f, 0xb8, 0xc1]);
        // Without the `F3` the encoding is nothing at all.
        faults(&v2pc(), 6, &[0x0f, 0xb8, 0xc1]);
        assert_eq!(leaf1_ecx(without) & (1 << 23), 0);
        assert_ne!(leaf1_ecx(all()) & (1 << 23), 0, "CPUID.01H:ECX.POPCNT");
        let impossible = Features {
            popcnt: true,
            ..Features::I80486
        };
        assert!(impossible.validate().is_err());
    }
}

/// AVX, in its own file: `tests/avx.rs`.
mod avx;

/// AVX2, FMA and F16C: `tests/avx2.rs`.
mod avx2;
/// BMI1 and BMI2, in their own file: `tests/bmi.rs`.
mod bmi;
