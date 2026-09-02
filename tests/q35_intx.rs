//! A PCI function's interrupt, from the card's pin to the processor's vector —
//! and what happens when the guest moves the routing under it.
//!
//! Everything else in this repository that touches `INTx#` stops one step
//! short: `tests/nvme_board.rs` watches an 8259A's interrupt request register,
//! `src/dev/q35/tests.rs` watches the router's output pin. This file runs the
//! whole path, with **real x86 instructions** doing the half a driver does:
//!
//! ```text
//!   NVMe completion queue           (a command the controller finished)
//!     -> INTA# on the function      (level, held while it is unacknowledged)
//!     -> the fabric's INTA# net     (device 4, so the swizzle is by zero)
//!     -> PIRQA on the ICH9          (the bridge collects the fabric's nets)
//!     -> PIRQ[A]_ROUT               (what the *guest* writes decides)
//!     -> the 8259A's IR5 or IR7     (level triggered, through the ELCR)
//!     -> the processor              (vector 0x0d or 0x0f)
//!     -> a handler that writes down which vector it was
//! ```
//!
//! # The claim worth the trouble
//!
//! The guest programs `PIRQ[A]_ROUT` to IRQ5, takes the interrupt, and then —
//! **with the same controller still asserting, having acknowledged nothing** —
//! reprograms the router to IRQ7 and takes the *same* interrupt again on a
//! different vector. That is only true of a model in which `INTx#` is a level
//! and the router re-derives its outputs from it; a model that latched an edge
//! when the completion arrived would deliver the first vector and never the
//! second, and would pass every test that only looked at the first.
//!
//! # Why this board and not `machines/q35.machine`
//!
//! Because the shipped q35 has no PCI function that raises an interrupt, and
//! putting an NVMe controller on it would make every q35 build link one. The
//! board below is the shipped board's chipset — the same `q35.mch`, `q35.lpc`
//! and `pc.pic` objects, wired the same way — with the controller added and
//! everything this file does not use left out.
//!
//! # Sources
//!
//! *PCI Local Bus Specification* Rev 2.1 §2.2.6 for `INTx#` being a shared,
//! level-sensitive line and §6.2.4 for the Interrupt Pin register;
//! *PCI-to-PCI Bridge Architecture Specification* Rev 1.1 §9.1 for the swizzle;
//! *Intel I/O Controller Hub 9 (ICH9) Family Datasheet* 316972-004 §13.1.17 for
//! `PIRQ[n]_ROUT`; *NVM Express* 1.4 §7.5.1.1 for the controller's pin;
//! the *Intel 8259A* data sheet for the initialisation sequence below.
//!
//! No emulator source and no firmware source was consulted (`CLAUDE.md`).

#![cfg(all(
    feature = "cpu-x86",
    feature = "dev-pc",
    feature = "dev-q35",
    feature = "dev-nvme"
))]

use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{Machine, build};

// ---------------------------------------------------------------------------
// the board
// ---------------------------------------------------------------------------

/// The q35 chipset, an 8259A, and one NVMe controller at device 4.
///
/// `pirq-routes` is deliberately **absent**, so every router comes up at
/// §13.1.17's own `80h` — not routed — and the guest below has to program one
/// before anything reaches a controller. That is the datasheet's power-up state
/// and it is what makes the first assertion below mean something.
const BOARD: &str = r#"
machine "q35-intx" {
  param ram = 15M

  osc cpu   = 25000000 Hz
  osc pmtmr = 315000000/88 Hz

  space mem  { width = 32, unassigned = read-as-ones }
  space port { width = 16, unassigned = read-as-ones }

  object cpu0 "cpu.x86" {
    clock   = cpu
    space   = mem
    iospace = "port"
    model   = "80486"
    engine  = "interp"
  }

  object ram_low  "ram" { size = 640K }
  object ram_high "ram" { size = ram }
  object bios "pc.rom" { image = "bios", size = 64K, align = "top" }

  object mch "q35.mch" { clock = cpu, space = mem, bus = "pci0" }
  object lpc "q35.lpc" {
    clock     = pmtmr
    bus       = "pci0"
    iospace   = "port"
    device-id = 0x2918
  }

  object pic1 "pc.pic" { mode = "master" }

  object nvme "nvme.controller" {
    space  = mem
    bus    = "pci0"
    device = 4
    image  = "nvme0"
    size   = 1M
  }

  map mem 0x00000000 size 640K = ram_low
  map mem 0x000f0000 size 64K  = bios
  map mem 0x00100000 size ram  = ram_high
  map mem 0x100000000 - 64K size 64K = bios

  map port 0x0020 size 0x0002 = pic1.regs
  map port 0x04d0 size 0x0001 = pic1.elcr
  map port 0x0cf8 size 0x0008 = mch.config

  wire pic1.int -> cpu0.intr
  wire lpc.irq5 -> pic1.ir5
  wire lpc.irq7 -> pic1.ir7
}
"#;

// ---------------------------------------------------------------------------
// the program
// ---------------------------------------------------------------------------

/// How big the ROM socket is, which the board above also declares.
const ROM_LEN: usize = 64 * 1024;

/// Where in the image the two interrupt handlers are assembled. The socket is
/// based at `0xf0000`, so an image offset is also an offset within segment
/// `f000`.
const HANDLER5: u16 = 0x0200;
const HANDLER7: u16 = 0x0220;

/// Where the handlers leave what they saw, in the board's low RAM.
const SAW5: u64 = 0x0500;
const SAW7: u64 = 0x0501;
const DONE: u64 = 0x0502;

/// `CONFADD` for 00:1f.0 register `0x60`, which is `PIRQ[A]_ROUT`.
const CONFADD_PIRQA: u32 = 0x8000_0000 | (31 << 11) | 0x60;

/// The firmware: sixteen-bit code that initialises an 8259A, installs two
/// interrupt handlers, programs the interrupt router, and waits.
///
/// Hand-assembled, as `tests/pc_apic.rs` is and for the same reason — the point
/// is that a *processor* executes it, so an assembler that ran at build time
/// would be one more thing between the claim and the machine.
fn firmware() -> Vec<u8> {
    let mut rom = vec![0xf4u8; ROM_LEN]; // `hlt` everywhere nothing is assembled

    let mut code: Vec<u8> = Vec::new();
    // cli; xor ax,ax; mov ds,ax; mov ss,ax; mov sp,0x7c00
    code.extend_from_slice(&[0xfa, 0x31, 0xc0, 0x8e, 0xd8, 0x8e, 0xd0, 0xbc, 0x00, 0x7c]);
    // The real-mode interrupt vector table. IRQ5 is vector 0x0d and IRQ7 is
    // vector 0x0f, because the 8259A below is programmed with a base of 0x08.
    let ivt = |vector: u16, offset: u16, code: &mut Vec<u8>| {
        let at = vector * 4;
        // mov word [at], offset
        code.extend_from_slice(&[0xc7, 0x06]);
        code.extend_from_slice(&at.to_le_bytes());
        code.extend_from_slice(&offset.to_le_bytes());
        // mov word [at + 2], 0xf000
        code.extend_from_slice(&[0xc7, 0x06]);
        code.extend_from_slice(&(at + 2).to_le_bytes());
        code.extend_from_slice(&0xf000u16.to_le_bytes());
    };
    ivt(0x0d, HANDLER5, &mut code);
    ivt(0x0f, HANDLER7, &mut code);

    // The 8259A, initialised the way the data sheet's ICW sequence says: ICW1
    // with ICW4 to follow, a vector base of 0x08, no slave on any input, and
    // 8086 mode.
    for (port, value) in [
        (0x20u8, 0x11u8),
        (0x21, 0x08),
        (0x21, 0x04),
        (0x21, 0x01),
        // OCW1: everything masked but IR5.
        (0x21, 0xdf),
    ] {
        code.extend_from_slice(&[0xb0, value, 0xe6, port]);
    }
    // The edge/level control register at 0x4d0: IR5 and IR7 are *levels*. A PCI
    // interrupt is a level (§2.2.6) and an edge-triggered input would latch the
    // first one and miss every later one raised while the line was still low.
    // 0x4d0 does not fit an immediate port, so it goes through DX.
    code.extend_from_slice(&[0xba, 0xd0, 0x04, 0xb0, 0xa0, 0xee]);

    /// Write one byte into `PIRQ[A]_ROUT` through the configuration ports.
    fn route(value: u8, code: &mut Vec<u8>) {
        // mov dx,0xcf8 ; mov eax,CONFADD ; out dx,eax
        code.extend_from_slice(&[0xba, 0xf8, 0x0c, 0x66, 0xb8]);
        code.extend_from_slice(&CONFADD_PIRQA.to_le_bytes());
        code.extend_from_slice(&[0x66, 0xef]);
        // mov dx,0xcfc ; mov al,value ; out dx,al
        code.extend_from_slice(&[0xba, 0xfc, 0x0c, 0xb0, value, 0xee]);
    }

    // Route PIRQA to IRQ5 and let the interrupt in. The controller is already
    // asserting by the time this runs, so the vector is taken immediately.
    route(0x05, &mut code);
    code.push(0xfb); // sti

    /// `cmp byte [at], 0` then `je` back to the compare: spin until it moves.
    fn spin(at: u64, code: &mut Vec<u8>) {
        code.extend_from_slice(&[0x80, 0x3e]);
        code.extend_from_slice(&(at as u16).to_le_bytes());
        code.extend_from_slice(&[0x00, 0x74, 0xf9]);
    }
    spin(SAW5, &mut code);

    // The interesting half: with the controller still asserting and nothing
    // acknowledged, move the router to IRQ7 and unmask only that input.
    route(0x07, &mut code);
    code.extend_from_slice(&[0xb0, 0x7f, 0xe6, 0x21]);
    spin(SAW7, &mut code);

    // mov byte [DONE],0xa5 ; jmp $
    code.extend_from_slice(&[0xc6, 0x06]);
    code.extend_from_slice(&(DONE as u16).to_le_bytes());
    code.extend_from_slice(&[0xa5, 0xeb, 0xfe]);
    assert!(
        code.len() < HANDLER5 as usize,
        "the program has grown into the first handler"
    );
    rom[..code.len()].copy_from_slice(&code);

    // A handler records the vector it was entered on, masks its own input —
    // the line is still asserted, so returning without masking would re-enter
    // for ever — acknowledges the controller, and returns.
    let handler = |vector: u8, at: u64| -> Vec<u8> {
        let mut out = vec![0x50u8]; // push ax
        out.extend_from_slice(&[0xb0, 0xff, 0xe6, 0x21]); // mov al,0xff; out 0x21,al
        out.extend_from_slice(&[0xc6, 0x06]); // mov byte [at], vector
        out.extend_from_slice(&(at as u16).to_le_bytes());
        out.push(vector);
        out.extend_from_slice(&[0xb0, 0x20, 0xe6, 0x20]); // mov al,0x20; out 0x20,al
        out.extend_from_slice(&[0x58, 0xcf]); // pop ax; iret
        out
    };
    let h5 = handler(0x0d, SAW5);
    rom[HANDLER5 as usize..HANDLER5 as usize + h5.len()].copy_from_slice(&h5);
    let h7 = handler(0x0f, SAW7);
    rom[HANDLER7 as usize..HANDLER7 as usize + h7.len()].copy_from_slice(&h7);

    // The reset vector: a 486 starts at 0xfffffff0 with `CS` based at
    // 0xffff0000, and this board maps the same socket there. `jmp f000:0000`
    // puts it in the other copy, in ordinary real mode.
    let reset = ROM_LEN - 0x10;
    rom[reset..reset + 5].copy_from_slice(&[0xea, 0x00, 0x00, 0x00, 0xf0]);
    rom
}

// ---------------------------------------------------------------------------
// building and driving
// ---------------------------------------------------------------------------

fn board() -> Machine {
    let mut options = rsemu::machine::catalog::build_options().expect("this build's options");
    options.realize.media.insert("bios", firmware());
    options.realize.media.insert("nvme0", Vec::new());
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut machine = match build("q35-intx.machine", BOARD, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    };
    machine.reset(ResetKind::Cold);
    machine.sweep();
    machine
}

fn peek8(m: &Machine, at: u64) -> u8 {
    m.space("mem")
        .expect("the memory space")
        .read(at, Width::U8, MemAttrs::DEFAULT)
        .expect("mapped RAM") as u8
}

fn peek32(m: &Machine, at: u64) -> u32 {
    m.space("mem")
        .expect("the memory space")
        .read(at, Width::U32, MemAttrs::DEFAULT)
        .expect("mapped memory") as u32
}

fn poke32(m: &Machine, at: u64, value: u32) {
    m.space("mem")
        .expect("the memory space")
        .write(at, Width::U32, u64::from(value), MemAttrs::DEFAULT)
        .expect("mapped memory");
}

fn outb(m: &Machine, port: u64, value: u8) {
    m.space("port")
        .expect("the I/O space")
        .write(port, Width::U8, u64::from(value), MemAttrs::DEFAULT)
        .expect("a decoded port");
}

fn inb(m: &Machine, port: u64) -> u8 {
    m.space("port")
        .expect("the I/O space")
        .read(port, Width::U8, MemAttrs::DEFAULT)
        .expect("a decoded port") as u8
}

/// A configuration write to the NVMe function at 00:04.0.
fn config_write(m: &Machine, register: u16, value: u32) {
    let address = 0x8000_0000u32 | (4 << 11) | u32::from(register & 0xfc);
    m.space("port")
        .expect("the I/O space")
        .write(0xcf8, Width::U32, u64::from(address), MemAttrs::DEFAULT)
        .expect("CONFADD");
    m.space("port")
        .expect("the I/O space")
        .write(0xcfc, Width::U32, u64::from(value), MemAttrs::DEFAULT)
        .expect("CONFDATA");
}

/// Where this test's driver puts the controller's register window, and where
/// its admin queues go in the board's RAM.
const BAR_BASE: u64 = 0xf000_0000;
const ASQ: u64 = 0x0010_0000;
const ACQ: u64 = 0x0010_1000;
const DATA: u64 = 0x0020_0000;
const ADMIN_ENTRIES: u32 = 4;

/// Bring the controller up and leave **one completion unacknowledged**, which
/// is what holds `INTA#` asserted (NVMe §7.5.1.1).
///
/// Everything here is what a driver does, through the board's own spaces: size
/// and place the base address register, switch the function on, build an admin
/// queue pair, set `CC.EN`, submit one `Identify` and ring the doorbell. The
/// completion queue head doorbell is deliberately *not* written.
fn raise_the_controllers_interrupt(m: &Machine) {
    // §6.2.5.1's sizing protocol, then the address, then Memory Space and Bus
    // Master in the Command register.
    config_write(m, 0x10, BAR_BASE as u32);
    config_write(m, 0x14, 0);
    config_write(m, 0x04, 0x0006);

    // §3.1.8: both queue sizes are zero-based.
    poke32(
        m,
        BAR_BASE + 0x24,
        (ADMIN_ENTRIES - 1) | ((ADMIN_ENTRIES - 1) << 16),
    );
    poke32(m, BAR_BASE + 0x28, ASQ as u32);
    poke32(m, BAR_BASE + 0x2c, 0);
    poke32(m, BAR_BASE + 0x30, ACQ as u32);
    poke32(m, BAR_BASE + 0x34, 0);
    // §3.1.5: the NVM command set, 4 KiB pages, 64-byte submission entries,
    // 16-byte completion entries, and go.
    poke32(m, BAR_BASE + 0x14, 1 | (6 << 17) | (4 << 21));
    assert_eq!(
        peek32(m, BAR_BASE + 0x1c) & 1,
        1,
        "the controller never came ready"
    );

    // One `Identify Controller` (§5.15): opcode 06h, CNS 1 in CDW10.
    let mut sqe = [0u8; 64];
    sqe[0..4].copy_from_slice(&0x0001_0006u32.to_le_bytes()); // opcode 6, cid 1
    sqe[24..32].copy_from_slice(&DATA.to_le_bytes());
    sqe[40..44].copy_from_slice(&1u32.to_le_bytes());
    m.space("mem")
        .expect("the memory space")
        .write_bytes(ASQ, &sqe, MemAttrs::DEFAULT)
        .expect("mapped RAM");
    poke32(m, BAR_BASE + 0x1000, 1); // the admin submission tail doorbell

    // The completion is there, and its phase bit says so.
    assert_ne!(
        peek32(m, ACQ + 12) & (1 << 16),
        0,
        "the controller did not complete the command"
    );
}

// ---------------------------------------------------------------------------
// the tests
// ---------------------------------------------------------------------------

#[test]
fn a_guest_takes_a_pci_interrupt_and_moving_the_router_moves_the_vector() {
    let mut machine = board();
    raise_the_controllers_interrupt(&machine);

    // Nothing has reached the 8259A: §13.1.17's power-up value is 80h, "the
    // PIRQ is not routed to the 8259", and no firmware has run yet. The
    // controller *is* asserting — the Status register's Interrupt Status bit
    // (Rev 3.0 §6.2.3) says so — so this is the routing being open, not the
    // interrupt being absent.
    outb(&machine, 0x20, 0x0a); // OCW3: the next read of port 0 is the IRR
    assert_eq!(inb(&machine, 0x20), 0, "the router routes nothing yet");

    machine
        .run_for(GlobalTime::from_nanos(20_000_000))
        .expect("the board runs");

    assert_eq!(
        peek8(&machine, DONE),
        0xa5,
        "the guest never got both interrupts: it saw {:#04x} then {:#04x}",
        peek8(&machine, SAW5),
        peek8(&machine, SAW7)
    );
    assert_eq!(
        peek8(&machine, SAW5),
        0x0d,
        "with PIRQA routed to IRQ5 the vector is the 8259A's base plus 5"
    );
    assert_eq!(
        peek8(&machine, SAW7),
        0x0f,
        "and the *same* unacknowledged interrupt, after the guest moved the \
         router to IRQ7, arrives as vector 0x0f"
    );
}

#[test]
fn acknowledging_the_controller_releases_the_line_the_router_is_holding() {
    // The other direction, and the one an edge-triggered model gets away with
    // until two functions share a line: what the router drives is a *level*,
    // and it goes away when the controller's own condition does — not when the
    // processor acknowledges anything.
    let mut machine = board();
    raise_the_controllers_interrupt(&machine);
    machine
        .run_for(GlobalTime::from_nanos(20_000_000))
        .expect("the board runs");
    assert_eq!(peek8(&machine, DONE), 0xa5);

    // The guest's handler masked every input, so the request register is the
    // line itself. IR7 is where the routing left it.
    let irr = |m: &Machine| -> u8 {
        outb(m, 0x20, 0x0a);
        inb(m, 0x20)
    };
    assert_eq!(irr(&machine) & (1 << 7), 1 << 7, "still asserted");

    // Write the admin completion queue head doorbell — the one thing the
    // bring-up deliberately did not do — and the whole path goes low: the
    // controller's condition, its pin, the fabric's net, the router's output.
    poke32(&machine, BAR_BASE + 0x1004, 1);
    assert_eq!(
        irr(&machine),
        0,
        "acknowledging the completion did not release the interrupt"
    );
}

#[test]
fn the_board_snapshots_and_restores_with_the_interrupt_still_asserted() {
    // An interrupt that is *outstanding* across a snapshot is the interesting
    // case, and it is the one a model that saved the routed level would get
    // wrong: nothing between the controller's completion queue and the 8259A's
    // request register is architectural state. The pin, the fabric's net and
    // the router's outputs are all re-derived, and the check is that they come
    // back the same — bit for bit, in one state hash.
    let mut a = board();
    raise_the_controllers_interrupt(&a);
    a.run_for(GlobalTime::from_nanos(20_000_000))
        .expect("the board runs");
    assert_eq!(peek8(&a, DONE), 0xa5, "the guest took both interrupts");

    let snapshot = a.save().expect("the board saves");
    let hash = a.state_hash().expect("the board hashes");

    let mut b = board();
    b.load(&snapshot).expect("the board loads");
    assert_eq!(
        b.state_hash().expect("the board hashes"),
        hash,
        "the restored machine is not the machine that was saved"
    );

    // And the interrupt is still there afterwards, on the input the guest last
    // routed it to. A restore that dropped it would leave a driver waiting for
    // a completion the controller believes it already reported.
    outb(&b, 0x20, 0x0a);
    assert_eq!(
        inb(&b, 0x20) & (1 << 7),
        1 << 7,
        "the restored machine lost the outstanding interrupt"
    );
    poke32(&b, BAR_BASE + 0x1004, 1);
    outb(&b, 0x20, 0x0a);
    assert_eq!(inb(&b, 0x20), 0, "and acknowledging it still releases it");
}
