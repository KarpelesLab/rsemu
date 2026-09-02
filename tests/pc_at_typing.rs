//! Typing at the `pc-at` board: more than one key, all the way through.
//!
//! The 8042 hands the processor one byte at a time and the 8259A's IRQ1 input
//! is **edge triggered**, so the interrupt has to *fall* between two scan codes
//! or the second one is never announced. That is not visible in a unit test of
//! the controller — the byte is in the buffer either way, and the status
//! register says so — and it is not visible in a board test that presses one
//! key. It takes a running firmware, an enabled interrupt and a second
//! keystroke, which is what this file is.
//!
//! # What is being claimed
//!
//! 1. rsemu's own BIOS posts, boots a one-sector guest, and that guest sits
//!    halted with interrupts enabled — the state a real PC waits for a key in.
//! 2. Two keys typed at the host character port produce **two** entries in the
//!    BIOS Data Area's type-ahead ring at `0040:001E`, which only happens if
//!    `INT 09h` ran twice, which only happens if IRQ1 produced two edges.
//! 3. The master 8259A saw the second edge: its in-service and request
//!    registers are back to idle afterwards, which is the acknowledge having
//!    completed rather than a line stuck high.
//!
//! # Sources
//!
//! Intel 8259A data sheet (edge-triggered mode, the IRR latch and OCW3's
//! read-register command); Intel 8042 data sheet for the output buffer and
//! OBF; the *IBM Personal Computer AT Technical Reference* for the BDA layout.
//! No emulator source was consulted.

#![cfg(all(
    feature = "cpu-x86",
    feature = "dev-pc",
    feature = "dev-pc-video",
    feature = "dev-pc-floppy",
    feature = "dev-pc-ide",
    feature = "fw-pcbios",
    feature = "machine-pc-at"
))]

use std::sync::Arc;

use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::cpu::x86::{Variant, X86};
use rsemu::fw::asm16::{AX, Asm, DS, ES, SP, SS};
use rsemu::host::chardev::ports as charports;
use rsemu::machine::Machine;
use rsemu::machine::build;
use rsemu::machine::realize::Bindings;

/// The BDA's keyboard ring: head, tail, and the sixteen words themselves.
const BDA_KBHEAD: u64 = 0x41a;
const BDA_KBTAIL: u64 = 0x41c;
const BDA_KBBUF: u64 = 0x41e;
/// Sixteen two-byte entries.
const KBBUF_BYTES: u64 = 32;

/// Where the boot sector lands.
const BOOT_ADDRESS: u16 = 0x7c00;

/// A boot sector that does nothing but wait for an interrupt.
///
/// Interrupts *on*: the firmware's own `INT 18h` park runs with them off,
/// because `INT` clears `IF` and nothing in that handler sets it again, so a
/// machine with no bootable disk is not a machine waiting for a keystroke. A
/// guest that is genuinely idle with `IF` set is the state a typing test needs,
/// and one sector is the whole of it.
fn boot_sector() -> Vec<u8> {
    let mut a = Asm::new(usize::from(BOOT_ADDRESS) + 512, 0x00);
    a.seek(BOOT_ADDRESS);
    a.cli();
    a.movi(AX, 0);
    a.movsr(DS, AX);
    a.movsr(ES, AX);
    a.movsr(SS, AX);
    a.movi(SP, BOOT_ADDRESS);
    a.sti();
    let park = a.here_label();
    a.hlt();
    a.jmp(park);
    a.seek(BOOT_ADDRESS + 510);
    a.db(&[0x55, 0xaa]);
    let image = a.finish();
    image[usize::from(BOOT_ADDRESS)..].to_vec()
}

/// The board, its processor, and the host objects the build opened.
fn board() -> (Machine, Arc<X86>, Arc<rsemu::core::hosts::HostObjects>) {
    let cpus: Arc<Captured<X86>> = Arc::new(Captured::new());
    let mut b = Bindings::new();
    rsemu::machine::builtin::bind(&mut b).expect("ram and rom");
    rsemu::dev::pc::bind(&mut b).expect("the chipset");
    rsemu::dev::ata::bind(&mut b).expect("the hard disks");
    let kept = Arc::clone(&cpus);
    b.bind("cpu.x86", move |props| {
        let cpu = Arc::new(X86::from_props_defaulting(props, Variant::I80486)?);
        kept.push(&cpu);
        Ok(cpu)
    })
    .expect("nothing else in this table claims the name");

    let mut options = rsemu::machine::BuildOptions::new()
        .with_classes(rsemu::machine::catalog::classes())
        .with_bindings(b);
    options
        .realize
        .media
        .insert("bios", rsemu::fw::pcbios::image());
    options.realize.media.insert("vgabios", Vec::new());
    // The spinning guest goes on the diskette, so `INT 19h` tries the empty
    // fixed-disk bay first and falls through to the uPD765 — the same path
    // `tests/pc_at_boot.rs` takes for its floppy case.
    let mut floppy = boot_sector();
    floppy.resize(1_474_560, 0);
    options.realize.media.insert("floppy", floppy);
    options.realize.media.insert("hd0", Vec::new());
    options.realize.media.insert("hd1", Vec::new());

    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut m = build("pc-at.machine", rsemu::dev::pc::PC_AT, &registry, &options)
        .unwrap_or_else(|e| panic!("the board does not realize: {e}"));
    let cpu = cpus.take().expect("the constructor kept a handle");
    m.reset(ResetKind::Cold);
    m.sweep();
    (m, cpu, options.realize.hosts)
}

fn peek(m: &Machine, addr: u64) -> u8 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U8, MemAttrs::DEBUG)
        .unwrap_or(0xff) as u8
}

fn peek16(m: &Machine, addr: u64) -> u16 {
    u16::from(peek(m, addr)) | (u16::from(peek(m, addr + 1)) << 8)
}

/// Every character sitting in the BDA's type-ahead ring, oldest first.
///
/// Sixteen two-byte entries between the head and the tail offset, both of which
/// are offsets within segment `0x40` and both of which wrap at the end of the
/// ring. The low byte of an entry is the ASCII character and the high byte its
/// scan code; only the character is of interest here.
fn typed(m: &Machine) -> Vec<u8> {
    let (first, last) = (BDA_KBBUF - 0x400, BDA_KBBUF - 0x400 + KBBUF_BYTES);
    let tail = u64::from(peek16(m, BDA_KBTAIL));
    let mut at = u64::from(peek16(m, BDA_KBHEAD));
    let mut out = Vec::new();
    while at != tail && out.len() < 16 {
        out.push(peek(m, 0x400 + at));
        at = if at + 2 >= last { first } else { at + 2 };
    }
    out
}

#[test]
fn two_keys_typed_at_the_8042_both_reach_the_type_ahead_ring() {
    let (mut m, cpu, hosts) = board();

    // POST, `INT 19h`, and the boot sector's own halt loop.
    m.run_for(GlobalTime::from_nanos(200_000_000))
        .expect("the machine runs");
    let regs = cpu.regs();
    println!(
        "pc-at typing: parked at {:04x}:{:08x}, halted={}",
        regs.cs,
        regs.rip,
        cpu.is_halted()
    );

    assert!(
        cpu.is_halted(),
        "the guest should be waiting on an interrupt"
    );

    let port = charports::get(&hosts, "keyboard")
        .expect("the keyboard port")
        .expect("the board opened one");

    // Set 2, because that is what an AT keyboard sends over the cable and what
    // the 8042's translation bit turns into set 1: `A` down then `A` up.
    port.feed(&[0x1c, 0xf0, 0x1c]);
    m.run_for(GlobalTime::from_nanos(20_000_000))
        .expect("the machine runs");
    assert_eq!(typed(&m), vec![b'a'], "the first keystroke never arrived");

    // The second key. Nothing about the machine has changed — no port was
    // poked, no register rewritten — so if the first keystroke left IRQ1 high
    // this one is invisible: the 8259A's input is edge triggered and there is
    // no edge left to give it. That is what this whole file is for.
    port.feed(&[0x32, 0xf0, 0x32]);
    m.run_for(GlobalTime::from_nanos(20_000_000))
        .expect("the machine runs");

    let ring = typed(&m);
    println!(
        "pc-at typing: ring holds {:?} (head {:#06x}, tail {:#06x})",
        ring.iter().map(|&c| c as char).collect::<String>(),
        peek16(&m, BDA_KBHEAD),
        peek16(&m, BDA_KBTAIL)
    );
    assert_eq!(
        ring,
        vec![b'a', b'b'],
        "the second keystroke never produced an IRQ1 edge"
    );

    // And the controller is empty rather than holding a byte nobody was told
    // about: status bit 0 is OBF (8042 data sheet), and it reading back set
    // here was the visible half of the defect.
    let pspace = m.space("port").expect("the I/O space");
    let status = pspace
        .read(0x64, Width::U8, MemAttrs::DEBUG)
        .expect("the status port") as u8;
    println!("pc-at typing: 8042 status {status:#04x}");
    assert_eq!(status & 0x01, 0, "a scan code is still stuck in the 8042");

    // The 8259A agrees: nothing is requesting and nothing is in service, which
    // is the acknowledge cycle having completed for every edge it latched.
    // OCW3 selects which register a read of 0x20 returns (8259A data sheet).
    let ocw3 = |sel: u8| {
        pspace
            .write(0x20, Width::U8, u64::from(sel), MemAttrs::DEFAULT)
            .expect("OCW3");
        pspace
            .read(0x20, Width::U8, MemAttrs::DEFAULT)
            .expect("the selected register") as u8
    };
    let (irr, isr) = (ocw3(0x0a), ocw3(0x0b));
    println!("pc-at typing: master IRR {irr:#04x}, ISR {isr:#04x}");
    assert_eq!(irr & 0x02, 0, "IRQ1 is still latched and unserviced");
    assert_eq!(isr, 0x00, "an interrupt is still in service");
}
