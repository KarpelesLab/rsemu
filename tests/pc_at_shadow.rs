//! The 440FX's shadow windows, from the outside: what an option-ROM scan sees.
//!
//! Firmware's first act after sizing memory is to walk `0xc0000`-`0xdffff` on
//! 2 KiB boundaries looking for `0x55 0xAA`. Most of that range has nothing
//! behind it on this board, and an ISA bus with nothing driving it reads as
//! ones — which is what `pc-at.machine`'s `unassigned = read-as-ones` says and
//! what the scan is written against.
//!
//! The `pc.pmc` in front of it must not get in the way. Out of reset every PAM
//! register is `00h`, which the 82441FX datasheet (order 290549-001, §3.2.18)
//! defines as *Disabled*: "both read and write cycles are directed to the
//! expansion bus", i.e. the DRAM behind the window does not answer at all. A
//! bridge that claimed the range and then refused the cycle would turn a normal
//! POST into thirty-two bus faults.
//!
//! # What is being claimed
//!
//! 1. Every 2 KiB boundary of the option-ROM scan answers, and answers ones
//!    where the board decodes nothing.
//! 2. A real POST on rsemu's own firmware finishes with **zero** unanswered
//!    bus accesses.
//! 3. Shadowing still works: setting a PAM window to read/write puts DRAM
//!    there, and setting it back to disabled takes it away again.
//!
//! # Sources
//!
//! Intel 82441FX (PMC) datasheet, order 290549-001, §3.2.18 (PAM) and §3.1.1
//! (configuration mechanism #1). No emulator source was consulted.

#![cfg(all(
    feature = "cpu-x86",
    feature = "dev-pc",
    feature = "dev-pc-video",
    feature = "dev-pc-floppy",
    feature = "dev-pc-ide",
    feature = "machine-pc-at"
))]

use std::sync::Arc;

use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::cpu::x86::{Variant, X86};
use rsemu::machine::Machine;
use rsemu::machine::build;
use rsemu::machine::realize::Bindings;

/// Where the option-ROM scan starts and stops (`0xc0000`-`0xdffff`).
const SCAN_BASE: u64 = 0x000c_0000;
const SCAN_END: u64 = 0x000e_0000;

/// The board, with whatever is in the `bios` socket.
fn board(bios: Vec<u8>) -> (Machine, Arc<X86>) {
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
    options.realize.media.insert("bios", bios);
    // An empty option-ROM socket: 64 KiB of zeroes, no signature. That is the
    // case the scan has to survive, and the one where every read above
    // 0xc0000 that is not the socket itself lands on nothing.
    options.realize.media.insert("vgabios", Vec::new());
    options.realize.media.insert("floppy", vec![0u8; 1_474_560]);
    options.realize.media.insert("hd0", Vec::new());
    options.realize.media.insert("hd1", Vec::new());

    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut m = build("pc-at.machine", rsemu::dev::pc::PC_AT, &registry, &options)
        .unwrap_or_else(|e| panic!("the board does not realize: {e}"));
    let cpu = cpus.take().expect("the constructor kept a handle");
    m.reset(ResetKind::Cold);
    m.sweep();
    (m, cpu)
}

/// Write one byte to an I/O port, as an `OUT` would.
fn outb(m: &Machine, port: u64, value: u8) {
    m.space("port")
        .expect("the I/O space")
        .write(port, Width::U8, u64::from(value), MemAttrs::DEFAULT)
        .expect("a decoded port");
}

/// Write a dword, as the configuration address latch takes one.
fn outl(m: &Machine, port: u64, value: u32) {
    m.space("port")
        .expect("the I/O space")
        .write(port, Width::U32, u64::from(value), MemAttrs::DEFAULT)
        .expect("a decoded port");
}

/// Set one of the PMC's PAM bytes, through configuration mechanism #1.
fn set_pam(m: &Machine, offset: u16, value: u8) {
    // Bus 0, device 0, function 0; the enable bit and the dword-aligned
    // register number (82441FX §3.1.1).
    let addr = 0x8000_0000u32 | u32::from(offset & 0xfc);
    outl(m, 0xcf8, addr);
    outb(m, 0xcfc + u64::from(offset & 0x03), value);
}

#[test]
fn the_option_rom_scan_raises_no_bus_fault_anywhere_in_its_window() {
    let (m, _cpu) = board(vec![0u8; 128 * 1024]);
    let mem = m.space("mem").expect("the memory space");
    let mut refused = Vec::new();
    let mut at = SCAN_BASE;
    while at < SCAN_END {
        match mem.read(at, Width::U16, MemAttrs::DEFAULT) {
            Ok(v) => {
                // Nothing is in the option-ROM socket, so every one of these
                // is an unterminated bus and reads as ones.
                assert_eq!(v, 0xffff, "{at:#08x} answered {v:#06x}, not the bus");
            }
            Err(e) => refused.push((at, e)),
        }
        at += 0x800;
    }
    assert!(
        refused.is_empty(),
        "the scan was refused at {} of its {} probes, first at {:#08x}",
        refused.len(),
        (SCAN_END - SCAN_BASE) / 0x800,
        refused[0].0
    );
}

#[cfg(feature = "fw-pcbios")]
#[test]
fn a_post_on_rsemus_own_firmware_leaves_no_unanswered_access() {
    let (mut m, cpu) = board(rsemu::fw::pcbios::image());
    m.run_for(GlobalTime::from_nanos(200_000_000))
        .expect("the machine runs");
    let (faults, last) = cpu.bus_faults();
    println!("pc-at shadow: {faults} unanswered bus access(es), last at {last:08x}");
    assert_eq!(
        faults, 0,
        "the memory map refused {faults} accesses, last at {last:08x}"
    );
}

#[test]
fn a_pam_window_shadows_and_unshadows_the_range_under_it() {
    let (m, _cpu) = board(vec![0u8; 128 * 1024]);
    let mem = m.space("mem").expect("the memory space");
    // PAM3[3:0] governs 0xd0000-0xd3fff, and there is nothing else there.
    let pam3 = 0x5cu16;
    assert_eq!(
        mem.read(0x000d_0000, Width::U8, MemAttrs::DEFAULT),
        Ok(0xff),
        "disabled: the cycle goes to the expansion bus, which is not there"
    );

    // Read/write: the bridge's own DRAM answers both directions.
    set_pam(&m, pam3, 0x03);
    mem.write(0x000d_0000, Width::U8, 0x5a, MemAttrs::DEFAULT)
        .expect("shadow DRAM takes a write");
    assert_eq!(
        mem.read(0x000d_0000, Width::U8, MemAttrs::DEFAULT),
        Ok(0x5a)
    );

    // And back to disabled: the DRAM stops answering and the bus is bare
    // again, with its contents still there for the next enable.
    set_pam(&m, pam3, 0x00);
    assert_eq!(
        mem.read(0x000d_0000, Width::U8, MemAttrs::DEFAULT),
        Ok(0xff)
    );
    set_pam(&m, pam3, 0x01);
    assert_eq!(
        mem.read(0x000d_0000, Width::U8, MemAttrs::DEFAULT),
        Ok(0x5a),
        "read-only: what was shadowed is still there"
    );
}
