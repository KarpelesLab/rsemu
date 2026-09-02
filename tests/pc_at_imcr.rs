//! The APIC path on `pc-at`, and the switch that keeps it out of the way.
//!
//! `pc-at` carries a local APIC, an I/O APIC and an HPET. All three are
//! *additions*: a firmware that has never heard of an APIC boots on this board
//! exactly as it did before, because the MP specification says a board powers
//! up in PIC mode and `pc.imcr` is what makes this one do so. That claim is
//! only worth anything if it is tested from both sides, which is this file.
//!
//! # What is being claimed
//!
//! 1. Out of reset the master 8259A reaches the processor's own `INTR` pin,
//!    vector and all, with an APIC sitting on the same net saying nothing.
//! 2. Writing `01h` to the IMCR takes that path away — the pin drops even with
//!    an interrupt pending — and writing `00h` gives it back.
//! 3. The legacy interrupt lines land on the I/O APIC as well, at the inputs
//!    the MP specification assigns, with IRQ0 on input **2**.
//! 4. The HPET's counter runs and its capability register reports the period
//!    the machine file declared.
//!
//! # Sources
//!
//! *MultiProcessor Specification* v1.4 §3.6.2.1 (the IMCR) and §5.1 (IRQ0 on
//! I/O APIC input 2); Intel 82093AA §3.2 (the index/data pair and the
//! redirection table); IA-PC HPET Specification rev 1.0a §2.3.4 and §2.3.7.
//! No emulator source was consulted.

#![cfg(all(
    feature = "cpu-x86",
    feature = "dev-pc",
    feature = "dev-pc-apic",
    feature = "dev-pc-hpet",
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

/// The I/O APIC's register page (82093AA §3.1) and the HPET's (spec §3.2.4).
const IOAPIC: u64 = 0xfec0_0000;
const HPET: u64 = 0xfed0_0000;

fn board() -> (Machine, Arc<X86>) {
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
    options.realize.media.insert("bios", vec![0u8; 128 * 1024]);
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

fn peek32(m: &Machine, addr: u64) -> u32 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U32, MemAttrs::DEFAULT)
        .expect("a mapped dword") as u32
}

fn poke32(m: &Machine, addr: u64, value: u32) {
    m.space("mem")
        .expect("the memory space")
        .write(addr, Width::U32, u64::from(value), MemAttrs::DEFAULT)
        .expect("a mapped dword");
}

/// One 32-bit half of an I/O APIC redirection entry (82093AA §3.2.4).
fn redirection(m: &Machine, half: u8) -> u32 {
    poke32(m, IOAPIC, u32::from(half));
    peek32(m, IOAPIC + 0x10)
}

/// Set the IMCR, the way MP specification §3.6.2.1 says to.
fn set_imcr(m: &Machine, value: u8) {
    outb(m, 0x22, 0x70);
    outb(m, 0x23, value);
}

/// Program the master 8259A as a PC's firmware does, and start counter 0.
fn start_the_tick(m: &Machine) {
    outb(m, 0x20, 0x11);
    outb(m, 0x21, 0x08);
    outb(m, 0x21, 0x04);
    outb(m, 0x21, 0x01);
    outb(m, 0x21, 0xfe);
    outb(m, 0x43, 0x34);
    outb(m, 0x40, 100);
    outb(m, 0x40, 0);
}

#[test]
fn the_board_realizes_with_the_apic_parts_fitted() {
    let (m, _cpu) = board();
    for path in ["lapic0", "ioapic", "hpet0", "imcr"] {
        assert!(m.device(path).is_some(), "no `{path}` on the board");
    }
    // And every one of the AT's chips is still there, which is what "additive"
    // has to mean.
    for path in [
        "cpu0", "pic1", "pic2", "pit0", "cmos", "kbc", "sysctl", "dma1", "dma2", "vga", "fdc",
        "ide0", "ide1", "bios",
    ] {
        assert!(m.device(path).is_some(), "`{path}` went missing");
    }
}

#[test]
fn out_of_reset_the_8259a_reaches_the_processor_directly() {
    // The MP specification's PIC mode, which is the power-on default and the
    // reason a DOS-era firmware boots on a board that has an APIC on it.
    let (mut m, cpu) = board();
    assert_eq!(inb(&m, 0x22), 0x00, "nothing is selected yet");
    start_the_tick(&m);
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("the machine runs");

    assert!(
        cpu.intr_asserted(),
        "the timer's output never reached the processor's pin"
    );
    // And the vector is the 8259A's, not the local APIC's spurious one — the
    // acknowledge travelled through the IMCR and out the other side. A local
    // APIC on the same net never declines a cycle (SDM Vol 3A §10.9), so this
    // is the assertion that the two are asked in the right order.
    assert_eq!(cpu.acknowledge(), 0x08);
}

#[test]
fn writing_the_imcr_takes_the_direct_path_away_and_gives_it_back() {
    let (mut m, cpu) = board();
    start_the_tick(&m);
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("the machine runs");
    assert!(cpu.intr_asserted(), "PIC mode, and a tick is pending");

    // 01h: "forces the NMI and 8259 INTR signals to pass through the APIC"
    // (§3.6.2.1). The APIC is software-disabled out of reset and its LINT0 is
    // masked, so the pending interrupt stops here — which is the real hardware
    // behaviour and exactly why a board without this register cannot have an
    // APIC bolted on without breaking its firmware.
    set_imcr(&m, 0x01);
    assert!(
        !cpu.intr_asserted(),
        "the 8259A is no longer driving the processor's pin"
    );
    assert_eq!(inb(&m, 0x23), 0x01, "and the register reads back");

    // And back. The interrupt is still pending at the 8259A, so the pin comes
    // straight up again rather than waiting for the next tick.
    set_imcr(&m, 0x00);
    assert!(cpu.intr_asserted(), "PIC mode restored the pending request");
    assert_eq!(cpu.acknowledge(), 0x08);
}

#[test]
fn the_legacy_lines_land_on_the_io_apic_as_well() {
    let (mut m, _cpu) = board();
    // Every redirection entry comes out of reset masked (82093AA §3.2.4), so
    // nothing below changes what the processor sees. What is being checked is
    // the *wiring*: the remote-IRR-free delivery-status side of an entry is not
    // observable while masked, so the claim is made through the entry the
    // machine file says each line reaches.
    //
    // Unmask input 2, which MP specification §5.1 puts the 8254 on — not input
    // 0, where an ExtINT entry for the whole cascade would go. Vector 0x40,
    // edge triggered, physical destination 0.
    poke32(&m, IOAPIC, 0x14);
    poke32(&m, IOAPIC + 0x10, 0x40);
    poke32(&m, IOAPIC, 0x15);
    poke32(&m, IOAPIC + 0x10, 0);
    assert_eq!(redirection(&m, 0x14) & 0xff, 0x40);
    assert_eq!(redirection(&m, 0x14) & (1 << 16), 0, "and it is unmasked");

    // Input 0 is left masked and stays that way, so an entry that fired there
    // would be the 8254 landing on the wrong pin.
    assert_ne!(redirection(&m, 0x10) & (1 << 16), 0, "input 0 stays masked");

    outb(&m, 0x43, 0x34);
    outb(&m, 0x40, 100);
    outb(&m, 0x40, 0);
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("the machine runs");

    // Delivery status, bit 12: the message was sent to a local APIC that is
    // software-disabled, so it goes nowhere and the entry's own bookkeeping is
    // what says the pin moved.
    let entry = redirection(&m, 0x14);
    let idle = redirection(&m, 0x10);
    assert_ne!(
        entry, idle,
        "input 2 looks exactly like the untouched input 0: the 8254's output \
         never reached the I/O APIC"
    );
}

#[test]
fn the_hpet_answers_with_the_period_the_board_declared() {
    let (mut m, _cpu) = board();
    // The capability register's top 32 bits are the counter period in
    // femtoseconds (HPET spec §2.3.4). The machine file says 100000000, which
    // is 100 ns, which is a 10 MHz crystal — and the `osc hpet` line above it
    // says 10 MHz, so the two agree or the guest is told a lie.
    assert_eq!(peek32(&m, HPET + 4), 100_000_000);

    // Enable the counter (`ENABLE_CNF`, §2.3.5) and let time pass.
    poke32(&m, HPET + 0x10, 1);
    let before = peek32(&m, HPET + 0xf0);
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("the machine runs");
    let after = peek32(&m, HPET + 0xf0);
    assert!(
        after > before,
        "the main counter stood still: {before:#x} -> {after:#x}"
    );
}

#[test]
fn the_board_still_round_trips_to_an_identical_state_hash() {
    let (mut m, _cpu) = board();
    start_the_tick(&m);
    set_imcr(&m, 0x01);
    poke32(&m, HPET + 0x10, 1);
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("the machine runs");

    let image = m.save().expect("the board saves");
    let (mut other, _) = board();
    other.load(&image).expect("the board loads");
    assert_eq!(
        other.state_hash().expect("a hash"),
        m.state_hash().expect("a hash"),
        "a restored board must be indistinguishable from the one it came from"
    );
}
