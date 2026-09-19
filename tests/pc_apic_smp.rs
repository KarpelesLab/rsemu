//! Can one processor start another with `INIT` and Start-Up?
//!
//! This drives the MultiProcessor Specification's universal startup algorithm
//! (§B.4) end to end: a bootstrap processor **executes** three writes to its
//! local APIC's interrupt command register, the messages travel the APIC bus,
//! the application processor's local APIC takes the `INIT`, resets itself and
//! waits, the Start-Up hands over the page the second processor is to begin
//! executing at — and that processor's own execution path picks it up, runs its
//! INIT sequence, enters the wait-for-SIPI state and starts at
//! `CS:IP = page << 8 : 0`.
//!
//! **Nothing in the test body touches the second processor's registers.** It is
//! started by the guest's own instructions, which is what `ROADMAP.md` Phase
//! 7's "≥ 2 vCPUs" gate needs to be true.
//!
//! # And what it can find out once it is running
//!
//! Starting a processor is half the job. The tests at the bottom of this file
//! are the other half, and they exist because every test above them passed
//! while a real kernel still brought up one processor: the application
//! processor started, ran the identification probe of *Intel SDM* Vol 1
//! §3.4.3.3 in the real mode a Start-Up leaves it in, could not make
//! `EFLAGS.ID` hold a bit, concluded the part predates `CPUID`, and halted.
//! So `the_started_processor_finds_its_own_cpuid_and_long_mode` runs the §B.4
//! sequence **in full** — including the second Start-Up — and then has the
//! *bootstrap* processor spin in guest code on a word the other one writes,
//! which is the shape of what a kernel is waiting for when it says a processor
//! failed to report alive.
//!
//! # No scaffolding
//!
//! A processor asks its own local interrupt controller what it has once per
//! instruction boundary, through `core::wire`'s `LocalController` — the same
//! shape as the acknowledge cycle it already runs against an 8259A, and offered
//! along the same `intr` net, so a machine file needs no new syntax. Both
//! halves exist: `src/dev/pc/apic.rs` offers the controller through
//! [`Device::local_controller`](rsemu::core::device::Device::local_controller)
//! and the core takes it through `attach_local_controller`, which is exactly
//! what `machines/pc-apic.machine`'s two processors are wired with. Nothing in
//! this file stands between them.
//!
//! # Why these APICs are never reset
//!
//! [`Device::reset`](rsemu::core::device::Device::reset) on a **non-bootstrap**
//! local APIC parks its processor in wait-for-SIPI at once, because that is
//! what the MP initialization protocol does at power-up (SDM Vol 3A 8.4.3): an
//! application processor on a real board never fetches the reset vector. This
//! rig deliberately does not run that reset, so the second processor here comes
//! up *running*, and the INIT below is then demonstrably what stops it rather
//! than a state it was already in. The board's version of the same sequence,
//! with the reset, is `machines/pc-apic.machine`.
//!
//! # Two APIC pages, not one
//!
//! On real hardware both local APICs answer at `0xfee00000`, and each processor
//! sees its own — the aperture is *per processor*, not per board. rsemu has one
//! address space per bus, so the second APIC's page is mapped at `0xfef00000`
//! here. Nothing in this test reaches it; it is mapped so that the two parts
//! are wired the same way, and it is the reason `machines/pc-apic.machine`
//! ships one processor rather than two.
//!
//! # Sources
//!
//! Intel SDM Volume 3A §10.6.1 for the interrupt command register's fields,
//! §8.4.3 for where a Start-Up leaves a processor, Table 9-1 for what an INIT
//! resets, and the *MultiProcessor Specification* v1.4 §B.4 for the sequence.

#![cfg(all(feature = "cpu-x86", feature = "dev-pc", feature = "dev-pc-apic"))]

use std::sync::Arc;

use rsemu::core::device::{Deferred, Device, RealizeCtx};
use rsemu::core::hosts::HostObjects;
use rsemu::core::props::Props;
use rsemu::core::space::{AddressSpace, MemAttrs, RamStore, Region, RequesterId};
use rsemu::core::value::Width;
use rsemu::core::wire::{LocalController, Wire, WireIdAllocator, WireSource};
use rsemu::cpu::x86::isa::seg;
use rsemu::cpu::x86::prot::{SegReg, ar};
use rsemu::cpu::x86::{Config, Regs, Variant, X86, flags};
use rsemu::dev::pc::apic::{ApicBus, LocalApic};

/// Where the bootstrap processor's program sits.
const BSP_CODE: u32 = 0x1000;
/// The page the Start-Up names. `0x08` means linear `0x8000`.
const AP_PAGE: u8 = 0x08;
/// Where the application processor writes to say it is alive.
const MARKER: u32 = 0x4000;
/// What it writes there.
const ALIVE: u16 = 0xa55a;

/// The two local APIC register pages. The second is where it is for the reason
/// the module documentation gives.
const LAPIC0_BASE: u64 = 0xfee0_0000;
const LAPIC1_BASE: u64 = 0xfef0_0000;

/// A board with two processors, two local APICs and one megabyte of RAM.
struct Rig {
    mem: Arc<AddressSpace>,
    ram: Arc<RamStore>,
    cpus: [Arc<X86>; 2],
    apics: [Arc<LocalApic>; 2],
}

/// Run a device's `realize`, which is what puts a local APIC on its bus.
fn realize(device: &dyn Device) {
    let hosts = HostObjects::new();
    let mut deferred = Deferred::new();
    let mut ctx = RealizeCtx::new("test", RequesterId::default(), &mut deferred, &hosts);
    device.realize(&mut ctx).expect("realize cannot fail here");
    deferred.drain();
}

fn rig() -> Rig {
    rig_of(Variant::I80486)
}

/// The same, as a named part. Only the model-specific registers care: a 486 has
/// none, so the guest that reads `IA32_APIC_BASE` needs a later one.
fn rig_of(variant: Variant) -> Rig {
    let mem = AddressSpace::new("mem", 32);
    let ram = Arc::new(RamStore::new(0x10_0000));
    mem.topology()
        .map(Region::ram("ram", Arc::clone(&ram)), 0)
        .expect("a megabyte at zero");

    let bus = Arc::new(ApicBus::new());
    let apics = [
        Arc::new(LocalApic::with_bus(0, true, Arc::clone(&bus))),
        Arc::new(LocalApic::with_bus(1, false, Arc::clone(&bus))),
    ];
    for (apic, base) in apics.iter().zip([LAPIC0_BASE, LAPIC1_BASE]) {
        realize(&**apic);
        mem.topology()
            .map(apic.region("regs").expect("the register page"), base)
            .expect("a page at the top of the space");
    }

    let mem = Arc::new(mem);
    let ids = WireIdAllocator::new();
    let cpus = [(); 2].map(|()| {
        let cpu = Arc::new(
            X86::from_props_defaulting(&Props::new(), variant)
                .expect("a preset part with no properties"),
        );
        cpu.attach_space(Arc::clone(&mem));
        cpu
    });
    for (apic, cpu) in apics.iter().zip(&cpus) {
        // The local APIC drives the processor's `INTR` and answers its
        // acknowledge cycle, exactly as an 8259A does.
        let src = ids.alloc();
        let pin = cpu
            .sink("intr", &[src])
            .expect("the processor has an INTR pin");
        let wire = Wire::builder()
            .source(src)
            .sink(pin.sink, pin.line)
            .build_shared();
        apic.connect("intr", WireSource::new(wire, src))
            .expect("a local APIC drives intr");
        let ack = apic.int_ack("intr").expect("and answers the acknowledge");
        cpu.attach_int_ack("intr", Arc::downgrade(&ack));
        // And this is *this* processor's own controller — the link an INIT and
        // a Start-Up travel. Offered along the same net as the interrupt,
        // because on the hardware it is the same connection: the controller is
        // inside the processor it interrupts.
        let peer: Arc<dyn LocalController> = apic
            .local_controller("intr")
            .expect("a local APIC is its processor's own controller");
        cpu.attach_local_controller("intr", Arc::downgrade(&peer));
        // The device owns that `Arc` — the core keeps a `Weak` — so dropping
        // this one here is what the realizer does too.
        drop(peer);
        // Consume the reset sequence, so the first step of a run is an
        // instruction rather than a restart at the reset vector.
        cpu.step();
    }

    Rig {
        mem,
        ram,
        cpus,
        apics,
    }
}

impl Rig {
    /// Put a processor into flat 32-bit protected mode at `eip`.
    ///
    /// What the twenty bytes of firmware in `tests/pc_apic.rs` leave behind
    /// when they have run: a null GDT entry nobody looks at, a flat code
    /// segment and a flat data segment. Installed rather than executed here
    /// because this file is about what happens *after* that.
    fn enter_flat_protected(&self, index: usize, eip: u32) {
        let cpu = &self.cpus[index];
        let code = SegReg {
            selector: 0x08,
            base: 0,
            limit: 0xffff_ffff,
            ar: ar::PRESENT | ar::S | ar::CODE | ar::RW | ar::ACCESSED | ar::DB | ar::GRANULAR,
        };
        let data = SegReg {
            selector: 0x10,
            ar: ar::PRESENT | ar::S | ar::RW | ar::ACCESSED | ar::DB | ar::GRANULAR,
            ..code
        };
        let mut sys = cpu.sys();
        sys.segs[seg::CS as usize] = code;
        for which in [seg::DS, seg::ES, seg::SS, seg::FS, seg::GS] {
            sys.segs[which as usize] = data;
        }
        sys.cr0 |= 1; // PE
        cpu.set_sys(sys);
        let mut regs = cpu.regs();
        regs.cs = 0x08;
        regs.ds = 0x10;
        regs.es = 0x10;
        regs.ss = 0x10;
        regs.fs = 0x10;
        regs.gs = 0x10;
        regs.rip = u64::from(eip);
        regs.rsp = 0xf000;
        cpu.set_regs(regs);
    }

    fn load(&self, at: u32, code: &[u8]) {
        for (i, byte) in code.iter().enumerate() {
            self.ram
                .write_u8(u64::from(at) + i as u64, *byte)
                .expect("inside the megabyte");
        }
    }

    fn peek16(&self, at: u32) -> u16 {
        self.mem
            .read(u64::from(at), Width::U16, MemAttrs::DEFAULT)
            .expect("a mapped word") as u16
    }

    fn peek32(&self, at: u32) -> u32 {
        self.mem
            .read(u64::from(at), Width::U32, MemAttrs::DEFAULT)
            .expect("a mapped dword") as u32
    }
}

/// Append a little-endian 32-bit word.
fn dw(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_le_bytes());
}

/// `mov dword [edi+disp32], imm32`.
fn store_at(out: &mut Vec<u8>, disp: u32, value: u32) {
    out.extend_from_slice(&[0xc7, 0x87]);
    dw(out, disp);
    dw(out, value);
}

/// The bootstrap processor's program: the MP specification's startup sequence,
/// written as three interrupt command register writes.
fn bsp_program() -> Vec<u8> {
    let mut code = Vec::new();
    code.push(0xbf); // mov edi, 0xfee00000
    dw(&mut code, LAPIC0_BASE as u32);
    // The destination half first: APIC ID 1, in bits 24-31.
    store_at(&mut code, 0x310, 1 << 24);
    // INIT, level, assert. Delivery mode 101 in bits 8-10, level in bit 14,
    // trigger mode in bit 15: 0xc500, the value every startup routine writes.
    store_at(&mut code, 0x300, 0x0000_c500);
    // INIT, level, de-assert: 0x8500.
    store_at(&mut code, 0x300, 0x0000_8500);
    // Start-Up, delivery mode 110, carrying the page as its vector.
    store_at(&mut code, 0x300, 0x0000_0600 | u32::from(AP_PAGE));
    code.extend_from_slice(&[0xeb, 0xfe]); // jmp $
    code
}

/// The application processor's program, in the real mode a Start-Up leaves it
/// in: say it is alive, then spin.
fn ap_program() -> Vec<u8> {
    let mut code = Vec::new();
    code.extend_from_slice(&[0x31, 0xc0]); // xor ax, ax
    code.extend_from_slice(&[0x8e, 0xd8]); // mov ds, ax
    code.extend_from_slice(&[0xc7, 0x06]); // mov word [MARKER], ALIVE
    code.extend_from_slice(&(MARKER as u16).to_le_bytes());
    code.extend_from_slice(&ALIVE.to_le_bytes());
    code.extend_from_slice(&[0xeb, 0xfe]); // jmp $
    code
}

#[test]
fn a_second_processor_is_started_by_init_and_start_up() {
    let rig = rig();
    rig.load(BSP_CODE, &bsp_program());
    rig.load(u32::from(AP_PAGE) << 12, &ap_program());
    rig.enter_flat_protected(0, BSP_CODE);

    // Give the application processor's APIC something to lose, so the INIT
    // reset is visible rather than a coincidence of two zeroed structures.
    rig.mem
        .write(LAPIC1_BASE + 0x080, Width::U32, 0x50, MemAttrs::DEFAULT)
        .expect("the task priority register");
    assert!(!rig.apics[1].waiting_for_startup());
    assert_eq!(rig.peek16(MARKER), 0, "and nothing has run yet");

    // Round-robin one instruction at a time, which is what the deterministic
    // threading mode does with a multiprocessor machine (`ROADMAP.md` §4.2).
    // **Both processors execute throughout**: the second one is running its own
    // reset vector when the INIT arrives, is stopped by the INIT rather than by
    // this test declining to schedule it, and is started again by the Start-Up.
    assert!(
        rig.cpus[1].run(1) > 0,
        "the second processor is executing its own reset vector to begin with"
    );
    let mut waited = false;
    for _ in 0..200 {
        rig.cpus[0].run(1);
        rig.cpus[1].run(1);
        waited |= rig.cpus[1].is_waiting_for_startup();
    }
    assert!(
        waited,
        "the INIT stopped it, and only the Start-Up started it again"
    );

    assert_eq!(
        rig.mem
            .read(LAPIC1_BASE + 0x080, Width::U32, MemAttrs::DEFAULT)
            .expect("the task priority register"),
        0,
        "the INIT reset the second APIC (SDM Vol 3A 10.4.7.1)"
    );
    assert_eq!(
        rig.apics[1].id(),
        1,
        "except for its ID, which an INIT preserves"
    );
    assert!(
        !rig.apics[1].init_asserted(),
        "and the de-assert dropped the line again"
    );
    assert!(
        !rig.apics[1].waiting_for_startup(),
        "the Start-Up ended the wait"
    );

    // Nothing in this test has touched the second processor's registers: it
    // asked its own controller, took the INIT, took the Start-Up behind it, and
    // started.
    assert_eq!(
        rig.cpus[1].sys().segs[seg::CS as usize].base,
        u64::from(AP_PAGE) << 12,
        "the Start-Up put CS's base at page << 12 (SDM Vol 3A 8.4.3)"
    );
    assert_eq!(
        rig.peek16(MARKER),
        ALIVE,
        "the second processor executed from the page the Start-Up named"
    );
}

#[test]
fn an_init_on_its_own_leaves_a_processor_waiting_for_a_start_up() {
    // The halt nothing but a Start-Up ends. Sent as the two halves of the
    // level-triggered pair and nothing else, so the processor is left in the
    // state the third message would have taken it out of.
    let rig = rig();
    let mut code = Vec::new();
    code.push(0xbf); // mov edi, 0xfee00000
    dw(&mut code, LAPIC0_BASE as u32);
    store_at(&mut code, 0x310, 1 << 24);
    store_at(&mut code, 0x300, 0x0000_c500); // INIT, level, assert
    store_at(&mut code, 0x300, 0x0000_8500); // INIT, level, de-assert
    code.extend_from_slice(&[0xeb, 0xfe]); // jmp $
    rig.load(BSP_CODE, &code);
    rig.load(u32::from(AP_PAGE) << 12, &ap_program());
    rig.enter_flat_protected(0, BSP_CODE);
    rig.cpus[0].run(2_000);

    // The second processor runs its INIT sequence and stops there. It charges
    // for the sequence and then nothing, which is how this core tells a
    // scheduler it has stopped rather than that it is looping.
    assert!(!rig.cpus[1].is_waiting_for_startup());
    let charged = rig.cpus[1].run(2_000);
    assert!(charged > 0, "the INIT sequence itself is charged for");
    assert!(
        rig.cpus[1].is_waiting_for_startup(),
        "and it left the processor waiting rather than at the reset vector"
    );
    assert_eq!(rig.cpus[1].run(2_000), 0, "which is a full stop");
    assert_eq!(rig.peek16(MARKER), 0, "nothing has executed");

    // That an interrupt does not end this halt — the difference between it and
    // a `HLT` — is `cpu::x86`'s own
    // `an_interrupt_does_not_leave_the_wait_for_sipi_state`, because asserting
    // one here would mean writing this processor's flags, and no test in this
    // file touches its registers.

    // The Start-Up the bootstrap processor never sent, sent now.
    rig.mem
        .write(LAPIC0_BASE + 0x310, Width::U32, 1 << 24, MemAttrs::DEFAULT)
        .expect("the destination half");
    rig.mem
        .write(
            LAPIC0_BASE + 0x300,
            Width::U32,
            0x0000_0600 | u64::from(AP_PAGE),
            MemAttrs::DEFAULT,
        )
        .expect("the command half");
    rig.cpus[1].run(2_000);
    assert!(!rig.cpus[1].is_waiting_for_startup());
    assert_eq!(
        rig.peek16(MARKER),
        ALIVE,
        "the second processor started at the page the Start-Up named"
    );
}

#[test]
fn a_guest_reads_and_writes_ia32_apic_base() {
    // The register `RDMSR` and `WRMSR` reach that is not in the processor at
    // all (SDM Vol 3A 10.4.4). Clearing its enable bit is the write with a
    // visible consequence: a hardware-disabled APIC is transparent.
    let rig = rig_of(Variant::X86_64);
    let mut code = Vec::new();
    // mov ecx, IA32_APIC_BASE ; rdmsr ; mov esi, eax
    code.extend_from_slice(&[0xb9]);
    dw(&mut code, 0x1b);
    code.extend_from_slice(&[0x0f, 0x32]);
    code.extend_from_slice(&[0x89, 0xc6]);
    // and eax, ~(1 << 11) ; wrmsr — clear the global enable
    code.extend_from_slice(&[0x25]);
    dw(&mut code, !(1u32 << 11));
    code.extend_from_slice(&[0x0f, 0x30]);
    code.extend_from_slice(&[0xeb, 0xfe]); // jmp $
    rig.load(BSP_CODE, &code);
    rig.enter_flat_protected(0, BSP_CODE);

    assert_eq!(rig.apics[0].apic_base() & (1 << 11), 1 << 11);
    rig.cpus[0].run(2_000);

    assert_eq!(
        rig.cpus[0].regs().rsi & 0xffff_ffff,
        LAPIC0_BASE | (1 << 11) | (1 << 8),
        "the read reported the page, the enable and the bootstrap flag"
    );
    assert_eq!(
        rig.apics[0].apic_base() & (1 << 11),
        0,
        "and the write turned the local APIC off"
    );
    assert!(
        rig.mem
            .read(LAPIC0_BASE + 0x020, Width::U32, MemAttrs::DEFAULT)
            .is_err(),
        "a hardware-disabled APIC has no register page at all"
    );
}

#[test]
fn a_start_up_to_a_processor_that_is_not_waiting_is_ignored() {
    // Which is why the MP specification's algorithm sends two of them and does
    // not care that the second is redundant (B.4).
    let rig = rig();
    rig.load(BSP_CODE, &bsp_program());
    rig.enter_flat_protected(0, BSP_CODE);
    rig.cpus[0].run(2_000);
    assert_eq!(rig.apics[1].take_startup(), Some(AP_PAGE));

    // A second Start-Up, sent by hand through the same register the program
    // used: the processor is no longer waiting, so nothing is latched.
    rig.mem
        .write(LAPIC0_BASE + 0x310, Width::U32, 1 << 24, MemAttrs::DEFAULT)
        .expect("the destination half");
    rig.mem
        .write(
            LAPIC0_BASE + 0x300,
            Width::U32,
            0x0000_0600 | u64::from(AP_PAGE),
            MemAttrs::DEFAULT,
        )
        .expect("the command half");
    assert_eq!(rig.apics[1].take_startup(), None);
}

#[test]
fn one_processor_interrupts_another() {
    // The other half of what an interrupt command register is for, and what
    // every reschedule and every TLB shootdown in an SMP kernel is built on.
    let rig = rig();
    let mut code = Vec::new();
    code.push(0xbf); // mov edi, 0xfee00000
    dw(&mut code, LAPIC0_BASE as u32);
    // Software-enable this APIC, so its own vector table is live; the message
    // inbox works either way, and the *destination's* enable is what matters.
    store_at(&mut code, 0x0f0, 0x1ff);
    store_at(&mut code, 0x310, 1 << 24); // destination: APIC 1
    store_at(&mut code, 0x300, 0x0000_0042); // fixed delivery, vector 0x42
    code.extend_from_slice(&[0xeb, 0xfe]); // jmp $
    rig.load(BSP_CODE, &code);
    rig.enter_flat_protected(0, BSP_CODE);

    rig.mem
        .write(LAPIC1_BASE + 0x0f0, Width::U32, 0x1ff, MemAttrs::DEFAULT)
        .expect("software-enable the destination");
    assert!(!rig.apics[1].intr_asserted());

    rig.cpus[0].run(2_000);

    assert!(
        rig.apics[1].intr_asserted(),
        "the second processor's INTR pin went up"
    );
    assert!(
        !rig.apics[0].intr_asserted(),
        "and the sender's did not: this was addressed to one APIC ID"
    );
    // The acknowledge cycle the second processor would run answers with the
    // vector the first one sent.
    let vector = rig.apics[1]
        .int_ack("intr")
        .expect("a local APIC answers the acknowledge")
        .acknowledge(rsemu::core::wire::IntAckCycle::vector_only());
    assert_eq!(vector, rsemu::core::wire::IntAckResponse::Vector(0x42));
}

// ---------------------------------------------------------------------------
// what the processor that was started can find out about itself
// ---------------------------------------------------------------------------

/// Where the application processor leaves its verdict.
///
/// A dword rather than [`MARKER`]'s word, because the bootstrap processor
/// waits on it with a 32-bit compare — and three distinct sentinels rather
/// than a flag, because "never executed" (the zero the RAM comes up with),
/// "executed and could not find `CPUID`" and "executed and `CPUID` says no
/// long mode" are three different failures and the assertion should say which
/// one it saw.
const VERDICT: u32 = 0x4010;
const HAS_CPUID_AND_LONG: u32 = 0x600d_c0de;
const NO_CPUID: u32 = 0xdead_0001;
const NO_LONG: u32 = 0xdead_0002;
/// Where the bootstrap processor records that its **own** wait ended.
const BSP_SAW: u32 = 0x4014;
const SAW: u32 = 0x5a0f_5a0f;

/// `mov dword [disp32], imm32` — an absolute store from a flat segment.
fn store_abs(out: &mut Vec<u8>, at: u32, value: u32) {
    out.extend_from_slice(&[0xc7, 0x05]);
    dw(out, at);
    dw(out, value);
}

/// `mov dword [disp16], imm32` — the same store in a 16-bit address size.
///
/// `66` widens the *operand* to thirty-two bits and leaves the address at
/// sixteen, which is what a real-mode program writing a dword emits — and what
/// the application processor below is stuck with, because a Start-Up leaves it
/// in real mode (*Intel SDM* Vol 3A §8.4.3).
fn store_abs16(out: &mut Vec<u8>, at: u16, value: u32) {
    out.extend_from_slice(&[0x66, 0xc7, 0x06]);
    out.extend_from_slice(&at.to_le_bytes());
    dw(out, value);
}

/// The application processor's program: find out whether this part has
/// `CPUID`, and whether it has long mode, and say so.
///
/// That is the whole of what a real bring-up trampoline establishes before it
/// switches modes, and it is done here in the **sixteen-bit real mode** a
/// Start-Up leaves a processor in — which is the half of it that had never
/// been exercised anywhere in this tree.
///
/// 1. `PUSHFD`, flip bit 21, `POPFD`, `PUSHFD`, compare. "The ability of a
///    program to set or clear this flag is one way to determine whether a
///    processor supports the `CPUID` instruction" (*Intel SDM* Vol 1
///    §3.4.3.3). A part that answers *no* has no `CPUID` to ask anything else
///    of, so the program stops there.
/// 2. `CPUID` leaf `8000_0001`, `EDX` bit 29 — `LM` (*AMD64 Architecture
///    Programmer's Manual* Vol 3, `CPUID Fn8000_0001_EDX[29]`).
///
/// Both answers are yes on the part this rig builds, so a run that writes
/// anything but [`HAS_CPUID_AND_LONG`] has found a defect rather than a
/// configuration.
fn ap_identifies_itself() -> Vec<u8> {
    let mut code: Vec<u8> = Vec::new();
    code.push(0xfa); // cli
    code.extend_from_slice(&[0x31, 0xc0]); // xor ax, ax
    code.extend_from_slice(&[0x8e, 0xd8]); // mov ds, ax
    code.extend_from_slice(&[0x8e, 0xd0]); // mov ss, ax
    code.extend_from_slice(&[0xbc, 0x00, 0x20]); // mov sp, 0x2000

    // The identification flag, toggled and read back.
    code.extend_from_slice(&[0x66, 0x9c]); // pushfd
    code.extend_from_slice(&[0x66, 0x58]); // pop eax
    code.extend_from_slice(&[0x66, 0x89, 0xc3]); // mov ebx, eax
    code.extend_from_slice(&[0x66, 0x35]); // xor eax, imm32
    dw(&mut code, flags::ID);
    code.extend_from_slice(&[0x66, 0x50]); // push eax
    code.extend_from_slice(&[0x66, 0x9d]); // popfd
    code.extend_from_slice(&[0x66, 0x9c]); // pushfd
    code.extend_from_slice(&[0x66, 0x58]); // pop eax
    code.extend_from_slice(&[0x66, 0x31, 0xd8]); // xor eax, ebx
    code.extend_from_slice(&[0x66, 0x25]); // and eax, imm32
    dw(&mut code, flags::ID);
    let to_no_cpuid = code.len();
    code.extend_from_slice(&[0x74, 0x00]); // jz no_cpuid

    // It moved, so there is a `CPUID` here to ask about long mode.
    code.extend_from_slice(&[0x66, 0xb8]); // mov eax, imm32
    dw(&mut code, 0x8000_0001);
    code.extend_from_slice(&[0x0f, 0xa2]); // cpuid
    code.extend_from_slice(&[0x66, 0x81, 0xe2]); // and edx, imm32
    dw(&mut code, 1 << 29);
    let to_no_long = code.len();
    code.extend_from_slice(&[0x74, 0x00]); // jz no_long

    store_abs16(&mut code, VERDICT as u16, HAS_CPUID_AND_LONG);
    code.extend_from_slice(&[0xeb, 0xfe]); // jmp $

    // A `rel8` is measured from the byte after the displacement.
    let here = code.len();
    code[to_no_cpuid + 1] = (here - (to_no_cpuid + 2)) as u8;
    store_abs16(&mut code, VERDICT as u16, NO_CPUID);
    code.extend_from_slice(&[0xeb, 0xfe]); // jmp $

    let here = code.len();
    code[to_no_long + 1] = (here - (to_no_long + 2)) as u8;
    store_abs16(&mut code, VERDICT as u16, NO_LONG);
    code.extend_from_slice(&[0xeb, 0xfe]); // jmp $
    code
}

/// The bootstrap processor's program: the *MultiProcessor Specification* §B.4
/// universal algorithm in full — `INIT` assert, `INIT` de-assert, **two**
/// Start-Ups — and then a spin on the word the other processor is to write.
///
/// The second Start-Up is what [`bsp_program`] leaves out. The specification
/// sends it because a processor that has already left wait-for-SIPI ignores
/// it; that it is ignored here rather than restarting the processor is what
/// `a_start_up_to_a_processor_that_is_not_waiting_is_ignored` asserts.
///
/// The spin is the point of this one. A test that polled the marker from
/// outside would be asserting that the *harness* can see the write; a
/// bootstrap processor that leaves its own loop has seen it through the memory
/// system the guest uses, which is what a kernel's "failed to report alive
/// state" is waiting for and what this file had never made a guest wait on.
fn bsp_starts_and_waits() -> Vec<u8> {
    let mut code: Vec<u8> = Vec::new();
    code.push(0xbf); // mov edi, LAPIC0_BASE
    dw(&mut code, LAPIC0_BASE as u32);
    // Nothing an APIC delivers is reliable until it is software-enabled
    // (*Intel SDM* Vol 3A §10.4.7.2), an interprocessor interrupt included.
    store_at(&mut code, 0x0f0, 0x1ff);
    store_at(&mut code, 0x310, 1 << 24); // destination: APIC ID 1
    store_at(&mut code, 0x300, 0x0000_c500); // INIT, level, assert
    store_at(&mut code, 0x300, 0x0000_8500); // INIT, level, de-assert
    store_at(&mut code, 0x300, 0x0000_0600 | u32::from(AP_PAGE)); // Start-Up
    store_at(&mut code, 0x300, 0x0000_0600 | u32::from(AP_PAGE)); // and again

    let spin = code.len();
    code.extend_from_slice(&[0x81, 0x3d]); // cmp dword [VERDICT], imm32
    dw(&mut code, VERDICT);
    dw(&mut code, HAS_CPUID_AND_LONG);
    code.push(0x75); // jne spin
    let from = code.len() + 1;
    code.push((spin as isize - from as isize) as u8);

    store_abs(&mut code, BSP_SAW, SAW);
    code.extend_from_slice(&[0xeb, 0xfe]); // jmp $
    code
}

/// **The processor a Start-Up starts can identify itself.**
///
/// The regression test for a defect that let every assertion above pass and
/// still left a real operating system with one processor: `EFLAGS.ID` had no
/// storage, so step 1 of [`ap_identifies_itself`] answered *this part predates
/// `CPUID`* — and a Linux application-processor trampoline, which asks that
/// question in the real mode a Start-Up leaves it in, halted instead of
/// entering long mode. `q35-linux-smp` printed `CPU1 failed to report alive
/// state` with the second processor sitting on a `HLT` four kilobytes into its
/// own trampoline, `EAX` holding a failure code and `EFLAGS` reading `0x2`.
/// The bootstrap processor's path never asks, so one processor was fine.
///
/// Nothing else in this file could see it: the other tests start a processor
/// and look at *where* it started, and the flags register is not part of that.
#[test]
fn the_started_processor_finds_its_own_cpuid_and_long_mode() {
    let rig = rig_of(Variant::X86_64);
    rig.load(BSP_CODE, &bsp_starts_and_waits());
    rig.load(u32::from(AP_PAGE) << 12, &ap_identifies_itself());
    rig.enter_flat_protected(0, BSP_CODE);

    // Round-robin, a few cycles each: the deterministic threading mode in
    // miniature. Neither processor is stepped by hand and neither is run to
    // completion before the other starts.
    for _ in 0..20_000 {
        rig.cpus[0].run(8);
        rig.cpus[1].run(8);
        if rig.peek32(BSP_SAW) == SAW {
            break;
        }
    }

    let verdict = rig.peek32(VERDICT);
    assert_eq!(
        verdict,
        HAS_CPUID_AND_LONG,
        "the application processor reported {verdict:#010x}: {}",
        match verdict {
            0 => "it never executed at all",
            NO_CPUID =>
                "EFLAGS.ID would not hold the bit it was given, so the \
                         probe of SDM Vol 1 3.4.3.3 concluded this part predates CPUID",
            NO_LONG => "CPUID 8000_0001 reported no long mode",
            _ => "something else entirely",
        }
    );
    assert_eq!(
        rig.peek32(BSP_SAW),
        SAW,
        "the bootstrap processor never came out of its own wait"
    );
    assert!(
        !rig.cpus[1].is_waiting_for_startup(),
        "and the processor that wrote the word had left wait-for-SIPI"
    );
}

/// The flag on its own, at the core's seam.
///
/// [`the_started_processor_finds_its_own_cpuid_and_long_mode`] is why it
/// matters; this is the one-line statement of what was wrong, and of the tie
/// that keeps it honest — the bit is storage exactly where `CPUID` is an
/// instruction, because announcing `CPUID` is the only thing it does.
#[test]
fn the_identification_flag_has_storage_exactly_where_cpuid_does() {
    for cfg in [Config::I80486, Config::X86_64] {
        assert_eq!(
            Regs::normalise_flags(cfg, flags::ID) & flags::ID,
            flags::ID,
            "{:?} has CPUID, so a program can set EFLAGS.ID and read it back",
            cfg.variant
        );
    }
    // The same part with the instruction taken away — an early 80486, which a
    // machine file spells `cpuid = false`. The flag goes with it: one that
    // toggled in front of an instruction raising `#UD` would be a part that
    // never shipped.
    let mut features = Config::I80486.features;
    features.extras_486 = false;
    let early = Config::I80486.with_features(features);
    assert_eq!(
        Regs::normalise_flags(early, flags::ID) & flags::ID,
        0,
        "a part with no CPUID has nothing for EFLAGS.ID to announce"
    );
    // And nothing below the 80486 ever had either.
    assert_eq!(
        Regs::normalise_flags(Config::I80386, flags::ID) & flags::ID,
        0
    );
}
