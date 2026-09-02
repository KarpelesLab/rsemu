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
//! # The one piece of scaffolding, and exactly what would replace it
//!
//! A processor asks its own local interrupt controller what it has once per
//! instruction boundary, through `core::wire`'s [`LocalController`] — the same
//! shape as the acknowledge cycle it already runs against an 8259A, and offered
//! along the same `intr` net, so a machine file needs no new syntax. What is
//! missing is the *controller's* half: `src/dev/pc/apic.rs` has all five
//! accessors ([`LocalApic::waiting_for_startup`], `init_asserted`,
//! `take_startup`, `apic_base`, `set_apic_base`) and does not yet offer them
//! through
//! [`Device::local_controller`](rsemu::core::device::Device::local_controller).
//! `ApicLink` below is that eight-line forwarder, and the day the APIC grows it
//! this file deletes `ApicLink` and asserts exactly the same things.
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
use std::sync::atomic::{AtomicBool, Ordering};

use rsemu::core::device::{Deferred, Device, RealizeCtx};
use rsemu::core::hosts::HostObjects;
use rsemu::core::props::Props;
use rsemu::core::space::{AddressSpace, MemAttrs, RamStore, Region, RequesterId};
use rsemu::core::value::Width;
use rsemu::core::wire::{LocalController, Startup, Wire, WireIdAllocator, WireSource};
use rsemu::cpu::x86::isa::seg;
use rsemu::cpu::x86::prot::{SegReg, ar};
use rsemu::cpu::x86::{Variant, X86};
use rsemu::dev::pc::apic::{ApicBus, LocalApic};

/// The processor-side view of a local APIC, which `src/dev/pc/apic.rs` does not
/// offer yet — see the module documentation.
///
/// Everything it reports is a public accessor on `LocalApic`; the only state of
/// its own is one bit of edge detection, because the APIC exposes an accepted
/// INIT as a *condition* (`waiting_for_startup`) and a processor needs the
/// *edge*.
#[derive(Debug)]
struct ApicLink {
    apic: Arc<LocalApic>,
    /// Whether the INIT currently in flight has already been reported.
    reported: AtomicBool,
}

impl LocalController for ApicLink {
    fn take_startup(&self) -> Startup {
        let held = self.apic.init_asserted();
        let waiting = self.apic.waiting_for_startup();
        let page = self.apic.take_startup();
        // One INIT is in flight from the moment the message is accepted until
        // the Start-Up behind it has been handed over. A processor that was not
        // running for any of that sees the whole sequence in a single ask,
        // which is what `Startup`'s three separate fields are for.
        let pending = held || waiting || page.is_some();
        let init = pending && !self.reported.swap(pending, Ordering::AcqRel);
        Startup { init, held, page }
    }

    fn base_register(&self) -> u64 {
        self.apic.apic_base()
    }

    fn set_base_register(&self, value: u64) {
        self.apic.set_apic_base(value);
    }
}

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
    /// Kept alive here because a processor holds its controller *weakly*, for
    /// the same reason it holds the thing that answers its acknowledge cycle
    /// weakly: the machine owns devices and a wire merely refers to them.
    _links: [Arc<ApicLink>; 2],
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
    let links = apics.clone().map(|apic| {
        Arc::new(ApicLink {
            apic,
            reported: AtomicBool::new(false),
        })
    });
    for ((apic, cpu), link) in apics.iter().zip(&cpus).zip(&links) {
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
        let peer: Arc<dyn LocalController> = Arc::clone(link) as Arc<dyn LocalController>;
        cpu.attach_local_controller("intr", Arc::downgrade(&peer));
        // Consume the reset sequence, so the first step of a run is an
        // instruction rather than a restart at the reset vector.
        cpu.step();
    }

    Rig {
        mem,
        ram,
        cpus,
        apics,
        _links: links,
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
