//! **A firmware-booting board with two accelerated processors** — phase 7's
//! gate, the other half.
//!
//! `ROADMAP.md` phase 7 asks that *"the phase-6 machines boot under KVM **with
//! ≥ 2 vCPUs**"*, and until now no single board did both halves.
//! `tests/kvm_smp.rs` runs two accelerated processors on
//! `machines/pc-apic.machine`, whose firmware is written by that test;
//! `tests/kvm_pc_at_boot.rs` boots `machines/pc-at.machine` on
//! [`rsemu::fw::pcbios`], the firmware this repository assembles, with the one
//! processor that board declares. This file is both at once: **`pc-at`, its
//! own firmware, POST, `INT 19h`, a boot sector off the diskette controller —
//! and two processors, the second started by the guest's own `INIT` and
//! Start-Up, executing in hardware.**
//!
//! # What the second processor cost, measured rather than estimated
//!
//! `pc-at` already carries the whole APIC half of the board — a local APIC
//! with `bus = "apic"`, an I/O APIC, and the MP specification's IMCR in front
//! of the 8259A. So the board change is **five additions and one changed id**,
//! and this test makes
//! them textually, from `machines/pc-at.machine` itself, so that what it costs
//! is *visible in the diff* rather than described:
//!
//! ```text
//! + object cpu1 "cpu.x86" { clock = cpu, space = mem, iospace = "port", … }
//! + object lapic1 "pc.lapic" { clock = bus, id = 1, bus = "apic" }
//! ~ object ioapic "pc.ioapic" { id = 2, … }   # was id = 1, which lapic1 takes
//! + map mem 0xfef00000 size 0x1000 = lapic1.regs
//! + wire lapic1.intr -> cpu1.intr
//! + wire lapic1.nmi  -> cpu1.nmi
//! ```
//!
//! That is exactly the `pc-apic` shape, and nothing else moves. **No `accel/`
//! code was needed**: [`AccelCpus`] already allocates one vCPU per `cpu.x86`
//! object in declaration order, and a processor whose local APIC reports
//! itself not to be the bootstrap processor parks in wait-for-SIPI having
//! executed nothing. The patch is applied here rather than in `machines/`
//! because a second processor on the shipped board is a board decision with a
//! firmware consequence, and this test is the evidence for taking it, not the
//! taking of it.
//!
//! # What it does *not* buy, and this is the honest half
//!
//! **An operating system still will not find the second processor.**
//! Enumeration is firmware's job — an MP specification floating pointer and
//! configuration table in the `0xf0000` segment, or an ACPI RSDP and MADT —
//! and [`rsemu::fw::pcbios`] writes neither. `src/dev/q35/acpi.rs` builds a
//! MADT from a realized machine, so the table generator exists; what does not
//! exist is a legacy BIOS that publishes one. So this test's guest starts the
//! application processor **the way firmware would**, out of a boot sector it
//! wrote itself, and what it proves is that *the board and the hypervisor are
//! ready for the firmware that publishes the table* — not that FreeDOS, or
//! anything else, would use it.
//!
//! Concretely, the three remaining pieces, in the order they would land:
//!
//! 1. the six machine-file lines above, in `machines/pc-at.machine` (or a
//!    `pc-at-smp.machine` beside it, if a one-processor AT is worth keeping);
//! 2. an MP specification §4 floating pointer and configuration table in
//!    `src/fw/pcbios`, listing the processors, the bus, the I/O APIC and the
//!    interrupt assignments — the *only* new firmware code, and it is a table
//!    rather than an algorithm;
//! 3. nothing in `src/accel/`.
//!
//! # Why the guest's spins are all `hlt`
//!
//! A guest that takes no exits is not preemptible (`accel::kvm`), and under
//! [`ThreadingMode::Parallel`] a scheduler round does not end until every
//! runnable returns — so a `jmp $` inside `KVM_RUN` stops the *machine's*
//! virtual time, not just one processor's. `hlt` leaves hardware.
//!
//! Skips cleanly with no `/dev/kvm`.

#![cfg(all(
    feature = "accel-kvm",
    feature = "cpu-x86",
    feature = "dev-pc",
    feature = "dev-pc-apic",
    feature = "dev-pc-video",
    feature = "dev-pc-floppy",
    feature = "dev-pc-ide",
    feature = "dev-pc-hpet",
    feature = "fw-pcbios",
    feature = "machine-pc-at",
    target_os = "linux",
    target_arch = "x86_64"
))]

use std::sync::Arc;

use rsemu::accel::cpu::AccelCpus;
use rsemu::accel::kvm::Kvm;
use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::sched::ThreadingMode;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{BuildOptions, Machine, build};

// ---------------------------------------------------------------------------
// the board, plus a second processor
// ---------------------------------------------------------------------------

/// `machines/pc-at.machine`, with a second processor and its local APIC.
///
/// Every anchor is asserted to have been found, because a silent
/// `str::replace` that matched nothing would leave a one-processor board that
/// then failed a long way from here.
fn two_processor_at() -> String {
    let mut text = String::from(rsemu::dev::pc::PC_AT);

    // A second processor, declared immediately after `cpu0`. Order matters:
    // construction order is vCPU order, so `cpu0` must stay first and is
    // therefore vCPU 0 — the processor the board's local APIC calls the
    // bootstrap processor (`accel::cpu`).
    const CPU0: &str = "  object cpu0 \"cpu.x86\" {\n\
                        \x20   clock   = cpu\n\
                        \x20   space   = mem\n\
                        \x20   iospace = \"port\"\n\
                        \x20   model   = \"80486\"\n\
                        \x20   engine  = \"interp\"\n\
                        \x20 }\n";
    assert!(text.contains(CPU0), "the `cpu0` object moved");
    text = text.replace(CPU0, &format!("{CPU0}{}", CPU0.replace("cpu0", "cpu1")));

    // A local APIC for it. The I/O APIC's id moves out of the way, which is
    // exactly what `machines/pc-apic.machine` does for the same reason.
    const APICS: &str = "  object lapic0 \"pc.lapic\"  { clock = bus, id = 0, bus = \"apic\" }\n  \
                         object ioapic \"pc.ioapic\" { id = 1, bus = \"apic\" }";
    assert!(text.contains(APICS), "the APIC objects moved");
    text = text.replace(
        APICS,
        "  object lapic0 \"pc.lapic\"  { clock = bus, id = 0, bus = \"apic\" }\n  \
         object lapic1 \"pc.lapic\"  { clock = bus, id = 1, bus = \"apic\" }\n  \
         object ioapic \"pc.ioapic\" { id = 2, bus = \"apic\" }",
    );

    // Its register page, at the address `machines/pc-apic.machine` gives a
    // second local APIC: rsemu models each one as its own device, so they
    // cannot share the architectural 0xfee00000 the way real per-processor
    // APICs do.
    const LAPIC0_MAP: &str = "  map mem 0xfee00000 size 0x1000   = lapic0.regs";
    assert!(text.contains(LAPIC0_MAP), "the lapic0 mapping moved");
    text = text.replace(
        LAPIC0_MAP,
        "  map mem 0xfee00000 size 0x1000   = lapic0.regs\n  \
         map mem 0xfef00000 size 0x1000   = lapic1.regs",
    );

    // And its two pins.
    const LAPIC0_WIRES: &str = "  wire lapic0.intr -> cpu0.intr\n  wire lapic0.nmi  -> cpu0.nmi";
    assert!(text.contains(LAPIC0_WIRES), "the lapic0 wires moved");
    text = text.replace(
        LAPIC0_WIRES,
        "  wire lapic0.intr -> cpu0.intr\n  \
         wire lapic0.nmi  -> cpu0.nmi\n  \
         wire lapic1.intr -> cpu1.intr\n  \
         wire lapic1.nmi  -> cpu1.nmi",
    );
    text
}

// ---------------------------------------------------------------------------
// the guest: a boot sector that starts the other processor
// ---------------------------------------------------------------------------

/// Where the boot sector lands, and what its labels are relative to.
const BOOT: u16 = 0x7c00;
/// Offsets inside the 512-byte sector, fixed so a far jump can name one.
const OFF_GDT: u16 = 0x0060;
const OFF_GDT_PTR: u16 = 0x0080;
const OFF_PM: u16 = 0x0090;

/// Where the application processor's trampoline is written, and the page a
/// Start-Up names to reach it.
const AP_TRAMPOLINE: u32 = 0x8000;
const AP_PAGE: u8 = 0x08;

/// Where each processor says it is alive, and what it says. Both in the block
/// at 0x0500 that every PC has left free since 1981.
const BSP_MARKER: u32 = 0x0500;
const BSP_ALIVE: u32 = 0x0000_b005;
const AP_MARKER: u32 = 0x0504;
const AP_ALIVE: u16 = 0xa55a;

/// The board's local APIC register page.
const LAPIC0: u32 = 0xfee0_0000;

/// Append a little-endian 32-bit word.
fn dw(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_le_bytes());
}

/// `mov dword [edi+disp32], imm32`, for a register-relative device page.
fn store_at(out: &mut Vec<u8>, disp: u32, value: u32) {
    out.extend_from_slice(&[0xc7, 0x87]);
    dw(out, disp);
    dw(out, value);
}

/// `mov dword [disp32], imm32` — an absolute store, for a flat data segment.
fn store_abs(out: &mut Vec<u8>, at: u32, value: u32) {
    out.extend_from_slice(&[0xc7, 0x05]);
    dw(out, at);
    dw(out, value);
}

/// The application processor's real-mode trampoline, which the *guest* writes
/// into the board's own RAM.
///
/// Real mode, because that is the mode a Start-Up leaves a processor in
/// (*Intel SDM* Vol 3A §8.4.3). It says it is alive and halts: the claim being
/// made is that a second processor executed guest instructions on host
/// silicon, and anything further would be `tests/kvm_smp.rs`'s claim, which
/// that file already makes.
fn trampoline() -> Vec<u8> {
    let mut out = Vec::new();
    out.extend_from_slice(&[0x31, 0xc0]); // xor ax, ax
    out.extend_from_slice(&[0x8e, 0xd8]); // mov ds, ax
    // mov word [AP_MARKER], AP_ALIVE — a 16-bit displacement, so the marker
    // has to live in the first 64 KiB, which it does.
    out.extend_from_slice(&[0xc7, 0x06]);
    out.extend_from_slice(&(AP_MARKER as u16).to_le_bytes());
    out.extend_from_slice(&AP_ALIVE.to_le_bytes());
    out.push(0xf4); // hlt
    out.extend_from_slice(&[0xeb, 0xfd]); // jmp back to the hlt
    out
}

/// A 1.44 MB diskette whose boot sector starts the other processor.
///
/// The sector is what firmware would be: it enters protected mode off a GDT
/// of its own — **in RAM**, so the accessed bit a descriptor load sets is a
/// write a hypervisor can land, which a ROM-resident GDT would not be
/// (`accel::board`) — writes the trampoline, software-enables its local APIC
/// and sends the *MultiProcessor Specification* §B.4 sequence through its own
/// interrupt command register.
fn bootable_diskette() -> Vec<u8> {
    let mut sector = vec![0u8; 512];

    // -- real mode ----------------------------------------------------------
    let mut entry: Vec<u8> = Vec::new();
    entry.push(0xfa); // cli
    entry.extend_from_slice(&[0x31, 0xc0]); // xor ax, ax
    entry.extend_from_slice(&[0x8e, 0xd8]); // mov ds, ax
    entry.extend_from_slice(&[0x8e, 0xc0]); // mov es, ax
    entry.extend_from_slice(&[0x8e, 0xd0]); // mov ss, ax
    entry.extend_from_slice(&[0xbc]); // mov sp, imm16
    entry.extend_from_slice(&BOOT.to_le_bytes());
    // lgdt [BOOT + OFF_GDT_PTR] — no operand-size prefix, so the base comes
    // from 24 bits, and 0x7c60 fits in 24 with room to spare.
    entry.extend_from_slice(&[0x0f, 0x01, 0x16]);
    entry.extend_from_slice(&(BOOT + OFF_GDT_PTR).to_le_bytes());
    entry.extend_from_slice(&[0x0f, 0x20, 0xc0]); // mov eax, cr0
    entry.extend_from_slice(&[0x0c, 0x01]); // or al, 1
    entry.extend_from_slice(&[0x0f, 0x22, 0xc0]); // mov cr0, eax
    entry.extend_from_slice(&[0x66, 0xea]); // jmp far 0x08:pm
    dw(&mut entry, u32::from(BOOT) + u32::from(OFF_PM));
    entry.extend_from_slice(&[0x08, 0x00]);
    assert!(
        entry.len() <= OFF_GDT as usize,
        "the real-mode entry ran into the GDT"
    );
    sector[..entry.len()].copy_from_slice(&entry);

    // -- the descriptor table -----------------------------------------------
    //
    // The accessed bit is set in both descriptors. It is not needed here — a
    // GDT in RAM is a writable slot — but it is set anyway, because a reader
    // copying this into a ROM-resident table would otherwise inherit the
    // `KVM_RUN` that never returns which `accel::board` documents.
    let gdt: [u8; 24] = [
        0, 0, 0, 0, 0, 0, 0, 0, // the null descriptor
        0xff, 0xff, 0, 0, 0, 0x9b, 0xcf, 0, // a flat 4 GiB code segment, ring 0
        0xff, 0xff, 0, 0, 0, 0x93, 0xcf, 0, // and a flat data segment
    ];
    let at = OFF_GDT as usize;
    sector[at..at + gdt.len()].copy_from_slice(&gdt);

    let mut gdt_ptr = Vec::new();
    gdt_ptr.extend_from_slice(&(gdt.len() as u16 - 1).to_le_bytes());
    dw(&mut gdt_ptr, u32::from(BOOT) + u32::from(OFF_GDT));
    let at = OFF_GDT_PTR as usize;
    sector[at..at + gdt_ptr.len()].copy_from_slice(&gdt_ptr);

    // -- protected mode -----------------------------------------------------
    let mut pm: Vec<u8> = Vec::new();
    pm.extend_from_slice(&[0xb8, 0x10, 0x00, 0x00, 0x00]); // mov eax, 0x10
    pm.extend_from_slice(&[0x8e, 0xd8]); // mov ds, ax
    pm.extend_from_slice(&[0x8e, 0xc0]); // mov es, ax
    pm.extend_from_slice(&[0x8e, 0xd0]); // mov ss, ax
    pm.push(0xbc); // mov esp, imm32
    dw(&mut pm, 0x7000);

    // The other processor's trampoline, into the board's own RAM.
    let tramp = trampoline();
    for (i, word) in tramp.chunks(4).enumerate() {
        let mut bytes = [0u8; 4];
        bytes[..word.len()].copy_from_slice(word);
        store_abs(
            &mut pm,
            AP_TRAMPOLINE + 4 * i as u32,
            u32::from_le_bytes(bytes),
        );
    }

    // Software-enable this processor's local APIC, with 0xff as the spurious
    // vector: nothing it delivers — including an IPI it sends — is reliable
    // until this is written (*Intel SDM* Vol 3A §10.4.7.2).
    pm.push(0xbf); // mov edi, LAPIC0
    dw(&mut pm, LAPIC0);
    store_at(&mut pm, 0xf0, 0x1ff);

    // The *MultiProcessor Specification* §B.4 sequence: the destination half,
    // `INIT` assert, `INIT` de-assert, Start-Up carrying the page.
    store_at(&mut pm, 0x310, 1 << 24);
    store_at(&mut pm, 0x300, 0x0000_c500);
    store_at(&mut pm, 0x300, 0x0000_8500);
    store_at(&mut pm, 0x300, 0x0000_0600 | u32::from(AP_PAGE));

    store_abs(&mut pm, BSP_MARKER, BSP_ALIVE);
    pm.push(0xf4); // hlt
    pm.extend_from_slice(&[0xeb, 0xfd]); // jmp back to the hlt
    let at = OFF_PM as usize;
    assert!(at + pm.len() <= 510, "the protected-mode half overran");
    sector[at..at + pm.len()].copy_from_slice(&pm);

    // What `INT 19h` looks for, and the reason a blank diskette is not booted.
    sector[510] = 0x55;
    sector[511] = 0xaa;

    let mut image = vec![0u8; 1_474_560];
    image[..sector.len()].copy_from_slice(&sector);
    image
}

// ---------------------------------------------------------------------------
// the board
// ---------------------------------------------------------------------------

/// The build options: rsemu's own firmware and that diskette.
fn options() -> BuildOptions {
    let mut options = rsemu::machine::catalog::build_options().expect("this build's classes");
    options
        .realize
        .media
        .insert("bios", rsemu::fw::pcbios::image());
    options.realize.media.insert("vgabios", Vec::new());
    options.realize.media.insert("optionrom", vec![0u8; 65536]);
    options.realize.media.insert("floppy", bootable_diskette());
    for slot in ["disk", "hd0", "hd1", "hd2", "hd3", "cd0", "cd1"] {
        options.realize.media.insert(slot, Vec::new());
    }
    options
}

/// A dword of guest memory, read as a debugger would.
fn peek32(m: &Machine, addr: u64) -> u32 {
    let mem = m.space("mem").expect("the memory space");
    mem.read(addr, Width::U32, MemAttrs::DEBUG).unwrap_or(0) as u32
}

/// Run `m` until both processors have said they are alive, or for `rounds`
/// milliseconds of virtual time.
fn run(m: &mut Machine, rounds: usize) {
    for _ in 0..rounds {
        m.run_for(GlobalTime::from_nanos(1_000_000))
            .expect("the board runs");
        if peek32(m, u64::from(BSP_MARKER)) == BSP_ALIVE
            && peek32(m, u64::from(AP_MARKER)) & 0xffff == u32::from(AP_ALIVE)
        {
            return;
        }
    }
}

/// **The gate.** `pc-at`, its own firmware, two processors, both in hardware.
#[test]
fn the_at_boots_its_own_firmware_and_starts_a_second_processor_in_hardware() {
    if !Kvm::is_available() {
        println!("kvm pc-at smp: no usable /dev/kvm on this host, skipping");
        return;
    }
    let accel = match AccelCpus::open(ThreadingMode::Parallel) {
        Ok(accel) => accel,
        Err(e) if e.is_unavailable() => return,
        Err(e) => panic!("/dev/kvm is present but unusable: {e}"),
    };

    let text = two_processor_at();
    let mut opts = options();
    opts.realize.scheduler.mode = ThreadingMode::Parallel;
    accel.install(&mut opts.bindings);
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut m = build("pc-at-smp.machine", &text, &registry, &opts)
        .unwrap_or_else(|e| panic!("the two-processor AT does not realize: {e}"));
    m.reset(ResetKind::Cold);
    m.sweep();

    let cpus = accel.cpus();
    assert_eq!(cpus.len(), 2, "the board declares two processors");
    let (bsp, ap) = (Arc::clone(&cpus[0]), Arc::clone(&cpus[1]));
    assert_eq!(bsp.id(), 0, "cpu0 is vCPU 0, the bootstrap processor");
    assert_eq!(ap.id(), 1);

    // Before anything runs: the application processor is parked by its local
    // APIC's reset and has entered hardware exactly zero times.
    assert_eq!(
        ap.entries(),
        0,
        "the application processor ran before the guest started it"
    );

    run(&mut m, 400);

    println!(
        "kvm pc-at smp: bsp {} entries, ap {} entries; bsp {:04x}:{:08x}, ap {:04x}:{:08x}",
        bsp.entries(),
        ap.entries(),
        bsp.shell().regs().cs,
        bsp.shell().regs().rip,
        ap.shell().regs().cs,
        ap.shell().regs().rip,
    );
    assert!(!bsp.is_stopped(), "cpu0 stopped: {:?}", bsp.failure());
    assert!(!ap.is_stopped(), "cpu1 stopped: {:?}", ap.failure());

    // POST ran, on host silicon, on the board's own firmware.
    let mem = m.space("mem").expect("the memory space");
    let basemem = mem
        .read(0x413, Width::U16, MemAttrs::DEBUG)
        .expect("the BDA");
    assert_eq!(basemem, 639, "POST did not fill the BIOS data area");

    // `INT 19h` found the diskette and its boot sector ran.
    assert_eq!(
        peek32(&m, u64::from(BSP_MARKER)),
        BSP_ALIVE,
        "the boot sector never reached protected mode"
    );

    // And the second processor executed guest instructions in hardware.
    assert_eq!(
        peek32(&m, u64::from(AP_MARKER)) & 0xffff,
        u32::from(AP_ALIVE),
        "the application processor never started: it has made {} guest entries",
        ap.entries()
    );
    assert!(
        ap.entries() > 0,
        "the marker is there but the application processor entered no guest, \
         which would mean something else wrote it"
    );
}

/// The same board on the interpreter, so that "two engines, one machine" is
/// checked rather than assumed — and so the machine-file patch is known to be
/// a *board* change rather than an accelerator trick.
#[test]
fn the_same_two_processor_at_boots_on_the_interpreter() {
    let text = two_processor_at();
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut m = build("pc-at-smp.machine", &text, &registry, &options())
        .unwrap_or_else(|e| panic!("the two-processor AT does not realize: {e}"));
    m.reset(ResetKind::Cold);
    m.sweep();
    run(&mut m, 2000);

    let mem = m.space("mem").expect("the memory space");
    assert_eq!(
        mem.read(0x413, Width::U16, MemAttrs::DEBUG)
            .expect("the BDA"),
        639
    );
    assert_eq!(
        peek32(&m, u64::from(BSP_MARKER)),
        BSP_ALIVE,
        "the boot sector never reached protected mode on the interpreter"
    );
    assert_eq!(
        peek32(&m, u64::from(AP_MARKER)) & 0xffff,
        u32::from(AP_ALIVE),
        "the interpreter's second processor never started"
    );
}
