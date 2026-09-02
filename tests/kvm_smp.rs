//! **Two processors of one board, on host silicon, started by the guest.**
//!
//! `ROADMAP.md` phase 7's gate: *"the phase-6 machines boot under KVM **with
//! ≥ 2 vCPUs**"*. `tests/kvm_pc_board.rs` got a board's memory map into a
//! hypervisor and ran its reset vector, and said plainly what it still was
//! not: the vCPU was created *beside* the board rather than being one of its
//! processors, so no interrupt could reach it and no second one could be
//! started.
//!
//! This file closes that. `machines/pc-apic.machine` is used **verbatim** —
//! two `object … "cpu.x86"`, `engine = "interp"` and all — and
//! [`AccelCpus::install`](rsemu::accel::cpu::AccelCpus::install) replaces the
//! class's constructor through
//! [`Bindings::replace`](rsemu::machine::Bindings::replace), which is the
//! supported way a host builds something else for a class the file already
//! names. Everything the board says about wiring, addresses and interrupt
//! routing is unchanged; the engine underneath its processors is not.
//!
//! What the firmware does, and therefore what is asserted:
//!
//! 1. `cpu0` runs the board's reset vector on the host's silicon, enters
//!    protected mode, builds an interrupt gate and software-enables its local
//!    APIC.
//! 2. It writes a trampoline into the board's own RAM and sends the
//!    *MultiProcessor Specification* §B.4 sequence — `INIT` assert, `INIT`
//!    de-assert, Start-Up — through its own interrupt command register.
//! 3. `cpu1`, which was parked in wait-for-SIPI by its local APIC's reset and
//!    had executed nothing, leaves that state and begins executing **in
//!    hardware** at `page << 12`. It says so in RAM, enters protected mode of
//!    its own, and sends a **fixed-delivery inter-processor interrupt** back to
//!    APIC ID 0.
//! 4. `cpu0` is idle in `HLT` when it arrives. The board's local APIC raises
//!    `INTR`, the accelerated processor runs the acknowledge cycle against it,
//!    the vector comes back, and the guest's own handler runs and counts.
//!
//! Nothing in the test body touches either processor's registers, and the only
//! thing it writes into the machine is the firmware image.
//!
//! # Why every spin here is a `HLT`
//!
//! A guest that takes no exits is not preemptible — `accel::kvm`'s module
//! documentation says so, and on a board it bites harder than it does on a
//! harness: under `ThreadingMode::Parallel` a scheduler round does not end
//! until every runnable returns, so a `jmp $` inside `KVM_RUN` would stop the
//! machine's virtual time rather than only its own processor's. `hlt; jmp $-1`
//! is the idle loop that leaves hardware, and it is also what real firmware
//! writes.
//!
//! Skips cleanly with no `/dev/kvm`.

#![cfg(all(
    feature = "accel-kvm",
    feature = "cpu-x86",
    feature = "dev-pc",
    feature = "dev-pc-apic",
    feature = "dev-pc-hpet",
    target_os = "linux",
    target_arch = "x86_64"
))]

use std::sync::Arc;

use rsemu::accel::cpu::AccelCpus;
use rsemu::accel::kvm::Kvm;
use rsemu::core::clock::GlobalTime;
use rsemu::core::device::{Device, ResetKind};
use rsemu::core::sched::ThreadingMode;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{Machine, build};

/// The board, verbatim.
const PC_APIC: &str = include_str!("../machines/pc-apic.machine");

/// The ROM socket's size, which the machine file also declares.
const ROM_LEN: usize = 128 * 1024;
/// Where segment `0xf000` starts inside the image: the socket is based at
/// `0xe0000`, so `0xf0000` is 64 KiB in.
const SEG_F000: usize = 0x1_0000;
/// The reset vector, sixteen bytes below the top of the socket.
const RESET_VECTOR: usize = ROM_LEN - 0x10;

// Offsets inside segment `0xf000`, so the firmware can name its own pieces.
const OFF_ENTRY: usize = 0x0000;
const OFF_GDT: usize = 0x0100;
const OFF_GDT_PTR: usize = 0x0120;
const OFF_IDT_PTR: usize = 0x0128;
const OFF_PM: usize = 0x0200;
const OFF_HANDLER: usize = 0x0400;
const OFF_AP_PM: usize = 0x0500;

/// A linear address for something at `off` inside segment `0xf000`.
const fn lin(off: usize) -> u32 {
    0xf_0000 + off as u32
}

/// Where the interrupt-descriptor table is built, in low RAM.
const IDT_BASE: u32 = 0x2000;
/// Where the handler counts.
const COUNTER: u32 = 0x3000;
/// Where the application processor's trampoline is written, and the page a
/// Start-Up names to get there.
const AP_TRAMPOLINE: u32 = 0x8000;
const AP_PAGE: u8 = 0x08;
/// Where the application processor says it is alive, and what it says.
const AP_MARKER: u32 = 0x3200;
const ALIVE: u32 = 0x0000_a55a;
/// And where it says it has sent its interrupt.
const AP_SENT: u32 = 0x3204;
const SENT: u32 = 0x0000_600d;

/// The vector the application processor sends to the bootstrap processor.
const IPI_VECTOR: u8 = 0x40;

/// The two local APIC register pages, as `machines/pc-apic.machine` maps them.
const LAPIC0: u32 = 0xfee0_0000;
const LAPIC1: u32 = 0xfef0_0000;

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

/// `mov dword [disp32], imm32` — an absolute store, for a flat data segment.
fn store_abs(out: &mut Vec<u8>, at: u32, value: u32) {
    out.extend_from_slice(&[0xc7, 0x05]);
    dw(out, at);
    dw(out, value);
}

/// Place `bytes` at `off` inside segment `0xf000`.
fn put(rom: &mut [u8], off: usize, bytes: &[u8]) {
    let at = SEG_F000 + off;
    rom[at..at + bytes.len()].copy_from_slice(bytes);
}

/// Enter protected mode: the seventeen bytes every PC firmware writes.
///
/// `DS` must already be `0xf000`, because the GDT pointer is in the ROM.
fn enter_protected(out: &mut Vec<u8>, target: u32) {
    // lgdt [0x0120] — no operand-size prefix, so the base is loaded from 24
    // bits, and 0x000f0100 fits in 24.
    out.extend_from_slice(&[0x0f, 0x01, 0x16]);
    out.extend_from_slice(&(OFF_GDT_PTR as u16).to_le_bytes());
    out.extend_from_slice(&[0x0f, 0x20, 0xc0]); // mov eax, cr0
    out.extend_from_slice(&[0x0c, 0x01]); // or al, 1
    out.extend_from_slice(&[0x0f, 0x22, 0xc0]); // mov cr0, eax
    out.extend_from_slice(&[0x66, 0xea]); // jmp far 0x08:target
    dw(out, target);
    out.extend_from_slice(&[0x08, 0x00]);
}

/// Load the flat data selector into every data segment and set a stack.
fn flat_data(out: &mut Vec<u8>, esp: u32) {
    out.extend_from_slice(&[0xb8, 0x10, 0x00, 0x00, 0x00]); // mov eax, 0x10
    out.extend_from_slice(&[0x8e, 0xd8]); // mov ds, ax
    out.extend_from_slice(&[0x8e, 0xc0]); // mov es, ax
    out.extend_from_slice(&[0x8e, 0xd0]); // mov ss, ax
    out.push(0xbc); // mov esp, imm32
    dw(out, esp);
}

/// `hlt; jmp $-1` — the idle loop, which leaves hardware on every pass.
fn idle(out: &mut Vec<u8>) {
    out.push(0xf4); // hlt
    out.extend_from_slice(&[0xeb, 0xfd]); // jmp back to the hlt
}

/// The application processor's real-mode trampoline, which the *guest* writes
/// into RAM.
///
/// Real mode however the bootstrap processor is running, because that is the
/// mode a Start-Up leaves a processor in (*Intel SDM* Vol 3A §8.4.3).
fn trampoline() -> Vec<u8> {
    let mut out = Vec::new();
    out.extend_from_slice(&[0x31, 0xc0]); // xor ax, ax
    out.extend_from_slice(&[0x8e, 0xd8]); // mov ds, ax
    // mov word [AP_MARKER], ALIVE — a 16-bit displacement, so the marker has
    // to live in the first 64 KiB, which it does.
    out.extend_from_slice(&[0xc7, 0x06]);
    out.extend_from_slice(&(AP_MARKER as u16).to_le_bytes());
    out.extend_from_slice(&(ALIVE as u16).to_le_bytes());
    out.push(0xfa); // cli
    out.extend_from_slice(&[0xb8, 0x00, 0xf0]); // mov ax, 0xf000
    out.extend_from_slice(&[0x8e, 0xd8]); // mov ds, ax
    enter_protected(&mut out, lin(OFF_AP_PM));
    out
}

/// The firmware image.
fn firmware() -> Vec<u8> {
    let mut rom = vec![0u8; ROM_LEN];

    // -- the reset vector ---------------------------------------------------
    rom[RESET_VECTOR..RESET_VECTOR + 5].copy_from_slice(&[0xea, 0x00, 0x00, 0x00, 0xf0]);

    // -- real mode: enter protected mode ------------------------------------
    let mut entry: Vec<u8> = Vec::new();
    entry.push(0xfa); // cli
    entry.extend_from_slice(&[0xb8, 0x00, 0xf0]); // mov ax, 0xf000
    entry.extend_from_slice(&[0x8e, 0xd8]); // mov ds, ax
    enter_protected(&mut entry, lin(OFF_PM));
    put(&mut rom, OFF_ENTRY, &entry);

    // -- the descriptor tables ----------------------------------------------
    // **The accessed bit is set in both descriptors, and that is not
    // decoration.** A descriptor load sets `A` if it is clear — a *write* to
    // the descriptor — and this GDT lives in the firmware socket, which under
    // acceleration is a `KVM_MEM_READONLY` memory slot. A guest write a
    // hypervisor cannot land makes the far jump into protected mode
    // unfinishable, and because the processor never leaves hardware to say so,
    // it presents as a `KVM_RUN` that does not return rather than as an error.
    // Real firmware sets these bits for the same reason; the interpreter, which
    // drops a write to a `RomStore`, does not care either way.
    let gdt: [u8; 24] = [
        0, 0, 0, 0, 0, 0, 0, 0, // the null descriptor
        0xff, 0xff, 0, 0, 0, 0x9b, 0xcf, 0, // a flat 4 GiB code segment, ring 0
        0xff, 0xff, 0, 0, 0, 0x93, 0xcf, 0, // and a flat data segment
    ];
    put(&mut rom, OFF_GDT, &gdt);

    let mut gdt_ptr = Vec::new();
    gdt_ptr.extend_from_slice(&(gdt.len() as u16 - 1).to_le_bytes());
    dw(&mut gdt_ptr, lin(OFF_GDT));
    put(&mut rom, OFF_GDT_PTR, &gdt_ptr);

    let mut idt_ptr = Vec::new();
    idt_ptr.extend_from_slice(&0x07ffu16.to_le_bytes());
    dw(&mut idt_ptr, IDT_BASE);
    put(&mut rom, OFF_IDT_PTR, &idt_ptr);

    // -- the bootstrap processor, in protected mode -------------------------
    let mut pm: Vec<u8> = Vec::new();
    flat_data(&mut pm, 0xf000);

    // One interrupt gate, at IDT_BASE + 8 * vector. RAM comes out of a cold
    // reset zeroed, so every other entry is a not-present gate already.
    pm.push(0xbf); // mov edi, gate
    dw(&mut pm, IDT_BASE + 8 * u32::from(IPI_VECTOR));
    pm.push(0xb8); // mov eax, handler
    dw(&mut pm, lin(OFF_HANDLER));
    pm.extend_from_slice(&[0x66, 0x89, 0x07]); // mov [edi], ax
    pm.extend_from_slice(&[0x66, 0xc7, 0x47, 0x02, 0x08, 0x00]); // mov word [edi+2], 8
    pm.extend_from_slice(&[0xc6, 0x47, 0x04, 0x00]); // mov byte [edi+4], 0
    pm.extend_from_slice(&[0xc6, 0x47, 0x05, 0x8e]); // mov byte [edi+5], 0x8e
    pm.extend_from_slice(&[0xc1, 0xe8, 0x10]); // shr eax, 16
    pm.extend_from_slice(&[0x66, 0x89, 0x47, 0x06]); // mov [edi+6], ax
    pm.extend_from_slice(&[0x0f, 0x01, 0x1d]); // lidt [idt_ptr]
    dw(&mut pm, lin(OFF_IDT_PTR));

    // Software-enable this processor's local APIC, with 0xff as the spurious
    // vector. Nothing it delivers reaches the processor until this is written
    // (SDM Vol 3A §10.4.7.2).
    pm.push(0xbf); // mov edi, 0xfee00000
    dw(&mut pm, LAPIC0);
    store_at(&mut pm, 0xf0, 0x1ff);

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

    // Then the *MultiProcessor Specification* §B.4 sequence, through this
    // processor's own interrupt command register: the destination half, INIT
    // assert, INIT de-assert, Start-Up carrying the page.
    store_at(&mut pm, 0x310, 1 << 24);
    store_at(&mut pm, 0x300, 0x0000_c500);
    store_at(&mut pm, 0x300, 0x0000_8500);
    store_at(&mut pm, 0x300, 0x0000_0600 | u32::from(AP_PAGE));

    pm.push(0xfb); // sti
    idle(&mut pm);
    put(&mut rom, OFF_PM, &pm);

    // -- the application processor, in protected mode -----------------------
    let mut ap: Vec<u8> = Vec::new();
    flat_data(&mut ap, 0xe000);
    ap.push(0xbf); // mov edi, 0xfef00000
    dw(&mut ap, LAPIC1);
    store_at(&mut ap, 0xf0, 0x1ff);
    // Destination: APIC ID 0, physical mode.
    store_at(&mut ap, 0x310, 0);
    // Fixed delivery, assert level, edge triggered — the ordinary IPI
    // (SDM Vol 3A §10.6.1's interrupt command register).
    store_at(&mut ap, 0x300, 0x0000_4000 | u32::from(IPI_VECTOR));
    store_abs(&mut ap, AP_SENT, SENT);
    idle(&mut ap);
    put(&mut rom, OFF_AP_PM, &ap);

    // -- the interrupt handler ----------------------------------------------
    let mut handler: Vec<u8> = Vec::new();
    handler.extend_from_slice(&[0xff, 0x05]); // inc dword [COUNTER]
    dw(&mut handler, COUNTER);
    store_abs(&mut handler, LAPIC0 + 0xb0, 0); // end of interrupt
    handler.push(0xcf); // iret
    put(&mut rom, OFF_HANDLER, &handler);

    rom
}

/// The board, with that firmware in its socket and its processors accelerated.
fn accelerated_board() -> Option<(Machine, Arc<AccelCpus>)> {
    if !Kvm::is_available() {
        return None;
    }
    // `Parallel` rather than `Deterministic`, and not by preference:
    // `AccelCpus::open` refuses a mode that claims reproducibility, because a
    // run on host silicon is not reproducible and a state hash taken over one
    // would be a number a regression suite would then bless.
    let accel = match AccelCpus::open(ThreadingMode::Parallel) {
        Ok(accel) => accel,
        Err(e) if e.is_unavailable() => return None,
        Err(e) => panic!("/dev/kvm is present but unusable: {e}"),
    };

    let mut options = rsemu::machine::catalog::build_options().expect("this build's classes");
    options.realize.media.insert("bios", firmware());
    options.realize.scheduler.mode = ThreadingMode::Parallel;
    options.realize.scheduler.workers = 2;
    accel.install(&mut options.bindings);

    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let machine = match build("pc-apic.machine", PC_APIC, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize under acceleration: {e}"),
    };
    Some((machine, accel))
}

/// Read a doubleword out of the board's own memory space.
fn peek(m: &Machine, at: u32) -> u32 {
    m.space("mem")
        .expect("the memory space")
        .read(u64::from(at), Width::U32, MemAttrs::DEBUG)
        .expect("low RAM answers") as u32
}

/// Everything either processor has to say about why it stopped, for a failure
/// message that is worth reading.
fn report(m: &Machine, accel: &AccelCpus) -> String {
    use std::fmt::Write as _;
    let mut out = String::new();
    for cpu in accel.cpus() {
        let _ = writeln!(
            out,
            "cpu{}: {} entries, rip {:#x}, halted {}, stopped {} intr={} if={}{}",
            cpu.id(),
            cpu.entries(),
            cpu.shell().regs().rip,
            cpu.is_halted(),
            cpu.is_stopped(),
            cpu.shell().intr_asserted(),
            cpu.vcpu().map(|v| v.interrupts_enabled()).unwrap_or(false),
            cpu.failure().map(|f| format!(" ({f})")).unwrap_or_default(),
        );
    }
    let _ = writeln!(
        out,
        "arrivals: marker {:#x}, sent {:#x}, counter {}",
        peek(m, AP_MARKER),
        peek(m, AP_SENT),
        peek(m, COUNTER)
    );
    if let Some(plan) = accel.plan() {
        out.push_str(&plan.describe());
    }
    out
}

#[test]
fn a_boards_processors_are_accelerated_without_touching_its_machine_file() {
    // No hypervisor needed for the half that matters most: the class is
    // replaced, the file is unchanged, and the board still realizes.
    let Some((m, accel)) = accelerated_board() else {
        return;
    };
    assert_eq!(
        accel.cpus().len(),
        2,
        "the board declares two processors and both were built here"
    );
    for (i, cpu) in accel.cpus().iter().enumerate() {
        assert_eq!(cpu.id() as usize, i, "vCPU ids follow declaration order");
        assert!(cpu.vcpu().is_some(), "each one bound a vCPU");
        assert_eq!(
            cpu.class().name,
            "cpu.x86",
            "and reports the class the machine file named"
        );
    }
    assert_eq!(
        m.device("cpu0").expect("cpu0").class().name,
        "cpu.x86",
        "the machine sees the class it asked for"
    );
    let plan = accel.plan().expect("the board's memory map was installed");
    assert!(
        plan.covers(0xffff_fff0),
        "the reset vector has to be fetchable in hardware:\n{}",
        plan.describe()
    );
}

#[test]
fn two_processors_run_on_host_silicon_and_the_second_is_started_by_the_first() {
    let Some((mut m, accel)) = accelerated_board() else {
        return;
    };
    m.reset(ResetKind::Cold);
    m.sweep();

    assert_eq!(peek(&m, AP_MARKER), 0, "nothing has run yet");

    // Up to two hundred milliseconds of the board's own virtual time, a
    // scheduler quantum at a time. A span *shorter* than the quantum would
    // advance the clock without running anything — `run_for` is additive and
    // declines a round its deadline falls inside (`Machine::run_until`) — and
    // the firmware needs only a handful of rounds once it is running.
    for _ in 0..200 {
        m.run_for(GlobalTime::from_nanos(1_000_000))
            .expect("the board runs");
        if peek(&m, COUNTER) > 0 {
            break;
        }
    }

    let cpus = accel.cpus();
    assert_eq!(cpus.len(), 2);
    assert!(
        cpus[0].entries() > 0,
        "the bootstrap processor never entered the guest\n{}",
        report(&m, &accel)
    );
    assert_eq!(
        peek(&m, AP_MARKER),
        ALIVE,
        "the second processor never ran: it is started only by the guest's own \
         INIT and Start-Up\n{}",
        report(&m, &accel)
    );
    assert!(
        cpus[1].entries() > 0,
        "the second processor's instructions did not run in hardware\n{}",
        report(&m, &accel)
    );
    assert_eq!(
        peek(&m, AP_SENT),
        SENT,
        "the second processor did not reach its interrupt command register\n{}",
        report(&m, &accel)
    );
    assert!(
        peek(&m, COUNTER) > 0,
        "the bootstrap processor never took the interrupt the second one sent\n{}",
        report(&m, &accel)
    );
}

#[test]
fn a_snapshot_of_an_accelerated_processor_is_the_interpreters_own_chunk() {
    let Some((mut m, accel)) = accelerated_board() else {
        return;
    };
    m.reset(ResetKind::Cold);
    m.sweep();
    for _ in 0..50 {
        m.run_for(GlobalTime::from_nanos(1_000_000))
            .expect("the board runs");
        if peek(&m, AP_MARKER) == ALIVE {
            break;
        }
    }
    let saved = m.save().expect("an accelerated machine saves");

    // The same board, with interpreters, loads what hardware produced. That is
    // the cross-engine half of phase 7's gate: one chunk format, version 7,
    // written by whichever engine happened to be running.
    let mut options = rsemu::machine::catalog::build_options().expect("this build's classes");
    options.realize.media.insert("bios", firmware());
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut interpreted =
        build("pc-apic.machine", PC_APIC, &registry, &options).expect("the interpreted board");
    interpreted
        .load(&saved)
        .expect("a snapshot taken under KVM restores under the interpreter");

    // Byte for byte. Nothing in the chunk is engine-specific, so a board that
    // loaded what hardware wrote writes exactly the same thing back — which is
    // a stronger statement than comparing a register at a time, and the one
    // that would break first if this device ever grew a format of its own.
    let round_trip = interpreted.save().expect("and saves again");
    assert_eq!(
        saved, round_trip,
        "a snapshot taken under KVM does not survive a round trip through the interpreter"
    );

    // And it is a *live* state, not just bytes that compare equal: the
    // interpreted board picks the guest up where the accelerator left it and
    // runs it to the interrupt the second processor sends.
    assert!(
        accel.cpus()[0].shell().regs().rip > 0,
        "the accelerated processor had actually run"
    );
    assert_eq!(peek(&interpreted, AP_MARKER), ALIVE);
    for _ in 0..200 {
        interpreted
            .run_for(GlobalTime::from_nanos(1_000_000))
            .expect("the restored board runs");
        if peek(&interpreted, COUNTER) > 0 {
            break;
        }
    }
    assert!(
        peek(&interpreted, COUNTER) > 0,
        "the guest did not carry on under the other engine"
    );
}

/// Says out loud whether the tests above actually ran.
#[test]
fn report_whether_this_host_can_run_two_vcpus() {
    if Kvm::is_available() {
        // Nothing to assert; the tests above did the asserting.
    } else {
        // Deliberately not a failure: `cargo test` stays hermetic.
    }
}
