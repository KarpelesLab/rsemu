//! Does an x86 processor see the board's clocks move **inside** a scheduler
//! round?
//!
//! # The defect this file is the proof against
//!
//! `cpu.x86` published no [`TickCursor`] position at all. A runnable reports
//! what it consumed only when its `run` call returns, so for the length of one
//! round the clock forest stood where the round began — and every lazily
//! advanced device on the board was therefore caught up to the round boundary
//! and no further, whatever cycle the instruction that touched it was on. A
//! guest that latched the 8254 in a loop read *pairs of equal counts and then
//! a jump of 1 193*, which is one millisecond of a 105/88 MHz crystal: the
//! length of a round. The same staircase was under the HPET's main counter,
//! the ACPI power-management timer and the local APIC's current count, on a
//! one-processor board exactly as on a two-processor one.
//!
//! That distorts the one thing a PC guest uses those counters for. A Linux
//! guest calibrates its TSC and its delay loop against the 8254 and the HPET,
//! and watches its clocksource against them; against a staircase, a
//! calibration measures the round rather than the crystal. On `pc64`, whose
//! core is declared at 100 MHz, a stock Gentoo `6.6.67` kernel read
//! `tsc: Detected 96.780 MHz` under `rsemu run` and `97.530 MHz` under
//! `tests/pc64_linux.rs`; it now reads `100.004 MHz` under both.
//! `docs/techniques/execution-budgets.md` has the whole matrix, and says which
//! kernel and which driving pattern every figure came from — both change the
//! answer, and this defect needed a companion fix in `core::sched` before
//! either column was worth quoting.
//!
//! # What has to hold now
//!
//! * **Resolution.** Consecutive samples of each counter, taken a few tens of
//!   guest cycles apart, differ — and differ by about what the two crystals'
//!   declared ratio says those cycles are worth. No two adjacent samples in a
//!   round are equal.
//! * **Monotonicity.** No counter ever runs backwards, across rounds included:
//!   a reader's last value in one round is never above its first in the next
//!   (`ReadLine`'s cap).
//! * **A two-processor board too.** `pc-apic` puts both processors on one
//!   crystal, where the scheduler arms *no* live view — the one that runs
//!   first must not drag a device into the future of the one that runs second.
//!   A read view catches nothing up, so it is available there as well, and the
//!   read path of each of these four devices uses it.
//! * **No comparator fires early, and none can be read past.** A read moves no
//!   device, so the 8254's terminal count, an HPET comparator and the local
//!   APIC timer's expiry still arrive on the tick the scheduler delivers them
//!   on; and a guest polling a counter sees it climb to its comparator and no
//!   further until the interrupt has been taken.
//! * **A timer starts where it was armed.** On one processor a write, like a
//!   read, now lands at the cycle of the instruction that made it, so a timer
//!   fires its whole interval after the instruction that armed it — where
//!   before it started at the round's beginning and fired up to a round early.
//!   On a crystal two processors share, a write still lands where the round
//!   began, exactly as before; the alarm tests pin both.
//! * **A halted processor's own counter counts the halt.** `HLT`, and the
//!   wait-for-SIPI an INIT leaves an application processor in, stop the
//!   *processor*, not its time-stamp counter (*Intel SDM* vol. 3B §17.17.1).
//!   That was a separate defect with a separate cause — `X86::run_budget`
//!   consumed a halted core's budget and charged it nothing — and the three
//!   tests at the bottom of this file are what it cost a guest.
//! * **Determinism.** The same program gives the same samples under
//!   [`ThreadingMode::Parallel`], because a read at one's own position
//!   involves nobody else.
//!
//! # Sources
//!
//! *Intel 82C54 CHMOS Programmable Interval Timer* datasheet (order 231244),
//! "Read Operations" — the counter-latch command and the two-byte read-back —
//! and the mode descriptions for what a counting element does between events.
//! *IA-PC HPET Specification* 1.0a §2.3.5 and §2.3.7 for the main counter and
//! what starts it, §2.3.8 for a comparator. The *Intel I/O Controller Hub 9
//! (ICH9) Family* datasheet §13.8.3.4 for `PM1_TMR`, its 24 bits and its
//! 3.579545 MHz clock. *Intel SDM* volume 3A §10.5.4 for the local APIC
//! timer's current-count register and §17.17 for the time-stamp counter. The
//! *MultiProcessor Specification* 1.4 §3.6.2.1 for the IMCR.
//!
//! [`TickCursor`]: rsemu::core::sched::TickCursor
//! [`ThreadingMode::Parallel`]: rsemu::core::sched::ThreadingMode::Parallel

#![cfg(all(
    feature = "cpu-x86",
    feature = "dev-pc",
    feature = "dev-pc-apic",
    feature = "dev-pc-hpet",
    feature = "std"
))]

extern crate alloc;
use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::sched::ThreadingMode;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{Machine, build};

// ---------------------------------------------------------------------------
// the boards
// ---------------------------------------------------------------------------

/// A two-processor board: both cores are on `osc cpu`, so neither is armed a
/// live view of anything and the read view is the only path there is.
const PC_APIC: &str = include_str!("../machines/pc-apic.machine");

/// A one-processor board that also carries the ACPI power-management timer.
#[cfg(feature = "dev-q35")]
const Q35: &str = include_str!("../machines/q35.machine");

/// How a board's BIOS socket is laid out: its size, and the linear address it
/// is based at.
#[derive(Debug, Clone, Copy)]
struct Socket {
    len: usize,
    base: u32,
}

/// `pc-apic` decodes 128 KiB at `0xe0000`; `q35` decodes 64 KiB at `0xf0000`.
/// Either way the firmware below lives in segment `0xf000`.
const PC_APIC_SOCKET: Socket = Socket {
    len: 128 * 1024,
    base: 0xe_0000,
};
#[cfg(feature = "dev-q35")]
const Q35_SOCKET: Socket = Socket {
    len: 64 * 1024,
    base: 0xf_0000,
};

impl Socket {
    /// Where segment `0xf000` starts inside the image.
    const fn seg_f000(self) -> usize {
        (0xf_0000 - self.base) as usize
    }
}

// Offsets within segment 0xf000. Linear addresses are `0xf0000 + off`.
const OFF_ENTRY: usize = 0x0000;
const OFF_GDT: usize = 0x0100;
const OFF_GDT_PTR: usize = 0x0120;
const OFF_PM: usize = 0x0200;

/// The linear address of an offset in segment 0xf000.
const fn lin(off: usize) -> u32 {
    0xf_0000 + off as u32
}

/// Where the guest writes its samples.
const SAMPLES: u32 = 0x4000;

/// How many times round the sampling loop. Each turn records a **pair** of
/// samples a few tens of cycles apart and then spins [`DELAY`], so the record
/// carries both questions at once: does a counter move between two adjacent
/// instructions, and does it keep moving across the round boundaries the
/// spinning crosses.
const PAIRS: u32 = 12;
const COUNT: u32 = 2 * PAIRS;

/// Turns of the spin between one pair and the next. Two instructions a turn on
/// a 25 MHz core is a few hundred microseconds, so a pair lands in most rounds
/// and the twelve of them span rather more than a dozen.
const DELAY: u32 = 2_000;

/// One sample: 8254, HPET, PM timer, local APIC current count, TSC.
const FIELDS: u32 = 5;
const STRIDE: u32 = 4 * FIELDS;

/// Which field of a sample.
const F_PIT: usize = 0;
const F_HPET: usize = 1;
const F_PMTMR: usize = 2;
const F_APIC: usize = 3;
const F_TSC: usize = 4;

/// The ACPI power-management timer's port on both q35 boards: `pmbase` is
/// 0x600 and `PM1_TMR` sits eight bytes into the block (ICH9 §13.8.3.4).
const PM_TMR_PORT: u16 = 0x608;

// ---------------------------------------------------------------------------
// the firmware
// ---------------------------------------------------------------------------

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

/// `out imm8, al`, for a port below 256.
fn outb(out: &mut Vec<u8>, port: u8, value: u8) {
    out.extend_from_slice(&[0xb0, value]); // mov al, imm8
    out.extend_from_slice(&[0xe6, port]); // out imm8, al
}

/// The sampling program: enter protected mode, arm every counter so that it
/// is moving and none of them can expire during the run, then take [`COUNT`]
/// samples of all five back to back.
///
/// `pm_timer` says whether the board has an ACPI block to read; a board
/// without one records zero in that field.
fn firmware(socket: Socket, pm_timer: bool) -> Vec<u8> {
    let (mut rom, mut pm) = boot(socket);
    sampler(&mut pm, pm_timer);
    put(&mut rom, socket, OFF_PM, &pm);
    rom
}

/// Everything both programs share: the reset vector, the switch to protected
/// mode through a flat GDT, and the protected-mode entry that loads the data
/// segments and a stack. Returns the image and the protected-mode code so far.
fn boot(socket: Socket) -> (Vec<u8>, Vec<u8>) {
    let mut rom = vec![0u8; socket.len];

    // -- the reset vector ---------------------------------------------------
    let reset = socket.len - 0x10;
    rom[reset..reset + 5].copy_from_slice(&[0xea, 0x00, 0x00, 0x00, 0xf0]);

    // -- real mode: enter protected mode ------------------------------------
    let mut entry: Vec<u8> = Vec::new();
    entry.push(0xfa); // cli
    entry.extend_from_slice(&[0xb8, 0x00, 0xf0]); // mov ax, 0xf000
    entry.extend_from_slice(&[0x8e, 0xd8]); // mov ds, ax
    entry.extend_from_slice(&[0x0f, 0x01, 0x16]); // lgdt [OFF_GDT_PTR]
    entry.extend_from_slice(&(OFF_GDT_PTR as u16).to_le_bytes());
    entry.extend_from_slice(&[0x0f, 0x20, 0xc0]); // mov eax, cr0
    entry.extend_from_slice(&[0x0c, 0x01]); // or al, 1
    entry.extend_from_slice(&[0x0f, 0x22, 0xc0]); // mov cr0, eax
    entry.extend_from_slice(&[0x66, 0xea]); // jmp far 0x08:OFF_PM
    dw(&mut entry, lin(OFF_PM));
    entry.extend_from_slice(&[0x08, 0x00]);
    put(&mut rom, socket, OFF_ENTRY, &entry);

    // -- the descriptor table -----------------------------------------------
    let gdt: [u8; 24] = [
        0, 0, 0, 0, 0, 0, 0, 0, // null
        0xff, 0xff, 0, 0, 0, 0x9a, 0xcf, 0, // flat code, ring 0, 32-bit
        0xff, 0xff, 0, 0, 0, 0x92, 0xcf, 0, // flat data, ring 0
    ];
    put(&mut rom, socket, OFF_GDT, &gdt);
    let mut gdt_ptr = Vec::new();
    gdt_ptr.extend_from_slice(&(gdt.len() as u16 - 1).to_le_bytes());
    dw(&mut gdt_ptr, lin(OFF_GDT));
    put(&mut rom, socket, OFF_GDT_PTR, &gdt_ptr);

    // -- protected mode -----------------------------------------------------
    let mut pm: Vec<u8> = Vec::new();
    pm.extend_from_slice(&[0xb8, 0x10, 0x00, 0x00, 0x00]); // mov eax, 0x10
    pm.extend_from_slice(&[0x8e, 0xd8]); // mov ds, ax
    pm.extend_from_slice(&[0x8e, 0xc0]); // mov es, ax
    pm.extend_from_slice(&[0x8e, 0xd0]); // mov ss, ax
    pm.push(0xbc); // mov esp, 0xf000
    dw(&mut pm, 0xf000);
    (rom, pm)
}

/// The sampling program proper, appended to [`boot`]'s protected-mode entry.
fn sampler(pm: &mut Vec<u8>, pm_timer: bool) {
    // The 8254's counter 0: mode 2, binary, low-then-high, and the largest
    // count that is not the modulus — 65 535 of a 105/88 MHz crystal is 54.9
    // ms, so the counter free-runs and its terminal count is far outside any
    // round this test takes. Not zero, which the datasheet's "Write
    // Operations" makes the full modulus and which therefore reads back as the
    // same 0 an unloaded counting element shows.
    outb(pm, 0x43, 0x34);
    outb(pm, 0x40, 0xff);
    outb(pm, 0x40, 0xff);

    // The HPET: start the main counter (`ENABLE_CNF`, §2.3.5). No comparator
    // is enabled, so nothing in this part has an event of its own.
    pm.push(0xbf); // mov edi, 0xfed00000
    dw(pm, 0xfed0_0000);
    store_at(pm, 0x010, 1);
    store_at(pm, 0x014, 0);

    // The local APIC: software-enable it, divide by one, and start a one-shot
    // timer whose LVT entry is **masked** — the count is what is being read,
    // and 0xffffffff of a 100 MHz bus clock is 42 seconds, so it cannot reach
    // zero here (SDM Vol 3A §10.5.4).
    pm.push(0xbf); // mov edi, 0xfee00000
    dw(pm, 0xfee0_0000);
    store_at(pm, 0x0f0, 0x1ff);
    store_at(pm, 0x3e0, 0b1011);
    store_at(pm, 0x320, (1 << 16) | 0x40);
    store_at(pm, 0x380, 0xffff_ffff);

    // -- the sampling loop --------------------------------------------------
    //
    // `edi` walks the record, `ecx` counts the pairs down. One turn records
    // two samples back to back — the tight pair — and then spins.
    pm.push(0xbf); // mov edi, SAMPLES
    dw(pm, SAMPLES);
    // One spin before the first pair, so that the clock pulse the 8254 takes
    // to transfer a written count into its counting element has certainly
    // happened and the first sample is a count rather than a null one.
    pm.push(0xb8); // mov eax, DELAY
    dw(pm, DELAY);
    pm.push(0x48); // dec eax
    pm.extend_from_slice(&[0x75, 0xfd]); // jnz -3
    pm.push(0xb9); // mov ecx, PAIRS
    dw(pm, PAIRS);

    /// One sample, written at `edi + slot * STRIDE`.
    fn sample(out: &mut Vec<u8>, slot: u32, pm_timer: bool) {
        let at = |field: usize| (slot * STRIDE + 4 * field as u32) as u8;
        // The 8254: a counter-latch command for counter 0, then the two bytes
        // it freezes (82C54 datasheet, "Counter Latch Command").
        out.extend_from_slice(&[0xb0, 0x00]); // mov al, 0
        out.extend_from_slice(&[0xe6, 0x43]); // out 0x43, al
        out.extend_from_slice(&[0xe4, 0x40]); // in al, 0x40
        out.extend_from_slice(&[0x88, 0xc3]); // mov bl, al
        out.extend_from_slice(&[0xe4, 0x40]); // in al, 0x40
        out.extend_from_slice(&[0x88, 0xc7]); // mov bh, al
        out.extend_from_slice(&[0x0f, 0xb7, 0xc3]); // movzx eax, bx
        out.extend_from_slice(&[0x89, 0x47, at(F_PIT)]);
        // The HPET main counter's low half (§2.3.7).
        out.push(0xa1); // mov eax, [0xfed000f0]
        dw(out, 0xfed0_00f0);
        out.extend_from_slice(&[0x89, 0x47, at(F_HPET)]);
        // The ACPI power-management timer.
        if pm_timer {
            out.extend_from_slice(&[0x66, 0xba]); // mov dx, PM_TMR_PORT
            out.extend_from_slice(&PM_TMR_PORT.to_le_bytes());
            out.push(0xed); // in eax, dx
        } else {
            out.extend_from_slice(&[0x31, 0xc0]); // xor eax, eax
        }
        out.extend_from_slice(&[0x89, 0x47, at(F_PMTMR)]);
        // The local APIC timer's current count.
        out.push(0xa1); // mov eax, [0xfee00390]
        dw(out, 0xfee0_0390);
        out.extend_from_slice(&[0x89, 0x47, at(F_APIC)]);
        // The time-stamp counter's low half.
        out.extend_from_slice(&[0x0f, 0x31]); // rdtsc
        out.extend_from_slice(&[0x89, 0x47, at(F_TSC)]);
    }

    let mut body: Vec<u8> = Vec::new();
    sample(&mut body, 0, pm_timer);
    sample(&mut body, 1, pm_timer);
    body.extend_from_slice(&[0x83, 0xc7, (2 * STRIDE) as u8]); // add edi, 2*STRIDE
    // The spin between one pair and the next.
    body.push(0xb8); // mov eax, DELAY
    dw(&mut body, DELAY);
    body.push(0x48); // dec eax
    body.extend_from_slice(&[0x75, 0xfd]); // jnz -3
    body.push(0x49); // dec ecx
    let back = -((body.len() + 2) as i32);
    body.extend_from_slice(&[0x0f, 0x85]); // jnz rel32 — the body is long
    dw(&mut body, (back - 4) as u32);
    pm.extend_from_slice(&body);

    pm.extend_from_slice(&[0xeb, 0xfe]); // jmp $
}

// ---------------------------------------------------------------------------
// the alarm program: when does a comparator fire, against the instruction
// that armed it?
// ---------------------------------------------------------------------------

/// Where the guest records the time-stamp counter immediately before the
/// arming access, and in the interrupt handler.
const ARMED_AT: u32 = 0x5000;
const SEEN_AT: u32 = 0x5008;
/// How many times the handler ran.
const TAKEN: u32 = 0x5010;

const OFF_IDT_PTR: usize = 0x0128;
const OFF_HANDLER: usize = 0x0180;
const IDT_BASE: u32 = 0x2000;
const ALARM_VECTOR: u8 = 0x40;

/// Every timer below is armed for about half a millisecond — 12 500 cycles of
/// the 25 MHz core both boards declare — and armed **mid-round**, after a
/// spin of about as long again, so that where in its round the arming access
/// lands is not zero.
const APIC_ALARM: u32 = 50_000; // bus ticks at 100 MHz
const HPET_ALARM: u32 = 5_000; // ticks at 10 MHz
const PIT_ALARM: u16 = 597; // ticks at 105/88 MHz: 12 508.6 cycles
/// The same half-millisecond, in core cycles, per source — the shortest time
/// after the arming access at which that source's interrupt may arrive.
const APIC_CYCLES: u64 = 12_500;
const HPET_CYCLES: u64 = 12_500;
const PIT_CYCLES: u64 = 12_508;
/// The spin before arming, in turns of `dec eax; jnz`.
const ARM_SPIN: u32 = 1_700;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Alarm {
    /// The local APIC timer, one-shot. Reaches the core without an I/O APIC.
    Apic,
    /// HPET comparator 0, one-shot, edge-triggered, through the I/O APIC
    /// input the board wires it to.
    Hpet(u8),
    /// The 8254's counter 0 in mode 0 — interrupt on terminal count — through
    /// I/O APIC input 2.
    Pit,
}

/// What the processor does between arming its timer and taking the interrupt.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Wait {
    /// `jmp $`: it executes, and charges, every cycle of the wait.
    Spin,
    /// `hlt`: it executes nothing until the interrupt arrives, which is where
    /// a time-stamp counter that stopped with it would show.
    Halt,
}

/// A program that arms one timer after a spin and records, with `RDTSC`, the
/// cycle it armed on and the cycle its interrupt handler first ran.
///
/// `imcr` says the board has an interrupt mode configuration register, which
/// powers up in PIC mode and hands the 8259A's output straight to the core
/// (MultiProcessor Specification 1.4 §3.6.2.1); the program selects symmetric
/// I/O mode through it first, or the 8254's edge would reach an unprogrammed
/// 8259A and arrive on a vector nothing handles.
fn alarm(socket: Socket, source: Alarm, imcr: bool, wait: Wait) -> Vec<u8> {
    let (mut rom, mut pm) = boot(socket);
    if imcr {
        outb(&mut pm, 0x22, 0x70); // select the IMCR
        outb(&mut pm, 0x23, 0x01); // and route the 8259A away from INTR
    }

    // One interrupt gate at IDT_BASE + 8 * vector.
    pm.push(0xbf); // mov edi, gate
    dw(&mut pm, IDT_BASE + 8 * u32::from(ALARM_VECTOR));
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

    // The local APIC, software-enabled (SDM Vol 3A §10.4.7.2).
    pm.push(0xbf); // mov edi, 0xfee00000
    dw(&mut pm, 0xfee0_0000);
    store_at(&mut pm, 0x0f0, 0x1ff);

    // The I/O APIC entry, edge-triggered, active high, fixed to APIC 0
    // (82093AA §3.2.4) — high half first, so it is never briefly live with a
    // destination nobody wrote.
    let input = match source {
        Alarm::Apic => None,
        Alarm::Hpet(input) => Some(input),
        Alarm::Pit => Some(2),
    };
    if let Some(input) = input {
        pm.push(0xbf); // mov edi, 0xfec00000
        dw(&mut pm, 0xfec0_0000);
        let index = 0x10 + 2 * u32::from(input);
        store_at(&mut pm, 0x00, index + 1);
        store_at(&mut pm, 0x10, 0);
        store_at(&mut pm, 0x00, index);
        store_at(&mut pm, 0x10, u32::from(ALARM_VECTOR));
    }

    // Everything but the arming access itself.
    match source {
        Alarm::Apic => {
            pm.push(0xbf); // mov edi, 0xfee00000
            dw(&mut pm, 0xfee0_0000);
            store_at(&mut pm, 0x3e0, 0b1011); // divide by one
            store_at(&mut pm, 0x320, u32::from(ALARM_VECTOR)); // one-shot, unmasked
        }
        Alarm::Hpet(_) => {
            // Comparator 0 enabled, edge-triggered, one-shot, matching
            // HPET_ALARM; the counter is at zero and halted until the arming
            // write sets `ENABLE_CNF` (§2.3.5, §2.3.8).
            pm.push(0xbf); // mov edi, 0xfed00000
            dw(&mut pm, 0xfed0_0000);
            store_at(&mut pm, 0x100, 0b100); // Tn_INT_ENB_CNF
            store_at(&mut pm, 0x104, 0);
            store_at(&mut pm, 0x108, HPET_ALARM);
            store_at(&mut pm, 0x10c, 0);
        }
        Alarm::Pit => {
            // Counter 0, mode 0, low-then-high, binary; the low byte now. The
            // count is armed by the high byte, and loaded on the next clock
            // (82C54 data sheet, mode 0).
            outb(&mut pm, 0x43, 0x30);
            outb(&mut pm, 0x40, PIT_ALARM as u8);
        }
    }

    // Land the arming access mid-round.
    pm.push(0xb8); // mov eax, ARM_SPIN
    dw(&mut pm, ARM_SPIN);
    pm.push(0x48); // dec eax
    pm.extend_from_slice(&[0x75, 0xfd]); // jnz -3

    pm.push(0xfb); // sti
    pm.extend_from_slice(&[0x0f, 0x31]); // rdtsc
    pm.push(0xa3); // mov [ARMED_AT], eax
    dw(&mut pm, ARMED_AT);
    pm.extend_from_slice(&[0x89, 0x15]); // mov [ARMED_AT+4], edx
    dw(&mut pm, ARMED_AT + 4);
    match source {
        Alarm::Apic => store_at(&mut pm, 0x380, APIC_ALARM),
        Alarm::Hpet(_) => store_at(&mut pm, 0x010, 1),
        Alarm::Pit => outb(&mut pm, 0x40, (PIT_ALARM >> 8) as u8),
    }
    if wait == Wait::Halt {
        pm.push(0xf4); // hlt
    }
    pm.extend_from_slice(&[0xeb, 0xfe]); // jmp $
    put(&mut rom, socket, OFF_PM, &pm);

    let mut idt_ptr = Vec::new();
    idt_ptr.extend_from_slice(&0x07ffu16.to_le_bytes());
    dw(&mut idt_ptr, IDT_BASE);
    put(&mut rom, socket, OFF_IDT_PTR, &idt_ptr);

    // The handler: the cycle it ran on, a count, an end-of-interrupt.
    let mut h: Vec<u8> = Vec::new();
    h.extend_from_slice(&[0x50, 0x52]); // push eax; push edx
    h.extend_from_slice(&[0x0f, 0x31]); // rdtsc
    h.push(0xa3); // mov [SEEN_AT], eax
    dw(&mut h, SEEN_AT);
    h.extend_from_slice(&[0x89, 0x15]); // mov [SEEN_AT+4], edx
    dw(&mut h, SEEN_AT + 4);
    h.extend_from_slice(&[0xff, 0x05]); // inc dword [TAKEN]
    dw(&mut h, TAKEN);
    h.extend_from_slice(&[0xc7, 0x05]); // mov dword [0xfee000b0], 0 — EOI
    dw(&mut h, 0xfee0_00b0);
    dw(&mut h, 0);
    h.extend_from_slice(&[0x5a, 0x58]); // pop edx; pop eax
    h.push(0xcf); // iret
    put(&mut rom, socket, OFF_HANDLER, &h);
    rom
}

fn put(rom: &mut [u8], socket: Socket, off: usize, bytes: &[u8]) {
    let at = socket.seg_f000() + off;
    rom[at..at + bytes.len()].copy_from_slice(bytes);
}

// ---------------------------------------------------------------------------
// running it
// ---------------------------------------------------------------------------

fn board(name: &str, text: &str, socket: Socket, pm_timer: bool, mode: ThreadingMode) -> Machine {
    board_with(name, text, firmware(socket, pm_timer), mode)
}

fn board_with(name: &str, text: &str, image: Vec<u8>, mode: ThreadingMode) -> Machine {
    let mut options = rsemu::machine::catalog::build_options().expect("this build's classes");
    options.realize.media.insert("bios", image);
    // `q35` declares a video option-ROM socket and a hard disk and refuses to
    // realize with nothing bound to either. An empty image is an empty socket
    // and a drive with no platter, which is what this test wants: it reads
    // clocks and nothing else.
    for slot in ["vgabios", "hd0", "hd1", "floppy"] {
        options.realize.media.insert(slot, Vec::new());
    }
    options.realize.threading = Some(mode);
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    match build(name, text, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("{name} does not realize: {e}"),
    }
}

/// Run the sampling program and hand back one column of the record.
fn samples(m: &mut Machine, field: usize) -> Vec<u32> {
    let mem = m.space("mem").expect("the memory space");
    (0..COUNT)
        .map(|i| {
            let at = u64::from(SAMPLES + i * STRIDE + 4 * field as u32);
            mem.read(at, Width::U32, MemAttrs::DEBUG)
                .expect("a mapped word") as u32
        })
        .collect()
}

/// Run long enough for the guest to finish sampling. Several rounds, so the
/// record spans round boundaries — which is where the staircase was visible
/// and where monotonicity across a boundary is asserted.
fn run(m: &mut Machine) {
    m.reset(ResetKind::Cold);
    m.sweep();
    m.run_for(GlobalTime::from_nanos(20_000_000))
        .expect("the machine runs");
}

/// The same run, driven the way `tests/x86boot` drives a kernel: one
/// millisecond of virtual time per call, so that every round boundary is also
/// a `run_for` boundary.
fn run_sliced(m: &mut Machine) {
    m.reset(ResetKind::Cold);
    m.sweep();
    for _ in 0..20 {
        m.run_for(GlobalTime::from_nanos(1_000_000))
            .expect("the machine runs");
    }
}

/// The gaps between consecutive samples of one column, as signed differences.
fn gaps(values: &[u32]) -> Vec<i64> {
    values
        .windows(2)
        .map(|w| i64::from(w[1]) - i64::from(w[0]))
        .collect()
}

/// The 8254 counts **down**, so its gaps are negative; every other counter
/// here counts up. Normalise to "how far the counter moved".
fn moved(values: &[u32], down: bool) -> Vec<i64> {
    gaps(values)
        .into_iter()
        .map(|d| if down { -d } else { d })
        .collect()
}

// ---------------------------------------------------------------------------
// what a guest sees
// ---------------------------------------------------------------------------

/// The measurement the defect was found by, on the board it was found on.
#[test]
fn every_counter_moves_between_two_reads_a_few_cycles_apart_on_two_processors() {
    let mut m = board(
        "pc-apic.machine",
        PC_APIC,
        PC_APIC_SOCKET,
        false,
        ThreadingMode::Deterministic,
    );
    run(&mut m);
    report(&mut m, "pc-apic, two processors on one crystal", false);
    assert_counters_move(&mut m, false);
}

#[cfg(feature = "dev-q35")]
#[test]
fn every_counter_moves_between_two_reads_a_few_cycles_apart_on_one_processor() {
    let mut m = board(
        "q35.machine",
        Q35,
        Q35_SOCKET,
        true,
        ThreadingMode::Deterministic,
    );
    run(&mut m);
    report(&mut m, "q35, one processor", true);
    assert_counters_move(&mut m, true);
}

/// Every counter this board has, as `(name, field, counts down)`.
fn columns(pm_timer: bool) -> Vec<(&'static str, usize, bool)> {
    let mut all = vec![
        ("8254 counter 0", F_PIT, true),
        ("HPET main counter", F_HPET, false),
        ("local APIC current count", F_APIC, true),
        ("TSC", F_TSC, false),
    ];
    if pm_timer {
        all.push(("ACPI PM timer", F_PMTMR, false));
    }
    all
}

/// Print the whole record. `cargo test -- --nocapture` is how the numbers in
/// this file's commit message were taken, before and after.
fn report(m: &mut Machine, what: &str, pm_timer: bool) {
    println!("-- {what}");
    for (name, field, down) in columns(pm_timer) {
        let values = samples(m, field);
        println!("   {name:<26} {values:?}");
        println!("   {:<26} {:?}", "  moved", moved(&values, down));
    }
}

fn assert_counters_move(m: &mut Machine, pm_timer: bool) {
    let tsc = samples(m, F_TSC);
    assert!(
        tsc.windows(2).all(|w| w[1] > w[0]),
        "the time-stamp counter is the core's own cycle count and was never \
         round-grained; if this fails the program did not run: {tsc:?}"
    );

    for (name, field, down) in [
        ("the 8254's counter 0", F_PIT, true),
        ("the HPET main counter", F_HPET, false),
        ("the local APIC timer's current count", F_APIC, true),
    ]
    .into_iter()
    .chain(pm_timer.then_some(("the ACPI PM timer", F_PMTMR, false)))
    {
        let values = samples(m, field);
        let steps = moved(&values, down);
        assert!(
            steps.iter().all(|d| *d > 0),
            "{name} stood still between two reads a few tens of cycles apart, \
             which is the staircase this file exists to refuse: {values:?}"
        );
    }
}

/// Nothing ever runs backwards, round boundary or not.
#[test]
fn no_counter_runs_backwards_across_a_round_boundary() {
    let mut m = board(
        "pc-apic.machine",
        PC_APIC,
        PC_APIC_SOCKET,
        false,
        ThreadingMode::Deterministic,
    );
    run(&mut m);
    for (name, field, down) in [
        ("the 8254's counter 0", F_PIT, true),
        ("the HPET main counter", F_HPET, false),
        ("the local APIC timer's current count", F_APIC, true),
        ("the time-stamp counter", F_TSC, false),
    ] {
        let values = samples(&mut m, field);
        assert!(
            moved(&values, down).iter().all(|d| *d >= 0),
            "{name} ran backwards: {values:?}"
        );
    }
}

/// A read at one's own position involves nobody else, so dispatching the round
/// over a task pool cannot change a single sample.
#[test]
fn the_same_program_reads_the_same_counters_under_a_dispatched_round() {
    let mut a = board(
        "pc-apic.machine",
        PC_APIC,
        PC_APIC_SOCKET,
        false,
        ThreadingMode::Deterministic,
    );
    let mut b = board(
        "pc-apic.machine",
        PC_APIC,
        PC_APIC_SOCKET,
        false,
        ThreadingMode::Parallel,
    );
    run(&mut a);
    run(&mut b);
    for field in 0..FIELDS as usize {
        assert_eq!(
            samples(&mut a, field),
            samples(&mut b, field),
            "column {field} differs between the deterministic and the \
             dispatched round"
        );
    }
}

// ---------------------------------------------------------------------------
// comparators: where they fire, against the instruction that armed them
// ---------------------------------------------------------------------------

/// Run the alarm program for `source` and report `(interrupts taken, cycles
/// from the arming access to the handler)`.
fn alarm_latency(
    name: &str,
    text: &str,
    socket: Socket,
    source: Alarm,
    imcr: bool,
    wait: Wait,
) -> (u32, u64) {
    let mut m = board_with(
        name,
        text,
        alarm(socket, source, imcr, wait),
        ThreadingMode::Deterministic,
    );
    m.reset(ResetKind::Cold);
    m.sweep();
    m.run_for(GlobalTime::from_nanos(5_000_000))
        .expect("the machine runs");
    let mem = m.space("mem").expect("the memory space");
    let peek = |at: u32| {
        mem.read(u64::from(at), Width::U64, MemAttrs::DEBUG)
            .expect("a mapped word")
    };
    let taken = peek(TAKEN) as u32;
    let latency = peek(SEEN_AT).wrapping_sub(peek(ARMED_AT));
    println!("-- {name:<16} {source:?}, {wait:?}: taken {taken}, {latency} cycles after arming");
    (taken, latency)
}

/// Every source, with the shortest interval after the arming access at which
/// its interrupt may arrive.
fn alarms(hpet_input: u8) -> [(Alarm, u64); 3] {
    [
        (Alarm::Apic, APIC_CYCLES),
        (Alarm::Hpet(hpet_input), HPET_CYCLES),
        (Alarm::Pit, PIT_CYCLES),
    ]
}

/// The whole interval, and not a cycle less.
///
/// Before `cpu.x86` published its position, the write that arms a timer was
/// applied to a device caught up only to where the round began, so the timer
/// started counting up to a round before the instruction that armed it and
/// fired that much early. Measured on this board with this program, before and
/// after, in cycles from the arming access to the handler against a 12 500
/// floor: APIC 3 615 → 12 646, HPET 3 450 → 12 646, 8254 3 532 → 12 662. What
/// is left above the floor is the `RDTSC` before the arming store and the
/// interrupt's own entry.
#[cfg(feature = "dev-q35")]
#[test]
fn a_timer_armed_on_one_processor_fires_its_whole_interval_after_the_arming_instruction() {
    for (source, floor) in alarms(16) {
        let (taken, latency) =
            alarm_latency("q35.machine", Q35, Q35_SOCKET, source, true, Wait::Spin);
        assert_eq!(taken, 1, "{source:?} interrupted exactly once");
        assert!(
            latency >= floor,
            "{source:?} fired {latency} cycles after the instruction that armed \
             it, short of the {floor} it was programmed for"
        );
        assert!(
            latency <= floor + 400,
            "{source:?} fired {latency} cycles after arming, far past {floor}"
        );
    }
}

/// On a crystal two processors share, nothing is caught up inside a round, so
/// an arming write still lands where the round began — as it did before this
/// file existed. The read path changed; the write path did not, and the
/// measured instants are the same to the cycle: APIC 3 659, HPET 3 494, 8254
/// 3 565, before and after. What is asserted is the bound that arrangement
/// keeps, and that the dispatched round agrees with the deterministic one.
#[test]
fn a_timer_armed_on_a_shared_crystal_fires_where_it_always_has() {
    // One round of the 25 MHz core.
    const ROUND_CYCLES: u64 = 25_000;
    for (source, floor) in alarms(20) {
        let (taken, latency) = alarm_latency(
            "pc-apic.machine",
            PC_APIC,
            PC_APIC_SOCKET,
            source,
            false,
            Wait::Spin,
        );
        assert_eq!(taken, 1, "{source:?} interrupted exactly once");
        assert!(
            latency + ROUND_CYCLES >= floor,
            "{source:?} fired {latency} cycles after arming: more than a round \
             short of {floor}"
        );
    }
}

// ---------------------------------------------------------------------------
// a reader cannot see past a comparator before it fires
// ---------------------------------------------------------------------------

/// Where the handler says it has run, and where the polling loop records.
const FIRED: u32 = 0x5100;
const POLL: u32 = 0x1_0000;
const POLL_END: u32 = 0x1_c000;
/// The comparator, in HPET ticks: a millisecond and a half, so it falls in the
/// middle of the second round.
const COMPARATOR: u32 = 15_000;

/// Arm HPET comparator 0 at [`COMPARATOR`], start the counter, and poll it as
/// fast as a loop can, recording beside each value whether the interrupt
/// handler had run by then.
fn poller(socket: Socket, input: u8) -> Vec<u8> {
    let (mut rom, mut pm) = boot(socket);
    pm.push(0xbf); // mov edi, gate
    dw(&mut pm, IDT_BASE + 8 * u32::from(ALARM_VECTOR));
    pm.push(0xb8); // mov eax, handler
    dw(&mut pm, lin(OFF_HANDLER));
    pm.extend_from_slice(&[0x66, 0x89, 0x07]); // mov [edi], ax
    pm.extend_from_slice(&[0x66, 0xc7, 0x47, 0x02, 0x08, 0x00]);
    pm.extend_from_slice(&[0xc6, 0x47, 0x04, 0x00]);
    pm.extend_from_slice(&[0xc6, 0x47, 0x05, 0x8e]);
    pm.extend_from_slice(&[0xc1, 0xe8, 0x10]); // shr eax, 16
    pm.extend_from_slice(&[0x66, 0x89, 0x47, 0x06]);
    pm.extend_from_slice(&[0x0f, 0x01, 0x1d]); // lidt
    dw(&mut pm, lin(OFF_IDT_PTR));

    pm.push(0xbf); // mov edi, 0xfee00000
    dw(&mut pm, 0xfee0_0000);
    store_at(&mut pm, 0x0f0, 0x1ff);
    pm.push(0xbf); // mov edi, 0xfec00000
    dw(&mut pm, 0xfec0_0000);
    let index = 0x10 + 2 * u32::from(input);
    store_at(&mut pm, 0x00, index + 1);
    store_at(&mut pm, 0x10, 0);
    store_at(&mut pm, 0x00, index);
    store_at(&mut pm, 0x10, u32::from(ALARM_VECTOR));

    pm.push(0xbf); // mov edi, 0xfed00000
    dw(&mut pm, 0xfed0_0000);
    store_at(&mut pm, 0x100, 0b100); // enabled, edge, one-shot
    store_at(&mut pm, 0x104, 0);
    store_at(&mut pm, 0x108, COMPARATOR);
    store_at(&mut pm, 0x10c, 0);
    store_at(&mut pm, 0x010, 1); // ENABLE_CNF
    pm.push(0xfb); // sti

    pm.push(0xbf); // mov edi, POLL
    dw(&mut pm, POLL);
    // The counter read is bracketed by two reads of the handler's flag, and
    // the record is their sum: 0 read wholly before the interrupt, 2 wholly
    // after, 1 with the interrupt taken somewhere in between.
    let top = pm.len();
    pm.extend_from_slice(&[0x8b, 0x1d]); // mov ebx, [FIRED]
    dw(&mut pm, FIRED);
    pm.push(0xa1); // mov eax, [0xfed000f0]
    dw(&mut pm, 0xfed0_00f0);
    pm.extend_from_slice(&[0x8b, 0x0d]); // mov ecx, [FIRED]
    dw(&mut pm, FIRED);
    pm.extend_from_slice(&[0x89, 0x07]); // mov [edi], eax
    pm.extend_from_slice(&[0x01, 0xcb]); // add ebx, ecx
    pm.extend_from_slice(&[0x89, 0x5f, 0x04]); // mov [edi+4], ebx
    pm.extend_from_slice(&[0x83, 0xc7, 0x08]); // add edi, 8
    pm.extend_from_slice(&[0x81, 0xff]); // cmp edi, POLL_END
    dw(&mut pm, POLL_END);
    let back = top as i32 - (pm.len() as i32 + 2);
    pm.extend_from_slice(&[0x72, back as i8 as u8]); // jb top
    pm.extend_from_slice(&[0xeb, 0xfe]); // jmp $
    put(&mut rom, socket, OFF_PM, &pm);

    let mut idt_ptr = Vec::new();
    idt_ptr.extend_from_slice(&0x07ffu16.to_le_bytes());
    dw(&mut idt_ptr, IDT_BASE);
    put(&mut rom, socket, OFF_IDT_PTR, &idt_ptr);

    let mut h: Vec<u8> = Vec::new();
    h.extend_from_slice(&[0xc7, 0x05]); // mov dword [FIRED], 1
    dw(&mut h, FIRED);
    dw(&mut h, 1);
    h.extend_from_slice(&[0xc7, 0x05]); // EOI
    dw(&mut h, 0xfee0_00b0);
    dw(&mut h, 0);
    h.push(0xcf); // iret
    put(&mut rom, socket, OFF_HANDLER, &h);
    rom
}

/// The HPET values a guest read before its comparator's interrupt was taken,
/// and the first one it read after.
fn poll(mode: ThreadingMode) -> (Vec<u32>, Option<u32>) {
    let mut m = board_with("pc-apic.machine", PC_APIC, poller(PC_APIC_SOCKET, 20), mode);
    m.reset(ResetKind::Cold);
    m.sweep();
    m.run_for(GlobalTime::from_nanos(5_000_000))
        .expect("the machine runs");
    let mem = m.space("mem").expect("the memory space");
    let mut before = Vec::new();
    let mut after = None;
    for at in (POLL..POLL_END).step_by(8) {
        let word = |a: u32| {
            mem.read(u64::from(a), Width::U32, MemAttrs::DEBUG)
                .expect("a mapped word") as u32
        };
        match (word(at), word(at + 4)) {
            (value, 0) => before.push(value),
            // The interrupt landed inside the bracket: this read is neither.
            (_, 1) => {}
            (value, _) => {
                after = Some(value);
                break;
            }
        }
    }
    (before, after)
}

/// **A guest cannot read its way past a comparator.** On a crystal two
/// processors share, the HPET is read at each reader's own position — and that
/// position is capped where the round closes, which is no later than the
/// comparator's own event. So the counter a guest polls climbs right up to the
/// comparator and no further until the interrupt has been delivered.
///
/// Measured with this program, the last value read before the interrupt: 10 000
/// before this change — the round's start, stale for half a millisecond — and
/// the comparator itself after.
#[test]
fn a_polled_counter_climbs_to_its_comparator_and_not_past_it_before_the_interrupt() {
    for mode in [ThreadingMode::Deterministic, ThreadingMode::Parallel] {
        let (before, after) = poll(mode);
        let last = *before.last().expect("the loop polled before the interrupt");
        println!(
            "-- {mode:?}: {} reads before the interrupt, last {last}, first after {after:?}",
            before.len()
        );
        assert!(
            before.iter().all(|v| *v <= COMPARATOR),
            "{mode:?}: a read before the interrupt was past the comparator at \
             {COMPARATOR}: {:?}",
            before
                .iter()
                .filter(|v| **v > COMPARATOR)
                .collect::<Vec<_>>()
        );
        assert!(
            last + 100 >= COMPARATOR,
            "{mode:?}: the last read before the interrupt was {last}, so the \
             reads were not live up to the comparator at {COMPARATOR}"
        );
        assert!(
            before.windows(2).all(|w| w[1] >= w[0]),
            "{mode:?}: the counter ran backwards"
        );
        assert!(
            after.is_some_and(|v| v >= COMPARATOR),
            "{mode:?}: the first read after the interrupt, {after:?}, was short of \
             the comparator that raised it"
        );
    }
}

// ---------------------------------------------------------------------------
// what publishing costs
// ---------------------------------------------------------------------------

/// A workload whose guest work cannot depend on any clock: a load, an add and
/// a store per turn over a 64 KiB buffer, no timer read, interrupts off, for
/// fifty milliseconds of `q35`. Every bus access — and under the interpreter
/// every fetch is one — publishes the core's position, so this is the hot path
/// at its most exposed, and two builds run exactly the same guest instructions
/// on it. Ignored: it is a measurement, taken as
///
/// ```text
/// valgrind --tool=callgrind <this test binary> --ignored --exact \
///     publishing_cost_workload
/// ```
///
/// on each build, and the difference in host instructions divided by the guest
/// instructions both executed.
#[cfg(feature = "dev-q35")]
#[test]
#[ignore = "a measurement: run it under callgrind"]
fn publishing_cost_workload() {
    let (mut rom, mut pm) = boot(Q35_SOCKET);
    pm.push(0xb9); // mov ecx, a count it never reaches
    dw(&mut pm, 0x7fff_ffff);
    pm.push(0xbe); // mov esi, 0x20000
    dw(&mut pm, 0x2_0000);
    let top = pm.len();
    pm.extend_from_slice(&[0x8b, 0x06]); // mov eax, [esi]
    pm.extend_from_slice(&[0x01, 0xc8]); // add eax, ecx
    pm.extend_from_slice(&[0x89, 0x06]); // mov [esi], eax
    pm.extend_from_slice(&[0x83, 0xc6, 0x04]); // add esi, 4
    pm.extend_from_slice(&[0x81, 0xe6]); // and esi, 0x2ffff
    dw(&mut pm, 0x2_ffff);
    pm.push(0x49); // dec ecx
    let back = top as i32 - (pm.len() as i32 + 2);
    pm.extend_from_slice(&[0x75, back as i8 as u8]); // jnz top
    pm.extend_from_slice(&[0xeb, 0xfe]); // jmp $
    put(&mut rom, Q35_SOCKET, OFF_PM, &pm);

    let mut m = board_with("q35.machine", Q35, rom, ThreadingMode::Deterministic);
    m.reset(ResetKind::Cold);
    m.sweep();
    m.run_for(GlobalTime::from_nanos(50_000_000))
        .expect("the machine runs");
    // The same hash on both builds is what says the guest work was identical.
    println!("state hash {:#018x}", m.state_hash().expect("a hash"));
}

/// **The rate a guest measures is the rate the machine file declares** — on
/// one processor and on two, and whether the caller ran the machine in one
/// call or in one-millisecond ones.
///
/// This is the shape of every clock calibration a guest performs: count one
/// counter against another over a window of many rounds. It is not enough
/// that a counter moves inside a round; the *rate* it moves at has to be the
/// declared one over the whole window, or a guest calibrates itself wrong.
/// Both boards put the core on 25 MHz and the 8254 on 105/88 MHz, so the
/// answer is 20.952 cycles per tick.
///
/// The tolerance is **0.5%**. The measurement's own quantisation is one 8254
/// tick over a window of some twelve thousand, under 0.01%, so the bound is
/// two orders of magnitude looser than the noise and two orders tighter than
/// either defect it stands against: reads pinned to the round's start read
/// 22.430 on two processors, and a passive crystal ageing through declined
/// fragments read 20.418 when the caller sliced.
#[test]
fn the_rate_a_guest_measures_is_the_declared_one_however_it_is_driven() {
    for (how, drive, board_name) in [
        ("two cpus, one call", run as fn(&mut Machine), "pc-apic"),
        ("two cpus, 1 ms", run_sliced as fn(&mut Machine), "pc-apic"),
        #[cfg(feature = "dev-q35")]
        ("one cpu, one call", run as fn(&mut Machine), "q35"),
        #[cfg(feature = "dev-q35")]
        ("one cpu, 1 ms", run_sliced as fn(&mut Machine), "q35"),
    ] {
        let mut m = match board_name {
            #[cfg(feature = "dev-q35")]
            "q35" => board(
                "q35.machine",
                Q35,
                Q35_SOCKET,
                true,
                ThreadingMode::Deterministic,
            ),
            _ => board(
                "pc-apic.machine",
                PC_APIC,
                PC_APIC_SOCKET,
                false,
                ThreadingMode::Deterministic,
            ),
        };
        drive(&mut m);
        let (pit, tsc) = (samples(&mut m, F_PIT), samples(&mut m, F_TSC));
        let (first, last) = (0, COUNT as usize - 1);
        let ticks = i64::from(pit[first]) - i64::from(pit[last]);
        let cycles = i64::from(tsc[last]) - i64::from(tsc[first]);
        // 25 MHz over 105/88 MHz, as the two integers rather than as 20.952:
        // `cycles × 105` against `ticks × 25 × 88`, which is exact.
        let (measured, declared) = (cycles * 105, ticks * 25 * 88);
        assert!(
            (measured - declared).abs() * 200 <= declared,
            "{how}: {cycles} cycles over {ticks} 8254 ticks is {:.4} per tick, \
             more than half a per cent from the 20.952 the crystals declare",
            cycles as f64 / ticks as f64
        );
    }
}

// ---------------------------------------------------------------------------
// what a halted processor does to its own time-stamp counter
// ---------------------------------------------------------------------------

/// Where the idle probe records `(TSC, HPET)` before and after.
const IDLE_BEFORE: u32 = 0x5200;
const IDLE_AFTER: u32 = 0x5210;
/// How many times it halts, each time until the next 8254 tick.
const IDLE_HALTS: u32 = 20;
/// The 8254 divisor that wakes it: 1 193 of 105/88 MHz is a millisecond.
const IDLE_TICK: u16 = 1_193;

/// A `jmp $` on this core: seven clocks and the bus cycles that refetch it. A
/// processor spinning on one notices an interrupt at the end of the jump it is
/// in, so it may take the interrupt up to this much later than one that was
/// halted — and that is the whole of the difference halting is allowed to make.
const ONE_JUMP: u64 = 16;

/// A halted processor wakes to a time-stamp counter that counted the wait.
///
/// *Intel SDM* volume 3B §17.17 gives this core's family (06H, model 0FH) a
/// counter that "increments at a constant rate", and §17.17.1 an invariant one
/// that "will run at a constant rate in all ACPI P-, C-. and T-states"; `HLT`
/// is C1. So a timer armed for a known interval and waited for with `HLT` must
/// show the handler's `RDTSC` that interval later, exactly as a processor
/// that spun through the same wait does — the spinning one is the reference,
/// because it charges every cycle by executing it.
///
/// Measured, cycles from the arming access to the handler, spinning against
/// halted, 12 500 of them programmed:
///
/// | board | source | spinning | halted, before | halted, after |
/// | --- | --- | --- | --- | --- |
/// | `q35` | APIC | 12 646 | 143 | 12 639 |
/// | `q35` | HPET | 12 646 | 143 | 12 637 |
/// | `q35` | 8254 | 12 662 | 137 | 12 654 |
/// | `pc-apic` | APIC | 3 659 | 143 | 3 650 |
/// | `pc-apic` | HPET | 3 494 | 143 | 3 487 |
/// | `pc-apic` | 8254 | 3 565 | 137 | 3 562 |
///
/// Before, `X86::run_budget` consumed a halted processor's budget and charged
/// the counter none of it, so the handler saw only the arming store and its
/// own entry. (`pc-apic`'s short intervals are the shared crystal's arming
/// write landing at the round's start, which
/// `a_timer_armed_on_a_shared_crystal_fires_where_it_always_has` pins; it
/// moves the interrupt, not the counter, and halting agrees with spinning
/// there as everywhere.)
#[test]
fn a_halted_processor_wakes_to_a_time_stamp_counter_that_counted_the_wait() {
    #[cfg(feature = "dev-q35")]
    halted_against_spinning("q35.machine", Q35, Q35_SOCKET, true, 16);
    halted_against_spinning("pc-apic.machine", PC_APIC, PC_APIC_SOCKET, false, 20);
}

/// Every alarm source on one board, waited for spinning and then halted.
fn halted_against_spinning(name: &str, text: &str, socket: Socket, imcr: bool, hpet: u8) {
    for (source, _) in alarms(hpet) {
        let (spun, spin) = alarm_latency(name, text, socket, source, imcr, Wait::Spin);
        let (halted, halt) = alarm_latency(name, text, socket, source, imcr, Wait::Halt);
        assert_eq!((spun, halted), (1, 1), "{name} {source:?} interrupted once");
        assert!(
            halt <= spin && spin - halt <= ONE_JUMP,
            "{name} {source:?}: the handler read the TSC {halt} cycles after \
             arming when the processor halted and {spin} when it spun — a \
             halted processor's counter must count the wait (SDM vol. 3B \
             §17.17), so the two may differ by the one `jmp` the spinning \
             processor was in, and no more"
        );
    }
}

/// Twenty halts in a row cost the counter nothing against a clock outside the
/// processor.
///
/// The guest halts [`IDLE_HALTS`] times, each until the 8254's next
/// millisecond tick, and compares its TSC with the HPET's main counter across
/// the whole stretch; then does the same wait by spinning on the handler's
/// count instead. Before, the HPET moved 199 744 ticks — 499 360 cycles of
/// this board's 25 MHz core — while the halted guest's TSC moved **2 926**.
/// Now it moves 499 481, and the spinning run 499 497 against 199 751.
///
/// Both runs sit about 120 cycles above the HPET's figure, and that is not a
/// rate: 45 halts sit the same 120 cycles above it (1 124 386 against 449 706
/// ticks). It is a fixed offset between the program's two samples of a clock
/// on another crystal, and halting leaves it exactly where spinning does, which
/// is what this asserts. Deterministic and dispatched rounds agree to the
/// cycle.
#[test]
fn a_halted_processors_time_stamp_counter_keeps_pace_with_the_hpet() {
    let halted = idle(ThreadingMode::Deterministic, Wait::Halt);
    assert_eq!(
        halted,
        idle(ThreadingMode::Parallel, Wait::Halt),
        "a dispatched round read different counters"
    );
    let spun = idle(ThreadingMode::Deterministic, Wait::Spin);
    for (how, (taken, _, _)) in [("halted", halted), ("spinning", spun)] {
        assert!(
            taken > u64::from(IDLE_HALTS),
            "{how}: the guest woke {taken} times, so it never reached the end of \
             the stretch this measures"
        );
    }
    assert!(
        halted.2 > 100_000,
        "the HPET moved {} ticks, too little idle time to measure against",
        halted.2
    );
    // This board's core is 25 MHz and its HPET 10 MHz: a tick is owed two and
    // a half cycles, so everything is kept doubled to stay in integers.
    let excess = |(_, tsc, hpet): (u64, u64, u64)| 2 * tsc as i64 - 5 * hpet as i64;
    let (halt, spin) = (excess(halted), excess(spun));
    assert!(
        (halt - spin).abs() <= 2 * ONE_JUMP as i64,
        "halted, the TSC moved {} cycles over {} HPET ticks; spinning, {} over \
         {}. Halting must cost the counter nothing, so the two stand the same \
         distance from the HPET's figure to within one instruction",
        halted.1,
        halted.2,
        spun.1,
        spun.2
    );
    assert!(
        halt.abs() * 1_000 <= 5 * halted.2 as i64,
        "halted, the TSC moved {} cycles against the {} the HPET's {} ticks are \
         worth — more than a thousandth apart",
        halted.1,
        halted.2 * 5 / 2,
        halted.2
    );
}

/// Where the bootstrap processor records the `RDTSC` it takes just before
/// sending the Start-Up, and the application processor its first.
const SIPI_SENT: u32 = 0x5300;
const AP_FIRST: u32 = 0x5308;
/// How long the bootstrap processor spins before it starts the other one, in
/// turns of `dec eax; jnz`: about thirty milliseconds of the 25 MHz core, during
/// which the application processor waits for a Start-Up.
const SIPI_SPIN: u32 = 100_000;
/// The Start-Up vector: page `0xe0`, the bottom of `pc-apic`'s socket.
const SIPI_PAGE: u8 = 0xe0;

/// An application processor's counter ran while it waited for its Start-Up.
///
/// *Intel SDM* volume 3A Table 9-1 has an INIT leave the time-stamp counter
/// "unchanged", and a counter that runs at a constant rate (vol. 3B §17.17)
/// goes on running through the wait-for-SIPI state that INIT leaves a
/// processor in, as through `HLT`. `pc-apic` parks its second processor at
/// reset; the first spins, reads its TSC, and starts the second, whose first
/// instruction reads its own. Both count from the same cold reset on one
/// crystal, so the second must read about what the first did — later by the
/// Start-Up's delivery, which on a crystal two processors share is at most a
/// round. Before, it read a counter that had stood still since the INIT.
#[test]
fn an_application_processor_starts_with_a_time_stamp_counter_that_counted_the_wait() {
    // One round of the 25 MHz core.
    const ROUND_CYCLES: u64 = 25_000;
    let (mut rom, mut pm) = boot(PC_APIC_SOCKET);
    // The local APIC, software-enabled.
    pm.push(0xbf); // mov edi, 0xfee00000
    dw(&mut pm, 0xfee0_0000);
    store_at(&mut pm, 0x0f0, 0x1ff);
    pm.push(0xb8); // mov eax, SIPI_SPIN
    dw(&mut pm, SIPI_SPIN);
    pm.push(0x48); // dec eax
    pm.extend_from_slice(&[0x75, 0xfd]); // jnz -3
    // The destination, APIC 1, first: the low half is what sends (SDM vol. 3A
    // §10.6.1).
    store_at(&mut pm, 0x310, 1 << 24);
    pm.extend_from_slice(&[0x0f, 0x31]); // rdtsc
    pm.push(0xa3); // mov [SIPI_SENT], eax
    dw(&mut pm, SIPI_SENT);
    pm.extend_from_slice(&[0x89, 0x15]); // mov [SIPI_SENT+4], edx
    dw(&mut pm, SIPI_SENT + 4);
    // Start-Up (delivery mode 110b), level asserted, to page SIPI_PAGE.
    store_at(&mut pm, 0x300, 0x0000_4600 | u32::from(SIPI_PAGE));
    pm.extend_from_slice(&[0xeb, 0xfe]); // jmp $
    put(&mut rom, PC_APIC_SOCKET, OFF_PM, &pm);

    // The application processor, in real mode at `SIPI_PAGE:0000` with the
    // data segment the INIT left at zero.
    let mut ap: Vec<u8> = Vec::new();
    ap.extend_from_slice(&[0x0f, 0x31]); // rdtsc
    ap.extend_from_slice(&[0x66, 0xa3]); // mov [AP_FIRST], eax
    ap.extend_from_slice(&(AP_FIRST as u16).to_le_bytes());
    ap.extend_from_slice(&[0x66, 0x89, 0x16]); // mov [AP_FIRST+4], edx
    ap.extend_from_slice(&((AP_FIRST + 4) as u16).to_le_bytes());
    ap.push(0xf4); // hlt
    ap.extend_from_slice(&[0xeb, 0xfd]); // jmp hlt
    let at = ((u32::from(SIPI_PAGE) << 12) - PC_APIC_SOCKET.base) as usize;
    rom[at..at + ap.len()].copy_from_slice(&ap);

    let mut m = board_with(
        "pc-apic.machine",
        PC_APIC,
        rom,
        ThreadingMode::Deterministic,
    );
    m.reset(ResetKind::Cold);
    m.sweep();
    m.run_for(GlobalTime::from_nanos(60_000_000))
        .expect("the machine runs");
    let mem = m.space("mem").expect("the memory space");
    let peek = |at: u32| {
        mem.read(u64::from(at), Width::U64, MemAttrs::DEBUG)
            .expect("a mapped word")
    };
    let (sent, first) = (peek(SIPI_SENT), peek(AP_FIRST));
    println!("-- pc-apic: Start-Up sent at TSC {sent}, the AP's first RDTSC {first}");
    assert!(
        sent > 500_000,
        "the bootstrap processor read {sent}: it never spun long enough for a \
         stopped counter to show"
    );
    assert!(
        first >= sent && first - sent <= ROUND_CYCLES,
        "the application processor's first RDTSC read {first}, and the \
         bootstrap processor's just before the Start-Up read {sent}: a counter \
         that ran through wait-for-SIPI reads the other's value, later by at \
         most the round the Start-Up is delivered in"
    );
}

/// Wait out [`IDLE_HALTS`] of the 8254's millisecond ticks on `pc-apic`, and
/// report `(interrupts taken, TSC moved, HPET moved)` across the whole
/// stretch.
///
/// [`Wait::Halt`] waits with `hlt` and counts the ticks by counting the
/// wakeups; [`Wait::Spin`] waits for the handler's own count to reach the same
/// number, executing all the way. The two differ in nothing else, which is
/// what makes the second the first's control.
fn idle(mode: ThreadingMode, wait: Wait) -> (u64, u64, u64) {
    let (mut rom, mut pm) = boot(PC_APIC_SOCKET);
    // A gate for the 8254's interrupt, and the tables.
    pm.push(0xbf); // mov edi, gate
    dw(&mut pm, IDT_BASE + 8 * u32::from(ALARM_VECTOR));
    pm.push(0xb8); // mov eax, handler
    dw(&mut pm, lin(OFF_HANDLER));
    pm.extend_from_slice(&[0x66, 0x89, 0x07]);
    pm.extend_from_slice(&[0x66, 0xc7, 0x47, 0x02, 0x08, 0x00]);
    pm.extend_from_slice(&[0xc6, 0x47, 0x04, 0x00]);
    pm.extend_from_slice(&[0xc6, 0x47, 0x05, 0x8e]);
    pm.extend_from_slice(&[0xc1, 0xe8, 0x10]);
    pm.extend_from_slice(&[0x66, 0x89, 0x47, 0x06]);
    pm.extend_from_slice(&[0x0f, 0x01, 0x1d]);
    dw(&mut pm, lin(OFF_IDT_PTR));
    // The local APIC, and I/O APIC input 2 — where this board wires IRQ 0.
    pm.push(0xbf);
    dw(&mut pm, 0xfee0_0000);
    store_at(&mut pm, 0x0f0, 0x1ff);
    pm.push(0xbf);
    dw(&mut pm, 0xfec0_0000);
    store_at(&mut pm, 0x00, 0x10 + 2 * 2 + 1);
    store_at(&mut pm, 0x10, 0);
    store_at(&mut pm, 0x00, 0x10 + 2 * 2);
    store_at(&mut pm, 0x10, u32::from(ALARM_VECTOR));
    // The HPET's main counter, which is the reference here.
    pm.push(0xbf);
    dw(&mut pm, 0xfed0_0000);
    store_at(&mut pm, 0x010, 1);
    store_at(&mut pm, 0x014, 0);
    // Counter 0, mode 2, ticking once a millisecond: what wakes it each time.
    outb(&mut pm, 0x43, 0x34);
    outb(&mut pm, 0x40, IDLE_TICK as u8);
    outb(&mut pm, 0x40, (IDLE_TICK >> 8) as u8);
    pm.push(0xfb); // sti

    /// `rdtsc` and the HPET's low half, into `at` and `at + 4`.
    fn snapshot(out: &mut Vec<u8>, at: u32) {
        out.extend_from_slice(&[0x0f, 0x31]); // rdtsc
        out.push(0xa3); // mov [at], eax
        dw(out, at);
        out.push(0xa1); // mov eax, [0xfed000f0]
        dw(out, 0xfed0_00f0);
        out.push(0xa3); // mov [at+4], eax
        dw(out, at + 4);
    }

    snapshot(&mut pm, IDLE_BEFORE);
    match wait {
        Wait::Halt => {
            pm.push(0xb9); // mov ecx, IDLE_HALTS
            dw(&mut pm, IDLE_HALTS);
            pm.push(0xf4); // hlt
            pm.push(0x49); // dec ecx
            pm.extend_from_slice(&[0x75, 0xfc]); // jnz hlt
        }
        Wait::Spin => {
            pm.extend_from_slice(&[0x8b, 0x1d]); // mov ebx, [TAKEN]
            dw(&mut pm, TAKEN);
            pm.extend_from_slice(&[0x81, 0xc3]); // add ebx, IDLE_HALTS
            dw(&mut pm, IDLE_HALTS);
            pm.extend_from_slice(&[0x39, 0x1d]); // cmp [TAKEN], ebx
            dw(&mut pm, TAKEN);
            pm.extend_from_slice(&[0x72, 0xf8]); // jb cmp
        }
    }
    snapshot(&mut pm, IDLE_AFTER);
    pm.extend_from_slice(&[0xeb, 0xfe]); // jmp $
    put(&mut rom, PC_APIC_SOCKET, OFF_PM, &pm);

    let mut idt_ptr = Vec::new();
    idt_ptr.extend_from_slice(&0x07ffu16.to_le_bytes());
    dw(&mut idt_ptr, IDT_BASE);
    put(&mut rom, PC_APIC_SOCKET, OFF_IDT_PTR, &idt_ptr);

    let mut h: Vec<u8> = Vec::new();
    h.extend_from_slice(&[0xff, 0x05]); // inc dword [TAKEN]
    dw(&mut h, TAKEN);
    h.extend_from_slice(&[0xc7, 0x05]); // EOI
    dw(&mut h, 0xfee0_00b0);
    dw(&mut h, 0);
    h.push(0xcf); // iret
    put(&mut rom, PC_APIC_SOCKET, OFF_HANDLER, &h);

    let mut m = board_with("pc-apic.machine", PC_APIC, rom, mode);
    m.reset(ResetKind::Cold);
    m.sweep();
    m.run_for(GlobalTime::from_nanos(60_000_000))
        .expect("the machine runs");
    let mem = m.space("mem").expect("the memory space");
    let word = |at: u32| {
        mem.read(u64::from(at), Width::U32, MemAttrs::DEBUG)
            .expect("a mapped word")
    };
    (
        word(TAKEN),
        word(IDLE_AFTER).wrapping_sub(word(IDLE_BEFORE)) & 0xffff_ffff,
        word(IDLE_AFTER + 4).wrapping_sub(word(IDLE_BEFORE + 4)) & 0xffff_ffff,
    )
}
