//! Does an interrupt get from an HPET comparator, through an I/O APIC, through
//! a local APIC and into a handler the guest wrote — and does the guest's
//! end-of-interrupt let go of it? And does firmware on this board's bootstrap
//! processor start its **second** processor?
//!
//! Every chip in this path has unit tests proving it works alone. This is the
//! one that runs **real x86 instructions**: the firmware image below is
//! hand-assembled machine code that enters protected mode, builds an interrupt
//! descriptor table, programs the I/O APIC's redirection entry, software-enables
//! the local APIC, arms an HPET comparator, sets `IF`, and spins. Everything
//! after that is the board doing its job.
//!
//! That is the standard the rest of this session's devices have been held to —
//! a real driver through real registers — and it is worth the hand assembly,
//! because the failure modes it catches are exactly the ones an isolated device
//! test cannot see: an address that decodes somewhere else, a wire connected to
//! the wrong pin, a vector that never reaches the processor because a mask bit
//! was still set.
//!
//! The same firmware, with one section swapped, arms the **local APIC's own
//! timer** instead — a path that never touches the I/O APIC — and that is where
//! the timing rule is asserted: the interrupt lands after a millisecond of
//! *virtual* time, so a run budgeted half a millisecond sees nothing and a run
//! budgeted three sees exactly one. A device that consulted a host clock could
//! not produce that pair of answers.
//!
//! # The program
//!
//! `machines/pc-apic.machine` puts a 128 KiB ROM socket at `0xe0000` and a
//! second copy of it at the top of the 32-bit space, so the reset vector at
//! `0xfffffff0` finds the same image. The layout below is written out once, as
//! constants, and every jump target is computed from them.

#![cfg(all(
    feature = "cpu-x86",
    feature = "dev-pc",
    feature = "dev-pc-apic",
    feature = "dev-pc-hpet"
))]

use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{Machine, build};

/// The board this file exercises.
const PC_APIC: &str = include_str!("../machines/pc-apic.machine");

/// The ROM socket's size, which the machine file also declares.
const ROM_LEN: usize = 128 * 1024;

/// Where the segment `0xf000` starts inside the image: the socket is based at
/// `0xe0000`, so `0xf0000` is 64 KiB in.
const SEG_F000: usize = 0x1_0000;

/// The reset vector, sixteen bytes below the top of the socket.
const RESET_VECTOR: usize = ROM_LEN - 0x10;

// Offsets within segment 0xf000. Linear addresses are `0xf0000 + off`.
const OFF_ENTRY: usize = 0x0000;
const OFF_GDT: usize = 0x0100;
const OFF_GDT_PTR: usize = 0x0120;
const OFF_IDT_PTR: usize = 0x0128;
const OFF_PM: usize = 0x0200;
const OFF_HANDLER: usize = 0x0400;

/// The linear address of an offset in segment 0xf000.
const fn lin(off: usize) -> u32 {
    0xf_0000 + off as u32
}

/// Where the guest puts its interrupt descriptor table, and where it counts.
const IDT_BASE: u32 = 0x2000;
const COUNTER: u32 = 0x3000;

/// Where the bootstrap processor writes the second one's trampoline, and the
/// page a Start-Up names to send it there: 0x8000 is page 0x08.
const AP_TRAMPOLINE: u32 = 0x8000;
const AP_PAGE: u8 = 0x08;

/// Where the second processor says it is alive, and what it writes there.
const AP_MARKER: u32 = 0x3200;
const ALIVE: u16 = 0xa55a;

/// How many processors have executed the firmware's entry point.
///
/// One, on a board whose application processor is parked in wait-for-SIPI —
/// which is the whole point of the second `pc.lapic`. Counted by the firmware
/// itself rather than asserted from the outside.
const ARRIVALS: u32 = 0x3100;

/// The vector the redirection entry carries.
const VECTOR: u8 = 0x40;

/// The vector the local APIC's own timer carries.
const TIMER_VECTOR: u8 = 0x41;

/// What the guest loads into the APIC timer's initial count. The board's bus
/// clock is 100 MHz and the divide configuration is one, so this is a
/// millisecond — long enough that the protected-mode setup before it cannot be
/// confused with it.
const TIMER_COUNT: u32 = 100_000;

/// What the guest loads into the APIC timer's initial count in *periodic* mode:
/// five milliseconds on the same 100 MHz bus clock.
///
/// Five rather than one, and the runs below are budgeted in whole periods with
/// most of a period to spare at each end. The delivery is exact — same run,
/// same tick, every time — but it is observed at the scheduler's next slice
/// rather than at the instant the count reaches zero, and a period on either
/// side of every assertion keeps that slice out of the arithmetic. What is
/// being asserted is that a periodic timer reloads, not what a slice is worth.
const TIMER_PERIOD: u32 = 500_000;

/// One period of either periodic source, in nanoseconds.
const PERIOD_NS: u64 = 5_000_000;

/// Which I/O APIC input the 8254's counter 0 is wired to, per
/// `machines/pc-apic.machine` — and per the MultiProcessor Specification §5.1,
/// which puts the 8254 on input **2** because input 0 is where an `ExtINT`
/// entry for the whole 8259A cascade goes. It is the input a kernel's
/// `..TIMER: vector=0x30 apic1=0 pin1=2` line names.
const PIT_INPUT: u8 = 2;

/// The 8254 divisor the firmware loads into counter 0. The board's 8254 crystal
/// is 105/88 MHz, so 5966 counts is 5.0001 ms — near enough one [`PERIOD_NS`]
/// that the two periodic sources can be budgeted the same way, and exact
/// because the oscillator is a rational rather than a rounded float.
const PIT_DIVISOR: u32 = 5966;

/// Which I/O APIC input the HPET's first comparator is wired to, per
/// `machines/pc-apic.machine`.
const HPET_INPUT: u8 = 20;

/// How many HPET ticks pass before the comparator matches. At 100 ns a tick,
/// 1000 of them is 100 microseconds.
const HPET_DELAY: u32 = 1_000;

/// The period of the *edge-triggered* comparator, in HPET ticks: 50 000 of them
/// at 100 ns is five milliseconds, the same interval [`TIMER_PERIOD`] gives the
/// local APIC timer, so the two periodic sources below count the same thing.
const HPET_PERIOD: u32 = 50_000;

/// Append a little-endian 32-bit word.
fn dw(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_le_bytes());
}

/// `mov dword [edi+disp32], imm32` — the form every register write below takes,
/// because an APIC register file is further than a signed byte from its base.
fn store_at(out: &mut Vec<u8>, disp: u32, value: u32) {
    out.extend_from_slice(&[0xc7, 0x87]);
    dw(out, disp);
    dw(out, value);
}

/// Which timer the firmware arms.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Source {
    /// An HPET comparator, level-triggered, routed through the I/O APIC.
    Hpet,
    /// A *periodic* HPET comparator in edge-triggered mode, routed through an
    /// I/O APIC redirection entry that is also edge-triggered — the other
    /// trigger mode, and the one a PC's 8254 arrives on.
    HpetEdge,
    /// The **8254's** counter 0 in mode 2, on I/O APIC input 2 through an
    /// edge-triggered entry: the exact path a Linux kernel checks in symmetric
    /// I/O mode, and the one behind `MP-BIOS bug: 8254 timer not connected to
    /// IO-APIC`.
    Pit,
    /// The local APIC's own timer, one-shot, which reaches the processor
    /// without going near the I/O APIC.
    ApicTimer,
    /// The same timer in periodic mode: one arming, an interrupt every period,
    /// with no further register write from the guest.
    ApicTimerPeriodic,
    /// No timer at all: the firmware starts the board's *second processor*
    /// instead, with the MultiProcessor Specification's three interrupt
    /// command register writes (v1.4 B.4).
    Smp,
}

/// The firmware image: real-mode entry, a GDT, protected-mode setup, the driver
/// and its interrupt handler.
fn firmware(source: Source) -> Vec<u8> {
    let mut rom = vec![0u8; ROM_LEN];

    // -- the reset vector ---------------------------------------------------
    //
    // `jmp far 0xf000:0x0000`, which is how every PC firmware image begins: the
    // far jump recomputes CS's base as `selector << 4` and drops the processor
    // out of the top of the address space and into the first megabyte.
    rom[RESET_VECTOR..RESET_VECTOR + 5].copy_from_slice(&[0xea, 0x00, 0x00, 0x00, 0xf0]);

    // -- real mode: enter protected mode ------------------------------------
    let mut entry: Vec<u8> = Vec::new();
    // Say that a processor got here. `DS` is zero out of reset and this is
    // real mode, so the address is physical and lands in `ram_low`, which a
    // cold reset leaves zeroed.
    entry.extend_from_slice(&[0xff, 0x06]); // inc word [ARRIVALS]
    entry.extend_from_slice(&(ARRIVALS as u16).to_le_bytes());
    entry.push(0xfa); // cli
    entry.extend_from_slice(&[0xb8, 0x00, 0xf0]); // mov ax, 0xf000
    entry.extend_from_slice(&[0x8e, 0xd8]); // mov ds, ax
    // lgdt [0x0120]. No operand-size prefix: without one the base is loaded
    // from 24 bits, and 0x000f0100 fits in 24.
    entry.extend_from_slice(&[0x0f, 0x01, 0x16]);
    entry.extend_from_slice(&(OFF_GDT_PTR as u16).to_le_bytes());
    entry.extend_from_slice(&[0x0f, 0x20, 0xc0]); // mov eax, cr0
    entry.extend_from_slice(&[0x0c, 0x01]); // or al, 1
    entry.extend_from_slice(&[0x0f, 0x22, 0xc0]); // mov cr0, eax
    // jmp far 0x08:pm — a 32-bit offset, so the operand-size prefix.
    entry.extend_from_slice(&[0x66, 0xea]);
    dw(&mut entry, lin(OFF_PM));
    entry.extend_from_slice(&[0x08, 0x00]);
    put(&mut rom, OFF_ENTRY, &entry);

    // -- the descriptor tables ----------------------------------------------
    let gdt: [u8; 24] = [
        // The null descriptor.
        0, 0, 0, 0, 0, 0, 0, 0, // A flat 4 GiB code segment, ring 0, 32-bit.
        0xff, 0xff, 0, 0, 0, 0x9a, 0xcf, 0, // A flat 4 GiB data segment, ring 0.
        0xff, 0xff, 0, 0, 0, 0x92, 0xcf, 0,
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

    // -- protected mode: the driver -----------------------------------------
    let mut pm: Vec<u8> = Vec::new();
    pm.extend_from_slice(&[0xb8, 0x10, 0x00, 0x00, 0x00]); // mov eax, 0x10
    pm.extend_from_slice(&[0x8e, 0xd8]); // mov ds, ax
    pm.extend_from_slice(&[0x8e, 0xc0]); // mov es, ax
    pm.extend_from_slice(&[0x8e, 0xd0]); // mov ss, ax
    pm.push(0xbc); // mov esp, 0xf000
    dw(&mut pm, 0xf000);

    // Build one interrupt gate, for this source's vector, at
    // IDT_BASE + 8 * vector. RAM comes out of a cold reset zeroed, so every
    // other entry is a not-present gate already.
    let vector = match source {
        Source::Hpet | Source::HpetEdge | Source::Pit => VECTOR,
        Source::ApicTimer | Source::ApicTimerPeriodic | Source::Smp => TIMER_VECTOR,
    };
    pm.push(0xbf); // mov edi, gate
    dw(&mut pm, IDT_BASE + 8 * u32::from(vector));
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

    if matches!(source, Source::Hpet | Source::HpetEdge | Source::Pit) {
        // The I/O APIC. Two indirect writes per half: the index register, then
        // the data window. The high half first, so the entry is never briefly
        // unmasked with a destination nobody has written.
        pm.push(0xbf); // mov edi, 0xfec00000
        dw(&mut pm, 0xfec0_0000);
        let input = if source == Source::Pit {
            PIT_INPUT
        } else {
            HPET_INPUT
        };
        let index = 0x10 + 2 * u32::from(input);
        store_at(&mut pm, 0x00, index + 1);
        store_at(&mut pm, 0x10, 0); // destination: APIC ID 0, physical
        store_at(&mut pm, 0x00, index);
        // Vector, unmasked, fixed delivery to APIC 0, and the trigger mode this
        // source asks for: bit 15 set is level, clear is edge (82093AA §3.2.4).
        let trigger = if source == Source::Hpet { 1 << 15 } else { 0 };
        store_at(&mut pm, 0x10, trigger | u32::from(VECTOR));
    }

    // The local APIC: software-enable it, with 0xff as the spurious vector.
    // Nothing this part delivers reaches the processor until this is written
    // (SDM Vol 3A 10.4.7.2).
    pm.push(0xbf); // mov edi, 0xfee00000
    dw(&mut pm, 0xfee0_0000);
    store_at(&mut pm, 0xf0, 0x1ff);

    match source {
        Source::ApicTimer | Source::ApicTimerPeriodic => {
            // Divide by one, the timer entry at `TIMER_VECTOR`, and the count
            // that starts it. The order matters and is the architecture's: the
            // LVT entry says where the interrupt goes and in which mode, and
            // "writing to the initial count register starts the timer" (SDM
            // Vol 3A 10.5.4).
            //
            // Bits 18:17 of the timer entry are the mode: `00` one-shot, `01`
            // periodic (SDM Vol 3A Figure 10-8). A periodic timer reloads the
            // initial count itself, so this is the last register write the
            // guest makes — every interrupt after the first comes from the
            // part's own arithmetic.
            let mode = if source == Source::ApicTimerPeriodic {
                1 << 17
            } else {
                0
            };
            let count = if source == Source::ApicTimerPeriodic {
                TIMER_PERIOD
            } else {
                TIMER_COUNT
            };
            store_at(&mut pm, 0x3e0, 0b1011);
            store_at(&mut pm, 0x320, mode | u32::from(TIMER_VECTOR));
            store_at(&mut pm, 0x380, count);
        }
        Source::Smp => {
            // The other processor. First its trampoline, written into RAM as
            // three doublewords — real-mode code, because that is the mode a
            // Start-Up leaves a processor in however this one is running (SDM
            // Vol 3A 8.4.3):
            //
            //     xor ax, ax ; mov ds, ax ; mov word [AP_MARKER], ALIVE ; jmp $
            let mut tramp = Vec::new();
            tramp.extend_from_slice(&[0x31, 0xc0, 0x8e, 0xd8, 0xc7, 0x06]);
            tramp.extend_from_slice(&(AP_MARKER as u16).to_le_bytes());
            tramp.extend_from_slice(&ALIVE.to_le_bytes());
            tramp.extend_from_slice(&[0xeb, 0xfe]);
            pm.push(0xbf); // mov edi, 0
            dw(&mut pm, 0);
            for (i, word) in tramp.chunks(4).enumerate() {
                let mut bytes = [0u8; 4];
                bytes[..word.len()].copy_from_slice(word);
                store_at(
                    &mut pm,
                    AP_TRAMPOLINE + 4 * i as u32,
                    u32::from_le_bytes(bytes),
                );
            }
            // Then the sequence itself, through this processor's own interrupt
            // command register: the destination half, INIT assert, INIT
            // de-assert, Start-Up carrying the page.
            pm.push(0xbf); // mov edi, 0xfee00000
            dw(&mut pm, 0xfee0_0000);
            store_at(&mut pm, 0x310, 1 << 24);
            store_at(&mut pm, 0x300, 0x0000_c500);
            store_at(&mut pm, 0x300, 0x0000_8500);
            store_at(&mut pm, 0x300, 0x0000_0600 | u32::from(AP_PAGE));
        }
        Source::Hpet => {
            // Comparator 0 level-triggered and enabled, matching 1000 ticks
            // from now, and then the main counter started.
            pm.push(0xbf); // mov edi, 0xfed00000
            dw(&mut pm, 0xfed0_0000);
            store_at(&mut pm, 0x100, 0b110); // Tn_INT_ENB_CNF | Tn_INT_TYPE_CNF
            store_at(&mut pm, 0x104, 0);
            store_at(&mut pm, 0x108, HPET_DELAY);
            store_at(&mut pm, 0x10c, 0);
            store_at(&mut pm, 0x010, 1); // ENABLE_CNF
            store_at(&mut pm, 0x014, 0);
        }
        Source::HpetEdge => {
            // The same comparator, *edge*-triggered and periodic:
            // `Tn_INT_ENB_CNF` (bit 2) with `Tn_INT_TYPE_CNF` (bit 1) clear is
            // the edge mode, `Tn_TYPE_CNF` (bit 3) is periodic, and
            // `Tn_VAL_SET_CNF` (bit 6) makes the next comparator write set the
            // accumulator as well as the period (IA-PC HPET §2.3.9).
            //
            // An edge-triggered comparator pulses its output line and leaves it
            // low, so nothing here holds a level: every interrupt the guest
            // takes below is a rising edge the redirection entry had to notice
            // on its own.
            pm.push(0xbf); // mov edi, 0xfed00000
            dw(&mut pm, 0xfed0_0000);
            store_at(&mut pm, 0x100, 0b100_1100);
            store_at(&mut pm, 0x104, 0);
            store_at(&mut pm, 0x108, HPET_PERIOD);
            store_at(&mut pm, 0x10c, 0);
            store_at(&mut pm, 0x010, 1); // ENABLE_CNF
            store_at(&mut pm, 0x014, 0);
        }
        Source::Pit => {
            // The 8254, programmed the way every PC operating system programs
            // it for its tick: control word 34h is counter 0, both count bytes,
            // **mode 2** (rate generator), binary; then the divisor, low byte
            // first (Intel 8254 datasheet, "Mode 2" and the control word
            // format).
            //
            // Mode 2 holds OUT high and takes it low for a single input clock
            // at terminal count, so what arrives at the interrupt controller is
            // a pulse and the interrupt is its rising edge. There is no level
            // anywhere on this path for an entry to latch.
            for byte in [0x34, (PIT_DIVISOR & 0xff) as u8, (PIT_DIVISOR >> 8) as u8] {
                pm.extend_from_slice(&[0xb0, byte]); // mov al, imm8
                pm.extend_from_slice(&[0xe6, if byte == 0x34 { 0x43 } else { 0x40 }]);
            }
        }
    }

    pm.push(0xfb); // sti
    pm.extend_from_slice(&[0xeb, 0xfe]); // jmp $
    put(&mut rom, OFF_PM, &pm);

    // -- the interrupt handler ----------------------------------------------
    //
    // What a level-triggered driver has to do, in the order it has to do it:
    // quiet the device first, then count, then end the interrupt. If the
    // end-of-interrupt did not reach the I/O APIC's remote IRR, the line would
    // never interrupt again; if the remote IRR were not held in the first
    // place, this handler would be re-entered before it finished.
    let mut handler: Vec<u8> = Vec::new();
    if source == Source::Hpet {
        handler.extend_from_slice(&[0xc7, 0x05]); // mov dword [0xfed00020], 1
        dw(&mut handler, 0xfed0_0020);
        dw(&mut handler, 1);
    }
    handler.extend_from_slice(&[0xff, 0x05]); // inc dword [COUNTER]
    dw(&mut handler, COUNTER);
    handler.extend_from_slice(&[0xc7, 0x05]); // mov dword [0xfee000b0], 0
    dw(&mut handler, 0xfee0_00b0);
    dw(&mut handler, 0);
    handler.push(0xcf); // iret
    put(&mut rom, OFF_HANDLER, &handler);

    rom
}

/// Place `bytes` at `off` inside segment 0xf000.
fn put(rom: &mut [u8], off: usize, bytes: &[u8]) {
    let at = SEG_F000 + off;
    rom[at..at + bytes.len()].copy_from_slice(bytes);
}

/// Build the board with that firmware in its socket.
fn board(source: Source) -> Machine {
    let mut options = rsemu::machine::catalog::build_options().expect("this build's classes");
    options.realize.media.insert("bios", firmware(source));
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    match build("pc-apic.machine", PC_APIC, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    }
}

/// Read a 32-bit word of guest memory.
fn peek32(m: &Machine, addr: u64) -> u32 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U32, MemAttrs::DEFAULT)
        .expect("a mapped word") as u32
}

#[test]
fn the_board_realizes_with_every_apic_page_where_its_specification_puts_it() {
    let m = board(Source::Hpet);
    assert_eq!(m.name(), "pc-apic");
    let mem = m.space("mem").expect("the memory space");
    // The I/O APIC's version register, reached the way a driver reaches it:
    // write the index, read the window (82093AA 3.1).
    mem.write(0xfec0_0000, Width::U32, 1, MemAttrs::DEFAULT)
        .expect("the index register decodes at 0xfec00000");
    let version = mem
        .read(0xfec0_0010, Width::U32, MemAttrs::DEFAULT)
        .expect("and the data window sixteen bytes above it");
    assert_eq!(version & 0xff, 0x11, "an 82093AA");
    assert_eq!((version >> 16) & 0xff, 23, "with twenty-four inputs");

    // The local APIC's version register (SDM Vol 3A Table 10-1).
    let version = mem
        .read(0xfee0_0030, Width::U32, MemAttrs::DEFAULT)
        .expect("the local APIC page decodes at 0xfee00000");
    assert_eq!(version & 0xff, 0x14, "an integrated local APIC");

    // The HPET's capability register, and the period the machine file declared.
    let period = mem
        .read(0xfed0_0004, Width::U32, MemAttrs::DEFAULT)
        .expect("the HPET decodes at 0xfed00000");
    assert_eq!(
        period, 100_000_000,
        "100 ns in femtoseconds, matching the 10 MHz oscillator the file declares"
    );
}

#[test]
fn a_guest_driver_programs_the_io_apic_takes_the_interrupt_and_ends_it() {
    let mut m = board(Source::Hpet);
    m.reset(ResetKind::Cold);
    m.sweep();

    // A hundred microseconds of HPET is where the comparator matches; five
    // milliseconds is room for the processor to have got there and back.
    m.run_for(GlobalTime::from_nanos(5_000_000))
        .expect("the machine runs");

    assert_eq!(
        peek32(&m, u64::from(COUNTER)),
        1,
        "the handler ran exactly once: the comparator matched, the I/O APIC \
         turned the line into a message, the local APIC offered the vector, \
         and the processor took it"
    );

    // The redirection entry's remote IRR is clear, which is what the guest's
    // end-of-interrupt bought: the I/O APIC was told the vector had been
    // acknowledged and let go of the line.
    let mem = m.space("mem").expect("the memory space");
    let index = 0x10 + 2 * u32::from(HPET_INPUT);
    mem.write(0xfec0_0000, Width::U32, u64::from(index), MemAttrs::DEFAULT)
        .expect("the index register");
    let entry = mem
        .read(0xfec0_0010, Width::U32, MemAttrs::DEFAULT)
        .expect("the data window") as u32;
    assert_eq!(
        entry & 0xff,
        u32::from(VECTOR),
        "the vector the guest wrote"
    );
    assert_eq!(entry & (1 << 14), 0, "and remote IRR is clear again");
    assert_eq!(entry & (1 << 16), 0, "with the entry still unmasked");

    // And the local APIC has nothing in service, because the guest ended it.
    let isr = mem
        .read(
            0xfee0_0100 + 0x10 * u64::from(VECTOR >> 5),
            Width::U32,
            MemAttrs::DEFAULT,
        )
        .expect("the in-service register");
    assert_eq!(isr, 0);
}

#[test]
fn the_same_run_twice_lands_in_the_same_place() {
    // Determinism, on the path that matters here: the interrupt is delivered on
    // a tick the scheduler chose from the HPET's own arithmetic, so two runs of
    // the same board agree exactly.
    let counts: Vec<u32> = (0..2)
        .map(|_| {
            let mut m = board(Source::Hpet);
            m.reset(ResetKind::Cold);
            m.sweep();
            m.run_for(GlobalTime::from_nanos(5_000_000))
                .expect("the machine runs");
            peek32(&m, u64::from(COUNTER))
        })
        .collect();
    assert_eq!(counts[0], counts[1]);
    assert_eq!(counts[0], 1);
}

/// Run the board with `source` armed for `nanos` of virtual time, and report
/// how many times the guest's handler ran.
fn interrupts_in(source: Source, nanos: u64) -> u32 {
    let mut m = board(source);
    m.reset(ResetKind::Cold);
    m.sweep();
    m.run_for(GlobalTime::from_nanos(nanos))
        .expect("the machine runs");
    peek32(&m, u64::from(COUNTER))
}

#[test]
fn the_apic_timer_fires_at_a_tick_the_scheduler_chose() {
    // The guest arms a one-shot for 100 000 bus ticks with a divisor of one.
    // The board's bus crystal is 100 MHz, so that is one millisecond of virtual
    // time — a number that comes from the machine file's oscillator and the
    // timer's own arithmetic and from nothing else.
    //
    // **This is the test that fails if the device reads a clock.** Virtual time
    // is the only thing that moves here: `run_for` is a budget, not a delay, and
    // the whole run takes well under a millisecond of wall time. A timer that
    // consulted the host would fire in the short run and might fire many times
    // in the long one; a timer that counts its own domain's ticks fires exactly
    // once, and only in the run long enough to contain it.
    assert_eq!(
        interrupts_in(Source::ApicTimer, 500_000),
        0,
        "half a millisecond of virtual time is not a millisecond"
    );
    assert_eq!(
        interrupts_in(Source::ApicTimer, 3_000_000),
        1,
        "three milliseconds contains exactly one expiry of a one-shot"
    );
}

/// A periodic local APIC timer keeps interrupting after the first expiry.
///
/// The one-shot test above is the model, and *only* a one-shot: it arms a timer
/// whose mode field is `00`, so nothing in this file ever exercised the reload.
/// A guest that arms `01` — which is what a kernel does for its tick, and what
/// `APIC delta adjusted to PM-Timer` is the calibration for — is asking for an
/// interrupt every period with no further register write, and one interrupt is
/// as wrong an answer as none.
///
/// "If the timer is set for periodic mode, the current-count register is
/// automatically reloaded from the initial-count register whenever the
/// current-count register reaches 0 and an interrupt is generated" (SDM Vol 3A
/// §10.5.4). The count below is the whole assertion: 100 000 bus ticks of a
/// 100 MHz crystal is a millisecond, so three and a half milliseconds of
/// virtual time contains exactly three expiries — and the reload, not the
/// arming, produced two of them.
#[test]
fn a_periodic_apic_timer_interrupts_once_a_period() {
    let count = interrupts_in(Source::ApicTimerPeriodic, 3 * PERIOD_NS + PERIOD_NS / 2);
    assert!(
        count > 1,
        "a periodic timer delivered {count} interrupt(s): it never reloaded"
    );
    assert_eq!(
        count, 3,
        "three whole periods, and the reload made two of them"
    );

    // And the interval is the timer's own, not an artefact of how long the run
    // was: half a period is no interrupt at all, and twice as long is twice as
    // many.
    assert_eq!(
        interrupts_in(Source::ApicTimerPeriodic, PERIOD_NS / 2),
        0,
        "half a period of virtual time is not a period"
    );
    assert_eq!(
        interrupts_in(Source::ApicTimerPeriodic, 6 * PERIOD_NS + PERIOD_NS / 2),
        6,
        "and six periods deliver six"
    );

    // The guest wrote the initial count once. A periodic timer that had run
    // down to zero and stopped would read zero here; one that reloads is
    // somewhere inside its period, and never above the count that was written.
    let mut m = board(Source::ApicTimerPeriodic);
    m.reset(ResetKind::Cold);
    m.sweep();
    m.run_for(GlobalTime::from_nanos(3 * PERIOD_NS + PERIOD_NS / 2))
        .expect("the machine runs");
    let mem = m.space("mem").expect("the memory space");
    let current = mem
        .read(0xfee0_0390, Width::U32, MemAttrs::DEFAULT)
        .expect("the current count register") as u32;
    assert!(
        current > 0 && current <= TIMER_PERIOD,
        "the current count is inside the period ({current})"
    );
    let initial = mem
        .read(0xfee0_0380, Width::U32, MemAttrs::DEFAULT)
        .expect("the initial count register") as u32;
    assert_eq!(initial, TIMER_PERIOD, "and the initial count is untouched");
}

/// The determinism rule, on the periodic path.
#[test]
fn the_periodic_apic_timer_lands_in_the_same_place_every_run() {
    let counts: Vec<u32> = (0..3)
        .map(|_| interrupts_in(Source::ApicTimerPeriodic, 3 * PERIOD_NS + PERIOD_NS / 2))
        .collect();
    assert_eq!(counts, [3, 3, 3]);
}

/// An **edge-triggered** redirection entry delivers, and keeps delivering.
///
/// The level-triggered test above is the only delivery this file used to
/// assert, and the two trigger modes are different code paths in the I/O APIC:
/// a level is a condition the entry re-evaluates and latches remote IRR for,
/// an edge is an event forwarded once with no latch (82093AA §3.2.4). A PC's
/// 8254 arrives on an edge-triggered input, which is what makes this the mode a
/// kernel checks first — `MP-BIOS bug: 8254 timer not connected to IO-APIC` is
/// Linux having counted zero of these.
///
/// The comparator behind it is periodic and edge-triggered, so its output line
/// pulses and returns low: nothing holds a level anywhere on this path, and
/// every interrupt counted is a rising edge the entry noticed by itself.
#[test]
fn an_edge_triggered_redirection_entry_delivers_every_edge() {
    let count = interrupts_in(Source::HpetEdge, 3 * PERIOD_NS + PERIOD_NS / 2);
    assert!(
        count > 0,
        "an edge-triggered redirection entry delivered nothing"
    );
    assert!(
        count > 1,
        "the first edge arrived and no later one did ({count}): the entry is \
         behaving like a level with remote IRR stuck"
    );
    assert_eq!(count, 3, "three whole periods, three rising edges taken");
    assert_eq!(
        interrupts_in(Source::HpetEdge, PERIOD_NS / 2),
        0,
        "and half a period is no edge at all"
    );

    // Where the level case latched remote IRR and needed the guest's
    // end-of-interrupt to let go, this entry never latches it: an edge is an
    // event, and there is no condition to hold.
    let mut m = board(Source::HpetEdge);
    m.reset(ResetKind::Cold);
    m.sweep();
    m.run_for(GlobalTime::from_nanos(3 * PERIOD_NS + PERIOD_NS / 2))
        .expect("the machine runs");
    let mem = m.space("mem").expect("the memory space");
    let index = 0x10 + 2 * u32::from(HPET_INPUT);
    mem.write(0xfec0_0000, Width::U32, u64::from(index), MemAttrs::DEFAULT)
        .expect("the index register");
    let entry = mem
        .read(0xfec0_0010, Width::U32, MemAttrs::DEFAULT)
        .expect("the data window") as u32;
    assert_eq!(
        entry & 0xff,
        u32::from(VECTOR),
        "the vector the guest wrote"
    );
    assert_eq!(entry & (1 << 15), 0, "the entry is edge-triggered");
    assert_eq!(
        entry & (1 << 14),
        0,
        "and an edge-triggered entry never latches remote IRR"
    );
    assert_eq!(entry & (1 << 16), 0, "with the entry still unmasked");
}

/// The **8254** reaching the processor through I/O APIC input 2 — the exact
/// path a kernel checks first, and the one it names when it gives up.
///
/// `..TIMER: vector=0x30 apic1=0 pin1=2`, then `..MP-BIOS bug: 8254 timer not
/// connected to IO-APIC`: Linux programs counter 0 as a rate generator, routes
/// it through the redirection entry the MADT's interrupt source override names,
/// and counts ticks. Every part of that is here — the same counter in the same
/// mode on the same input through an entry with the same trigger mode — because
/// a synthetic pulse from an HPET comparator is not quite the same claim as the
/// chip a PC actually ticks on.
///
/// Mode 2 holds OUT high and drops it for a single input clock per period, so
/// the interrupt is the *rising* edge at the end of that pulse: nothing on this
/// path is a level, and nothing latches. Loading the counter takes OUT high
/// from the low it idles at out of reset, which is one further rising edge and
/// the first interrupt below — a real 8254 does exactly this, and it is why a
/// PC's first tick arrives early.
#[test]
fn the_8254_reaches_the_processor_through_io_apic_input_2() {
    // Half a period: the arming edge only.
    assert_eq!(
        interrupts_in(Source::Pit, PERIOD_NS / 2),
        1,
        "loading counter 0 takes OUT high, which is a rising edge on the input"
    );
    // Three whole periods on top of it.
    assert_eq!(
        interrupts_in(Source::Pit, 3 * PERIOD_NS + PERIOD_NS / 2),
        4,
        "the arming edge and one per period: an 8254 on an edge-triggered \
         redirection entry keeps interrupting"
    );
}

/// The level-triggered entry still works, unchanged, beside the edge one.
///
/// The pair is the point: `tests/q35_intx.rs` asserts a route by showing that
/// what arrives changes when the routing changes, and this is the same shape.
/// Both sources are the same comparator on the same input pin with the same
/// vector; only the trigger mode differs, and both counts are what that mode
/// says they should be.
#[test]
fn both_trigger_modes_deliver_and_they_are_not_the_same_path() {
    // One-shot and level: one interrupt, and the entry held remote IRR until
    // the guest ended it.
    assert_eq!(interrupts_in(Source::Hpet, 5_000_000), 1);
    // Periodic and edge, over a run long enough for two whole periods: more
    // than one, because no latch was in the way.
    assert!(interrupts_in(Source::HpetEdge, 2 * PERIOD_NS + PERIOD_NS / 2) > 1);
}

#[test]
fn the_apic_timer_lands_in_the_same_place_every_run() {
    let counts: Vec<u32> = (0..3)
        .map(|_| interrupts_in(Source::ApicTimer, 3_000_000))
        .collect();
    assert_eq!(counts, [1, 1, 1]);
}

#[test]
fn the_application_processor_is_parked_and_the_bootstrap_one_runs() {
    // The board carries two processors and only one of them executes firmware.
    // Nothing in this file makes that true: `pc.lapic`'s own reset parks the
    // processor in front of it when it is not the bootstrap one, because that
    // is what the MP initialization protocol does at power-up (Intel SDM Vol 3A
    // 8.4.3), and it says so over the `LocalController` link the machine file's
    // `lapic1.intr -> cpu1.intr` wire carries.
    //
    // The count is the firmware's own: the first instruction at the entry point
    // increments it, so a second processor arriving at 0xfffffff0 would be
    // visible as a two. Scheduler accounting cannot answer this — a stopped
    // core still consumes its budget, which is how it tells the scheduler not
    // to spin on it.
    let mut m = board(Source::Hpet);
    assert_eq!(peek32(&m, ARRIVALS.into()), 0, "nothing has executed yet");
    m.run_for(GlobalTime::from_nanos(5_000_000))
        .expect("the machine runs");
    assert_eq!(
        peek32(&m, ARRIVALS.into()),
        1,
        "one processor ran the firmware; the other is waiting for a Start-Up"
    );
    assert!(
        peek32(&m, COUNTER.into()) > 0,
        "and the one that ran it is the one taking interrupts"
    );
}

#[test]
fn the_guest_starts_the_second_processor_on_the_board() {
    // The Phase 7 gate, on a machine file rather than in a rig: firmware
    // running on `cpu0` writes a trampoline into RAM, sends `INIT` and a
    // Start-Up through its own local APIC's interrupt command register, and
    // `cpu1` — which until then had executed nothing at all — starts at the
    // page the Start-Up named and says so.
    //
    // Nothing here touches the second processor. The whole path is the board's:
    // `lapic0`'s ICR, the `apic` message bus, `lapic1`'s INIT and Start-Up, and
    // the `LocalController` it offers on the `lapic1.intr -> cpu1.intr` wire.
    let mut m = board(Source::Smp);
    assert_eq!(peek32(&m, AP_MARKER.into()), 0, "nothing has run yet");

    m.run_for(GlobalTime::from_nanos(5_000_000))
        .expect("the machine runs");

    assert_eq!(
        peek32(&m, ARRIVALS.into()),
        1,
        "only the bootstrap processor ever ran the firmware"
    );
    assert_eq!(
        peek32(&m, AP_MARKER.into()) & 0xffff,
        u32::from(ALIVE),
        "and the second processor executed the trampoline it was sent to"
    );
}
