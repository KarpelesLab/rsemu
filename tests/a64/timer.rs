// A64 conformance: the generic timer, from a guest that uses it as a kernel
// would.
//
// Copyright (c) Karpeles Lab Inc. MIT. Written from DDI 0487; no emulator
// source of any licence was consulted.
//
// ---------------------------------------------------------------------------
// Where the expectations come from, said plainly
// ---------------------------------------------------------------------------
//
// **They are ours.** Nothing in ordinary Rust reaches `CNTP_CTL_EL0`, so
// unlike `integer.rs` or `fp_natural.rs` this guest is not a record of what
// LLVM chose to emit, and unlike the floating-point guests it has no
// independent oracle computing the same function. Every case here is a rule
// transcribed from DDI 0487 D11.2 and then checked against this core — two
// parts of one head agreeing, exactly as `fp_rules.rs` says of itself.
//
// What it buys, which the crate's own unit tests cannot, is the *route*: a
// vector table the guest installed, an `ERET` back out of an interrupt handler,
// a `WFI` that ends because of a comparator and not because a test poked a pin,
// an excursion to EL0 and back, and a trapped `MRS` whose syndrome the handler
// reads to decide what to do. Those are not properties of one instruction. They
// are the shape a kernel tick has, and nothing short of running one exercises
// it.
//
// ---------------------------------------------------------------------------
// What the counter is counting
// ---------------------------------------------------------------------------
//
// The core derives `CNTPCT_EL0` from its own tick counter — one tick per bus
// access — divided by a board-supplied integer. The conformance runner builds
// a core with that divisor at its default of one, so here one count is one bus
// access and a deadline of 500 counts is 500 accesses away. Nothing below
// depends on the ratio; it is written down so the magnitudes make sense.

#![no_std]
#![no_main]

use core::ptr::{read_volatile, write_volatile};

include!("rt.rs");

/// How many times the IRQ handler has run.
#[unsafe(no_mangle)]
static mut TICKS: u64 = 0;

/// `CNTP_CTL_EL0` as the handler saw it, before it disarmed the timer.
#[unsafe(no_mangle)]
static mut CTL_SEEN: u64 = 0;

/// `ESR_EL1` from the last exception the lower-EL synchronous vector handled
/// that was not an `SVC`.
#[unsafe(no_mangle)]
static mut ESR_SEEN: u64 = 0;

// ---------------------------------------------------------------------------
// The vector table, and the excursion to EL0
// ---------------------------------------------------------------------------
//
// `VBAR_EL1` needs 2 KiB alignment and the sixteen entries are 0x80 apart, so
// the table is `.balign 2048` and each entry is padded to `.balign 128`. Only
// three of them do anything; the rest report case 90 through the runner's own
// protocol, so a stray exception says *which vector* rather than hanging.
//
// The lower-EL synchronous handler is the interesting one: a trapped
// system-register access (`ESR_EL1.EC` 0x18) is recorded and stepped over,
// which is precisely what a kernel virtualising the counter for a process does.
//
// **Getting back from EL0 is the timer's job too**, which is not a contrivance
// so much as a consequence: the runner arms `ExitMask::SYSCALL`, so an `SVC`
// leaves the core instead of vectoring and cannot be the return path. So
// `el0_probe` arms the physical timer before it drops, spins at EL0, and is
// interrupted back out through the *lower-EL* IRQ vector — which is an
// interrupt taken across an exception level, and a thing worth proving on its
// own.
core::arch::global_asm!(
    ".section .text.vectors,\"ax\"",
    ".balign 2048",
    ".globl a64_vectors",
    "a64_vectors:",
    // -- current EL with SP_EL0 ------------------------------------------
    ".balign 128", "mov x0, #90", "mov x3, #0x000", "brk #0",
    ".balign 128", "mov x0, #90", "mov x3, #0x080", "brk #0",
    ".balign 128", "mov x0, #90", "mov x3, #0x100", "brk #0",
    ".balign 128", "mov x0, #90", "mov x3, #0x180", "brk #0",
    // -- current EL with SP_ELx ------------------------------------------
    ".balign 128", "mov x0, #90", "mov x3, #0x200", "brk #0",
    // The one this guest exists for: the EL1h IRQ vector.
    ".balign 128",
    "stp x9, x10, [sp, #-16]!",
    "mrs x10, cntp_ctl_el0",           // ENABLE | ISTATUS, IMASK clear
    "adrp x9, {ctl_seen}",
    "add  x9, x9, :lo12:{ctl_seen}",
    "str  x10, [x9]",
    // Disarm. The output is a *level*: a handler that returned without
    // clearing the condition would take the same interrupt on the very next
    // instruction, forever.
    "msr cntp_ctl_el0, xzr",
    "msr cntv_ctl_el0, xzr",
    "adrp x9, {ticks}",
    "add  x9, x9, :lo12:{ticks}",
    "ldr  x10, [x9]",
    "add  x10, x10, #1",
    "str  x10, [x9]",
    "ldp x9, x10, [sp], #16",
    "eret",
    ".balign 128", "mov x0, #90", "mov x3, #0x300", "brk #0",
    ".balign 128", "mov x0, #90", "mov x3, #0x380", "brk #0",
    // -- lower EL, AArch64 -----------------------------------------------
    ".balign 128",
    "mrs x11, esr_el1",
    "lsr x12, x11, #26",
    "cmp x12, #0x18",
    "b.ne 30f",
    // A trapped system-register access: record the syndrome and step over the
    // instruction, which is the whole of trap-and-emulate.
    "adrp x13, {esr_seen}",
    "add  x13, x13, :lo12:{esr_seen}",
    "str  x11, [x13]",
    "mrs  x14, elr_el1",
    "add  x14, x14, #4",
    "msr  elr_el1, x14",
    "eret",
    // Anything else at this vector is unexpected, and says which class it was.
    "30:",
    "mov x0, #90",
    "mov x3, #0x400",
    "mov x1, x12",
    "brk #0",
    // The lower-EL IRQ vector: the timer, fired while the guest was at EL0.
    // Disarm it and go back to EL1h at the return address `el0_probe` parked
    // in x9.
    ".balign 128",
    "msr cntp_ctl_el0, xzr",
    "msr elr_el1, x9",
    "mov x15, #0x3c5",                 // DAIF set, M[3:0] = 0b0101: EL1h
    "msr spsr_el1, x15",
    "eret",
    ".balign 128", "mov x0, #90", "mov x3, #0x500", "brk #0",
    ".balign 128", "mov x0, #90", "mov x3, #0x580", "brk #0",

    // -----------------------------------------------------------------
    // Drop to EL0, read the counter there, and come back through `SVC`.
    // -----------------------------------------------------------------
    ".section .text.el0probe,\"ax\"",
    ".balign 4",
    ".globl el0_probe",
    "el0_probe:",
    "mov x9, x30",                     // where to return to, for the handler
    "mov x0, #200",                    // the ride home, armed before we leave
    "msr cntp_tval_el0, x0",
    "mov x0, #1",                      // ENABLE, IMASK clear
    "msr cntp_ctl_el0, x0",
    "adr x0, 20f",
    "msr elr_el1, x0",
    "msr spsr_el1, xzr",               // EL0t, every mask clear
    "eret",
    "20:",                             // ---- EL0 from here ----
    "mrs x1, cntvct_el0",              // trapped unless CNTKCTL_EL1 allows it
    "21:",
    "b 21b",                           // until the timer interrupts us out
    ticks = sym TICKS,
    ctl_seen = sym CTL_SEEN,
    esr_seen = sym ESR_SEEN,
);

unsafe extern "C" {
    fn el0_probe();
    static a64_vectors: u8;
}

// ---------------------------------------------------------------------------
// The registers, one accessor each
// ---------------------------------------------------------------------------

macro_rules! reader {
    ($name:ident, $reg:literal) => {
        fn $name() -> u64 {
            let v: u64;
            unsafe {
                core::arch::asm!(
                    concat!("mrs {}, ", $reg),
                    out(reg) v,
                    options(nomem, nostack),
                )
            };
            v
        }
    };
}

macro_rules! writer {
    ($name:ident, $reg:literal) => {
        fn $name(value: u64) {
            unsafe {
                core::arch::asm!(
                    concat!("msr ", $reg, ", {}"),
                    "isb",
                    in(reg) value,
                    options(nostack),
                )
            };
        }
    };
}

reader!(cntfrq, "cntfrq_el0");
reader!(cntpct, "cntpct_el0");
reader!(cntvct, "cntvct_el0");
reader!(cntp_ctl, "cntp_ctl_el0");
reader!(cntp_cval, "cntp_cval_el0");
reader!(cntp_tval, "cntp_tval_el0");
reader!(cntv_ctl, "cntv_ctl_el0");
writer!(set_cntfrq, "cntfrq_el0");
writer!(set_cntkctl, "cntkctl_el1");
writer!(set_cntp_ctl, "cntp_ctl_el0");
writer!(set_cntp_cval, "cntp_cval_el0");
writer!(set_cntp_tval, "cntp_tval_el0");
writer!(set_cntv_ctl, "cntv_ctl_el0");
writer!(set_cntv_tval, "cntv_tval_el0");
writer!(set_vbar, "vbar_el1");

fn unmask_irq() {
    unsafe { core::arch::asm!("msr daifclr, #2", options(nomem, nostack)) };
}
fn mask_irq() {
    unsafe { core::arch::asm!("msr daifset, #2", options(nomem, nostack)) };
}
fn wfi() {
    unsafe { core::arch::asm!("wfi", options(nomem, nostack)) };
}

fn ticks() -> u64 {
    unsafe { read_volatile(&raw const TICKS) }
}
fn ctl_seen() -> u64 {
    unsafe { read_volatile(&raw const CTL_SEEN) }
}
fn esr_seen() -> u64 {
    unsafe { read_volatile(&raw const ESR_SEEN) }
}
fn clear_esr() {
    unsafe { write_volatile(&raw mut ESR_SEEN, 0) };
}

/// Spin until the handler has run `want` times, or give up.
///
/// Bounded because a timer that never fires must fail the case rather than the
/// suite's access budget: "no result within the access budget" says nothing
/// about which of eleven cases was the one that hung.
fn wait_for_ticks(want: u64) -> bool {
    for _ in 0..100_000u64 {
        if ticks() >= want {
            return true;
        }
    }
    false
}

/// `ENABLE`, `IMASK` and `ISTATUS` as `CNT{P,V}_CTL_EL0` spells them.
const ENABLE: u64 = 1;
const IMASK: u64 = 2;
const ISTATUS: u64 = 4;

fn run() -> Report {
    // A vector table of the guest's own, first, so anything unexpected from
    // here on reports which vector it went to rather than hanging.
    set_vbar(&raw const a64_vectors as u64);

    // ------------------------------------------------------------------
    // 1. `CNTFRQ_EL0` is RW at the highest implemented exception level
    // ------------------------------------------------------------------
    //
    // Which is EL1 here, because this core has no EL2 and no EL3. It holds
    // whatever firmware put there and nothing checks it against the hardware —
    // so the only thing that can be asserted is that it holds what was written.
    set_cntfrq(24_000_000);
    if cntfrq() != 24_000_000 {
        return (1, cntfrq(), 24_000_000, 0);
    }

    // ------------------------------------------------------------------
    // 2. The count advances, and never backwards
    // ------------------------------------------------------------------
    let first = cntpct();
    let second = cntpct();
    if second < first {
        return (2, second, first, 0);
    }
    let mut later = second;
    for _ in 0..1000 {
        later = cntpct();
        if later > first {
            break;
        }
    }
    if later <= first {
        return (2, later, first, 1);
    }

    // ------------------------------------------------------------------
    // 3. Without EL2 the virtual count *is* the physical one
    // ------------------------------------------------------------------
    //
    // `CNTVOFF_EL2` is architecturally zero when EL2 is not implemented. The
    // two cannot be read at the same instant, so the assertable form of "they
    // are equal" is that a virtual count read between two physical ones falls
    // between them.
    let before = cntpct();
    let virt = cntvct();
    let after = cntpct();
    if virt < before || virt > after {
        return (3, virt, before, after);
    }

    // ------------------------------------------------------------------
    // 4. `TVAL` is a countdown relative to now
    // ------------------------------------------------------------------
    set_cntp_tval(10_000);
    let cval = cntp_cval();
    let now = cntpct();
    if cval < now || cval - now > 10_000 {
        return (4, cval, now, 0);
    }
    // ...and reading it back gives the distance that is left. `remaining` was
    // sampled before `now2`, so `remaining + now2` lands on or just past the
    // comparator, never short of it.
    let remaining = cntp_tval();
    let now2 = cntpct();
    if remaining + now2 < cval || remaining + now2 > cval + 64 {
        return (4, remaining + now2, cval, 1);
    }

    // A negative `TVAL` is a deadline in the past, which is how a driver asks
    // for "fire at once". Zero-extending it instead would put the deadline four
    // billion counts away and hang the guest.
    set_cntp_tval(0xffff_ffff);
    let cval = cntp_cval();
    if cntpct().wrapping_sub(cval) > 64 {
        return (4, cval, cntpct(), 2);
    }

    // ------------------------------------------------------------------
    // 5. `ISTATUS` is computed, read-only, and gated on `ENABLE`
    // ------------------------------------------------------------------
    //
    // The comparator is still in the past from the case above.
    set_cntp_ctl(0);
    if cntp_ctl() != 0 {
        return (5, cntp_ctl(), 0, 0);
    }
    // Writing `ISTATUS` must not store it: a driver that reads the register
    // and writes it straight back would otherwise pin the bit forever.
    set_cntp_ctl(ENABLE | IMASK | ISTATUS);
    if cntp_ctl() != ENABLE | IMASK | ISTATUS {
        return (5, cntp_ctl(), ENABLE | IMASK | ISTATUS, 1);
    }
    // `IMASK` gates the output and not the status, so with interrupts unmasked
    // nothing is taken even though the condition is met.
    unmask_irq();
    for _ in 0..64 {
        core::hint::spin_loop();
    }
    mask_irq();
    if ticks() != 0 {
        return (5, ticks(), 0, 2);
    }
    // Clearing `ENABLE` clears `ISTATUS` with it, whatever the comparator says.
    set_cntp_ctl(0);
    if cntp_ctl() & ISTATUS != 0 {
        return (5, cntp_ctl(), 0, 3);
    }

    // ------------------------------------------------------------------
    // 6. A tick: the timer interrupts the guest that armed it
    // ------------------------------------------------------------------
    set_cntp_tval(500);
    set_cntp_ctl(ENABLE);
    unmask_irq();
    let arrived = wait_for_ticks(1);
    mask_irq();
    if !arrived {
        return (6, ticks(), 1, 0);
    }
    if ctl_seen() != ENABLE | ISTATUS {
        return (6, ctl_seen(), ENABLE | ISTATUS, 1);
    }
    if cntp_ctl() != 0 {
        return (6, cntp_ctl(), 0, 2);
    }

    // ------------------------------------------------------------------
    // 7. `WFI` ends because the timer fired
    // ------------------------------------------------------------------
    //
    // With `PSTATE.I` **set**. DDI 0487 D1: `WFI` ends on a wake-up event, and
    // a pending interrupt is one even when the mask would stop it being taken.
    // That distinction is the whole reason an idle kernel can sleep with
    // interrupts off and still be woken by its own tick.
    set_cntp_tval(500);
    set_cntp_ctl(ENABLE);
    let before = ticks();
    wfi();
    if ticks() != before {
        return (7, ticks(), before, 0);
    }
    if cntp_ctl() & ISTATUS == 0 {
        return (7, cntp_ctl(), ISTATUS, 1);
    }
    // ...and now let it in.
    unmask_irq();
    let arrived = wait_for_ticks(before + 1);
    mask_irq();
    if !arrived {
        return (7, ticks(), before + 1, 2);
    }

    // ------------------------------------------------------------------
    // 7b. The idle loop, which is `WFI` with interrupts *enabled*
    // ------------------------------------------------------------------
    //
    // The wake-up event ends the stall and the interrupt is taken on top of
    // it, so the handler returns to the instruction after the `WFI` and the
    // guest goes on.
    //
    // This is the sequence, not a trap for it: the core got the `State::wfi`
    // bookkeeping wrong here and this case still passed, because the flag it
    // leaked was cleared again by the next step. What pins that is a unit
    // test (`a_wfi_ended_by_a_taken_interrupt_does_not_stall_again`), and the
    // gap is worth naming — a guest sees an idle loop work, and cannot see
    // that the core believed itself asleep while running the handler.
    let before = ticks();
    set_cntp_tval(500);
    set_cntp_ctl(ENABLE);
    unmask_irq();
    wfi();
    // Reached only because the `WFI` completed *and* the handler returned
    // here. `wfi` is not a loop; if the stall had been re-entered this would
    // never run and the case below would report the tick count it saw.
    let after = ticks();
    mask_irq();
    if after != before + 1 {
        return (71, after, before + 1, 0);
    }
    if cntp_ctl() != 0 {
        return (71, cntp_ctl(), 0, 1);
    }

    // ------------------------------------------------------------------
    // 8. The virtual timer is a second timer, not an alias of the first
    // ------------------------------------------------------------------
    let before = ticks();
    set_cntv_tval(500);
    set_cntv_ctl(ENABLE);
    if cntp_ctl() != 0 {
        return (8, cntp_ctl(), 0, 0);
    }
    unmask_irq();
    let arrived = wait_for_ticks(before + 1);
    mask_irq();
    if !arrived {
        return (8, ticks(), before + 1, 1);
    }
    if cntv_ctl() != 0 {
        return (8, cntv_ctl(), 0, 2);
    }

    // ------------------------------------------------------------------
    // 9. A comparator far in the future does not fire
    // ------------------------------------------------------------------
    //
    // A comparator half the counter away is not met, so nothing fires. This
    // catches an inverted or truncated comparison; it does **not** catch an
    // unsigned one, which agrees here and only disagrees across the counter's
    // wrap — 2⁶⁴ counts away, which no guest can reach. That half is a unit
    // test (`the_timer_comparison_is_signed`), and saying so is the point:
    // mutating the core to `count >= cval` leaves this guest passing.
    set_cntp_cval(1u64 << 62);
    set_cntp_ctl(ENABLE);
    if cntp_ctl() & ISTATUS != 0 {
        return (9, cntp_ctl(), 0, 0);
    }
    let before = ticks();
    unmask_irq();
    for _ in 0..1000 {
        core::hint::spin_loop();
    }
    mask_irq();
    if ticks() != before {
        return (9, ticks(), before, 1);
    }
    set_cntp_ctl(0);

    // ------------------------------------------------------------------
    // 10. EL0 reaches nothing until `CNTKCTL_EL1` says so
    // ------------------------------------------------------------------
    //
    // And the refusal is a *trap*, EC 0x18, carrying the encoding of the
    // instruction — not an UNDEFINED. The handler above proves the difference
    // is usable: it reads the syndrome, steps over the `MRS` and returns, which
    // is the shape of a kernel emulating a register it has not delegated.
    set_cntkctl(0);
    clear_esr();
    unsafe { el0_probe() };
    let esr = esr_seen();
    if esr >> 26 != 0x18 {
        return (10, esr >> 26, 0x18, 0);
    }
    let iss = esr & 0x01ff_ffff;
    // DDI 0487 D17.2.37: Op0 21:20, Op2 19:17, Op1 16:14, CRn 13:10, Rt 9:5,
    // CRm 4:1, Direction bit 0. `CNTVCT_EL0` is op0 3, op1 3, CRn 14, CRm 0,
    // op2 2, read, into x1.
    let want = (3 << 20) | (2 << 17) | (3 << 14) | (14 << 10) | (1 << 5) | 1;
    if iss != want {
        return (10, iss, want, 1);
    }

    // With the right bit set the same instruction is an ordinary read, and
    // with the wrong one it is not: `EL0PCTEN` does not open `CNTVCT_EL0`.
    set_cntkctl(1); // EL0PCTEN
    clear_esr();
    unsafe { el0_probe() };
    if esr_seen() == 0 {
        return (10, 0, 1, 2);
    }
    set_cntkctl(2); // EL0VCTEN
    clear_esr();
    unsafe { el0_probe() };
    if esr_seen() != 0 {
        return (10, esr_seen(), 0, 3);
    }

    PASS
}
