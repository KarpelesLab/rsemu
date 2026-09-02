// The shared runtime every A64 conformance guest is built around.
//
// `include!`d rather than made a module, because each guest is compiled
// standalone by `rustc --target aarch64-unknown-none` with no Cargo project
// around it — see scripts/build-a64-tests.sh.
//
// Copyright (c) Karpeles Lab Inc. MIT, like the rest of rsemu. Written from
// DDI 0487; no emulator source of any licence was consulted.
//
// ---------------------------------------------------------------------------
// The protocol
// ---------------------------------------------------------------------------
//
// A guest ends at a `BRK #0` with:
//
//   x0   0 on success, otherwise a 1-based case number
//   x1   what the case produced
//   x2   what it should have produced
//   x3   a tag naming the subtest, so a failure says which of several
//        properties of one case went wrong
//
// The runner reads those four registers. Nothing is written to memory and
// there is no symbol lookup, which keeps the ELF reader on the host side down
// to the program headers.
//
// `#![no_std]` and `#![no_main]` are *not* here: inner attributes must be the
// first thing in a crate root, and an `include!` expands too late for that.
// Each guest carries the two lines itself, above its `include!`.

// Not every guest uses it, and an unused import is not worth a `cfg`.
#[allow(unused_imports)]
use core::hint::black_box;

/// What a guest reports.
type Report = (u64, u64, u64, u64);

/// Success.
const PASS: Report = (0, 0, 0, 0);

/// The guest's stack. 16 KiB is far more than any of these needs; it is sized
/// so an accidental recursion faults on unmapped memory rather than quietly
/// running into `.bss`.
#[repr(C, align(16))]
struct Stack([u8; 16384]);

#[unsafe(no_mangle)]
static mut STACK: Stack = Stack([0; 16384]);

core::arch::global_asm!(
    ".section .text.start,\"ax\"",
    ".globl _start",
    "_start:",
    // The stack pointer. `adrp`+`add` rather than a literal pool, so the
    // prologue needs no `.ltorg` and no data in `.text`.
    "adrp x9, {stack}",
    "add x9, x9, :lo12:{stack}",
    "add x9, x9, #4, lsl #12",
    "mov sp, x9",
    // `CPACR_EL1.FPEN = 0b11`: EL0 and EL1 may use SIMD and floating point.
    // Out of reset this field is zero and the first floating-point
    // instruction traps — that is the architecture (DDI 0487 D), and every
    // real firmware does exactly this before touching an FP register.
    "mov x9, #(3 << 20)",
    "msr cpacr_el1, x9",
    "isb",
    "b {entry}",
    stack = sym STACK,
    entry = sym entry,
);

/// Run the guest and report through `BRK #0`.
#[unsafe(no_mangle)]
extern "C" fn entry() -> ! {
    let (code, got, want, tag) = run();
    unsafe {
        core::arch::asm!(
            "brk #0",
            in("x0") code,
            in("x1") got,
            in("x2") want,
            in("x3") tag,
            options(noreturn, nostack),
        )
    }
}

/// A panic is a failure with a code no case number can collide with.
#[panic_handler]
fn panicked(_: &core::panic::PanicInfo) -> ! {
    unsafe {
        core::arch::asm!(
            "brk #0",
            in("x0") u64::MAX,
            options(noreturn, nostack),
        )
    }
}

/// Compare two `binary64` results.
///
/// **NaN payloads are deliberately not compared.** The expected value comes
/// from `rustc`'s constant evaluator, and the reference says plainly that the
/// bit pattern of a NaN produced by constant folding is not guaranteed to
/// match what the target's hardware produces. So a NaN expectation asserts
/// only that the result is *a* NaN; which NaN it is, and its sign, are
/// asserted in the crate's own unit tests against DDI 0487's
/// `FPProcessNaNs` instead. This is the exact edge of what a
/// const-evaluated oracle can prove, and it is drawn here rather than left
/// implicit.
#[allow(dead_code)]
fn same_f64(got: u64, want: u64) -> bool {
    let is_nan = |bits: u64| bits & 0x7ff0_0000_0000_0000 == 0x7ff0_0000_0000_0000
        && bits & 0x000f_ffff_ffff_ffff != 0;
    if is_nan(want) {
        return is_nan(got);
    }
    got == want
}

/// The same, for `binary32` in the low half of a `u64`.
#[allow(dead_code)]
fn same_f32(got: u64, want: u64) -> bool {
    let is_nan =
        |bits: u64| bits & 0x7f80_0000 == 0x7f80_0000 && bits & 0x007f_ffff != 0;
    if is_nan(want) {
        return is_nan(got);
    }
    got == want
}

#[allow(dead_code)]
/// Read `FPSR`.
fn fpsr() -> u64 {
    let value: u64;
    unsafe { core::arch::asm!("mrs {}, fpsr", out(reg) value, options(nomem, nostack)) };
    value
}

#[allow(dead_code)]
/// Write `FPSR`, which is how the sticky exception flags are cleared.
fn set_fpsr(value: u64) {
    unsafe { core::arch::asm!("msr fpsr, {}", in(reg) value, options(nomem, nostack)) };
}

/// Read `FPCR`.
#[allow(dead_code)]
fn fpcr() -> u64 {
    let value: u64;
    unsafe { core::arch::asm!("mrs {}, fpcr", out(reg) value, options(nomem, nostack)) };
    value
}

/// Write `FPCR`, and synchronise: a rounding-mode change must be in force for
/// the next instruction.
#[allow(dead_code)]
fn set_fpcr(value: u64) {
    unsafe {
        core::arch::asm!("msr fpcr, {}", "isb", in(reg) value, options(nomem, nostack));
    }
}
