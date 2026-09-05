//! The level-3 dynamic-linking guest: an `ET_DYN` executable with a
//! `PT_INTERP`, which is what almost every program on a real system is.
//!
//! `scripts/fetch-testdata.sh usermode-guests` links this against
//! [`lib.rs`](lib) and points its `PT_INTERP` at `/lib/ld-linux-aarch64.so.1`,
//! which the script stages from a cross sysroot on the host. The loader that
//! then runs is a **real** one, doing real relocation processing;
//! `src/usermode/proof.rs` supplies only what an operating system supplies —
//! the two images at the bases it chose, and an auxiliary vector that
//! describes each of them to the other.
//!
//! There is no libc here on purpose. `hello.rs` is the guest that proves a
//! compiler's whole runtime works; this one isolates the loader, so that a
//! failure is a failure of the loading rather than of the four hundred
//! instructions a `__libc_start_main` executes on the way to `main`. It also
//! keeps the guest honest about what it is measuring: the string it prints
//! lives in the *shared object*, and the length it prints it with comes from a
//! function there, so nothing is on screen unless a data relocation and a
//! function relocation both resolved.
//!
//! Written against the raw kernel ABI rather than a libc, so `svc`/`ecall`/
//! `syscall` is written out. The x86-64 arm exists so the same program can be
//! built for the host and run under `strace`, which is how every level-3
//! syscall in this tree was settled (`docs/system/usermode-abi.md`).

#![no_std]
#![no_main]

use core::arch::asm;

/// A guest with no libc has no unwinder; a panic is an exit status.
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    exit(101)
}

unsafe extern "C" {
    /// Resolved by the loader as a data relocation.
    static GREETING: [u8; 24];
    /// Resolved by the loader as a function relocation.
    fn greeting_len() -> usize;
}

/// `write(2)`, by the syscall the architecture uses.
fn write(fd: u64, buf: *const u8, len: usize) -> i64 {
    let ret: i64;
    unsafe {
        #[cfg(target_arch = "aarch64")]
        asm!(
            "svc #0",
            in("x8") 64u64,
            inlateout("x0") fd => ret,
            in("x1") buf,
            in("x2") len,
            options(nostack)
        );
        #[cfg(target_arch = "riscv64")]
        asm!(
            "ecall",
            in("a7") 64u64,
            inlateout("a0") fd => ret,
            in("a1") buf,
            in("a2") len,
            options(nostack)
        );
        // `syscall` clobbers `rcx` and `r11` itself, which is the one thing
        // x86-64's ABI does that the `asm-generic` architectures do not.
        #[cfg(target_arch = "x86_64")]
        asm!(
            "syscall",
            inlateout("rax") 1u64 => ret,
            in("rdi") fd,
            in("rsi") buf,
            in("rdx") len,
            lateout("rcx") _,
            lateout("r11") _,
            options(nostack)
        );
    }
    ret
}

/// `exit_group(2)`.
fn exit(status: i32) -> ! {
    unsafe {
        #[cfg(target_arch = "aarch64")]
        asm!("svc #0", in("x8") 94u64, in("x0") status, options(noreturn, nostack));
        #[cfg(target_arch = "riscv64")]
        asm!("ecall", in("a7") 94u64, in("a0") status, options(noreturn, nostack));
        #[cfg(target_arch = "x86_64")]
        asm!("syscall", in("rax") 231u64, in("rdi") status, options(noreturn, nostack));
    }
}

/// Where the dynamic loader hands control once it has relocated everything.
///
/// The stack it is entered on is the one the auxiliary vector was built on,
/// and this guest never looks at it — which is deliberate. `hello.rs` reads
/// `argv`, the environment and `AT_PHDR`; if *this* one needed them too, a
/// malformed stack would fail both tests and neither would say which layer
/// was wrong.
#[unsafe(no_mangle)]
pub extern "C" fn _start() -> ! {
    let len = unsafe { greeting_len() };
    write(1, &raw const GREETING as *const u8, len);
    exit(0)
}
