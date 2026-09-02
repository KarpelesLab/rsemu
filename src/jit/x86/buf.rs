//! The W^X code buffer, and the three system calls it needs.
//!
//! **This is the JIT code buffer** — one of the six subsystems `ROADMAP.md`
//! §0 allows to opt back into `unsafe`, and the only file in `jit/` that does.
//! Everything else in this backend is safe Rust that builds a `Vec<u8>`; this
//! file is where those bytes become memory the processor will execute, and
//! where that memory is entered.
//!
//! # W^X
//!
//! The mapping is **never writable and executable at the same time**. It is
//! created `PROT_READ|PROT_WRITE`, and every transition is explicit:
//! [`CodeBuf::push`] takes it back to `RW` and [`CodeBuf::entry`] flips it to
//! `PROT_READ|PROT_EXEC` before handing out an address to call. The state is
//! tracked so the flips are lazy — two `mprotect` calls per compile-then-run
//! transition, not two per block — and a debug assertion pins the invariant
//! rather than leaving it to the reader.
//!
//! # Raw syscalls, not libc
//!
//! CLAUDE.md, "Dependency policy": *OS interaction is by raw syscall (the
//! `purestd` pattern), not via `libc`.* `accel::sys` is the audited precedent
//! and the shape here is deliberately its shape — one `asm!` block, wrapped by
//! small functions that each establish what one kernel entry point needs. It
//! is not *reused*, because `accel::sys` is compiled only with the
//! `accel-kvm` feature, and a JIT that worked only when the KVM feature
//! happened to be on would be a worse answer than a second transcription of
//! three syscall numbers.
//!
//! ## Sources
//!
//! * The x86-64 System V syscall convention: number in `rax`, arguments in
//!   `rdi`, `rsi`, `rdx`, `r10`, `r8`, `r9`, result in `rax`, with `rcx` and
//!   `r11` destroyed by the `syscall` instruction (*Intel SDM* volume 2,
//!   `SYSCALL`; System V AMD64 ABI, "AMD64 Linux Kernel Conventions").
//! * Syscall numbers from the x86-64 table
//!   (`arch/x86/entry/syscalls/syscall_64.tbl`): `mmap` 9, `mprotect` 10,
//!   `munmap` 11. Stable ABI since 2.6.
//! * `PROT_*` and `MAP_*` values from `asm-generic/mman-common.h`, likewise
//!   stable ABI.

#![allow(unsafe_code)]

#[cfg(test)]
use alloc::vec::Vec;

/// The default code buffer: one mebibyte.
///
/// A block compiles to a few hundred bytes, so this holds a few thousand of
/// them — the same order as [`BlockCache`](crate::jit::BlockCache)'s default
/// capacity, which is what decides how many are live at once. When it fills,
/// [`CodeBuf::reset`] throws the lot away and bumps a generation, so running
/// out costs re-compilation rather than an allocation failure.
pub const DEFAULT_CAPACITY: u64 = 1 << 20;

const SYS_MMAP: u64 = 9;
const SYS_MPROTECT: u64 = 10;
const SYS_MUNMAP: u64 = 11;

const PROT_READ: u64 = 0x1;
const PROT_WRITE: u64 = 0x2;
const PROT_EXEC: u64 = 0x4;
const MAP_PRIVATE: u64 = 0x02;
const MAP_ANONYMOUS: u64 = 0x20;

/// The host page size this module assumes.
///
/// Hard-coded, as in `accel::sys`: `mmap` and `mprotect` both round to it, and
/// a host with a larger base page would need this file recompiled regardless.
/// Only used to round the buffer's length up.
const PAGE: u64 = 4096;

/// Issue a system call with six arguments.
///
/// # Safety
///
/// The caller must uphold whatever the named system call requires of its
/// arguments. Nothing here can check any of it; the four callers below are
/// each a wrapper that establishes exactly what one entry point needs, which
/// is why this function is private.
#[inline]
unsafe fn syscall6(n: u64, a1: u64, a2: u64, a3: u64, a4: u64, a5: u64, a6: u64) -> i64 {
    let ret: i64;
    // SAFETY: the register assignment is the x86-64 Linux kernel calling
    // convention (see the module's Sources). `rcx` and `r11` are declared
    // clobbered because the `syscall` instruction itself overwrites them with
    // the return address and the saved flags; not declaring them is the
    // classic way to corrupt a caller. `nostack` is correct because `syscall`
    // neither pushes nor uses the red zone. Whether the *arguments* are
    // meaningful for `n` is the caller's obligation, stated above.
    unsafe {
        core::arch::asm!(
            "syscall",
            inlateout("rax") n => ret,
            in("rdi") a1,
            in("rsi") a2,
            in("rdx") a3,
            in("r10") a4,
            in("r8") a5,
            in("r9") a6,
            lateout("rcx") _,
            lateout("r11") _,
            options(nostack)
        );
    }
    ret
}

/// Whether a raw return is an error.
///
/// The kernel returns `-errno` in the last page's worth of values and anything
/// else is success — the same test every libc's syscall stub makes, and the
/// reason `mmap` can return an address whose signed value is negative.
#[inline]
fn failed(ret: i64) -> bool {
    (-4095..0).contains(&ret)
}

/// Which protection the mapping currently carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Prot {
    /// Readable and writable. Never executable.
    Write,
    /// Readable and executable. Never writable.
    Exec,
}

/// An executable code buffer with a write phase and an execute phase.
///
/// Not `Clone`, not `Copy`, and its address is never handed out except as a
/// function pointer through [`CodeBuf::entry`], so the mapping's lifetime is
/// exactly this value's.
#[derive(Debug)]
pub struct CodeBuf {
    addr: u64,
    len: u64,
    used: u64,
    prot: Prot,
    generation: u64,
}

impl CodeBuf {
    /// Map `len` bytes, rounded up to a page, readable and writable.
    ///
    /// `None` if the kernel refused, which a caller treats as *no compiled
    /// backend on this host* rather than as a failure: the IR interpreter is
    /// always the fallback (`ROADMAP.md` §9, "Backends").
    #[must_use]
    pub fn new(len: u64) -> Option<CodeBuf> {
        let len = len.max(PAGE).next_multiple_of(PAGE);
        // SAFETY: a null hint lets the kernel choose the address, so no
        // existing mapping of this process can be replaced. `len` is a
        // non-zero multiple of the page size. The descriptor is -1 and the
        // offset 0, which is what `MAP_ANONYMOUS` requires. `mmap`
        // dereferences nothing, and the region it returns is owned by the
        // `CodeBuf` built from it and unmapped in its `Drop`.
        let ret = unsafe {
            syscall6(
                SYS_MMAP,
                0,
                len,
                PROT_READ | PROT_WRITE,
                MAP_PRIVATE | MAP_ANONYMOUS,
                -1i64 as u64,
                0,
            )
        };
        if failed(ret) {
            return None;
        }
        Some(CodeBuf {
            addr: ret as u64,
            len,
            used: 0,
            prot: Prot::Write,
            generation: 1,
        })
    }

    /// How many bytes are committed.
    #[inline]
    #[must_use]
    pub fn used(&self) -> u64 {
        self.used
    }

    /// How many bytes the mapping holds.
    #[inline]
    #[must_use]
    pub fn capacity(&self) -> u64 {
        self.len
    }

    /// Which generation of code this buffer is serving.
    ///
    /// Bumped by [`CodeBuf::reset`], so an offset handed out before a reset can
    /// be told from one handed out after. That is the whole of the buffer's
    /// invalidation story: a stale reference is *rejected* rather than
    /// followed, and the block behind it is compiled again.
    #[inline]
    #[must_use]
    pub fn generation(&self) -> u64 {
        self.generation
    }

    /// Forget every byte, and every offset handed out so far.
    pub fn reset(&mut self) {
        self.used = 0;
        self.generation = self.generation.wrapping_add(1);
    }

    /// Append `code`, returning the offset it landed at.
    ///
    /// `None` when it does not fit; the caller resets and tries again, or gives
    /// up and interprets.
    pub fn push(&mut self, code: &[u8]) -> Option<u64> {
        let len = code.len() as u64;
        if self.used.checked_add(len)? > self.len {
            return None;
        }
        self.protect(Prot::Write)?;
        let at = self.used;
        // SAFETY: `self.addr` is a live mapping of `self.len` bytes, currently
        // `PROT_READ|PROT_WRITE` — `protect` above returned `Some`, and
        // nothing but `protect` changes `prot`. `at + len <= self.len` was
        // checked immediately above, so the destination range is wholly inside
        // the mapping. The source is a `&[u8]` of exactly `len` bytes, and the
        // mapping is private and anonymous so the two cannot overlap. `u8`
        // needs no alignment.
        unsafe {
            core::ptr::copy_nonoverlapping(code.as_ptr(), (self.addr + at) as *mut u8, code.len());
        }
        self.used += len;
        Some(at)
    }

    /// The function at `offset`, ready to call.
    ///
    /// Flips the mapping to `PROT_READ|PROT_EXEC` first, so the returned
    /// pointer never names writable memory.
    ///
    /// # Safety
    ///
    /// `offset` must name the first byte of a function this buffer holds,
    /// pushed in the current [`CodeBuf::generation`], that follows the System V
    /// AMD64 calling convention for [`Entry`] and that is sound to execute with
    /// the argument the caller passes. None of that is checkable here: it is
    /// the code generator's obligation, and `Compiled` is the only type outside
    /// this file's tests that constructs one.
    pub unsafe fn entry(&mut self, offset: u64) -> Option<Entry> {
        if offset >= self.used {
            return None;
        }
        self.protect(Prot::Exec)?;
        let addr = self.addr + offset;
        // SAFETY: `addr` is inside a live mapping that is
        // `PROT_READ|PROT_EXEC` — `protect` above returned `Some`. That the
        // bytes there are a function of this signature is the caller's
        // obligation, restated in this function's own `# Safety` section.
        Some(unsafe { core::mem::transmute::<u64, Entry>(addr) })
    }

    /// Move the mapping to `want`, if it is not there already.
    fn protect(&mut self, want: Prot) -> Option<()> {
        if self.prot == want {
            return Some(());
        }
        let bits = match want {
            Prot::Write => PROT_READ | PROT_WRITE,
            Prot::Exec => PROT_READ | PROT_EXEC,
        };
        debug_assert!(
            bits & PROT_WRITE == 0 || bits & PROT_EXEC == 0,
            "W^X: the code buffer is never writable and executable at once"
        );
        // SAFETY: `addr` and `len` are exactly what a successful `mmap` in
        // this module returned and name a live mapping this value owns; `bits`
        // is one of the two constants immediately above. `mprotect`
        // dereferences nothing. No borrow of the mapping's contents outlives
        // this call: the only thing that escapes is an `Entry` from `entry`,
        // which borrows `&mut self`, so no caller can hold one across a flip
        // back to writable.
        let ret = unsafe { syscall6(SYS_MPROTECT, self.addr, self.len, bits, 0, 0, 0) };
        if failed(ret) {
            return None;
        }
        self.prot = want;
        Some(())
    }
}

/// A compiled block's entry point.
///
/// One argument, the execution context; one result, the status code the
/// generated epilogue leaves in `rax`. Everything else a block needs is
/// reachable from the context, which is what keeps this signature from having
/// to know anything about a guest.
pub type Entry = unsafe extern "sysv64" fn(*mut core::ffi::c_void) -> u64;

impl Drop for CodeBuf {
    fn drop(&mut self) {
        // SAFETY: `addr`/`len` are exactly what a successful `mmap` in this
        // module returned and have never been handed to `munmap` before —
        // `CodeBuf` is not `Copy` and offers no way to unmap early. The only
        // things that escape are `Entry` function pointers from `entry`, which
        // borrow `&mut self` and so cannot outlive this drop.
        unsafe {
            let _ = syscall6(SYS_MUNMAP, self.addr, self.len, 0, 0, 0, 0);
        }
    }
}

/// Bytes that return their argument's low 32 bits, for testing the buffer
/// without the code generator.
///
/// `mov eax, edi; ret` — three bytes, and the smallest thing that proves a
/// mapping was really made executable and really entered.
#[cfg(test)]
pub(crate) fn identity_stub() -> Vec<u8> {
    alloc::vec![0x89, 0xf8, 0xc3]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_buffer_maps_writes_and_executes() {
        let mut buf = CodeBuf::new(4096).expect("mmap");
        let at = buf.push(&identity_stub()).expect("it fits");
        assert_eq!(at, 0);
        // SAFETY: `at` names the first byte of `identity_stub`, pushed in this
        // generation, which is `mov eax, edi; ret` — a function of exactly this
        // signature that dereferences nothing and returns its first argument.
        let f = unsafe { buf.entry(at) }.expect("mprotect");
        // SAFETY: as above; the stub does not dereference its argument.
        let got = unsafe { f(0x1234 as *mut core::ffi::c_void) };
        assert_eq!(got, 0x1234);
    }

    #[test]
    fn a_buffer_is_never_writable_and_executable_at_once() {
        let mut buf = CodeBuf::new(4096).expect("mmap");
        assert_eq!(buf.prot, Prot::Write);
        let at = buf.push(&identity_stub()).expect("it fits");
        // SAFETY: as in the test above.
        let _ = unsafe { buf.entry(at) }.expect("mprotect");
        assert_eq!(buf.prot, Prot::Exec);
        // and writing again takes it back, rather than widening.
        buf.push(&identity_stub()).expect("it fits");
        assert_eq!(buf.prot, Prot::Write);
    }

    #[test]
    fn a_full_buffer_refuses_rather_than_growing() {
        let mut buf = CodeBuf::new(4096).expect("mmap");
        let big = alloc::vec![0x90u8; 4096];
        assert_eq!(buf.push(&big), Some(0));
        assert_eq!(buf.push(&[0x90]), None);
        // A reset is the whole invalidation story, and it is visible.
        let before = buf.generation();
        buf.reset();
        assert_eq!(buf.generation(), before + 1);
        assert_eq!(buf.push(&[0x90]), Some(0));
    }

    #[test]
    fn an_offset_past_the_end_has_no_entry_point() {
        let mut buf = CodeBuf::new(4096).expect("mmap");
        buf.push(&identity_stub()).expect("it fits");
        // SAFETY: the call refuses before forming a pointer, so the obligation
        // is discharged vacuously — which is what the assertion checks.
        assert!(unsafe { buf.entry(99) }.is_none());
    }
}
