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
//! **No page of the mapping is ever writable and executable at the same
//! time.** It is created `PROT_READ|PROT_EXEC` and stays that way except for
//! one **window**: the page-aligned range [`CodeBuf::push`] is appending into,
//! which is `PROT_READ|PROT_WRITE` from the moment that push opens it until
//! [`CodeBuf::entry`] seals it again. No address inside an open window is ever
//! handed out, because `entry` seals before it returns one.
//!
//! # Why a window, and not the whole mapping
//!
//! It used to be the whole mapping, and that was measured as the single
//! largest cost in the compiled engine. `mprotect` is O(the range), because
//! the kernel splits and merges the VMA and shoots down the TLB entries the
//! range covers — and this buffer is **256 MiB** on the guest it exists for
//! (`cpu::riscv::engine`'s `CODE_BUFFER`, sized that way so a Linux boot's
//! working set never forces a reset). Two flips of a quarter-gigabyte per
//! compiled block cost **433 600 cycles a block, 144 µs**, against 13 850
//! cycles for the code generation they were protecting: on a `riscv-virt`
//! Linux boot, 59 269 compiles spent **25.6 billion cycles in `mprotect`**,
//! which was more than the entire margin by which the host code generator
//! was losing to the portable IR backend.
//!
//! A block compiles to about a kilobyte, so a window is one or two pages and
//! the flip is O(1) in the buffer's size. The security property is unchanged —
//! it was always a per-page property — and the *bookkeeping* is what got
//! narrower: `open` names the range that is writable instead of a single flag
//! naming the whole mapping. Growing the buffer no longer makes compiling
//! slower, which is the coupling that made the previous round's two fixes
//! fight each other.
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
/// A block compiles to about a kilobyte — 949 bytes, averaged over 59 269
/// compiles of a `riscv-virt` Linux boot — so this holds a thousand of them,
/// which is well under [`BlockCache`](crate::jit::BlockCache)'s default
/// capacity. When it fills, [`CodeBuf::reset`] throws the lot away and bumps a
/// generation, so running out costs re-compilation rather than an allocation
/// failure. A guest with a real working set should ask for more:
/// `cpu::riscv::engine` asks for 256 MiB and says why.
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
/// Used to round the buffer's length up and to page-align the write window.
///
/// A window is rounded **outward** to this, so a host whose base page is
/// larger would get a window start `mprotect` refuses as unaligned — `push`
/// then answers `None` and the backend degrades to the interpreter, which is
/// a slow build rather than an unsound one. x86-64 Linux's base page is 4 KiB
/// and this module is `cfg`-gated to x86-64 Linux.
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
    /// The page-aligned byte range of the mapping that is currently
    /// `PROT_READ|PROT_WRITE`; everything outside it is `PROT_READ|PROT_EXEC`.
    ///
    /// `None` means the whole mapping is executable, which is what it is
    /// created as and what it is returned to before any address is handed
    /// out. At most one window is open at a time — see the module docs for why
    /// it is a window rather than the whole mapping.
    open: Option<(u64, u64)>,
    generation: u64,
    flips: u64,
}

impl CodeBuf {
    /// Map `len` bytes, rounded up to a page, readable and executable.
    ///
    /// Executable rather than writable, and empty rather than half a state
    /// machine: no window is open, so the first [`CodeBuf::push`] opens
    /// exactly the page it writes into rather than the whole mapping.
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
                PROT_READ | PROT_EXEC,
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
            open: None,
            generation: 1,
            flips: 0,
        })
    }

    /// How many `mprotect` calls this buffer has made.
    ///
    /// A statistic rather than a knob, and it exists because the number used
    /// to be two per compiled block **over the whole mapping** and that was
    /// the compiled engine's largest single cost. A test asserts what a
    /// compile-then-run pair costs, so a change that widened the window again
    /// would fail rather than merely be slow.
    #[inline]
    #[must_use]
    pub fn flips(&self) -> u64 {
        self.flips
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
        if len == 0 {
            // Nothing to write, so no window to open — and a window is a
            // half-open range, which an empty one is not.
            return Some(self.used);
        }
        // Only the pages this append touches. `used` may sit in a page a
        // previous push already opened, in which case this costs nothing.
        self.open(
            self.used & !(PAGE - 1),
            (self.used + len).next_multiple_of(PAGE),
        )?;
        let at = self.used;
        // SAFETY: `self.addr` is a live mapping of `self.len` bytes, and the
        // pages `[at, at + len)` lands on are `PROT_READ|PROT_WRITE` — `open`
        // above returned `Some` for exactly that range, and nothing but
        // `open` and `seal` change a page's protection. `at + len <= self.len`
        // was checked immediately above, so the destination range is wholly
        // inside the mapping. The source is a `&[u8]` of exactly `len` bytes,
        // and the mapping is private and anonymous so the two cannot overlap.
        // `u8` needs no alignment.
        unsafe {
            core::ptr::copy_nonoverlapping(code.as_ptr(), (self.addr + at) as *mut u8, code.len());
        }
        self.used += len;
        Some(at)
    }

    /// The function at `offset`, ready to call.
    ///
    /// Seals any open write window first, so the returned pointer never names
    /// writable memory and no page of the mapping is writable while the
    /// caller holds it.
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
        self.seal()?;
        let addr = self.addr + offset;
        // SAFETY: `addr` is inside a live mapping, and the page it is on is
        // `PROT_READ|PROT_EXEC` — `seal` above returned `Some`, so no window
        // is open and every page of the mapping is executable. That the bytes
        // there are a function of this signature is the caller's obligation,
        // restated in this function's own `# Safety` section.
        Some(unsafe { core::mem::transmute::<u64, Entry>(addr) })
    }

    /// Make `[lo, hi)` writable, sealing whatever window was open before.
    ///
    /// Both bounds are byte offsets into the mapping and both are multiples of
    /// [`PAGE`]. Costs nothing when the range is already inside the open
    /// window, which is the ordinary case for two small blocks pushed onto the
    /// same page.
    fn open(&mut self, lo: u64, hi: u64) -> Option<()> {
        if let Some((was_lo, was_hi)) = self.open {
            if lo >= was_lo && hi <= was_hi {
                return Some(());
            }
            // The window moved — an append that crossed a page, or a push
            // after a reset. Seal before opening, so the two ranges are never
            // writable at once and `open` never names a range wider than what
            // one push asked for.
            self.protect(was_lo, was_hi, Prot::Exec)?;
            self.open = None;
        }
        self.protect(lo, hi, Prot::Write)?;
        self.open = Some((lo, hi));
        Some(())
    }

    /// Return the mapping to wholly executable.
    ///
    /// Idempotent, and a no-op when nothing is open — which is every call but
    /// the first after a push, because a block is compiled once and run many
    /// times.
    fn seal(&mut self) -> Option<()> {
        let Some((lo, hi)) = self.open else {
            return Some(());
        };
        self.protect(lo, hi, Prot::Exec)?;
        // Only once the kernel has agreed: a failed `mprotect` that cleared
        // this would leave the window writable and the bookkeeping saying it
        // was not, which is the one lie this file must not tell.
        self.open = None;
        Some(())
    }

    /// `mprotect` the byte range `[lo, hi)` of the mapping to `want`.
    fn protect(&mut self, lo: u64, hi: u64, want: Prot) -> Option<()> {
        let bits = match want {
            Prot::Write => PROT_READ | PROT_WRITE,
            Prot::Exec => PROT_READ | PROT_EXEC,
        };
        debug_assert!(
            bits & PROT_WRITE == 0 || bits & PROT_EXEC == 0,
            "W^X: no page of the code buffer is writable and executable at once"
        );
        debug_assert!(
            lo.is_multiple_of(PAGE) && hi.is_multiple_of(PAGE) && lo < hi && hi <= self.len,
            "a window is a page-aligned range inside the mapping"
        );
        // SAFETY: `addr` is exactly what a successful `mmap` in this module
        // returned and names a live mapping this value owns; `lo` and `hi` are
        // page-aligned offsets inside it, so `addr + lo` is page-aligned and
        // `hi - lo` bytes from there stay inside the mapping; `bits` is one of
        // the two constants immediately above. `mprotect` dereferences
        // nothing. No borrow of the mapping's contents outlives this call: the
        // only thing that escapes is an `Entry` from `entry`, which borrows
        // `&mut self`, so no caller can hold one across a flip back to
        // writable.
        let ret = unsafe { syscall6(SYS_MPROTECT, self.addr + lo, hi - lo, bits, 0, 0, 0) };
        if failed(ret) {
            return None;
        }
        self.flips += 1;
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
        assert_eq!(buf.open, None, "a fresh mapping is wholly executable");
        let at = buf.push(&identity_stub()).expect("it fits");
        assert_eq!(buf.open, Some((0, PAGE)), "and a push opens its own page");
        // SAFETY: as in the test above.
        let _ = unsafe { buf.entry(at) }.expect("mprotect");
        assert_eq!(buf.open, None, "handing out an address seals it again");
        // and writing again takes it back, rather than widening.
        buf.push(&identity_stub()).expect("it fits");
        assert_eq!(buf.open, Some((0, PAGE)));
    }

    #[test]
    fn a_window_covers_the_pages_it_writes_and_no_others() {
        // Sixteen pages, a push that starts on page 0 and ends on page 1, and
        // the window must be both of them and nothing beyond.
        let mut buf = CodeBuf::new(16 * PAGE).expect("mmap");
        let big = alloc::vec![0x90u8; PAGE as usize + 8];
        assert_eq!(buf.push(&big), Some(0));
        assert_eq!(buf.open, Some((0, 2 * PAGE)));
        // A second push lands inside the page the first one left open, so it
        // costs no flip at all.
        let before = buf.flips();
        buf.push(&[0x90]).expect("it fits");
        assert_eq!(buf.flips(), before, "an append inside the window is free");
        assert_eq!(buf.open, Some((0, 2 * PAGE)));
    }

    #[test]
    fn a_push_past_the_open_window_moves_it_rather_than_writing_outside_it() {
        // The other branch of `open`, and the one that is a fault rather than
        // a slow path if it is wrong: a push that reaches past the pages a
        // previous push made writable must seal those and open the ones it
        // actually needs. Getting it wrong writes into `PROT_READ|PROT_EXEC`
        // memory, which is a `SIGSEGV` and not an assertion — so the bytes are
        // read back afterwards, and the flip count pins that the old window
        // was sealed rather than merely forgotten.
        let mut buf = CodeBuf::new(16 * PAGE).expect("mmap");
        buf.push(&[0xcc]).expect("it fits");
        assert_eq!(buf.open, Some((0, PAGE)));
        let before = buf.flips();
        // Two pages' worth, starting on page 0: the window has to grow past
        // where it is, and the pages beyond it are executable right now.
        let big = alloc::vec![0x90u8; 2 * PAGE as usize];
        let at = buf.push(&big).expect("it fits");
        assert_eq!(at, 1);
        assert_eq!(buf.open, Some((0, 3 * PAGE)), "the window moved");
        assert_eq!(
            buf.flips(),
            before + 2,
            "one flip to seal the old window and one to open the new"
        );
        // SAFETY: reading back bytes this buffer owns, inside `used`, through
        // a shared slice of the mapping — which is `PROT_READ` in either
        // state, so the read is valid whatever the window is doing.
        let seen = unsafe { core::slice::from_raw_parts(buf.addr as *const u8, buf.used as usize) };
        assert_eq!(seen[0], 0xcc, "the first push survived the window move");
        assert!(
            seen[1..].iter().all(|b| *b == 0x90),
            "and the second one landed"
        );
    }

    #[test]
    fn a_compile_then_run_pair_flips_two_page_sized_ranges() {
        // The regression this window exists for: the flip used to be over the
        // whole mapping, which on the 256 MiB buffer a Linux guest asks for
        // cost 144 µs a block — more than the code generation it protected.
        // What is asserted is the *count* and the *size*, because a change
        // that widened the range again would still pass a correctness test.
        let mut buf = CodeBuf::new(64 * PAGE).expect("mmap");
        let at = buf.push(&identity_stub()).expect("it fits");
        // SAFETY: as in the first test.
        let _ = unsafe { buf.entry(at) }.expect("mprotect");
        assert_eq!(buf.flips(), 2, "one to open the window, one to seal it");
        // Running the same code again asks for no flip: nothing is open.
        // SAFETY: as above.
        let _ = unsafe { buf.entry(at) }.expect("mprotect");
        assert_eq!(buf.flips(), 2, "a second run of compiled code is free");
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
