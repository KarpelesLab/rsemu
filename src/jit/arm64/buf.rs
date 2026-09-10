//! The W^X code buffer, the three system calls it needs, and the cache
//! maintenance A64 needs and x86-64 does not.
//!
//! **This is the JIT code buffer** — the subsystem CLAUDE.md's "`unsafe`"
//! section sanctions, seen on a second host. It is not an eighth site: it is
//! the same one, and every obligation below is one `jit::x86::buf` already
//! states. What is new here is one paragraph of *architecture*, and it is the
//! reason this file is not a copy of that one with different syscall numbers.
//!
//! # Why A64 needs more than `mprotect`
//!
//! On x86-64, writing bytes and then executing them is enough: the instruction
//! cache is coherent with the data cache and the processor notices a store to
//! memory it has fetched. **A64 does not promise that.** The instruction cache
//! is not required to be coherent with the data cache, so bytes written
//! through a data store may sit in the data cache while the instruction fetch
//! reads stale memory — silently, and on some cores only some of the time.
//!
//! The Arm Architecture Reference Manual (**DDI 0487**) states the required
//! sequence under *Synchronization and coherency issues between data and
//! instruction accesses*, and [`CodeBuf::sync_range`] is that sequence:
//!
//! 1. `DC CVAU` for every cache line the new code touches — clean the data
//!    cache to the point of unification, so the bytes are where an instruction
//!    fetch will look.
//! 2. `DSB ISH` — wait for that to complete, across the inner-shareable
//!    domain.
//! 3. `IC IVAU` for every line — invalidate the instruction cache, so a stale
//!    fetch is not reused.
//! 4. `DSB ISH` — wait for *that*.
//! 5. `ISB` — flush this PE's pipeline, so instructions already fetched are
//!    fetched again.
//!
//! The line sizes are not fixed by the architecture either: `CTR_EL0` carries
//! `DminLine` and `IminLine`, each the log2 of the line size **in words**, and
//! a sequence that assumed 64 bytes would skip lines on a core with 32-byte
//! ones. So this reads the register (DDI 0487, *CTR_EL0, Cache Type Register*)
//! rather than guessing. `CTR_EL0` also carries `IDC` and `DIC`, which say
//! that step 1 and step 3 respectively are *not required* on this
//! implementation; both are honoured, because on a core that sets them the
//! loops are pure cost.
//!
//! Linux permits all of this from EL0: `SCTLR_EL1.UCT` enables the `CTR_EL0`
//! read and `SCTLR_EL1.UCI` enables `DC CVAU` and `IC IVAU`, and the kernel
//! sets both — a kernel that did not would trap and emulate them rather than
//! fault.
//!
//! ## The other PE
//!
//! `IC IVAU` is broadcast to the inner-shareable domain, but `ISB` is this
//! PE's alone: DDI 0487's *Concurrent modification and execution of
//! instructions* requires every PE that will execute the new instructions to
//! have executed a context synchronization event after the invalidation. That
//! is satisfied here because a [`CodeBuf`] is owned by one
//! [`Engine`](super::rt::Engine), an engine belongs to one CPU model, and a
//! thread that migrates between PEs does so through an exception entry and
//! return, which is itself a context synchronization event. A design that ever
//! compiled on one thread and ran on another would owe an extra `ISB` on the
//! running side, and that is written here so the next person who wants to
//! share a buffer knows what it costs.
//!
//! # W^X
//!
//! **No page of the mapping is ever writable and executable at the same
//! time.** It is created `PROT_READ|PROT_EXEC` and stays that way except for
//! one **window**: the granule-aligned range [`CodeBuf::push`] is appending
//! into, which is `PROT_READ|PROT_WRITE` from the moment that push opens it
//! until [`CodeBuf::entry`] seals it again. No address inside an open window is
//! ever handed out, because `entry` seals before it returns one. The window is
//! a range rather than the whole mapping for the reason `jit::x86::buf`
//! measured: `mprotect` is O(the range), and two flips of a quarter-gigabyte
//! per compiled block cost more than the code generation they protected.
//!
//! # The granule
//!
//! x86-64 Linux has one base page size and `jit::x86::buf` hard-codes it. A64
//! has **three** translation granules — 4 KiB, 16 KiB and 64 KiB (DDI 0487
//! §D8, *The AArch64 Virtual Memory System Architecture*) — and Linux is built
//! for one of them, so a hard-coded 4 KiB would be a buffer whose `mprotect`
//! calls fail with `EINVAL` on a 64 KiB kernel. That is not a fault: `push`
//! answers `None` and the backend degrades to the interpreter. But it is a
//! whole architecture's worth of hosts running slow for a constant.
//!
//! So this rounds to **64 KiB**, the largest of the three, without asking the
//! kernel anything: every smaller granule divides it, so a 64 KiB-aligned
//! address is page-aligned on all three. The cost is a coarser window — one
//! 64 KiB range writable at a time instead of one 4 KiB range — which changes
//! neither the W^X property (still per-page, still never both) nor the
//! asymptotics of a flip (still O(1) in the buffer's size).
//!
//! # Raw syscalls, not libc
//!
//! CLAUDE.md, "Dependency policy": *OS interaction is by raw syscall (the
//! `purestd` pattern), not via `libc`.*
//!
//! ## Sources
//!
//! * The aarch64 Linux syscall convention: number in `x8`, arguments in `x0`
//!   through `x5`, result in `x0`, entered with `SVC #0` (DDI 0487 C6.2, *SVC*,
//!   for the instruction; the register assignment is Linux's, and is the same
//!   in every arm64 syscall stub).
//! * Syscall numbers from the **generic** table
//!   (`include/uapi/asm-generic/unistd.h`), which arm64 uses rather than having
//!   its own: `munmap` 215, `mmap` 222, `mprotect` 226. Stable ABI since
//!   arm64 was merged.
//! * `PROT_*` and `MAP_*` values from `asm-generic/mman-common.h`, likewise
//!   stable ABI and identical to the x86-64 values.

#![allow(unsafe_code)]

#[cfg(test)]
use alloc::vec::Vec;

/// The default code buffer: one mebibyte.
///
/// A block compiles to a kilobyte or so, so this holds a thousand of them.
/// When it fills, [`CodeBuf::reset`] throws the lot away and bumps a
/// generation, so running out costs re-compilation rather than an allocation
/// failure.
pub const DEFAULT_CAPACITY: u64 = 1 << 20;

const SYS_MUNMAP: u64 = 215;
const SYS_MMAP: u64 = 222;
const SYS_MPROTECT: u64 = 226;

const PROT_READ: u64 = 0x1;
const PROT_WRITE: u64 = 0x2;
const PROT_EXEC: u64 = 0x4;
const MAP_PRIVATE: u64 = 0x02;
const MAP_ANONYMOUS: u64 = 0x20;

/// The alignment this module rounds every mapping and every window to.
///
/// 64 KiB: the largest of A64's three translation granules, which every
/// smaller one divides. See the module docs for why this is a constant rather
/// than a question asked of the kernel.
const GRANULE: u64 = 65536;

/// Issue a system call with six arguments.
///
/// # Safety
///
/// The caller must uphold whatever the named system call requires of its
/// arguments. Nothing here can check any of it; the wrappers below each
/// establish exactly what one entry point needs, which is why this function is
/// private.
#[inline]
unsafe fn syscall6(n: u64, a1: u64, a2: u64, a3: u64, a4: u64, a5: u64, a6: u64) -> i64 {
    let ret: i64;
    // SAFETY: the register assignment is the aarch64 Linux kernel calling
    // convention (see the module's Sources). `x8` is declared `inlateout` and
    // discarded rather than `in`, so nothing here assumes the kernel preserved
    // it. `nostack` is correct because `SVC` neither pushes nor uses any red
    // zone. Whether the *arguments* are meaningful for `n` is the caller's
    // obligation, stated above.
    unsafe {
        core::arch::asm!(
            "svc #0",
            inlateout("x8") n => _,
            inlateout("x0") a1 => ret,
            in("x1") a2,
            in("x2") a3,
            in("x3") a4,
            in("x4") a5,
            in("x5") a6,
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

/// `CTR_EL0`, the Cache Type Register.
///
/// Read once per [`CodeBuf`] rather than per flush: it is a constant of the
/// implementation, and on a big.LITTLE system the architecture requires every
/// PE to report the same values for the fields used here (that is what
/// `CTR_EL0` is *for* — the kernel emulates a uniform value where the hardware
/// disagrees).
#[derive(Debug, Clone, Copy)]
struct Ctr {
    /// Data cache line size in bytes, from `DminLine`.
    dline: u64,
    /// Instruction cache line size in bytes, from `IminLine`.
    iline: u64,
    /// `IDC`: a data cache clean to the point of unification is **not**
    /// required for instruction-to-data coherence.
    idc: bool,
    /// `DIC`: an instruction cache invalidation is **not** required.
    dic: bool,
}

impl Ctr {
    /// Read the register.
    fn read() -> Ctr {
        let raw: u64;
        // SAFETY: `MRS Xt, CTR_EL0` reads a register and touches no memory.
        // It is architecturally accessible from EL0 when `SCTLR_EL1.UCT` is
        // set, which Linux sets; where it is not, the kernel traps and
        // emulates the read rather than delivering a fault, so this cannot
        // fail on the one platform this file is compiled for.
        unsafe {
            core::arch::asm!("mrs {0}, ctr_el0", out(reg) raw, options(nomem, nostack, preserves_flags));
        }
        // DDI 0487, *CTR_EL0*: `IminLine` is bits [3:0] and `DminLine` bits
        // [19:16], each the log2 of the line size in **words** — so the byte
        // size is `4 << field`. `DIC` is bit 29 and `IDC` bit 28.
        Ctr {
            dline: 4u64 << ((raw >> 16) & 0xf),
            iline: 4u64 << (raw & 0xf),
            idc: (raw >> 28) & 1 == 1,
            dic: (raw >> 29) & 1 == 1,
        }
    }
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
    /// The granule-aligned byte range of the mapping that is currently
    /// `PROT_READ|PROT_WRITE`; everything outside it is `PROT_READ|PROT_EXEC`.
    open: Option<(u64, u64)>,
    /// The byte range written since the last seal, which is what
    /// [`CodeBuf::sync_range`] has to make visible to instruction fetch. A
    /// subset of the open window — usually a few hundred bytes of it — so
    /// cleaning it is cheaper than cleaning the window.
    dirty: Option<(u64, u64)>,
    ctr: Ctr,
    generation: u64,
    flips: u64,
    syncs: u64,
}

impl CodeBuf {
    /// Map `len` bytes, rounded up to a granule, readable and executable.
    ///
    /// `None` if the kernel refused, which a caller treats as *no compiled
    /// backend on this host* rather than as a failure: the IR interpreter is
    /// always the fallback (`ROADMAP.md` §9, "Backends").
    #[must_use]
    pub fn new(len: u64) -> Option<CodeBuf> {
        let len = len.max(GRANULE).next_multiple_of(GRANULE);
        // SAFETY: a null hint lets the kernel choose the address, so no
        // existing mapping of this process can be replaced. `len` is a
        // non-zero multiple of the granule. The descriptor is -1 and the
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
            dirty: None,
            ctr: Ctr::read(),
            generation: 1,
            flips: 0,
            syncs: 0,
        })
    }

    /// How many `mprotect` calls this buffer has made.
    #[inline]
    #[must_use]
    pub fn flips(&self) -> u64 {
        self.flips
    }

    /// How many cache maintenance sequences it has run.
    ///
    /// A statistic rather than a knob, and it exists because this is the one
    /// cost the x86 backend does not have: a test asserts that a
    /// compile-then-run pair costs exactly one, so a change that ran the
    /// sequence per *push* — or, worse, per entry — would fail rather than
    /// merely be slow.
    #[inline]
    #[must_use]
    pub fn syncs(&self) -> u64 {
        self.syncs
    }

    /// The address the mapping starts at.
    ///
    /// The one thing that turns a [`CodeRef`](crate::jit::CodeRef)'s chain
    /// offset into an address a link can branch to. Exposed rather than kept
    /// behind [`CodeBuf::entry`] because the resolver — `rt`'s chain thunk —
    /// runs while the engine that owns this buffer is executing and may not
    /// borrow it; see [`Linkage`](super::rt::Linkage).
    #[inline]
    #[must_use]
    pub fn base(&self) -> u64 {
        self.addr
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
    /// be told from one handed out after. A stale reference is *rejected*
    /// rather than followed, and the block behind it is compiled again.
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
            return Some(self.used);
        }
        // Only the granules this append touches. `used` may sit in a granule a
        // previous push already opened, in which case this costs nothing.
        self.open(
            self.used & !(GRANULE - 1),
            (self.used + len).next_multiple_of(GRANULE),
        )?;
        let at = self.used;
        // SAFETY: `self.addr` is a live mapping of `self.len` bytes, and the
        // range `[at, at + len)` is `PROT_READ|PROT_WRITE` — `open` above
        // returned `Some` for exactly the granules it lands on, and nothing but
        // `open` and `seal` change a page's protection. `at + len <= self.len`
        // was checked immediately above, so the destination range is wholly
        // inside the mapping. The source is a `&[u8]` of exactly `len` bytes,
        // and the mapping is private and anonymous so the two cannot overlap.
        // `u8` needs no alignment.
        unsafe {
            core::ptr::copy_nonoverlapping(code.as_ptr(), (self.addr + at) as *mut u8, code.len());
        }
        self.used += len;
        // Remember what has to reach the instruction stream. Merged rather
        // than replaced, because two pushes may precede one seal.
        self.dirty = Some(match self.dirty {
            Some((lo, hi)) => (lo.min(at), hi.max(self.used)),
            None => (at, self.used),
        });
        Some(at)
    }

    /// The function at `offset`, ready to call.
    ///
    /// Seals any open write window first — which is where the cache
    /// maintenance happens — so the returned pointer never names writable
    /// memory, no page of the mapping is writable while the caller holds it,
    /// and the bytes behind it are the bytes an instruction fetch will see.
    ///
    /// # Safety
    ///
    /// `offset` must name the first byte of a function this buffer holds,
    /// pushed in the current [`CodeBuf::generation`], that follows AAPCS64 for
    /// [`Entry`] and that is sound to execute with the argument the caller
    /// passes. None of that is checkable here: it is the code generator's
    /// obligation, and `Compiled` is the only type outside this file's tests
    /// that constructs one.
    pub unsafe fn entry(&mut self, offset: u64) -> Option<Entry> {
        if offset >= self.used {
            return None;
        }
        self.seal()?;
        let addr = self.addr + offset;
        // SAFETY: `addr` is inside a live mapping, and the page it is on is
        // `PROT_READ|PROT_EXEC` — `seal` above returned `Some`, so no window
        // is open and every page of the mapping is executable. `seal` also ran
        // the cache maintenance for everything written since the last one, so
        // an instruction fetch from `addr` reads the bytes `push` wrote rather
        // than whatever was in the instruction cache before. That the bytes
        // there are a function of this signature is the caller's obligation,
        // restated in this function's own `# Safety` section.
        Some(unsafe { core::mem::transmute::<u64, Entry>(addr) })
    }

    /// Make `[lo, hi)` writable, sealing whatever window was open before.
    ///
    /// Both bounds are byte offsets into the mapping and both are multiples of
    /// [`GRANULE`]. Costs nothing when the range is already inside the open
    /// window, which is the ordinary case for two small blocks pushed into the
    /// same granule.
    fn open(&mut self, lo: u64, hi: u64) -> Option<()> {
        if let Some((was_lo, was_hi)) = self.open {
            if lo >= was_lo && hi <= was_hi {
                return Some(());
            }
            // The window moved — an append that crossed a granule, or a push
            // after a reset. Seal before opening, so the two ranges are never
            // writable at once.
            self.seal()?;
        }
        self.protect(lo, hi, Prot::Write)?;
        self.open = Some((lo, hi));
        Some(())
    }

    /// Return the mapping to wholly executable, making what was written
    /// visible to instruction fetch on the way.
    ///
    /// Idempotent, and a no-op when nothing is open — which is every call but
    /// the first after a push, because a block is compiled once and run many
    /// times.
    ///
    /// The maintenance runs **before** the `mprotect` rather than after, and
    /// the order is not arbitrary: `DC CVAU` and `IC IVAU` require the address
    /// to be readable, which it is in both states, but a fetch may only happen
    /// once the range is executable — so doing the cleaning first means no
    /// window exists in which the range is executable and stale.
    fn seal(&mut self) -> Option<()> {
        let Some((lo, hi)) = self.open else {
            return Some(());
        };
        if let Some((dlo, dhi)) = self.dirty.take() {
            self.sync_range(dlo, dhi);
        }
        self.protect(lo, hi, Prot::Exec)?;
        // Only once the kernel has agreed: a failed `mprotect` that cleared
        // this would leave the window writable and the bookkeeping saying it
        // was not, which is the one lie this file must not tell.
        self.open = None;
        Some(())
    }

    /// The cache maintenance A64 requires between writing instructions and
    /// executing them, over the byte range `[lo, hi)` of the mapping.
    ///
    /// See the module docs for the sequence and its source. `IDC` and `DIC`
    /// each skip one of the two loops where the implementation says it is not
    /// needed; the barriers are unconditional, because they are what orders
    /// the *stores* against the fetch and not only the maintenance against
    /// itself.
    fn sync_range(&mut self, lo: u64, hi: u64) {
        self.syncs += 1;
        let start = self.addr + lo;
        let end = self.addr + hi;
        if !self.ctr.idc {
            let mut at = start & !(self.ctr.dline - 1);
            while at < end {
                // SAFETY: `at` is inside the live mapping this value owns
                // (rounded down to a cache line, and `lo` is inside it), and
                // the mapping is `PROT_READ` in either protection state. `DC
                // CVAU` cleans a cache line and writes no memory; it is
                // accessible from EL0 with `SCTLR_EL1.UCI` set, which Linux
                // sets, and is trapped and emulated where it is not.
                unsafe {
                    core::arch::asm!("dc cvau, {0}", in(reg) at, options(nostack, preserves_flags));
                }
                at += self.ctr.dline;
            }
        }
        // SAFETY: barriers and an instruction-cache invalidation over the same
        // range, on the same argument, with the same accessibility. `DSB ISH`
        // and `ISB` take no operand and touch no memory.
        unsafe {
            core::arch::asm!("dsb ish", options(nostack, preserves_flags));
            if !self.ctr.dic {
                let mut at = start & !(self.ctr.iline - 1);
                while at < end {
                    core::arch::asm!("ic ivau, {0}", in(reg) at, options(nostack, preserves_flags));
                    at += self.ctr.iline;
                }
                core::arch::asm!("dsb ish", options(nostack, preserves_flags));
            }
            core::arch::asm!("isb", options(nostack, preserves_flags));
        }
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
            lo.is_multiple_of(GRANULE) && hi.is_multiple_of(GRANULE) && lo < hi && hi <= self.len,
            "a window is a granule-aligned range inside the mapping"
        );
        // SAFETY: `addr` is exactly what a successful `mmap` in this module
        // returned and names a live mapping this value owns; `lo` and `hi` are
        // granule-aligned offsets inside it, so `addr + lo` is page-aligned on
        // any of A64's three granules and `hi - lo` bytes from there stay
        // inside the mapping; `bits` is one of the two constants immediately
        // above. `mprotect` dereferences nothing. No borrow of the mapping's
        // contents outlives this call: the only thing that escapes is an
        // `Entry` from `entry`, which borrows `&mut self`, so no caller can
        // hold one across a flip back to writable.
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
/// generated epilogue leaves in `x0`. `extern "C"` is AAPCS64 on this target,
/// and there is no other spelling for it.
pub type Entry = unsafe extern "C" fn(*mut core::ffi::c_void) -> u64;

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
/// `mov w0, w0; ret` — two instructions, and the smallest thing that proves a
/// mapping was really made executable, really synchronised and really entered.
#[cfg(test)]
pub(crate) fn identity_stub() -> Vec<u8> {
    let mut out = Vec::new();
    // `mov w0, w0` is `ORR W0, WZR, W0`.
    out.extend_from_slice(&0x2a00_03e0u32.to_le_bytes());
    out.extend_from_slice(&0xd65f_03c0u32.to_le_bytes());
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_buffer_maps_writes_and_executes() {
        let mut buf = CodeBuf::new(GRANULE).expect("mmap");
        let at = buf.push(&identity_stub()).expect("it fits");
        assert_eq!(at, 0);
        // SAFETY: `at` names the first byte of `identity_stub`, pushed in this
        // generation, which is `mov w0, w0; ret` — a function of exactly this
        // signature that dereferences nothing and returns its first argument.
        let f = unsafe { buf.entry(at) }.expect("mprotect");
        // SAFETY: as above; the stub does not dereference its argument.
        let got = unsafe { f(0x1234 as *mut core::ffi::c_void) };
        assert_eq!(got, 0x1234);
    }

    #[test]
    fn a_buffer_is_never_writable_and_executable_at_once() {
        let mut buf = CodeBuf::new(GRANULE).expect("mmap");
        assert_eq!(buf.open, None, "a fresh mapping is wholly executable");
        let at = buf.push(&identity_stub()).expect("it fits");
        assert_eq!(
            buf.open,
            Some((0, GRANULE)),
            "and a push opens its own granule"
        );
        // SAFETY: as in the test above.
        let _ = unsafe { buf.entry(at) }.expect("mprotect");
        assert_eq!(buf.open, None, "handing out an address seals it again");
        buf.push(&identity_stub()).expect("it fits");
        assert_eq!(buf.open, Some((0, GRANULE)));
    }

    #[test]
    fn a_compile_then_run_pair_flips_twice_and_synchronises_once() {
        // The x86 backend asserts the flip count for a cost it measured. This
        // asserts the flip count *and* the maintenance count, because the
        // second is the one this host adds: a sequence run per push, or per
        // entry, would still be correct and would put a loop over the whole
        // buffer on the hot path.
        let mut buf = CodeBuf::new(16 * GRANULE).expect("mmap");
        let at = buf.push(&identity_stub()).expect("it fits");
        // SAFETY: as in the first test.
        let _ = unsafe { buf.entry(at) }.expect("mprotect");
        assert_eq!(buf.flips(), 2, "one to open the window, one to seal it");
        assert_eq!(buf.syncs(), 1, "one maintenance sequence, at the seal");
        // Running the same code again asks for neither: nothing is open and
        // nothing is dirty.
        // SAFETY: as above.
        let _ = unsafe { buf.entry(at) }.expect("mprotect");
        assert_eq!(buf.flips(), 2, "a second run of compiled code is free");
        assert_eq!(buf.syncs(), 1);
    }

    #[test]
    fn a_push_past_the_open_window_moves_it_rather_than_writing_outside_it() {
        // Getting this wrong writes into `PROT_READ|PROT_EXEC` memory, which
        // is a `SIGSEGV` and not an assertion — so the bytes are read back
        // afterwards, and the flip count pins that the old window was sealed
        // rather than merely forgotten.
        let mut buf = CodeBuf::new(4 * GRANULE).expect("mmap");
        buf.push(&[0xcc]).expect("it fits");
        assert_eq!(buf.open, Some((0, GRANULE)));
        let before = buf.flips();
        let big = alloc::vec![0x1fu8; 2 * GRANULE as usize];
        let at = buf.push(&big).expect("it fits");
        assert_eq!(at, 1);
        assert_eq!(buf.open, Some((0, 3 * GRANULE)), "the window moved");
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
        assert!(seen[1..].iter().all(|b| *b == 0x1f));
    }

    #[test]
    fn a_full_buffer_refuses_rather_than_growing() {
        let mut buf = CodeBuf::new(GRANULE).expect("mmap");
        let big = alloc::vec![0u8; GRANULE as usize];
        assert_eq!(buf.push(&big), Some(0));
        assert_eq!(buf.push(&[0]), None);
        let before = buf.generation();
        buf.reset();
        assert_eq!(buf.generation(), before + 1);
        assert_eq!(buf.push(&[0]), Some(0));
    }

    #[test]
    fn an_offset_past_the_end_has_no_entry_point() {
        let mut buf = CodeBuf::new(GRANULE).expect("mmap");
        buf.push(&identity_stub()).expect("it fits");
        // SAFETY: the call refuses before forming a pointer, so the obligation
        // is discharged vacuously — which is what the assertion checks.
        assert!(unsafe { buf.entry(99) }.is_none());
    }

    #[test]
    fn the_cache_geometry_is_read_rather_than_assumed() {
        let ctr = Ctr::read();
        // Both line sizes are a power of two of at least four bytes: the
        // register holds a log2 of a count of words, so anything else means
        // the field was decoded wrong.
        assert!(ctr.dline >= 4 && ctr.dline.is_power_of_two());
        assert!(ctr.iline >= 4 && ctr.iline.is_power_of_two());
        assert!(ctr.dline <= 2048 && ctr.iline <= 2048);
    }
}
