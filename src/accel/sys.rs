//! The Linux system calls the acceleration backends need, by raw `syscall`
//! instruction.
//!
//! `CLAUDE.md`, "Dependency policy": *OS interaction is by raw syscall (the
//! `purestd` pattern), not via `libc`*. `ROADMAP.md` §10 says the same thing
//! about this subsystem in particular — KVM is *"reachable with raw `ioctl`
//! syscalls only, so it fits the no-foreign-code rule exactly"* — and §0
//! forbids a C toolchain in the tree, so there is no header to include and no
//! `bindgen` to run. What is here instead is a transcription, with the numbers
//! cited where they come from.
//!
//! # Sources
//!
//! * The x86-64 System V syscall convention: number in `rax`, arguments in
//!   `rdi`, `rsi`, `rdx`, `r10`, `r8`, `r9`, result in `rax`, and `rcx` and
//!   `r11` destroyed by the `syscall` instruction itself (*Intel SDM* volume 2,
//!   `SYSCALL`; System V AMD64 ABI supplement, "AMD64 Linux Kernel
//!   Conventions").
//! * The syscall numbers below are the x86-64 table
//!   (`arch/x86/entry/syscalls/syscall_64.tbl`), stable ABI since 2.6.
//! * `mmap`/`munmap` flag values: `asm-generic/mman-common.h`, likewise stable
//!   ABI.
//! * `openat(2)`, `ioctl(2)`, `mmap(2)`, `munmap(2)`, `close(2)` man pages for
//!   the argument order and the error set.
//! * For the interval timer that bounds a guest entry ([`IntervalTimer`], and
//!   [`preempt`](super::preempt) for why it exists): `timer_create(2)`,
//!   `timer_settime(2)`, `timer_delete(2)` and `gettid(2)`. The kernel's
//!   `struct sigevent` is **not** glibc's — the raw one is an eight-byte
//!   `sigval_t`, then `sigev_signo` and `sigev_notify` as `int`s, then a union
//!   whose first member is the thread id, padded to `SIGEV_MAX_SIZE` of 64
//!   bytes (`include/uapi/asm-generic/siginfo.h`, where the preamble size is
//!   `sizeof(int) * 2 + sizeof(sigval_t)`). `SIGEV_THREAD_ID` is 4 and
//!   `CLOCK_MONOTONIC` is 1 (`include/uapi/linux/time.h`). The kernel's
//!   `timer_t` is a plain `int` in the raw ABI, which is why nothing here is
//!   pointer-sized.
//!
//! # `unsafe`
//!
//! This file is one of the two sanctioned sites this subsystem uses — *"the
//! raw-syscall accel backends"* (`ROADMAP.md` §0's list of seven). Every block
//! carries its invariant. The shape was chosen to keep the count low: **one**
//! `asm!` block, wrapped by small functions that each establish what one
//! kernel entry point needs, so callers elsewhere in `accel/` write no
//! `unsafe` of their own except where the operation genuinely is one (an
//! `ioctl` with an arbitrary argument).

use core::fmt;

// ---------------------------------------------------------------------------
// errno
// ---------------------------------------------------------------------------

/// A kernel error number, as returned negated in `rax`.
///
/// A newtype rather than an enum because the list is the kernel's and grows;
/// only the handful this subsystem branches on are named.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(transparent)]
pub struct Errno(pub i32);

impl Errno {
    /// No such file or directory — what `/dev/kvm` gives on a host with no KVM
    /// module loaded.
    pub const ENOENT: Errno = Errno(2);
    /// Interrupted. `KVM_RUN` returns this when the run was cut short before
    /// the guest was entered, which is how a stop request gets out.
    pub const EINTR: Errno = Errno(4);
    /// Bad file descriptor.
    pub const EBADF: Errno = Errno(9);
    /// Argument list too long — what a `CPUID` table the kernel has more
    /// entries for than the caller offered room for comes back as.
    pub const E2BIG: Errno = Errno(7);
    /// Permission denied — `/dev/kvm` exists and this user is not in its group.
    pub const EACCES: Errno = Errno(13);
    /// No such device.
    pub const ENODEV: Errno = Errno(19);
    /// Invalid argument.
    pub const EINVAL: Errno = Errno(22);
    /// Not a typewriter: what an unrecognised `ioctl` request returns.
    pub const ENOTTY: Errno = Errno(25);
    /// Function not implemented — an ioctl this kernel does not have.
    pub const ENOSYS: Errno = Errno(38);

    /// Whether this is the "there is no usable KVM here" family, which callers
    /// treat as *skip* rather than as *fail*.
    #[must_use]
    pub const fn is_unavailable(self) -> bool {
        matches!(self, Errno::ENOENT | Errno::EACCES | Errno::ENODEV)
    }
}

impl fmt::Display for Errno {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let name = match *self {
            Errno::ENOENT => "ENOENT",
            Errno::EINTR => "EINTR",
            Errno::E2BIG => "E2BIG",
            Errno::EBADF => "EBADF",
            Errno::EACCES => "EACCES",
            Errno::ENODEV => "ENODEV",
            Errno::EINVAL => "EINVAL",
            Errno::ENOTTY => "ENOTTY",
            Errno::ENOSYS => "ENOSYS",
            _ => return write!(f, "errno {}", self.0),
        };
        f.write_str(name)
    }
}

/// The result of a system call.
pub type SysResult<T> = Result<T, Errno>;

/// Turn a raw `rax` into a result.
///
/// The kernel returns `-errno` in the last page's worth of values and anything
/// else is success — the same test every libc's syscall stub makes, and the
/// reason `mmap` can return an address whose signed value is negative.
#[inline]
#[allow(clippy::cast_possible_truncation)]
fn decode(ret: i64) -> SysResult<i64> {
    if (-4095..0).contains(&ret) {
        Err(Errno(-ret as i32))
    } else {
        Ok(ret)
    }
}

// ---------------------------------------------------------------------------
// the one asm block
// ---------------------------------------------------------------------------

const SYS_CLOSE: u64 = 3;
const SYS_MMAP: u64 = 9;
const SYS_MUNMAP: u64 = 11;
const SYS_IOCTL: u64 = 16;
const SYS_GETTID: u64 = 186;
const SYS_TIMER_CREATE: u64 = 222;
const SYS_TIMER_SETTIME: u64 = 223;
const SYS_TIMER_DELETE: u64 = 226;
const SYS_OPENAT: u64 = 257;

/// `AT_FDCWD`, the "relative to the working directory" pseudo-descriptor.
const AT_FDCWD: u64 = -100i64 as u64;

/// Issue a system call with six arguments.
///
/// # Safety
///
/// The caller must uphold whatever the named system call requires of its
/// arguments — that a pointer is valid and correctly typed for `n`, that a
/// descriptor is open, that a length is right. Nothing here can check any of
/// it; every caller in this file is a wrapper that establishes exactly that
/// for one entry point, which is why this function is private.
#[allow(unsafe_code)]
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

// ---------------------------------------------------------------------------
// file descriptors
// ---------------------------------------------------------------------------

/// `O_RDWR`.
pub const O_RDWR: u64 = 2;
/// `O_CLOEXEC`. Always set: a forked child inheriting a vCPU descriptor is a
/// bug that only shows up under a test harness that shells out.
pub const O_CLOEXEC: u64 = 0o2_000_000;

/// An owned file descriptor that closes itself.
///
/// Not `std::os::fd::OwnedFd`, because `accel` reaches the kernel by raw
/// syscall and converting between the two would mean `std::os::unix`, which is
/// one more thing that does not exist on every target this crate builds for.
#[derive(Debug)]
pub struct Fd(i32);

impl Fd {
    /// The raw descriptor, for a syscall that needs it.
    #[must_use]
    pub const fn raw(&self) -> i32 {
        self.0
    }

    /// Adopt a descriptor obtained from a syscall in this crate.
    ///
    /// Crate-private on purpose: an [`Fd`] closes on drop, so adopting a
    /// descriptor somebody else owns would be a double close.
    pub(crate) const fn from_raw(fd: i32) -> Fd {
        Fd(fd)
    }
}

impl Drop for Fd {
    #[allow(unsafe_code)]
    fn drop(&mut self) {
        // SAFETY: `self.0` came from a successful `openat`, `KVM_CREATE_VM` or
        // `KVM_CREATE_VCPU` and has not been closed — `Fd` is not `Copy`, is
        // constructed only from a checked syscall return, and hands out no way
        // to close it early. `close` dereferences no memory.
        unsafe {
            let _ = syscall6(SYS_CLOSE, self.0 as u64, 0, 0, 0, 0, 0);
        }
    }
}

/// Open a path.
///
/// `path` must be NUL-terminated; the check is here rather than in a caller
/// because a missing terminator is the one way this call could read out of
/// bounds.
///
/// # Errors
///
/// [`Errno::EINVAL`] if `path` is not NUL-terminated, and whatever `openat`
/// itself returns otherwise — [`Errno::ENOENT`] where there is no KVM module,
/// [`Errno::EACCES`] where the caller is not in the `kvm` group.
#[allow(unsafe_code)]
#[allow(clippy::cast_possible_truncation)]
pub fn open(path: &[u8], flags: u64) -> SysResult<Fd> {
    if path.last() != Some(&0) {
        return Err(Errno::EINVAL);
    }
    // SAFETY: `path` is a NUL-terminated byte string (checked immediately
    // above) that outlives the call, which is the whole of what `openat`
    // requires of it; `AT_FDCWD` is the documented pseudo-descriptor to use
    // with an absolute path, and the mode argument is unused without
    // `O_CREAT`, which these flags never contain.
    let ret = unsafe { syscall6(SYS_OPENAT, AT_FDCWD, path.as_ptr() as u64, flags, 0, 0, 0) };
    decode(ret).map(|fd| Fd::from_raw(fd as i32))
}

/// Perform an `ioctl` whose argument is an integer or a pointer.
///
/// # Safety
///
/// `request` and `arg` must agree: if the request's UAPI definition names a
/// struct, `arg` must be the address of a live, correctly sized, correctly
/// aligned value of exactly that struct, writable if the request has the read
/// direction. None of that is checkable here — [`crate::accel::kvm`] pairs the
/// two in one place with a typed request table, and that is where the
/// obligation is discharged.
///
/// # Errors
///
/// Whatever the driver returns.
#[allow(unsafe_code)]
pub unsafe fn ioctl(fd: &Fd, request: u64, arg: u64) -> SysResult<i64> {
    // SAFETY: `fd` is open for as long as the borrow lasts, which is what the
    // descriptor argument requires. The pairing of `request` with `arg` is the
    // caller's obligation, restated in this function's own `# Safety` section,
    // and there is nothing further this frame can establish.
    let ret = unsafe { syscall6(SYS_IOCTL, fd.0 as u64, request, arg, 0, 0, 0) };
    decode(ret)
}

// ---------------------------------------------------------------------------
// memory maps
// ---------------------------------------------------------------------------

/// `PROT_READ | PROT_WRITE`.
pub const PROT_READ_WRITE: u64 = 0x1 | 0x2;
/// `MAP_SHARED`.
pub const MAP_SHARED: u64 = 0x01;
/// `MAP_PRIVATE`.
pub const MAP_PRIVATE: u64 = 0x02;
/// `MAP_ANONYMOUS`.
pub const MAP_ANONYMOUS: u64 = 0x20;

/// The page size this module assumes, and the granularity every KVM memory
/// region is required to have.
///
/// Hard-coded rather than asked of the kernel: `KVM_SET_USER_MEMORY_REGION`
/// rejects a guest address, a size or a host address that is not page aligned,
/// and x86-64 Linux has a 4 KiB base page. A host with a larger one would need
/// this file recompiled anyway.
pub const PAGE_SIZE: u64 = 4096;

/// An owned `mmap` region that unmaps itself.
///
/// The address is page aligned by construction, which is the property
/// `KVM_SET_USER_MEMORY_REGION` demands of `userspace_addr` and the whole
/// reason this type exists rather than a `Vec`.
#[derive(Debug)]
pub struct Mapping {
    addr: u64,
    len: u64,
}

impl Mapping {
    /// The host address of the first byte. Page aligned.
    #[must_use]
    pub const fn addr(&self) -> u64 {
        self.addr
    }

    /// The length in bytes. A whole number of pages.
    #[must_use]
    pub const fn len(&self) -> u64 {
        self.len
    }

    /// Whether the mapping is empty.
    ///
    /// Never true — a zero-length `mmap` is refused — but clippy asks for it
    /// beside `len`.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// The mapped bytes, as a slice of atomics.
    ///
    /// **The only place in `accel/` a host pointer is dereferenced.** Both
    /// users need it — guest RAM, which hardware writes while a vCPU is in the
    /// guest, and the `kvm_run` page, which the kernel writes on every exit —
    /// so keeping it in one function keeps the crate to a single
    /// `from_raw_parts`.
    ///
    /// `AtomicU8` rather than `u8` is the load-bearing choice: the other writer
    /// is outside this program entirely, and per-byte relaxed atomics are the
    /// strongest thing the Rust abstract machine can be told about that. It is
    /// also exactly the element type
    /// [`RamStore`](crate::core::space::RamStore) uses, for the same reason.
    #[allow(unsafe_code)]
    #[inline]
    #[must_use]
    pub fn cells(&self) -> &[core::sync::atomic::AtomicU8] {
        // SAFETY: `self` owns a live `mmap` of exactly `self.len` readable and
        // writable bytes at `self.addr`, held for as long as the returned
        // borrow — `Mapping` unmaps only in its own `Drop`, which cannot run
        // while `&self` is alive. `AtomicU8` is `#[repr(transparent)]` over
        // `UnsafeCell<u8>`, so it has size 1 and alignment 1 and every readable
        // byte is a valid one; an `mmap` region is page aligned, so the whole
        // slice is aligned. `self.len` is a multiple of `PAGE_SIZE` and came
        // from a successful `mmap`, so it is far below `isize::MAX` and the
        // slice cannot wrap the address space. The reference is shared, never
        // `&mut`, so concurrent writes by guest hardware or by the kernel are
        // expressed as relaxed atomic traffic rather than as a data race.
        unsafe {
            core::slice::from_raw_parts(
                self.addr as *const core::sync::atomic::AtomicU8,
                self.len as usize,
            )
        }
    }

    /// Load a little-endian `u8` at `offset`, or `None` if it is out of range.
    #[inline]
    #[must_use]
    pub fn load_u8(&self, offset: u64) -> Option<u8> {
        let at = usize::try_from(offset).ok()?;
        Some(
            self.cells()
                .get(at)?
                .load(core::sync::atomic::Ordering::Relaxed),
        )
    }

    /// Store a `u8` at `offset`, reporting whether it was in range.
    #[inline]
    pub fn store_u8(&self, offset: u64, value: u8) -> bool {
        let Ok(at) = usize::try_from(offset) else {
            return false;
        };
        match self.cells().get(at) {
            Some(cell) => {
                cell.store(value, core::sync::atomic::Ordering::Relaxed);
                true
            }
            None => false,
        }
    }

    /// Load `N` bytes at `offset` as a little-endian integer.
    ///
    /// Every field of `struct kvm_run` this crate reads goes through here, so
    /// the layout knowledge stays as a table of offsets rather than as a
    /// `#[repr(C)]` transcription of a union whose arms this build does not
    /// use.
    #[inline]
    #[must_use]
    pub fn load_le<const N: usize>(&self, offset: u64) -> Option<[u8; N]> {
        let at = usize::try_from(offset).ok()?;
        let cells = self.cells();
        let end = at.checked_add(N)?;
        if end > cells.len() {
            return None;
        }
        let mut out = [0u8; N];
        for (i, b) in out.iter_mut().enumerate() {
            *b = cells[at + i].load(core::sync::atomic::Ordering::Relaxed);
        }
        Some(out)
    }
}

impl Drop for Mapping {
    #[allow(unsafe_code)]
    fn drop(&mut self) {
        // SAFETY: `addr`/`len` are exactly what a successful `mmap` in this
        // module returned and have never been handed to `munmap` before —
        // `Mapping` is not `Copy` and offers no way to unmap early. Every user
        // of the region borrows `&self`, so no borrow outlives this drop.
        unsafe {
            let _ = syscall6(SYS_MUNMAP, self.addr, self.len, 0, 0, 0, 0);
        }
    }
}

/// Map anonymous, zeroed, private memory.
///
/// # Errors
///
/// [`Errno::EINVAL`] for a zero or unaligned length; otherwise whatever `mmap`
/// returns.
pub fn map_anonymous(len: u64) -> SysResult<Mapping> {
    map(len, MAP_PRIVATE | MAP_ANONYMOUS, None, 0)
}

/// Map `len` bytes of `fd` shared, starting at `offset`.
///
/// # Errors
///
/// [`Errno::EINVAL`] for a zero or unaligned length; otherwise whatever `mmap`
/// returns.
pub fn map_shared(fd: &Fd, len: u64, offset: u64) -> SysResult<Mapping> {
    map(len, MAP_SHARED, Some(fd), offset)
}

#[allow(unsafe_code)]
#[allow(clippy::cast_sign_loss)]
fn map(len: u64, flags: u64, fd: Option<&Fd>, offset: u64) -> SysResult<Mapping> {
    if len == 0 || !len.is_multiple_of(PAGE_SIZE) {
        return Err(Errno::EINVAL);
    }
    let raw_fd = match fd {
        Some(fd) => u64::from(fd.0 as u32),
        None => -1i64 as u64,
    };
    // SAFETY: a null hint lets the kernel choose the address, so no existing
    // mapping of this process can be replaced; `len` is a non-zero multiple of
    // the page size (checked above); `fd` is open for the duration of the call
    // or is `-1`, which is what `MAP_ANONYMOUS` requires. `mmap` dereferences
    // nothing, and the returned region is owned by the `Mapping` built from it.
    let ret = unsafe { syscall6(SYS_MMAP, 0, len, PROT_READ_WRITE, flags, raw_fd, offset) };
    decode(ret).map(|addr| Mapping {
        addr: addr as u64,
        len,
    })
}

// ---------------------------------------------------------------------------
// per-thread interval timers
// ---------------------------------------------------------------------------

/// `CLOCK_MONOTONIC`. Not `CLOCK_REALTIME`: a preemption interval must not
/// change length because somebody stepped the wall clock.
const CLOCK_MONOTONIC: u64 = 1;

/// `SIGEV_THREAD_ID`: deliver to the thread named in `sigev_notify_thread_id`
/// rather than to the process, so a two-processor board's vCPUs get their own
/// kicks instead of racing for one.
const SIGEV_THREAD_ID: i32 = 4;

/// A POSIX interval timer that signals **one thread**.
///
/// The kernel's `timer_t` is an `int` in the raw ABI (`__kernel_timer_t`);
/// glibc's opaque pointer is its own wrapper and does not apply here.
///
/// Dropping it deletes the timer, so a thread that armed one and then went
/// away leaves nothing behind to fire at a tid the kernel has reused.
#[derive(Debug)]
pub struct IntervalTimer(i32);

/// The kernel's `struct sigevent`, x86-64.
///
/// `SIGEV_MAX_SIZE` is 64 bytes: an eight-byte `sigval_t`, two `int`s, and a
/// twelve-`int` union whose first member is the thread id
/// (`include/uapi/asm-generic/siginfo.h`). Written out as an array of bytes
/// rather than as a struct with a padding array because only three fields
/// carry anything and the rest must read as zero.
#[repr(C, align(8))]
struct Sigevent {
    value: u64,
    signo: i32,
    notify: i32,
    tid: i32,
    pad: [i32; 11],
}

/// The kernel's `struct __kernel_itimerspec`: interval, then initial
/// expiration, each a `timespec` of two 64-bit words.
#[repr(C)]
struct Itimerspec {
    interval_sec: i64,
    interval_nsec: i64,
    value_sec: i64,
    value_nsec: i64,
}

/// This thread's kernel thread id, which is what [`IntervalTimer::for_this_thread`]
/// aims a timer at.
#[must_use]
#[allow(unsafe_code)]
#[allow(clippy::cast_possible_truncation)]
pub fn thread_id() -> i32 {
    // SAFETY: `gettid` takes no arguments, dereferences nothing and cannot
    // fail. There is nothing for a caller to establish.
    let ret = unsafe { syscall6(SYS_GETTID, 0, 0, 0, 0, 0, 0) };
    ret as i32
}

impl IntervalTimer {
    /// Create a timer that delivers `signal` to **the calling thread**.
    ///
    /// Must be called on the thread that is to be interrupted: the target is
    /// fixed at creation and there is no way to re-aim one afterwards.
    ///
    /// # Errors
    ///
    /// Whatever `timer_create(2)` returns — `EAGAIN` if the process is at its
    /// timer limit, `EINVAL` for a signal number the kernel will not take.
    #[allow(unsafe_code)]
    pub fn for_this_thread(signal: i32) -> SysResult<IntervalTimer> {
        let event = Sigevent {
            value: 0,
            signo: signal,
            notify: SIGEV_THREAD_ID,
            tid: thread_id(),
            pad: [0; 11],
        };
        let mut id: i32 = -1;
        // SAFETY: `timer_create`'s second argument is a `struct sigevent` the
        // kernel reads and its third an `int` the kernel writes; both locals
        // are live, correctly typed and correctly aligned for the length of
        // the call, and the kernel keeps no pointer into either. The first
        // argument is a clock id the kernel validates itself.
        let ret = unsafe {
            syscall6(
                SYS_TIMER_CREATE,
                CLOCK_MONOTONIC,
                (&raw const event) as u64,
                (&raw mut id) as u64,
                0,
                0,
                0,
            )
        };
        decode(ret)?;
        Ok(IntervalTimer(id))
    }

    /// Fire every `nanos` from now until [`IntervalTimer::disarm`].
    ///
    /// **Periodic rather than one-shot, deliberately.** A signal delivered in
    /// the window between a userspace check and the entry it was meant to
    /// prevent is consumed and lost; with an interval the next expiry recovers
    /// it, so the bound holds without any of the signal-mask machinery a
    /// one-shot design would need to close that race.
    ///
    /// A zero or sub-nanosecond interval is refused rather than silently
    /// turned into "never", which is what the kernel would make of it.
    ///
    /// # Errors
    ///
    /// `EINVAL` for a zero interval; otherwise whatever `timer_settime(2)`
    /// returns.
    #[allow(unsafe_code)]
    #[allow(clippy::cast_possible_wrap)]
    pub fn arm(&self, nanos: u64) -> SysResult<()> {
        if nanos == 0 {
            return Err(Errno::EINVAL);
        }
        let sec = (nanos / 1_000_000_000) as i64;
        let nsec = (nanos % 1_000_000_000) as i64;
        let spec = Itimerspec {
            interval_sec: sec,
            interval_nsec: nsec,
            value_sec: sec,
            value_nsec: nsec,
        };
        self.settime(&spec)
    }

    /// Stop it firing. Idempotent.
    ///
    /// # Errors
    ///
    /// Whatever `timer_settime(2)` returns.
    pub fn disarm(&self) -> SysResult<()> {
        self.settime(&Itimerspec {
            interval_sec: 0,
            interval_nsec: 0,
            value_sec: 0,
            value_nsec: 0,
        })
    }

    #[allow(unsafe_code)]
    fn settime(&self, spec: &Itimerspec) -> SysResult<()> {
        // SAFETY: `timer_settime`'s third argument is a `struct itimerspec`
        // the kernel reads; `spec` is a live, correctly typed and aligned
        // borrow for the length of the call. The fourth is a null `old_value`,
        // which the manual page documents as "do not report the previous
        // setting" rather than as a pointer the kernel writes. `self.0` is a
        // timer this process created and has not deleted, because deletion
        // consumes the `IntervalTimer`.
        let ret = unsafe {
            syscall6(
                SYS_TIMER_SETTIME,
                u64::from(self.0 as u32),
                0,
                (spec as *const Itimerspec) as u64,
                0,
                0,
                0,
            )
        };
        decode(ret).map(|_| ())
    }
}

impl Drop for IntervalTimer {
    #[allow(unsafe_code)]
    fn drop(&mut self) {
        // SAFETY: `timer_delete` takes one integer and dereferences nothing.
        // `self.0` is this process's own timer id, obtained from a checked
        // `timer_create` and deleted exactly once, since `IntervalTimer` is
        // neither `Copy` nor `Clone`.
        let _ = unsafe { syscall6(SYS_TIMER_DELETE, u64::from(self.0 as u32), 0, 0, 0, 0, 0) };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::ToString;

    #[test]
    fn a_path_without_a_terminator_is_refused_before_the_syscall() {
        assert_eq!(open(b"/dev/null", O_RDWR).unwrap_err(), Errno::EINVAL);
    }

    #[test]
    fn an_anonymous_mapping_is_page_aligned() {
        let m = map_anonymous(2 * PAGE_SIZE).expect("anonymous mmap");
        assert_eq!(m.addr() % PAGE_SIZE, 0);
        assert_eq!(m.len(), 2 * PAGE_SIZE);
        assert!(!m.is_empty());
    }

    #[test]
    fn an_unaligned_length_is_refused_before_the_syscall() {
        assert_eq!(map_anonymous(1).unwrap_err(), Errno::EINVAL);
        assert_eq!(map_anonymous(0).unwrap_err(), Errno::EINVAL);
    }

    #[test]
    fn errnos_name_themselves() {
        assert_eq!(Errno::EINTR.to_string(), "EINTR");
        assert_eq!(Errno(1234).to_string(), "errno 1234");
        assert!(Errno::ENOENT.is_unavailable());
        assert!(Errno::EACCES.is_unavailable());
        assert!(!Errno::EINVAL.is_unavailable());
    }

    #[test]
    fn a_negative_return_in_the_errno_window_is_an_error() {
        assert_eq!(decode(-2), Err(Errno::ENOENT));
        assert_eq!(decode(0), Ok(0));
        assert_eq!(decode(7), Ok(7));
        // A large negative value is an address, not an errno: `mmap` may
        // legitimately return one in the top half of the address space, and a
        // stub that tested `ret < 0` would turn it into a spurious failure.
        assert_eq!(decode(-4096), Ok(-4096));
    }

    #[test]
    fn opening_a_file_that_is_not_there_says_so() {
        assert_eq!(
            open(b"/nonexistent/rsemu/accel\0", O_RDWR).unwrap_err(),
            Errno::ENOENT
        );
    }

    #[test]
    fn a_descriptor_round_trips_through_a_real_open() {
        let fd = open(b"/dev/null\0", O_RDWR | O_CLOEXEC).expect("/dev/null opens");
        assert!(fd.raw() >= 0);
    }

    /// The transcription is right if the kernel accepts it, and the only way
    /// to find out is to ask. `SIGEV_THREAD_ID` with a bad `_tid` is refused
    /// with `EINVAL`, so a timer that is created at all is one whose structure
    /// the kernel read the way this file laid it out.
    #[test]
    fn a_per_thread_timer_is_created_armed_and_deleted() {
        assert!(thread_id() > 0, "gettid never fails");
        // 36 is `Signal::PREEMPT`; this test does not install a disposition
        // for it and does not arm long enough to be delivered before the
        // disarm below, so nothing reaches the default action.
        let timer = IntervalTimer::for_this_thread(36).expect("timer_create");
        // An hour, so it cannot fire inside this test whatever the host is
        // doing, and then off again.
        assert_eq!(timer.arm(3_600_000_000_000), Ok(()));
        assert_eq!(timer.disarm(), Ok(()));
        // Idempotent, which is what the `Hold` guard relies on.
        assert_eq!(timer.disarm(), Ok(()));
        // And the drop deletes it; a second delete of the same id would be
        // `EINVAL`, which is why `IntervalTimer` is neither `Copy` nor `Clone`.
    }

    /// A zero interval means *never* to the kernel and *no bound* to a caller,
    /// which are opposite things. Refused before the syscall rather than
    /// turned into a timer that silently does nothing.
    #[test]
    fn a_zero_interval_is_refused_before_the_syscall() {
        let timer = IntervalTimer::for_this_thread(36).expect("timer_create");
        assert_eq!(timer.arm(0), Err(Errno::EINVAL));
    }

    /// A signal number the kernel will not take is an error rather than a
    /// timer that never fires.
    #[test]
    fn an_impossible_signal_is_refused_by_the_kernel() {
        assert_eq!(
            IntervalTimer::for_this_thread(1234).map(|_| ()),
            Err(Errno::EINVAL)
        );
    }
}
