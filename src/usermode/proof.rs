//! The proof consumer: the level-3 half rsemu deliberately does **not** ship.
//!
//! `ROADMAP.md` §2.1 draws the line at *"is it hardware?"* and puts the ELF
//! loader, the syscall table, file descriptors, errno and the process model on
//! the other side of it, in [`KarpelesLab/nixvm`]. `Cargo.toml`'s `usermode`
//! feature says the same thing in the same words. So none of that is public
//! API here and none of it ever will be — **this module is `#[cfg(test)]`**,
//! and every line of it is the *consumer's* half written out longhand so that
//! rsemu's half can be proven sufficient.
//!
//! That is not a workaround. §2.1's stated reason for keeping the two crates
//! apart is that *"a downstream consumer that needs cores, memory, a syscall
//! exit and accel — and needs them through the public API, from another crate
//! — will find every place that surface is not actually usable"*. A stand-in
//! that runs a **real** statically linked Linux binary is that consumer,
//! in-tree, on `cargo test`, without rsemu depending on anything.
//!
//! # What it proves
//!
//! `src/usermode/tests.rs` already carries phase 5b's literal gate: a
//! hand-assembled RV64 program that writes to fd 1 and exits. That proves the
//! *seam*. It does not prove the seam is **enough**, because a program somebody
//! wrote by hand to fit the seam always fits it. A compiler's output does not:
//! a static `musl` binary starts by finding its own program headers through the
//! auxiliary vector, setting up thread-local storage, sizing a heap with `brk`,
//! installing signal handlers, and asking for entropy — none of which the hand
//! assembled guest does, and every one of which goes through this module's
//! public surface.
//!
//! # The host-filesystem policy, decided before `openat` was written
//!
//! > **A level-3 guest may be told about itself. It may not be told about the
//! > host.**
//!
//! A guest asking `openat("/etc/shadow")` is asking the *host*, and the whole
//! appeal of level 3 (§2, *"run this program somewhere it cannot hurt me"*)
//! evaporates the moment that question can be answered. The rule above is the
//! one worth holding because it is the one that is **checkable**: every answer
//! this module gives comes from [`UserMemory`] or from a `Vec<u8>` the harness
//! owns, and there is no code path from a guest pointer to a host path.
//! "Which paths are safe" is not a question with a checkable answer.
//!
//! Concretely:
//!
//! * `faccessat`, `readlinkat` and `newfstatat` answer `-ENOENT` **without
//!   looking at the path at all**. The guest sees an empty namespace, which is
//!   a coherent thing for a filesystem to be, rather than a permission error
//!   that invites a retry.
//! * `openat` compares the path against exactly **one** name,
//!   `/proc/self/maps`, and answers `-ENOENT` for everything else. That one is
//!   not an exception to the rule: it is rendered from
//!   [`UserMemory::mappings`] — the guest's own address space, which
//!   `usermode::mem`'s documentation names `/proc/self/maps` as the reason for
//!   keeping — and consults nothing outside the machine.
//! * Descriptors 0, 1 and 2 are the harness's: `write` to 1 or 2 appends to a
//!   buffer the test asserts on, and `read` from 0 is a clean end of file.
//!   Descriptors above 2 exist only as the result of that one `openat`.
//! * `mmap` is anonymous-only. A file-backed mapping is `-ENODEV`, which
//!   follows from the above rather than being a second rule: there is no
//!   descriptor for a host file to map.
//!
//! There is no `--allow` flag to add to, because the moment there is one this
//! module stops being a proof of the seam and starts being a sandbox with a
//! policy to get wrong. A real consumer will need passthrough — `npm install`
//! reads files — and will have to design it; §2.1 already says that design is
//! nixvm's. "Nothing, until someone asks" is the default, and the someone is
//! not in this repository.
//!
//! The one place this module does touch the host is
//! [`guest_binary`], which reads the guest *executable* off the
//! disk before anything is running, in the harness, under `#[cfg(feature =
//! "std")]`. That is the test fixture, not a service the guest can reach.
//!
//! # `AT_RANDOM` and `getrandom` are the determinism seam
//!
//! Everything else a level-3 guest can observe is a function of the program:
//! virtual time comes from [`GuestClock`], the schedule from [`ThreadSet`],
//! and placement from [`UserMemory`]'s deterministic top-down search. Entropy
//! is the exception — it is genuinely outside the machine — so both doors it
//! comes through are [`Journal::ask`] calls and neither reaches the entropy
//! source directly:
//!
//! * the sixteen bytes the auxiliary vector's `AT_RANDOM` points at, asked for
//!   once while the stack is being built, before the guest has executed an
//!   instruction; and
//! * every `getrandom(2)`, asked for at the virtual instant of the `ecall`.
//!
//! [`Kernel::replay_guard`] is what makes that a checked claim rather than a
//! comment: in [`JournalMode::Replay`] the entropy closure is replaced by one
//! that panics, so a run that reached the host anywhere else fails the test
//! instead of quietly producing a different answer.
//!
//! [`KarpelesLab/nixvm`]: https://github.com/KarpelesLab/nixvm

use alloc::collections::BTreeMap;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;

use crate::core::clock::GlobalTime;
use crate::core::exec::{ExitReason, ExitingCore};

use super::{
    Answer, GuestClock, Journal, JournalMode, PAGE_SIZE, Prot, Tag, ThreadId, ThreadSet,
    ThreadState, UserMemory,
};

// ---------------------------------------------------------------------------
// A minimal ELF64 program loader
// ---------------------------------------------------------------------------
//
// Field offsets are transcribed from the System V gABI's `Elf64_Ehdr` and
// `Elf64_Phdr`, and the initial-stack layout from the same document's
// "Process Initialization" plus Linux's `fs/binfmt_elf.c` *behaviour* as it is
// described in `getauxval(3)` and `elf(5)` — documentation, not source.

/// What went wrong reading an image. A `String` because this is test
/// scaffolding and the message is the whole value of the error.
type LoadResult<T> = core::result::Result<T, String>;

const ET_EXEC: u16 = 2;
const ET_DYN: u16 = 3;
const PT_LOAD: u32 = 1;
const PT_INTERP: u32 = 3;
const PF_X: u32 = 1;
const PF_W: u32 = 2;
const PF_R: u32 = 4;

/// Everything the initial process image needs that only the file knows.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Image {
    /// `e_entry`, where the first instruction is.
    entry: u64,
    /// Where the program header table ended up in guest memory — `AT_PHDR`.
    ///
    /// A static binary finds its own `PT_TLS` through this, so getting it
    /// wrong is the classic "starts and immediately faults".
    phdr: u64,
    /// `e_phentsize` — `AT_PHENT`.
    phent: u64,
    /// `e_phnum` — `AT_PHNUM`.
    phnum: u64,
    /// The first page above every loaded segment: where `brk` starts.
    brk: u64,
}

fn at(file: &[u8], off: u64, len: u64) -> LoadResult<&[u8]> {
    let end = off
        .checked_add(len)
        .ok_or_else(|| "ELF: an offset overflowed".to_string())?;
    let (off, end) = (off as usize, end as usize);
    if end > file.len() || off > end {
        return Err(format!(
            "ELF: {len} byte(s) at {off:#x} run off the end of a {}-byte file",
            file.len()
        ));
    }
    Ok(&file[off..end])
}

fn u16_at(file: &[u8], off: u64) -> LoadResult<u16> {
    Ok(u16::from_le_bytes(at(file, off, 2)?.try_into().unwrap()))
}

fn u32_at(file: &[u8], off: u64) -> LoadResult<u32> {
    Ok(u32::from_le_bytes(at(file, off, 4)?.try_into().unwrap()))
}

fn u64_at(file: &[u8], off: u64) -> LoadResult<u64> {
    Ok(u64::from_le_bytes(at(file, off, 8)?.try_into().unwrap()))
}

const fn page_down(a: u64) -> u64 {
    a & !(PAGE_SIZE - 1)
}

const fn page_up(a: u64) -> u64 {
    (a + PAGE_SIZE - 1) & !(PAGE_SIZE - 1)
}

/// One `PT_LOAD`, as read.
#[derive(Debug, Clone, Copy)]
struct Phdr {
    kind: u32,
    flags: u32,
    offset: u64,
    vaddr: u64,
    filesz: u64,
    memsz: u64,
}

/// Map every `PT_LOAD` of `file` into `mem` and say where the entry, the
/// program headers and the break are.
///
/// The three things that are easy to get wrong and are therefore spelled out:
/// **`p_memsz` beyond `p_filesz` is zeroed** (that is `.bss`, and a static
/// binary's uninitialised globals are load-bearing); **segments are page
/// granular and may share a page**, so the map is built from the *union* of
/// their page ranges with the *union* of their permissions rather than one
/// mapping per segment; and **`AT_PHDR` is a guest address**, derived from the
/// `PT_LOAD` whose file range covers `e_phoff`, not from `e_phoff` itself.
fn load(mem: &UserMemory, file: &[u8], machine: u16) -> LoadResult<Image> {
    if at(file, 0, 4)? != b"\x7fELF" {
        return Err("ELF: bad magic — not an ELF file".to_string());
    }
    match at(file, 4, 1)?[0] {
        2 => {}
        c => return Err(format!("ELF: class {c}, and this loader reads ELF64 only")),
    }
    match at(file, 5, 1)?[0] {
        1 => {}
        d => return Err(format!("ELF: data encoding {d}, expected little-endian")),
    }
    let etype = u16_at(file, 16)?;
    if etype != ET_EXEC {
        return Err(format!(
            "ELF: e_type {etype}, and this loader takes ET_EXEC ({ET_EXEC}) only{}",
            if etype == ET_DYN {
                " — a position-independent executable needs a relocation pass, \
                  which is an operating system's job (ROADMAP.md §2.1)"
            } else {
                ""
            }
        ));
    }
    let em = u16_at(file, 18)?;
    if em != machine {
        return Err(format!("ELF: e_machine {em}, expected {machine}"));
    }

    let entry = u64_at(file, 24)?;
    let phoff = u64_at(file, 32)?;
    let phent = u64::from(u16_at(file, 54)?);
    let phnum = u64::from(u16_at(file, 56)?);
    if phent < 56 {
        return Err(format!("ELF: e_phentsize {phent} is below Elf64_Phdr's 56"));
    }
    if phnum == 0 {
        return Err("ELF: no program headers, so nothing to load".to_string());
    }
    // Bounds-check the whole table once, before reading any of it.
    at(file, phoff, phnum * phent)?;

    let mut loads: Vec<Phdr> = Vec::new();
    for i in 0..phnum {
        let base = phoff + i * phent;
        let ph = Phdr {
            kind: u32_at(file, base)?,
            flags: u32_at(file, base + 4)?,
            offset: u64_at(file, base + 8)?,
            vaddr: u64_at(file, base + 16)?,
            filesz: u64_at(file, base + 32)?,
            memsz: u64_at(file, base + 40)?,
        };
        if ph.kind == PT_INTERP {
            return Err(
                "ELF: PT_INTERP — a dynamically linked binary needs an interpreter, \
                 and loading one is an operating system's job (ROADMAP.md §2.1)"
                    .to_string(),
            );
        }
        if ph.kind != PT_LOAD || ph.memsz == 0 {
            continue;
        }
        if ph.filesz > ph.memsz {
            return Err(format!(
                "ELF: segment {i} has p_filesz {:#x} above p_memsz {:#x}",
                ph.filesz, ph.memsz
            ));
        }
        // Read it now so a truncated file is refused before anything is mapped.
        at(file, ph.offset, ph.filesz)?;
        ph.vaddr
            .checked_add(ph.memsz)
            .ok_or_else(|| format!("ELF: segment {i} wraps the address space"))?;
        loads.push(ph);
    }
    if loads.is_empty() {
        return Err("ELF: no PT_LOAD segment with a non-zero p_memsz".to_string());
    }

    // The union of the segments' page ranges, coalesced, mapped writable so
    // the loader can fill it. Permissions are applied afterwards, because
    // `init_bytes` may not straddle two mappings and `protect` is what splits
    // one mapping into several.
    let mut pages: Vec<(u64, u64)> = loads
        .iter()
        .map(|p| (page_down(p.vaddr), page_up(p.vaddr + p.memsz)))
        .collect();
    pages.sort_unstable();
    let mut ranges: Vec<(u64, u64)> = Vec::new();
    for (lo, hi) in pages {
        match ranges.last_mut() {
            Some(last) if lo <= last.1 => last.1 = last.1.max(hi),
            _ => ranges.push((lo, hi)),
        }
    }
    for &(lo, hi) in &ranges {
        mem.map_at(lo, hi - lo, Prot::RW, "elf")
            .map_err(|e| format!("ELF: mapping {lo:#x}..{hi:#x}: {e}"))?;
    }

    for (i, ph) in loads.iter().enumerate() {
        mem.init_bytes(ph.vaddr, at(file, ph.offset, ph.filesz)?)
            .map_err(|e| format!("ELF: filling segment {i} at {:#x}: {e}", ph.vaddr))?;
        // `p_memsz` beyond `p_filesz` is `.bss`. The mapping is fresh and
        // therefore already zero, but a well-formed ELF is allowed to place a
        // later segment's file data in a page an earlier segment's `.bss`
        // reaches into, so the zeroing is done explicitly and in file order
        // rather than left to luck.
        let bss = ph.memsz - ph.filesz;
        if bss > 0 {
            mem.init_bytes(ph.vaddr + ph.filesz, &vec![0u8; bss as usize])
                .map_err(|e| format!("ELF: zeroing segment {i}'s bss: {e}"))?;
        }
    }

    // Permissions, per page, as the union over every segment covering it —
    // then applied over maximal runs so the map stays a handful of ranges.
    let mut prot: BTreeMap<u64, Prot> = BTreeMap::new();
    for &(lo, hi) in &ranges {
        for page in (lo..hi).step_by(PAGE_SIZE as usize) {
            prot.insert(page, Prot::NONE);
        }
    }
    for ph in &loads {
        let (lo, hi) = (page_down(ph.vaddr), page_up(ph.vaddr + ph.memsz));
        let mut p = Prot::NONE;
        if ph.flags & PF_R != 0 {
            p = p.union(Prot::READ);
        }
        if ph.flags & PF_W != 0 {
            p = p.union(Prot::WRITE);
        }
        if ph.flags & PF_X != 0 {
            p = p.union(Prot::EXEC);
        }
        for page in (lo..hi).step_by(PAGE_SIZE as usize) {
            let slot = prot.get_mut(&page).expect("every page was just inserted");
            *slot = slot.union(p);
        }
    }
    let mut run: Option<(u64, u64, Prot)> = None;
    for (&page, &p) in &prot {
        match run {
            Some((base, end, have)) if end == page && have == p => {
                run = Some((base, page + PAGE_SIZE, p));
            }
            Some((base, end, have)) => {
                mem.protect(base, end - base, have)
                    .map_err(|e| e.to_string())?;
                run = Some((page, page + PAGE_SIZE, p));
            }
            None => run = Some((page, page + PAGE_SIZE, p)),
        }
    }
    if let Some((base, end, have)) = run {
        mem.protect(base, end - base, have)
            .map_err(|e| e.to_string())?;
    }

    // `AT_PHDR` is the *guest* address of the program header table: find the
    // segment whose file range covers `e_phoff` and translate through it.
    let table = phnum * phent;
    let phdr = loads
        .iter()
        .find(|p| p.offset <= phoff && phoff + table <= p.offset + p.filesz)
        .map(|p| p.vaddr + (phoff - p.offset))
        .ok_or_else(|| {
            format!("ELF: the program header table at {phoff:#x} is in no PT_LOAD segment")
        })?;

    let brk = loads
        .iter()
        .map(|p| page_up(p.vaddr + p.memsz))
        .max()
        .expect("loads is non-empty");

    Ok(Image {
        entry,
        phdr,
        phent,
        phnum,
        brk,
    })
}

// ---------------------------------------------------------------------------
// The initial stack
// ---------------------------------------------------------------------------

/// `AT_*`, the ones a static binary actually reads.
mod auxv {
    /// End of the vector.
    pub(super) const NULL: u64 = 0;
    /// Program header table address.
    pub(super) const PHDR: u64 = 3;
    /// Size of one program header.
    pub(super) const PHENT: u64 = 4;
    /// Number of program headers.
    pub(super) const PHNUM: u64 = 5;
    /// Page size.
    pub(super) const PAGESZ: u64 = 6;
    /// The program's entry point.
    pub(super) const ENTRY: u64 = 9;
    /// Real user id.
    pub(super) const UID: u64 = 11;
    /// Effective user id.
    pub(super) const EUID: u64 = 12;
    /// Real group id.
    pub(super) const GID: u64 = 13;
    /// Effective group id.
    pub(super) const EGID: u64 = 14;
    /// Hardware capability bitmap.
    pub(super) const HWCAP: u64 = 16;
    /// Clock ticks per second.
    pub(super) const CLKTCK: u64 = 17;
    /// Whether the image is setuid.
    pub(super) const SECURE: u64 = 23;
    /// Sixteen bytes of entropy.
    pub(super) const RANDOM: u64 = 25;
}

/// One auxiliary-vector value: a number, or bytes that get a stack address.
#[derive(Debug, Clone)]
enum Aux {
    /// The value is the number.
    Num(u64),
    /// The bytes are pushed onto the stack and the value is their address.
    Bytes(Vec<u8>),
}

/// Build the initial stack and return the value the stack pointer starts at.
///
/// The layout is the System V one every Linux `_start` walks:
///
/// ```text
///   sp -> argc
///         argv[0] .. argv[argc-1], NULL
///         envp[0] .. envp[n-1],    NULL
///         auxv pairs .., AT_NULL 0
///         (gap)
///         the strings and AT_RANDOM's bytes
///   top ->
/// ```
///
/// `sp` is sixteen-byte aligned, which the RISC-V psABI requires of every
/// stack pointer and which `_start` does not re-establish.
fn build_stack(
    mem: &UserMemory,
    top: u64,
    argv: &[&str],
    envp: &[&str],
    aux: &[(u64, Aux)],
) -> LoadResult<u64> {
    let mut p = top;
    let push = |bytes: &[u8], p: &mut u64| -> LoadResult<u64> {
        *p -= bytes.len() as u64;
        mem.write_bytes(*p, bytes).map_err(|e| e.to_string())?;
        Ok(*p)
    };

    let mut argv_ptrs = Vec::new();
    for s in argv {
        let mut b = s.as_bytes().to_vec();
        b.push(0);
        argv_ptrs.push(push(&b, &mut p)?);
    }
    let mut envp_ptrs = Vec::new();
    for s in envp {
        let mut b = s.as_bytes().to_vec();
        b.push(0);
        envp_ptrs.push(push(&b, &mut p)?);
    }
    let mut aux_pairs = Vec::new();
    for (key, value) in aux {
        let v = match value {
            Aux::Num(n) => *n,
            Aux::Bytes(b) => push(b, &mut p)?,
        };
        aux_pairs.push((*key, v));
    }

    let words = 1                       // argc
        + argv_ptrs.len() as u64 + 1    // argv, NULL
        + envp_ptrs.len() as u64 + 1    // envp, NULL
        + 2 * aux_pairs.len() as u64 + 2; // auxv, AT_NULL
    let sp = (p - words * 8) & !15;

    let mut w = sp;
    let word = |value: u64, w: &mut u64| -> LoadResult<()> {
        mem.write_bytes(*w, &value.to_le_bytes())
            .map_err(|e| e.to_string())?;
        *w += 8;
        Ok(())
    };
    word(argv_ptrs.len() as u64, &mut w)?;
    for a in &argv_ptrs {
        word(*a, &mut w)?;
    }
    word(0, &mut w)?;
    for e in &envp_ptrs {
        word(*e, &mut w)?;
    }
    word(0, &mut w)?;
    for (k, v) in &aux_pairs {
        word(*k, &mut w)?;
        word(*v, &mut w)?;
    }
    word(auxv::NULL, &mut w)?;
    word(0, &mut w)?;
    Ok(sp)
}

// ---------------------------------------------------------------------------
// The syscall stand-in
// ---------------------------------------------------------------------------
//
// Linux's `asm-generic` syscall numbers, which RISC-V, AArch64 and every
// architecture added since 2012 share. Facts about an ABI the *consumer* owns
// (§2.1) — nothing outside this module names them, and rsemu does not know
// they exist.

/// `include/uapi/asm-generic/unistd.h`.
mod nr {
    pub(super) const IOCTL: u64 = 29;
    pub(super) const FACCESSAT: u64 = 48;
    pub(super) const OPENAT: u64 = 56;
    pub(super) const CLOSE: u64 = 57;
    pub(super) const LSEEK: u64 = 62;
    pub(super) const READ: u64 = 63;
    pub(super) const WRITE: u64 = 64;
    pub(super) const WRITEV: u64 = 66;
    pub(super) const PPOLL: u64 = 73;
    pub(super) const READLINKAT: u64 = 78;
    pub(super) const NEWFSTATAT: u64 = 79;
    pub(super) const FSTAT: u64 = 80;
    pub(super) const EXIT: u64 = 93;
    pub(super) const EXIT_GROUP: u64 = 94;
    pub(super) const SET_TID_ADDRESS: u64 = 96;
    pub(super) const FUTEX: u64 = 98;
    pub(super) const SET_ROBUST_LIST: u64 = 99;
    pub(super) const NANOSLEEP: u64 = 101;
    pub(super) const CLOCK_GETRES: u64 = 114;
    pub(super) const CLOCK_GETTIME: u64 = 113;
    pub(super) const CLOCK_NANOSLEEP: u64 = 115;
    pub(super) const SCHED_YIELD: u64 = 124;
    pub(super) const SCHED_GETAFFINITY: u64 = 123;
    pub(super) const TKILL: u64 = 130;
    pub(super) const TGKILL: u64 = 131;
    pub(super) const SIGALTSTACK: u64 = 132;
    pub(super) const RT_SIGACTION: u64 = 134;
    pub(super) const RT_SIGPROCMASK: u64 = 135;
    pub(super) const UNAME: u64 = 160;
    pub(super) const GETPID: u64 = 172;
    pub(super) const GETPPID: u64 = 173;
    pub(super) const GETUID: u64 = 174;
    pub(super) const GETEUID: u64 = 175;
    pub(super) const GETGID: u64 = 176;
    pub(super) const GETEGID: u64 = 177;
    pub(super) const GETTID: u64 = 178;
    pub(super) const BRK: u64 = 214;
    pub(super) const MUNMAP: u64 = 215;
    pub(super) const MREMAP: u64 = 216;
    pub(super) const MMAP: u64 = 222;
    pub(super) const MPROTECT: u64 = 226;
    pub(super) const MADVISE: u64 = 233;
    pub(super) const PRLIMIT64: u64 = 261;
    pub(super) const CLONE: u64 = 220;
    pub(super) const GETRANDOM: u64 = 278;
    pub(super) const MEMBARRIER: u64 = 283;
    pub(super) const RSEQ: u64 = 293;
}

/// `CLONE_*`, from the same header.
///
/// Only the ones a threaded libc sets. `CLONE_VM | CLONE_THREAD |
/// CLONE_SIGHAND` together are what makes a `clone` a *thread* rather than a
/// process, and this stand-in insists on all three rather than guessing from
/// one.
mod cl {
    /// Share the address space.
    pub(super) const VM: u64 = 0x0000_0100;
    /// Share the signal handlers.
    pub(super) const SIGHAND: u64 = 0x0000_0800;
    /// Join the caller's thread group.
    pub(super) const THREAD: u64 = 0x0001_0000;
    /// The `tls` argument sets the child's thread pointer.
    pub(super) const SETTLS: u64 = 0x0008_0000;
    /// Write the child's id into `*ptid`.
    pub(super) const PARENT_SETTID: u64 = 0x0010_0000;
    /// Zero `*ctid` and wake it when the child exits — `pthread_join`.
    pub(super) const CHILD_CLEARTID: u64 = 0x0020_0000;
    /// Write the child's id into `*ctid`.
    pub(super) const CHILD_SETTID: u64 = 0x0100_0000;
}

/// `FUTEX_*` operations, from `include/uapi/linux/futex.h`.
mod fx {
    /// Sleep while the word still holds the expected value.
    pub(super) const WAIT: u64 = 0;
    /// Wake waiters.
    pub(super) const WAKE: u64 = 1;
    /// `WAIT` with a bitset and an *absolute* timeout.
    pub(super) const WAIT_BITSET: u64 = 9;
    /// `WAKE` with a bitset.
    pub(super) const WAKE_BITSET: u64 = 10;
    /// The futex is process private. Always true here, so it is masked off
    /// rather than checked: a level-3 process is alone in its address space.
    pub(super) const PRIVATE_FLAG: u64 = 128;
    /// The timeout is against the realtime clock. There is one clock here and
    /// it is virtual, so this is masked off too.
    pub(super) const CLOCK_REALTIME: u64 = 256;
}

/// The errno values this stand-in returns. Negated into `a0`, which is how
/// every `asm-generic` architecture reports a failure.
mod errno {
    /// No such file or directory. The answer to every path.
    pub(super) const NOENT: i64 = 2;
    /// Bad file descriptor.
    pub(super) const BADF: i64 = 9;
    /// Try again — what a `FUTEX_WAIT` whose word already changed gets.
    pub(super) const AGAIN: i64 = 11;
    /// Out of memory.
    pub(super) const NOMEM: i64 = 12;
    /// No such device — what a file-backed `mmap` gets.
    pub(super) const NODEV: i64 = 19;
    /// Invalid argument.
    pub(super) const INVAL: i64 = 22;
    /// Not a typewriter.
    pub(super) const NOTTY: i64 = 25;
    /// Function not implemented.
    pub(super) const NOSYS: i64 = 38;
    /// A timed wait reached its deadline.
    pub(super) const TIMEDOUT: i64 = 110;
}

/// `MAP_*` and `PROT_*`, `asm-generic` values.
mod mm {
    /// Put the mapping exactly where asked.
    pub(super) const MAP_FIXED: u64 = 0x10;
    /// The mapping is not backed by a file.
    pub(super) const MAP_ANONYMOUS: u64 = 0x20;
    /// Readable.
    pub(super) const PROT_READ: u64 = 1;
    /// Writable.
    pub(super) const PROT_WRITE: u64 = 2;
    /// Executable.
    pub(super) const PROT_EXEC: u64 = 4;
}

// ---------------------------------------------------------------------------
// The architectures
// ---------------------------------------------------------------------------
//
// §2.1's claim is that a syscall exit is *"a property of a core"* — not a
// property of RISC-V. The only way to find that out is to run the same
// consumer on a second one, so everything above this comment (the loader, the
// initial stack, the file descriptors, the policy, the journal) is written
// once, and what follows is the whole of what an architecture contributes:
// which register carries a syscall number, which carry its arguments, and what
// state a core has to be in before a `_start` will run on it.
//
// The measure of whether the seam is architecture-shaped is how short this
// section is. `Thread` has eight methods and none of them is a special case.

/// One guest thread, reached the way a syscall kernel reaches one.
///
/// Deliberately *not* rsemu's [`ExitingCore`]: that trait is the seam for
/// **running** a core and says nothing about registers, because rsemu does not
/// know which register carries a syscall number and refuses to guess. This
/// trait is the consumer's other half of the same seam — the ABI knowledge —
/// and it lives here because §2.1 puts it here.
trait Thread: Send + Sync + core::fmt::Debug {
    /// The core the scheduler runs.
    fn core(&self) -> Arc<dyn ExitingCore>;

    /// The syscall number the guest just asked for.
    fn nr(&self) -> u64;

    /// Argument `i`, zero based. Every `asm-generic` architecture passes six.
    fn arg(&self, i: u32) -> u64;

    /// Write what the call returns — negative for an errno.
    ///
    /// It lands in the same register [`arg`](Thread::arg)`(0)` reads, on both
    /// architectures and on every other `asm-generic` one, which is why a test
    /// can read a call's result back without a second accessor.
    fn set_ret(&self, value: i64);

    /// Put a call's number and arguments in place, the way an instruction
    /// would have.
    ///
    /// A core is a register file to a syscall kernel, and this is that and
    /// nothing else: no instruction is executed. It exists so the policy can
    /// be tested without assembling a guest for every question.
    fn set_call(&self, nr: u64, args: &[u64]);

    /// Point the guest at a stack.
    fn set_sp(&self, sp: u64);

    /// Set the thread pointer, which is what `clone`'s `tls` argument means
    /// and what a libc reads thread-local storage through.
    fn set_tls(&self, tls: u64);

    /// Where the guest will resume.
    fn pc(&self) -> u64;

    /// A second thread of the same process: this thread's registers exactly,
    /// on a new stack, with **zero** where the call's result goes.
    ///
    /// That zero is the whole of how a child of `clone` tells itself apart
    /// from its parent — every libc's `__clone` branches on it — and it is the
    /// same fact on both architectures, which is why it is stated here and not
    /// in either implementation.
    fn spawn(&self, sp: u64) -> Arc<dyn Thread>;
}

/// The four encodings the synthetic guests below are assembled from.
///
/// A struct of function pointers rather than a trait, because this is a
/// four-instruction assembler and a trait would be more ceremony than code.
/// Registers are named by **role** — `0..6` are the syscall's arguments — so
/// one program text assembles for both architectures and the ABI's register
/// mapping is the only thing that differs.
#[derive(Debug, Clone, Copy)]
struct Asm {
    /// Load a 64-bit constant into a role register, in a number of
    /// instructions that does not depend on the constant, so an address
    /// *inside* the code can be computed before the code exists.
    li: fn(role: u32, value: u64) -> Vec<u32>,
    /// Load the doubleword at `[base]` into role register `dst`.
    ld: fn(dst: u32, base: u32) -> u32,
    /// Store role register `src` to `[base]`.
    st: fn(src: u32, base: u32) -> u32,
    /// Reserve `[base]` and load it into `dst` — `lr.d`, `ldxr`.
    lr: fn(dst: u32, base: u32) -> u32,
    /// Store `src` to `[base]` **if the reservation still holds**, reporting
    /// in `status`: zero means it stored, on both architectures.
    sc: fn(status: u32, src: u32, base: u32) -> u32,
    /// Do nothing. What a program that needs to take up time is made of.
    nop: u32,
    /// The environment call.
    syscall: u32,
}

impl Asm {
    /// The role of the register carrying the syscall number.
    const NR: u32 = 6;
    /// The role of a register a synthetic guest may scribble in.
    const TMP: u32 = 7;
}

/// What an architecture contributes to a level-3 run.
#[derive(Debug, Clone, Copy)]
struct Arch {
    /// What this is called in a test name and a diagnostic.
    name: &'static str,
    /// `e_machine`, from the psABI.
    machine: u16,
    /// `AT_HWCAP`. Claimed honestly: a guest reading it is asking what this
    /// core has, and a bit set here that the core does not implement is a
    /// promise the first instruction breaks.
    hwcap: u64,
    /// `utsname.machine`.
    uname: &'static str,
    /// The suffix `scripts/fetch-testdata.sh usermode-guests` puts on a guest
    /// built for this architecture.
    ///
    /// Only a `std` build can read a built guest off a disk, and these two
    /// fields are facts *about* one, so they are gated with the tests that use
    /// them rather than sitting unread in a `no_std` build — which the
    /// per-feature sweep is entitled to complain about, and did.
    #[cfg(feature = "std")]
    suffix: &'static str,
    /// The first thread of a process: a core over `mem`, at `entry`, with
    /// `sp`, in whatever state that architecture calls unprivileged.
    start: fn(mem: &Arc<UserMemory>, entry: u64, sp: u64) -> Arc<dyn Thread>,
    /// The instructions the synthetic guests are made of.
    asm: Asm,
}

/// Every architecture this build can run a level-3 guest on.
///
/// A build with one CPU feature enabled runs one of them and every test below
/// still runs, which is the per-feature sweep's requirement and also the
/// honest shape: an architecture is a feature.
const ARCHES: &[&Arch] = &[
    #[cfg(feature = "cpu-riscv")]
    &riscv::ARCH,
    #[cfg(feature = "cpu-arm-a64")]
    &a64::ARCH,
];

/// RISC-V: `a7` carries the number, `a0`..`a5` the arguments, `a0` the result,
/// and `tp` is the thread pointer. Volume I's register-usage table plus the
/// `asm-generic` convention every architecture added since 2012 shares.
#[cfg(feature = "cpu-riscv")]
mod riscv {
    use alloc::sync::Arc;
    use alloc::vec;
    use alloc::vec::Vec;

    use crate::core::exec::{ExitMask, ExitingCore};
    use crate::core::space::AddressSpace;
    use crate::cpu::riscv::csr::Priv;
    use crate::cpu::riscv::{Config, Hart};

    use super::{Arch, Asm, Thread, UserMemory};

    /// The hardware register a role maps onto: `a0`..`a5` are `x10`..`x15`,
    /// the number is in `a7` (`x17`), and a synthetic guest's scratch is `t0`.
    const fn reg(role: u32) -> u32 {
        match role {
            Asm::NR => 17,
            Asm::TMP => 5,
            n => 10 + n,
        }
    }

    /// The `AT_HWCAP` bitmap for an RV64GC hart: one bit per single-letter
    /// extension, bit 0 being `A`, as `Documentation/arch/riscv/hwprobe.rst`
    /// describes `ELF_HWCAP`.
    const HWCAP: u64 = (1 << 0)   // A
        | (1 << 2)                 // C
        | (1 << 3)                 // D
        | (1 << 5)                 // F
        | (1 << 8)                 // I
        | (1 << 12); // M

    /// `addi rd, rs1, imm` — I-type, and with `rs1 = x0` the `li` a
    /// hand-assembled program is mostly made of.
    const fn addi(rd: u32, rs1: u32, imm: i32) -> u32 {
        ((imm as u32) << 20) | (rs1 << 15) | (rd << 7) | 0b001_0011
    }

    /// `lui rd, imm20` — U-type.
    const fn lui(rd: u32, imm20: u32) -> u32 {
        (imm20 << 12) | (rd << 7) | 0b011_0111
    }

    /// `ecall`.
    const ECALL: u32 = 0x0000_0073;

    /// The `lui`/`addi` pair a `li` expands to. The `addi` immediate is sign
    /// extended, so the upper half is pre-compensated when the lower half's
    /// top bit is set — a fact about the encoding, not a trick.
    fn li(role: u32, value: u64) -> Vec<u32> {
        let rd = reg(role);
        let lo = (value & 0xfff) as i32;
        let lo = if lo & 0x800 != 0 { lo - 0x1000 } else { lo };
        let hi = ((value as i64 - i64::from(lo)) >> 12) as u32 & 0xf_ffff;
        vec![lui(rd, hi), addi(rd, rd, lo)]
    }

    /// `ld rd, 0(rs1)` — I-type, opcode `0000011`, funct3 `011`.
    fn ld(dst: u32, base: u32) -> u32 {
        (reg(base) << 15) | (0b011 << 12) | (reg(dst) << 7) | 0b000_0011
    }

    /// `sd rs2, 0(rs1)` — S-type, opcode `0100011`, funct3 `011`. The
    /// immediate is zero, so both of its split halves are.
    fn st(src: u32, base: u32) -> u32 {
        (reg(src) << 20) | (reg(base) << 15) | (0b011 << 12) | 0b010_0011
    }

    /// `lr.d rd, (rs1)` — the A extension's R-type `AMO` format, funct5
    /// `00010`, with `aq` and `rl` clear. Volume I, "Load-Reserved/Store
    /// Conditional Instructions".
    fn lr(dst: u32, base: u32) -> u32 {
        (0b00010 << 27) | (reg(base) << 15) | (0b011 << 12) | (reg(dst) << 7) | 0b010_1111
    }

    /// `sc.d rd, rs2, (rs1)` — funct5 `00011`. `rd` is zero if the store
    /// happened and non-zero if the reservation had been broken.
    fn sc(status: u32, src: u32, base: u32) -> u32 {
        (0b00011 << 27)
            | (reg(src) << 20)
            | (reg(base) << 15)
            | (0b011 << 12)
            | (reg(status) << 7)
            | 0b010_1111
    }

    /// `nop`, which is `addi x0, x0, 0`.
    const NOP: u32 = 0x0000_0013;

    /// One RISC-V guest thread: a hart, and the map it shares with its
    /// siblings.
    #[derive(Debug)]
    struct Rv {
        hart: Arc<Hart>,
        space: Arc<AddressSpace>,
    }

    /// A hart in the state a level-3 guest runs in: unprivileged, over
    /// `space`, with `ecall` and a fault leaving the core instead of
    /// vectoring.
    fn hart(space: &Arc<AddressSpace>) -> Arc<Hart> {
        let hart = Arc::new(Hart::new(Config {
            pmp_count: 0,
            ..Config::rv64gc()
        }));
        hart.attach_space(Arc::clone(space));
        let mut csrs = hart.csrs();
        csrs.priv_mode = Priv::User;
        hart.set_csrs(csrs);
        hart.set_exit_mask(ExitMask::USER);
        hart
    }

    impl Thread for Rv {
        fn core(&self) -> Arc<dyn ExitingCore> {
            Arc::clone(&self.hart) as Arc<dyn ExitingCore>
        }

        fn nr(&self) -> u64 {
            self.hart.x(17)
        }

        fn arg(&self, i: u32) -> u64 {
            self.hart.x(10 + i)
        }

        fn set_ret(&self, value: i64) {
            self.hart.set_x(10, value as u64);
        }

        fn set_call(&self, nr: u64, args: &[u64]) {
            self.hart.set_x(17, nr);
            for (i, a) in args.iter().enumerate() {
                self.hart.set_x(10 + i as u32, *a);
            }
        }

        fn set_sp(&self, sp: u64) {
            self.hart.set_x(2, sp);
        }

        fn set_tls(&self, tls: u64) {
            self.hart.set_x(4, tls);
        }

        fn pc(&self) -> u64 {
            self.hart.pc()
        }

        fn spawn(&self, sp: u64) -> Arc<dyn Thread> {
            let child = hart(&self.space);
            // `x0` is hardwired zero, so the copy starts at one.
            for i in 1..32 {
                child.set_x(i, self.hart.x(i));
            }
            for i in 0..32 {
                child.set_f(i, self.hart.f(i));
            }
            child.set_csrs(self.hart.csrs());
            child.set_pc(self.hart.pc());
            child.set_x(2, sp);
            child.set_x(10, 0);
            Arc::new(Rv {
                hart: child,
                space: Arc::clone(&self.space),
            })
        }
    }

    fn start(mem: &Arc<UserMemory>, entry: u64, sp: u64) -> Arc<dyn Thread> {
        let space = Arc::clone(mem.space());
        let hart = hart(&space);
        hart.set_pc(entry);
        hart.set_x(2, sp);
        Arc::new(Rv { hart, space })
    }

    pub(super) const ARCH: Arch = Arch {
        name: "riscv64",
        // `EM_RISCV`.
        machine: 243,
        hwcap: HWCAP,
        uname: "riscv64",
        #[cfg(feature = "std")]
        suffix: "riscv64",
        start,
        asm: Asm {
            li,
            ld,
            st,
            lr,
            sc,
            nop: NOP,
            syscall: ECALL,
        },
    };
}

/// AArch64: `x8` carries the number, `x0`..`x5` the arguments, `x0` the
/// result, and `TPIDR_EL0` is the thread pointer. The AArch64 psABI plus the
/// same `asm-generic` convention — which is the point: two architectures, one
/// syscall table, and the only difference is five register numbers.
#[cfg(feature = "cpu-arm-a64")]
mod a64 {
    use alloc::sync::Arc;
    use alloc::vec::Vec;

    use crate::core::exec::{ExitMask, ExitingCore};
    use crate::core::space::AddressSpace;
    use crate::cpu::arm::a64::sysreg::El;
    use crate::cpu::arm::a64::{Config, Cpu};

    use super::{Arch, Asm, Thread, UserMemory};

    /// The hardware register a role maps onto. The arguments already *are*
    /// `x0`..`x5`; the number is `x8` and a synthetic guest's scratch is `x9`.
    const fn reg(role: u32) -> u32 {
        match role {
            Asm::NR => 8,
            Asm::TMP => 9,
            n => n,
        }
    }

    /// `AT_HWCAP`: `HWCAP_FP` and `HWCAP_ASIMD`, which is exactly what
    /// [`Config::cortex_a53`] implements.
    ///
    /// **`HWCAP_ATOMICS` is deliberately absent**, and that is not a detail:
    /// compiler-rt's out-of-line atomics read this bit to decide between
    /// `FEAT_LSE`'s `casal` and an `ldaxr`/`stlxr` loop, and a part without
    /// `FEAT_LSE` that claimed the bit would take an `UNDEFINED` on the first
    /// atomic a threaded guest executes.
    const HWCAP: u64 = (1 << 0) | (1 << 1);

    /// `svc #0`.
    const SVC: u32 = 0xd400_0001;

    /// A 64-bit constant, as `movz` plus three `movk` — always four
    /// instructions, whatever the constant, which is what lets a synthetic
    /// guest compute an address inside its own code.
    fn li(role: u32, value: u64) -> Vec<u32> {
        let rd = reg(role);
        (0..4u32)
            .map(|hw| {
                let imm = ((value >> (hw * 16)) & 0xffff) as u32;
                // `movz` for the first halfword (it zeroes the rest), `movk`
                // for the others (they keep it).
                let op = if hw == 0 { 0xd280_0000 } else { 0xf280_0000 };
                op | (hw << 21) | (imm << 5) | rd
            })
            .collect()
    }

    /// `ldr xt, [xn]` — the unsigned-offset form with a zero offset.
    fn ld(dst: u32, base: u32) -> u32 {
        0xf940_0000 | (reg(base) << 5) | reg(dst)
    }

    /// `str xt, [xn]`, likewise.
    fn st(src: u32, base: u32) -> u32 {
        0xf900_0000 | (reg(base) << 5) | reg(src)
    }

    /// `ldxr xt, [xn]` — DDI 0487's Load/store exclusive, size `11`, `Rs` and
    /// `Rt2` both `0b11111` because this is the non-pair, non-acquire form.
    fn lr(dst: u32, base: u32) -> u32 {
        0xc85f_7c00 | (reg(base) << 5) | reg(dst)
    }

    /// `stxr ws, xt, [xn]` — the same encoding with `L` clear and `Rs` naming
    /// the 32-bit register the result lands in: zero if it stored.
    fn sc(status: u32, src: u32, base: u32) -> u32 {
        0xc800_7c00 | (reg(status) << 16) | (reg(base) << 5) | reg(src)
    }

    /// `nop`.
    const NOP: u32 = 0xd503_201f;

    /// One AArch64 guest thread.
    #[derive(Debug)]
    struct A64 {
        cpu: Arc<Cpu>,
        space: Arc<AddressSpace>,
    }

    /// A core in the state a level-3 guest runs in.
    ///
    /// Three decisions, each of which a Linux kernel makes for a process it is
    /// about to enter:
    ///
    /// * **EL0**, because that is what unprivileged means here. It is also
    ///   what makes `SP` mean `SP_EL0` without a `PSTATE.SP` to get wrong.
    /// * **`CPACR_EL1.FPEN = 0b11`**, because the architecture resets it to
    ///   "trap", and a kernel that did not clear it would take an `UNDEFINED`
    ///   on the first `stp q0, q1` in a `memcpy`. Level 3 has no kernel to
    ///   take that trap, so the state is established up front.
    /// * **`SCTLR_EL1.M = 0`**: no MMU, so the map [`UserMemory`] builds is
    ///   the address space the guest sees. Level 3's whole memory model is
    ///   "there is no page table", and turning the MMU on would need one.
    fn cpu(space: &Arc<AddressSpace>) -> Arc<Cpu> {
        let cpu = Arc::new(Cpu::new(Config::cortex_a53()));
        cpu.attach_space(Arc::clone(space));
        let mut regs = cpu.sysregs();
        regs.el = El::El0;
        regs.cpacr = 0b11 << 20;
        regs.sctlr = 0;
        regs.daif = 0;
        cpu.set_sysregs(regs);
        cpu.set_exit_mask(ExitMask::USER);
        cpu
    }

    impl Thread for A64 {
        fn core(&self) -> Arc<dyn ExitingCore> {
            Arc::clone(&self.cpu) as Arc<dyn ExitingCore>
        }

        fn nr(&self) -> u64 {
            self.cpu.x(8)
        }

        fn arg(&self, i: u32) -> u64 {
            self.cpu.x(i)
        }

        fn set_ret(&self, value: i64) {
            self.cpu.set_x(0, value as u64);
        }

        fn set_call(&self, nr: u64, args: &[u64]) {
            self.cpu.set_x(8, nr);
            for (i, a) in args.iter().enumerate() {
                self.cpu.set_x(i as u32, *a);
            }
        }

        fn set_sp(&self, sp: u64) {
            self.cpu.set_sp(sp);
        }

        fn set_tls(&self, tls: u64) {
            let mut regs = self.cpu.sysregs();
            regs.tpidr_el0 = tls;
            self.cpu.set_sysregs(regs);
        }

        fn pc(&self) -> u64 {
            self.cpu.pc()
        }

        fn spawn(&self, sp: u64) -> Arc<dyn Thread> {
            let child = cpu(&self.space);
            // `x31` is the zero register or the stack pointer depending on the
            // instruction, and is not part of the general file.
            for i in 0..31 {
                child.set_x(i, self.cpu.x(i));
            }
            for i in 0..32 {
                child.set_v(i, self.cpu.v(i));
            }
            // The system registers carry `TPIDR_EL0`, `NZCV`, `FPCR` and
            // `SP_EL0`, so this must happen before the stack is set.
            child.set_sysregs(self.cpu.sysregs());
            child.set_pc(self.cpu.pc());
            child.set_sp(sp);
            child.set_x(0, 0);
            Arc::new(A64 {
                cpu: child,
                space: Arc::clone(&self.space),
            })
        }
    }

    fn start(mem: &Arc<UserMemory>, entry: u64, sp: u64) -> Arc<dyn Thread> {
        let space = Arc::clone(mem.space());
        let cpu = cpu(&space);
        cpu.set_pc(entry);
        cpu.set_sp(sp);
        Arc::new(A64 { cpu, space })
    }

    pub(super) const ARCH: Arch = Arch {
        name: "aarch64",
        // `EM_AARCH64`.
        machine: 183,
        hwcap: HWCAP,
        uname: "aarch64",
        #[cfg(feature = "std")]
        suffix: "aarch64",
        start,
        asm: Asm {
            li,
            ld,
            st,
            lr,
            sc,
            nop: NOP,
            syscall: SVC,
        },
    };
}

/// Where entropy comes from when the journal decides to actually ask.
///
/// A boxed closure rather than a call into `host/`, because the point of the
/// exercise is that **this** is the one place a level-3 run touches the
/// outside world, and a test can therefore replace it with something that
/// panics. See [`Kernel::replay_guard`].
type Entropy = alloc::boxed::Box<dyn FnMut(usize) -> Vec<u8> + Send>;

/// A counter-based entropy source, for the runs that are not measuring
/// entropy. Deterministic on purpose: a test that wants to prove the journal
/// is the only door uses [`Kernel::replay_guard`] instead.
fn counting_entropy() -> Entropy {
    let mut n: u8 = 0;
    alloc::boxed::Box::new(move |len| {
        (0..len)
            .map(|_| {
                n = n.wrapping_add(0x9d);
                n
            })
            .collect()
    })
}

/// An open descriptor above 2: a fixed byte string and a position in it.
///
/// The contents are produced *once*, at `openat`, from the guest's own state.
/// There is no host file behind one and no way to make there be.
#[derive(Debug)]
struct Vfile {
    bytes: Vec<u8>,
    pos: u64,
}

/// The one path a level-3 guest may open, and the reason it is not an
/// exception to the policy: it is a description of the guest's **own** address
/// space, produced from [`UserMemory::mappings`], and no host is consulted to
/// answer it.
///
/// `usermode::mem`'s own documentation names `/proc/self/maps` as the first of
/// the three reasons the mapping bookkeeping is a list rather than a bitmap, so
/// serving it here is what that list is *for*. The static `musl` guest below
/// never asks — it finds its stack from the auxiliary vector — but a threaded
/// one does: `pthread_getattr_np` locates the main thread's stack by looking
/// for the `[stack]` line, and a caller that is refused does not fail cleanly,
/// it proceeds with a stack base it made up.
///
/// The line this draws is the one worth stating precisely: **a level-3 guest
/// may be told about itself and may not be told about the host.** That
/// distinction is checkable — everything on the answer comes from `mem` — in a
/// way that "which paths are safe" never is.
const PROC_SELF_MAPS: &str = "/proc/self/maps";

/// Render the guest's map the way `fs/proc/task_mmu.c` prints it: sixteen hex
/// digits either side of a dash, `rwxp`, then a file offset, device, inode and
/// name that are all zero here because no mapping is file backed.
fn proc_self_maps(mem: &UserMemory) -> Vec<u8> {
    let mut out = String::new();
    for m in mem.mappings() {
        let name = if m.name.starts_with('[') { &m.name } else { "" };
        out.push_str(&format!(
            "{:016x}-{:016x} {}p 00000000 00:00 0 {}\n",
            m.base,
            m.base + m.len,
            m.prot,
            name
        ));
    }
    out.into_bytes()
}

/// One live guest thread, as the *consumer* models it.
///
/// rsemu's [`ThreadSet`] deliberately knows only that a thread is an
/// [`ExitingCore`] and when it may run; a thread id, a `clear_child_tid` word
/// and the ABI knowledge are all on this side of §2.1's line. This struct is
/// the whole of what that turned out to be.
#[derive(Debug)]
struct Task {
    thread: Arc<dyn Thread>,
    /// `set_tid_address(2)`, and `clone`'s `CLONE_CHILD_CLEARTID`: the word to
    /// zero and the futex to wake when this thread exits. Zero for "nobody is
    /// waiting", which is also its initial value.
    ///
    /// This is how `pthread_join` works and there is no other mechanism: the
    /// joiner waits on the word, and the exiting thread clears it and wakes
    /// whoever is there. Forget the wake and a threaded guest hangs at exit
    /// with every thread blocked — which is exactly what happened here first.
    clear_child_tid: u64,
    /// The alternate signal stack this thread last installed, if any.
    ///
    /// **Per thread, and finding out that it had to be is the whole reason
    /// the native trace gets run.** Held process-wide it is not obviously
    /// wrong — nothing crashes, the output is byte-identical — but the second
    /// thread's query then reads back the *first* thread's stack, concludes
    /// one is already installed, and skips installing its own. Every thread
    /// but the first silently loses its stack-overflow handler, and the trace
    /// is short by two `sigaltstack`s, one `mmap`, one `mprotect` and one
    /// `munmap` per thread. That count is how it was noticed: 130 calls here
    /// against the same program's 148 under `strace`, with identical output.
    ///
    /// It is the same defect the single-threaded proof already found once, in
    /// the same place, for a different reason — which is an argument for the
    /// method rather than about `sigaltstack`.
    altstack: Option<(u64, u64)>,
    /// The blocked-signal mask.
    ///
    /// **Per thread**, where the dispositions are per process — `clone` with
    /// `CLONE_SIGHAND` shares the handlers and gives the child a *copy* of its
    /// parent's mask — and that asymmetry is the reason the two live in
    /// different structs here rather than one place labelled "signals".
    sigmask: u64,
}

/// `sizeof(struct sigaction)` as the kernel sees it on an `asm-generic`
/// architecture with no `SA_RESTORER`: a handler pointer, a flags word and a
/// 64-bit signal mask. RISC-V and AArch64 are both of those.
const K_SIGACTION: usize = 24;

/// One thread parked on a futex word.
#[derive(Debug, Clone, Copy)]
struct Waiter {
    /// Who is waiting.
    thread: ThreadId,
    /// Where in the trace that thread's `FUTEX_WAIT` was recorded, so a wake
    /// can correct it from the timeout it was preloaded with.
    at: usize,
}

/// The consumer's half: file descriptors, errno, a heap, threads, and a
/// syscall table.
///
/// Everything §2.1 says is nixvm's, written out here in the smallest form that
/// runs a real binary, so rsemu's half has something to be proven against.
struct Kernel {
    /// Which architecture this process is running on. Reaches exactly two
    /// answers — `uname` and `AT_HWCAP` — which is itself the finding.
    arch: &'static Arch,
    mem: Arc<UserMemory>,
    clock: Arc<GuestClock>,
    journal: Arc<Journal>,
    entropy: Entropy,
    /// rsemu's scheduler. `clone` inserts into it, `futex` blocks and wakes in
    /// it, and `exit` removes from it.
    threads: Arc<ThreadSet>,
    /// Every live thread, by the scheduler's id. The guest's thread id *is*
    /// that id: there is one process, so there is nothing to translate.
    tasks: BTreeMap<ThreadId, Task>,
    /// Whose call is being serviced.
    current: ThreadId,
    /// The signal dispositions, as the guest supplied them: process-wide,
    /// because a level-3 process shares them across every thread.
    ///
    /// Nothing here ever *delivers* a signal — there is nothing in a level-3
    /// run to raise one — so this is a register and no more. It still has to
    /// be a real one: see [`Kernel::rt_sigaction`].
    sigactions: BTreeMap<u32, [u8; K_SIGACTION]>,
    /// Threads waiting on a futex word, keyed by address and in arrival order.
    ///
    /// A `BTreeMap` of `Vec`s rather than a hash of sets, because a wake of
    /// *one* waiter has to pick the same one every run or the whole journal
    /// stops replaying (CLAUDE.md, "Determinism").
    waiters: BTreeMap<u64, Vec<Waiter>>,
    /// Everything the guest wrote to fd 1.
    stdout: Vec<u8>,
    /// Everything the guest wrote to fd 2.
    stderr: Vec<u8>,
    brk_base: u64,
    brk: u64,
    /// Descriptors above 2, each a snapshot of something the *guest* can
    /// legitimately be told about itself. Never a host file — see the module
    /// documentation.
    files: Vec<Option<Vfile>>,
    /// `(number, return value)` for every call serviced, in order. The
    /// discovery tool: *implement from a trace, not from a list*.
    trace: Vec<(u64, i64)>,
    /// Numbers this stand-in refused, deduplicated, in first-asked order.
    refused: Vec<u64>,
    /// How many threads this process ever had, which is the cheapest evidence
    /// that a threaded guest actually threaded.
    spawned: u64,
}

impl Kernel {
    /// A kernel over `mem` with one thread, and a heap starting at `brk_base`.
    fn new(
        arch: &'static Arch,
        mem: Arc<UserMemory>,
        clock: Arc<GuestClock>,
        journal: Arc<Journal>,
        entropy: Entropy,
        brk_base: u64,
        main: Arc<dyn Thread>,
    ) -> Kernel {
        let threads = Arc::new(ThreadSet::new(Arc::clone(&clock)));
        let id = threads.insert(main.core());
        let mut tasks = BTreeMap::new();
        tasks.insert(
            id,
            Task {
                thread: main,
                clear_child_tid: 0,
                altstack: None,
                sigmask: 0,
            },
        );
        Kernel {
            arch,
            mem,
            clock,
            journal,
            entropy,
            threads,
            tasks,
            current: id,
            sigactions: BTreeMap::new(),
            waiters: BTreeMap::new(),
            stdout: Vec::new(),
            stderr: Vec::new(),
            brk_base,
            brk: brk_base,
            files: Vec::new(),
            trace: Vec::new(),
            refused: Vec::new(),
            spawned: 1,
        }
    }

    /// An entropy source that panics if it is ever called.
    ///
    /// What a [`JournalMode::Replay`] run is handed, so *"the closure never
    /// runs"* is a checked property of the run rather than a claim in a doc
    /// comment.
    fn replay_guard() -> Entropy {
        alloc::boxed::Box::new(|_| {
            panic!("a replayed run reached the host for entropy — the journal is not the only door")
        })
    }

    /// Ask for `len` bytes of entropy through the journal, at the current
    /// virtual instant, tagged with what wanted them.
    fn ask_entropy(&mut self, tag: Tag, len: usize) -> Vec<u8> {
        let at = self.clock.now();
        let source = &mut self.entropy;
        self.journal
            .ask(at, tag, || {
                let bytes = source(len);
                Answer::with_bytes(bytes.len() as u64, bytes)
            })
            .expect("the journal answered")
            .bytes
    }

    /// The sixteen bytes `AT_RANDOM` points at.
    ///
    /// Asked before the guest has executed an instruction, so at virtual
    /// instant zero, and tagged with a number no syscall has: the auxiliary
    /// vector is not a syscall, and a replay that found one where it expected
    /// the other should say so rather than hand over the wrong bytes.
    fn at_random(&mut self) -> Vec<u8> {
        self.ask_entropy(Tag(u32::MAX), 16)
    }

    /// Service the call the thread `id` has just exited on. `Some(status)`
    /// when the **process** is over — which is not the same as a thread
    /// exiting, and telling those apart is most of what `exit` versus
    /// `exit_group` means.
    fn service(&mut self, id: ThreadId) -> Option<i32> {
        self.current = id;
        let t = Arc::clone(&self.tasks.get(&id).expect("the thread that ran").thread);
        let a = |i: u32| t.arg(i);
        let nr = t.nr();
        let ret = match nr {
            // The whole process stops, however many threads are still in it.
            nr::EXIT_GROUP => {
                self.trace.push((nr, 0));
                return Some(a(0) as i32);
            }
            // One thread stops. The process outlives it unless it was the
            // last, and `main` returning is exactly that case.
            nr::EXIT => {
                self.trace.push((nr, 0));
                return self.exit_thread(id, a(0) as i32);
            }
            nr::WRITE => self.write(a(0), a(1), a(2)),
            nr::WRITEV => self.writev(a(0), a(1), a(2)),
            // fd 0 is at end of file, always: there is no host to read from.
            nr::READ if a(0) == 0 => 0,
            nr::READ => self.read(a(0), a(1), a(2)),
            nr::CLOSE if a(0) <= 2 => 0,
            nr::CLOSE => self.close(a(0)),
            nr::LSEEK => self.lseek(a(0), a(1) as i64, a(2)),
            nr::OPENAT => self.openat(a(1)),
            nr::BRK => self.set_brk(a(0)),
            nr::MMAP => self.mmap(a(0), a(1), a(2), a(3), a(4) as i64),
            nr::MUNMAP => match self.mem.unmap(page_down(a(0)), page_up(a(1))) {
                Ok(()) => 0,
                Err(_) => -errno::INVAL,
            },
            nr::MPROTECT => {
                let (base, end) = (page_down(a(0)), page_up(a(0) + a(1)));
                match self.mem.protect(base, end - base, prot_of(a(2))) {
                    Ok(()) => 0,
                    Err(_) => -errno::INVAL,
                }
            }
            // A `mremap` that cannot move is a `realloc` that falls back to
            // allocate, copy and free — correct, and slower.
            nr::MREMAP => -errno::NOMEM,
            nr::MADVISE => 0,
            nr::SET_TID_ADDRESS => {
                self.task_mut().clear_child_tid = a(0);
                i64::from(id.0)
            }
            nr::SET_ROBUST_LIST => 0,
            nr::CLONE => self.clone_thread(&t, a(0), a(1), a(2), a(3), a(4)),
            nr::FUTEX => self.futex(a(0), a(1), a(2), a(3)),
            // The quantum already ended somewhere; giving up the rest of it is
            // this scheduler's `run_next` coming round again.
            nr::SCHED_YIELD => 0,
            nr::NANOSLEEP => self.sleep_until(a(0), false),
            nr::CLOCK_NANOSLEEP => self.sleep_until(a(2), a(1) & 1 != 0),
            nr::CLOCK_GETTIME => self.clock_gettime(a(1)),
            nr::CLOCK_GETRES => self.clock_getres(a(1)),
            nr::GETRANDOM => self.getrandom(a(0), a(1)),
            nr::UNAME => self.uname(a(0)),
            nr::FSTAT => self.fstat(a(0), a(1)),
            // Every path answers the same way, and none of them is looked at.
            // See the module documentation: the filesystem policy is "there
            // isn't one", decided before `openat` was written.
            nr::FACCESSAT | nr::READLINKAT | nr::NEWFSTATAT => -errno::NOENT,
            nr::IOCTL => -errno::NOTTY,
            // The standard descriptors exist and nothing is ready. The caller
            // supplied a zeroed `revents` array and it stays that way.
            nr::PPOLL => 0,
            nr::RT_SIGACTION => self.rt_sigaction(a(0), a(1), a(2)),
            nr::RT_SIGPROCMASK => self.rt_sigprocmask(a(0), a(1), a(2)),
            nr::SIGALTSTACK => self.sigaltstack(a(0), a(1)),
            nr::TGKILL | nr::TKILL => 0,
            nr::PRLIMIT64 => 0,
            nr::SCHED_GETAFFINITY => self.sched_getaffinity(a(1), a(2)),
            nr::MEMBARRIER => self.membarrier(),
            nr::GETPID => 1,
            // The guest's thread id **is** the scheduler's, because there is
            // one process and therefore nothing to translate. That is only
            // honest while ids are never reused, which `ThreadSet` promises.
            nr::GETTID => i64::from(id.0),
            nr::GETPPID => 0,
            nr::GETUID | nr::GETEUID | nr::GETGID | nr::GETEGID => 0,
            nr::RSEQ => -errno::NOSYS,
            other => {
                if !self.refused.contains(&other) {
                    self.refused.push(other);
                }
                -errno::NOSYS
            }
        };
        self.trace.push((nr, ret));
        t.set_ret(ret);
        None
    }

    /// The current thread's bookkeeping.
    fn task_mut(&mut self) -> &mut Task {
        let id = self.current;
        self.tasks.get_mut(&id).expect("the thread that ran")
    }

    /// Ask for `nr` with `args` on behalf of the current thread, and report
    /// what it was told.
    ///
    /// The test-side spelling of an `ecall`, with no guest under it.
    fn ask(&mut self, nr: u64, args: &[u64]) -> i64 {
        let id = self.current;
        let t = Arc::clone(&self.tasks[&id].thread);
        t.set_call(nr, args);
        assert!(self.service(id).is_none(), "call {nr} ended the process");
        t.arg(0) as i64
    }

    /// The process's first thread, which is the only one that exists before
    /// the guest has run an instruction.
    fn main_thread(&self) -> Arc<dyn Thread> {
        Arc::clone(
            &self
                .tasks
                .values()
                .next()
                .expect("a process has a thread")
                .thread,
        )
    }

    /// Everything worth knowing when a run stops for the wrong reason.
    ///
    /// A trace tail and the state of every thread. Both earn their place: the
    /// trace is how this module was written at all, and with more than one
    /// thread *"which one is stuck and on what"* is the first question and was
    /// unanswerable without it.
    fn diagnose(&self, what: &str) -> String {
        let mut out = format!(
            "{what}, on {} after {} syscall(s) and {} tick(s)",
            self.arch.name,
            self.trace.len(),
            self.clock.ticks()
        );
        for (id, task) in &self.tasks {
            let waiting = self
                .waiters
                .iter()
                .find(|(_, q)| q.iter().any(|w| w.thread == *id))
                .map(|(addr, _)| *addr);
            out.push_str(&format!(
                "\n  thread {} at pc {:#x}: {:?}{}",
                id.0,
                task.thread.pc(),
                self.threads.state(*id).unwrap_or(ThreadState::Runnable),
                match waiting {
                    Some(addr) => format!(" on the futex at {addr:#x}"),
                    None => String::new(),
                }
            ));
        }
        out.push_str(&format!(
            "\n  last calls: {:?}",
            &self.trace[self.trace.len().saturating_sub(12)..]
        ));
        out
    }

    /// A thread exits.
    ///
    /// Returns the process's status only if this was the last thread, which is
    /// what makes `main` returning end the run without anything special being
    /// said about the main thread. It is not special: it is the one that has
    /// nobody left to outlive it.
    fn exit_thread(&mut self, id: ThreadId, status: i32) -> Option<i32> {
        // `CLONE_CHILD_CLEARTID` / `set_tid_address`: zero the word and wake
        // whoever is waiting on it. **This is the whole of `pthread_join`** —
        // the joiner is blocked on a futex over exactly this word — and it has
        // to happen before the thread leaves the set, or the joiner waits for
        // a thread that is already gone.
        let word = self.tasks.get(&id).map_or(0, |t| t.clear_child_tid);
        if word != 0 {
            let _ = self.mem.write_bytes(word, &0u32.to_le_bytes());
            self.wake(word, u64::from(u32::MAX));
        }
        self.tasks.remove(&id);
        self.threads.remove(id);
        self.forget_waiter(id);
        if self.tasks.is_empty() {
            Some(status)
        } else {
            None
        }
    }

    /// `clone(flags, stack, ptid, tls, ctid)` — a new thread of this process.
    ///
    /// **The argument order is the one an architecture selecting
    /// `CONFIG_CLONE_BACKWARDS` gets**, which RISC-V, AArch64 and x86-64 all
    /// do. It is not the order `clone(2)` documents for the libc wrapper, and
    /// swapping `tls` with `ctid` produces a guest that starts its first
    /// thread and faults inside its first thread-local access — a symptom
    /// several removes from the cause. The host `strace` of the same program
    /// prints the fields by name, which is how it was settled rather than
    /// guessed.
    fn clone_thread(
        &mut self,
        parent: &Arc<dyn Thread>,
        flags: u64,
        stack: u64,
        ptid: u64,
        tls: u64,
        ctid: u64,
    ) -> i64 {
        const THREAD: u64 = cl::VM | cl::THREAD | cl::SIGHAND;
        if flags & THREAD != THREAD || stack == 0 {
            // A `fork` makes a *process*, and this stand-in has one. rsemu's
            // half is already there — `UserMemory::duplicate` shares every
            // range copy-on-write and `resolve_write_fault` breaks it — so
            // what is missing is the process model, which §2.1 puts on this
            // side of the line and which nothing here has needed.
            return -errno::INVAL;
        }
        let inherited = self.tasks[&self.current].sigmask;
        let child = parent.spawn(stack);
        if flags & cl::SETTLS != 0 {
            child.set_tls(tls);
        }
        let id = self.threads.insert(child.core());
        let tid = id.0;
        if flags & cl::PARENT_SETTID != 0 && self.mem.write_bytes(ptid, &tid.to_le_bytes()).is_err()
        {
            self.threads.remove(id);
            return -errno::INVAL;
        }
        if flags & cl::CHILD_SETTID != 0 && self.mem.write_bytes(ctid, &tid.to_le_bytes()).is_err()
        {
            self.threads.remove(id);
            return -errno::INVAL;
        }
        self.tasks.insert(
            id,
            Task {
                thread: child,
                clear_child_tid: if flags & cl::CHILD_CLEARTID != 0 {
                    ctid
                } else {
                    0
                },
                // A new thread has no alternate signal stack. Linux does not
                // inherit one across `clone`, and a libc that was told it had
                // one would not install its own.
                altstack: None,
                // The mask, by contrast, *is* inherited — which is what
                // `pthread_create` relies on when it blocks everything around
                // the clone and the child unblocks in its own start routine.
                sigmask: inherited,
            },
        );
        self.spawned += 1;
        i64::from(tid)
    }

    /// `futex(uaddr, op, val, timeout)`, the two operations a threaded libc
    /// actually uses.
    ///
    /// Every futex here is private, because a level-3 process is alone in its
    /// address space, so `FUTEX_PRIVATE_FLAG` is masked off rather than
    /// checked. `FUTEX_CLOCK_REALTIME` likewise: there is one clock and it is
    /// virtual.
    fn futex(&mut self, uaddr: u64, op: u64, val: u64, timeout: u64) -> i64 {
        match op & !(fx::PRIVATE_FLAG | fx::CLOCK_REALTIME) {
            fx::WAIT => self.futex_wait(uaddr, val as u32, timeout, false),
            fx::WAIT_BITSET => self.futex_wait(uaddr, val as u32, timeout, true),
            fx::WAKE | fx::WAKE_BITSET => self.wake(uaddr, val),
            other => {
                let refused = 0x1_0000 | other;
                if !self.refused.contains(&refused) {
                    self.refused.push(refused);
                }
                -errno::NOSYS
            }
        }
    }

    /// `FUTEX_WAIT`: sleep while the word still holds `val`.
    ///
    /// The compare and the block are one step here because nothing else runs
    /// in between — a level-3 run has one thread executing at a time — which
    /// is the property that makes the classic lost-wakeup race impossible in
    /// this scheduler rather than merely unlikely.
    fn futex_wait(&mut self, uaddr: u64, val: u32, timeout: u64, absolute: bool) -> i64 {
        let mut seen = [0u8; 4];
        if self.mem.read_bytes(uaddr, &mut seen).is_err() {
            return -errno::INVAL;
        }
        if u32::from_le_bytes(seen) != val {
            return -errno::AGAIN;
        }
        let deadline = if timeout == 0 {
            None
        } else {
            match self.timespec(timeout) {
                Some(nanos) if absolute => Some(GlobalTime::from_nanos(nanos)),
                Some(nanos) => Some(
                    self.clock
                        .now()
                        .saturating_add(GlobalTime::from_nanos(nanos)),
                ),
                None => return -errno::INVAL,
            }
        };
        let id = self.current;
        self.waiters.entry(uaddr).or_default().push(Waiter {
            thread: id,
            at: self.trace.len(),
        });
        self.threads.block(id, deadline);
        // Preload the answer a wait that is *never* woken gets. A [`wake`]
        // overwrites both this register and the trace entry; a deadline that
        // fires leaves them, and the thread resumes with `-ETIMEDOUT` already
        // in place. That is what lets the consumer stay out of rsemu's way:
        // `ThreadSet` does not have to say *why* a thread became runnable,
        // because the two answers were written at the two moments they became
        // true.
        if deadline.is_some() {
            -errno::TIMEDOUT
        } else {
            0
        }
    }

    /// `FUTEX_WAKE`: make up to `count` waiters on `uaddr` runnable.
    ///
    /// In arrival order, which is a choice and has to be one: waking "some"
    /// waiter is what the kernel promises, and a consumer that picked by hash
    /// order would have a replay that diverges the first time two threads
    /// wait on the same lock.
    fn wake(&mut self, uaddr: u64, count: u64) -> i64 {
        let mut woken = Vec::new();
        if let Some(queue) = self.waiters.get_mut(&uaddr) {
            let take = (count as usize).min(queue.len());
            woken.extend(queue.drain(..take));
            if queue.is_empty() {
                self.waiters.remove(&uaddr);
            }
        }
        for w in &woken {
            self.threads.wake(w.thread);
            if let Some(task) = self.tasks.get(&w.thread) {
                task.thread.set_ret(0);
            }
            // Correct the trace as well as the register. A trace that says a
            // wait timed out when it was woken is worse than no trace: it is
            // the artefact the whole method depends on. (There is no entry
            // when a test drove `futex` directly rather than through a
            // guest's `ecall`, which is why this is a `get_mut`.)
            if let Some(slot) = self.trace.get_mut(w.at) {
                *slot = (nr::FUTEX, 0);
            }
        }
        woken.len() as i64
    }

    /// Take a thread off every wait queue, because it has exited.
    fn forget_waiter(&mut self, id: ThreadId) {
        self.waiters.retain(|_, queue| {
            queue.retain(|w| w.thread != id);
            !queue.is_empty()
        });
    }

    /// `nanosleep` / `clock_nanosleep`: block until a virtual instant.
    ///
    /// No host sleeps: [`ThreadSet`] jumps virtual time to the earliest
    /// deadline when nothing is runnable, so a guest that sleeps a second
    /// costs nothing and lands on the same instruction every run.
    fn sleep_until(&mut self, ts: u64, absolute: bool) -> i64 {
        let Some(nanos) = self.timespec(ts) else {
            return -errno::INVAL;
        };
        let now = self.clock.now();
        let at = if absolute {
            GlobalTime::from_nanos(nanos)
        } else {
            now.saturating_add(GlobalTime::from_nanos(nanos))
        };
        if at <= now {
            return 0;
        }
        self.threads.block(self.current, Some(at));
        0
    }

    /// Read a `struct timespec` and report it in nanoseconds.
    fn timespec(&self, at: u64) -> Option<u64> {
        let mut buf = [0u8; 16];
        self.mem.read_bytes(at, &mut buf).ok()?;
        let secs = u64::from_le_bytes(buf[..8].try_into().unwrap());
        let nanos = u64::from_le_bytes(buf[8..].try_into().unwrap());
        if nanos >= 1_000_000_000 {
            return None;
        }
        secs.checked_mul(1_000_000_000)?.checked_add(nanos)
    }

    /// `membarrier(MEMBARRIER_CMD_QUERY, ...)` and every other command.
    ///
    /// Zero is the honest answer to the query — this build supports no
    /// command — and it is also the honest answer to an actual barrier: one
    /// guest thread executes at a time, so every store a thread made is
    /// visible to every other before either of them runs again.
    fn membarrier(&mut self) -> i64 {
        0
    }

    /// Read a NUL-terminated path out of guest memory, bounded.
    ///
    /// Bounded because the guest chose the pointer: a level-3 kernel that
    /// scans until it finds a zero is a level-3 kernel a guest can hang.
    fn path_at(&self, mut at: u64) -> Option<String> {
        let mut out = Vec::new();
        while out.len() < 4096 {
            let mut b = [0u8; 1];
            self.mem.read_bytes(at, &mut b).ok()?;
            if b[0] == 0 {
                return String::from_utf8(out).ok();
            }
            out.push(b[0]);
            at += 1;
        }
        None
    }

    fn openat(&mut self, path: u64) -> i64 {
        // The policy: the path is compared against exactly one name, and that
        // name is not a host file. Everything else is an empty namespace.
        if self.path_at(path).as_deref() != Some(PROC_SELF_MAPS) {
            return -errno::NOENT;
        }
        let file = Vfile {
            bytes: proc_self_maps(&self.mem),
            pos: 0,
        };
        let slot = match self.files.iter().position(Option::is_none) {
            Some(i) => i,
            None => {
                self.files.push(None);
                self.files.len() - 1
            }
        };
        self.files[slot] = Some(file);
        slot as i64 + 3
    }

    fn file(&mut self, fd: u64) -> Option<&mut Vfile> {
        let index = fd.checked_sub(3)? as usize;
        self.files.get_mut(index)?.as_mut()
    }

    fn read(&mut self, fd: u64, buf: u64, len: u64) -> i64 {
        let Some(file) = self.file(fd) else {
            return -errno::BADF;
        };
        let start = file.pos.min(file.bytes.len() as u64);
        let n = len.min(file.bytes.len() as u64 - start);
        let bytes = file.bytes[start as usize..(start + n) as usize].to_vec();
        match self.mem.write_bytes(buf, &bytes) {
            Ok(()) => {
                self.file(fd).expect("still open").pos = start + n;
                n as i64
            }
            Err(_) => -errno::INVAL,
        }
    }

    fn lseek(&mut self, fd: u64, offset: i64, whence: u64) -> i64 {
        let Some(file) = self.file(fd) else {
            return -errno::BADF;
        };
        let end = file.bytes.len() as i64;
        let base = match whence {
            0 => 0,
            1 => file.pos as i64,
            2 => end,
            _ => return -errno::INVAL,
        };
        let Some(pos) = base.checked_add(offset).filter(|p| *p >= 0) else {
            return -errno::INVAL;
        };
        file.pos = pos as u64;
        pos
    }

    fn close(&mut self, fd: u64) -> i64 {
        match fd
            .checked_sub(3)
            .and_then(|i| self.files.get_mut(i as usize))
        {
            Some(slot @ Some(_)) => {
                *slot = None;
                0
            }
            _ => -errno::BADF,
        }
    }

    fn write(&mut self, fd: u64, buf: u64, len: u64) -> i64 {
        if !matches!(fd, 1 | 2) {
            return -errno::BADF;
        }
        let mut bytes = vec![0u8; len as usize];
        if self.mem.read_bytes(buf, &mut bytes).is_err() {
            return -errno::INVAL;
        }
        if fd == 1 {
            self.stdout.extend_from_slice(&bytes);
        } else {
            self.stderr.extend_from_slice(&bytes);
        }
        len as i64
    }

    fn writev(&mut self, fd: u64, iov: u64, count: u64) -> i64 {
        if !matches!(fd, 1 | 2) {
            return -errno::BADF;
        }
        let mut total = 0i64;
        for i in 0..count {
            let base = iov + i * 16;
            let (Ok(ptr), Ok(len)) = (self.mem.read_u64(base), self.mem.read_u64(base + 8)) else {
                return -errno::INVAL;
            };
            if len == 0 {
                continue;
            }
            let n = self.write(fd, ptr, len);
            if n < 0 {
                return if total > 0 { total } else { n };
            }
            total += n;
        }
        total
    }

    fn set_brk(&mut self, want: u64) -> i64 {
        if want < self.brk_base {
            return self.brk as i64;
        }
        let (have, need) = (page_up(self.brk), page_up(want));
        if need > have
            && self
                .mem
                .map_at(have, need - have, Prot::RW, "[heap]")
                .is_err()
        {
            return self.brk as i64;
        }
        if need < have {
            let _ = self.mem.unmap(need, have - need);
        }
        self.brk = want;
        want as i64
    }

    fn mmap(&mut self, addr: u64, len: u64, prot: u64, flags: u64, fd: i64) -> i64 {
        // The policy, enforced here rather than in `openat`: a level-3 guest
        // cannot obtain a descriptor for a host file, so it cannot map one.
        if flags & mm::MAP_ANONYMOUS == 0 || fd >= 0 {
            return -errno::NODEV;
        }
        let len = page_up(len);
        if len == 0 {
            return -errno::INVAL;
        }
        let prot = prot_of(prot);
        if flags & mm::MAP_FIXED != 0 && addr != 0 {
            let base = page_down(addr);
            return match self.mem.map_at(base, len, prot, "[anon]") {
                Ok(()) => base as i64,
                Err(_) => -errno::INVAL,
            };
        }
        match self.mem.map(len, prot, "[anon]") {
            Ok(base) => base as i64,
            Err(_) => -errno::NOMEM,
        }
    }

    fn clock_gettime(&mut self, ts: u64) -> i64 {
        // Virtual time, and therefore *not* a journal question — the whole
        // reason `usermode::clock` exists (`ROADMAP.md` §0, phase 5b).
        let nanos = self.clock.nanos();
        let mut buf = [0u8; 16];
        buf[..8].copy_from_slice(&(nanos / 1_000_000_000).to_le_bytes());
        buf[8..].copy_from_slice(&(nanos % 1_000_000_000).to_le_bytes());
        match self.mem.write_bytes(ts, &buf) {
            Ok(()) => 0,
            Err(_) => -errno::INVAL,
        }
    }

    /// `clock_getres`: one nanosecond, which is what [`GuestClock`]'s default
    /// rate actually resolves and therefore not a rounded-up claim.
    fn clock_getres(&mut self, ts: u64) -> i64 {
        let mut buf = [0u8; 16];
        buf[8..].copy_from_slice(&1u64.to_le_bytes());
        match self.mem.write_bytes(ts, &buf) {
            Ok(()) => 0,
            Err(_) => -errno::INVAL,
        }
    }

    fn getrandom(&mut self, buf: u64, len: u64) -> i64 {
        let bytes = self.ask_entropy(Tag(nr::GETRANDOM as u32), len as usize);
        match self.mem.write_bytes(buf, &bytes) {
            Ok(()) => bytes.len() as i64,
            Err(_) => -errno::INVAL,
        }
    }

    fn uname(&mut self, buf: u64) -> i64 {
        // Six 65-byte fields, as `struct utsname` has had since Linux 1.0.
        let mut out = vec![0u8; 6 * 65];
        let fields = [
            "Linux",
            "rsemu",
            "6.12.0-rsemu-usermode",
            "#1 rsemu level 3",
            self.arch.uname,
            "(none)",
        ];
        for (i, field) in fields.iter().enumerate() {
            out[i * 65..i * 65 + field.len()].copy_from_slice(field.as_bytes());
        }
        match self.mem.write_bytes(buf, &out) {
            Ok(()) => 0,
            Err(_) => -errno::INVAL,
        }
    }

    fn fstat(&mut self, fd: u64, buf: u64) -> i64 {
        const S_IFCHR: u32 = 0o020_000;
        const S_IFREG: u32 = 0o100_000;
        let mode = if fd <= 2 {
            S_IFCHR | 0o620
        } else if self.file(fd).is_some() {
            // Size zero, exactly as procfs reports: a file whose contents are
            // generated has no length until it is read, and stdio must not
            // try to allocate one buffer for the whole thing.
            S_IFREG | 0o444
        } else {
            return -errno::BADF;
        };
        // `struct stat` as `asm-generic/stat.h` lays it out: 128 bytes, of
        // which anything here looks only at `st_mode` and `st_blksize`.
        let mut out = vec![0u8; 128];
        out[16..20].copy_from_slice(&mode.to_le_bytes());
        out[20..24].copy_from_slice(&1u32.to_le_bytes());
        out[56..60].copy_from_slice(&(PAGE_SIZE as u32).to_le_bytes());
        match self.mem.write_bytes(buf, &out) {
            Ok(()) => 0,
            Err(_) => -errno::INVAL,
        }
    }

    /// `rt_sigaction(sig, act, oldact)`, stored and reported back.
    ///
    /// Not a stub returning zero, and the reason is the one `sigaltstack`
    /// already taught: **`oldact` is a query**, and a query that leaves the
    /// caller's buffer alone is answered by whatever was in it. A libc or a
    /// runtime asks "is a handler already installed for this signal?" before
    /// installing its own, and gets an answer this kernel did not give.
    ///
    /// It cost one `rt_sigaction` in the emulated trace against the same
    /// program's native one — 4 against 5 — with byte-identical output, which
    /// is precisely the signature of this class of defect and precisely why
    /// the trace is compared rather than the output.
    ///
    /// `struct sigaction` as `asm-generic` lays it out for an architecture
    /// with no `SA_RESTORER`, which RISC-V and AArch64 both are: a handler, a
    /// flags word and a 64-bit mask.
    fn rt_sigaction(&mut self, sig: u64, act: u64, old: u64) -> i64 {
        /// Signals 1..=64; `NSIG` is 64 on every `asm-generic` architecture.
        const NSIG: u64 = 64;
        if sig == 0 || sig > NSIG {
            return -errno::INVAL;
        }
        if old != 0 {
            let bytes = self
                .sigactions
                .get(&(sig as u32))
                .copied()
                .unwrap_or([0u8; K_SIGACTION]);
            if self.mem.write_bytes(old, &bytes).is_err() {
                return -errno::INVAL;
            }
        }
        if act != 0 {
            let mut bytes = [0u8; K_SIGACTION];
            if self.mem.read_bytes(act, &mut bytes).is_err() {
                return -errno::INVAL;
            }
            self.sigactions.insert(sig as u32, bytes);
        }
        0
    }

    /// `rt_sigprocmask(how, set, oldset)`, the same story one register along.
    ///
    /// `pthread_create` blocks every signal around the `clone` and the child
    /// restores the mask its parent saved, so a kernel that never wrote
    /// `oldset` would have the child restore a stack slot's leftovers. Nothing
    /// here delivers a signal, so nothing would visibly break — which is the
    /// argument for writing it down rather than against.
    fn rt_sigprocmask(&mut self, how: u64, set: u64, old: u64) -> i64 {
        /// `SIG_BLOCK`, `SIG_UNBLOCK`, `SIG_SETMASK`.
        const BLOCK: u64 = 0;
        const UNBLOCK: u64 = 1;
        const SETMASK: u64 = 2;
        let mask = self.tasks[&self.current].sigmask;
        if old != 0 && self.mem.write_bytes(old, &mask.to_le_bytes()).is_err() {
            return -errno::INVAL;
        }
        if set == 0 {
            return 0;
        }
        let mut bytes = [0u8; 8];
        if self.mem.read_bytes(set, &mut bytes).is_err() {
            return -errno::INVAL;
        }
        let arg = u64::from_le_bytes(bytes);
        self.task_mut().sigmask = match how {
            BLOCK => mask | arg,
            UNBLOCK => mask & !arg,
            SETMASK => arg,
            _ => return -errno::INVAL,
        };
        0
    }

    /// `sigaltstack(new, old)`, stored **per thread** and reported back.
    ///
    /// A stub that returned zero and wrote nothing was *worse than wrong*, and
    /// finding that out is what the native trace was for: a caller queries the
    /// current alternate stack first, and a query that leaves the guest's
    /// buffer untouched reads as `ss_flags == 0`, meaning "one is already
    /// installed". Rust's standard library then skips installing its own, so a
    /// stack-overflow handler that exists on Linux silently does not here.
    /// `SS_DISABLE` has to be *said*.
    ///
    /// `stack_t` is `{ void *ss_sp; int ss_flags; size_t ss_size; }` — 24
    /// bytes on every 64-bit `asm-generic` architecture.
    fn sigaltstack(&mut self, new: u64, old: u64) -> i64 {
        const SS_DISABLE: u32 = 2;
        if old != 0 {
            let mut out = [0u8; 24];
            match self.task_mut().altstack {
                Some((sp, size)) => {
                    out[..8].copy_from_slice(&sp.to_le_bytes());
                    out[16..24].copy_from_slice(&size.to_le_bytes());
                }
                None => out[8..12].copy_from_slice(&SS_DISABLE.to_le_bytes()),
            }
            if self.mem.write_bytes(old, &out).is_err() {
                return -errno::INVAL;
            }
        }
        if new != 0 {
            let mut buf = [0u8; 24];
            if self.mem.read_bytes(new, &mut buf).is_err() {
                return -errno::INVAL;
            }
            let sp = u64::from_le_bytes(buf[..8].try_into().unwrap());
            let flags = u32::from_le_bytes(buf[8..12].try_into().unwrap());
            let size = u64::from_le_bytes(buf[16..24].try_into().unwrap());
            self.task_mut().altstack = if flags & SS_DISABLE != 0 {
                None
            } else {
                Some((sp, size))
            };
        }
        0
    }

    fn sched_getaffinity(&mut self, len: u64, mask: u64) -> i64 {
        if len < 8 {
            return -errno::INVAL;
        }
        match self.mem.write_bytes(mask, &1u64.to_le_bytes()) {
            Ok(()) => 8,
            Err(_) => -errno::INVAL,
        }
    }
}

fn prot_of(bits: u64) -> Prot {
    let mut p = Prot::NONE;
    if bits & mm::PROT_READ != 0 {
        p = p.union(Prot::READ);
    }
    if bits & mm::PROT_WRITE != 0 {
        p = p.union(Prot::WRITE);
    }
    if bits & mm::PROT_EXEC != 0 {
        p = p.union(Prot::EXEC);
    }
    p
}

// ---------------------------------------------------------------------------
// The harness: load, start, run to exit
// ---------------------------------------------------------------------------

/// Where the stack goes, and how big it is. Both are the consumer's policy.
const STACK_TOP: u64 = 0x7fff_0000_0000;
const STACK_SIZE: u64 = 8 * 1024 * 1024;

/// What a finished run produced.
#[derive(Debug)]
struct Outcome {
    /// The status the guest exited with.
    status: i32,
    /// Everything it wrote to fd 1.
    stdout: Vec<u8>,
    /// Everything it wrote to fd 2.
    stderr: Vec<u8>,
    /// Every syscall it made, as `(number, return value)`.
    trace: Vec<(u64, i64)>,
    /// Every number this stand-in had to refuse. A `futex` operation it
    /// refused appears as `0x10000 | op`, so a missing operation is not
    /// hidden behind a number that was implemented.
    refused: Vec<u64>,
    /// Virtual ticks consumed.
    ticks: u64,
    /// Where the auxiliary vector's `AT_RANDOM` pointed, and to what.
    random: Vec<u8>,
    /// How many threads the process ever had, main included.
    threads: u64,
}

/// Load `file`, build its initial process image, and run it until its last
/// thread exits.
///
/// The whole consumer, end to end, through rsemu's public surface and nothing
/// else. `budget` caps virtual ticks so a guest that loops is a test failure
/// rather than a hung suite.
fn run(
    arch: &'static Arch,
    file: &[u8],
    argv: &[&str],
    envp: &[&str],
    journal: Arc<Journal>,
    entropy: Entropy,
    budget: u64,
) -> LoadResult<Outcome> {
    let mem = Arc::new(UserMemory::new(48));
    // Keep unplaced mappings — this consumer's `mmap` — below the stack, so
    // the two cannot collide however much the guest allocates.
    mem.set_placement(PAGE_SIZE, STACK_TOP - STACK_SIZE)
        .map_err(|e| e.to_string())?;
    let image = load(&mem, file, arch.machine)?;
    mem.map_at(STACK_TOP - STACK_SIZE, STACK_SIZE, Prot::RW, "[stack]")
        .map_err(|e| e.to_string())?;

    let clock = Arc::new(GuestClock::new());
    let main = (arch.start)(&mem, image.entry, STACK_TOP);
    let mut kernel = Kernel::new(
        arch,
        Arc::clone(&mem),
        Arc::clone(&clock),
        journal,
        entropy,
        image.brk,
        main,
    );

    let random = kernel.at_random();
    let sp = build_stack(
        &mem,
        STACK_TOP,
        argv,
        envp,
        &[
            (auxv::PHDR, Aux::Num(image.phdr)),
            (auxv::PHENT, Aux::Num(image.phent)),
            (auxv::PHNUM, Aux::Num(image.phnum)),
            (auxv::PAGESZ, Aux::Num(PAGE_SIZE)),
            (auxv::ENTRY, Aux::Num(image.entry)),
            (auxv::UID, Aux::Num(0)),
            (auxv::EUID, Aux::Num(0)),
            (auxv::GID, Aux::Num(0)),
            (auxv::EGID, Aux::Num(0)),
            (auxv::HWCAP, Aux::Num(arch.hwcap)),
            (auxv::CLKTCK, Aux::Num(100)),
            (auxv::SECURE, Aux::Num(0)),
            (auxv::RANDOM, Aux::Bytes(random.clone())),
        ],
    )?;
    kernel.main_thread().set_sp(sp);

    let threads = Arc::clone(&kernel.threads);
    loop {
        if clock.ticks() > budget {
            return Err(kernel.diagnose(&format!("the guest ran past its {budget}-tick budget")));
        }
        let Some(stop) = threads.run_next() else {
            // Every thread blocked with no deadline, or none left at all. The
            // second cannot happen here — the process ends when its last
            // thread exits — so this is a deadlock, and saying so is the whole
            // reason `run_next` reports it rather than spinning.
            return Err(kernel.diagnose("every thread is blocked: the guest deadlocked"));
        };
        let Some(exit) = stop.exit else { continue };
        match exit.reason {
            ExitReason::SYSCALL => {
                if let Some(status) = kernel.service(stop.thread) {
                    return Ok(Outcome {
                        status,
                        stdout: kernel.stdout,
                        stderr: kernel.stderr,
                        trace: kernel.trace,
                        refused: kernel.refused,
                        ticks: clock.ticks(),
                        random,
                        threads: kernel.spawned,
                    });
                }
            }
            ExitReason::FAULT => {
                return Err(kernel.diagnose(&format!(
                    "thread {} faulted at pc {:#x}: {:?} of {:#x} (cause {})",
                    stop.thread.0, exit.pc, exit.access, exit.address, exit.detail,
                )));
            }
            other => return Err(format!("the guest exited for {other}")),
        }
    }
}

// ---------------------------------------------------------------------------
// Synthetic guests: a real ELF file, assembled here, with no toolchain
// ---------------------------------------------------------------------------
//
// The loader is a parser, and a parser that is only ever handed one file has
// not been tested. These build well-formed and malformed ELF64 images in
// memory so the loader is exercised on `cargo test`, with nothing fetched and
// nothing installed — and so the real-binary tests below are measuring the
// guest rather than the loader.
//
// They are assembled for **every architecture this build has**, out of the
// four encodings in [`Asm`]. One program text, two instruction sets: that is
// the cheapest possible statement of what a level-3 seam is supposed to be.

fn words(code: &[u32]) -> Vec<u8> {
    code.iter().flat_map(|w| w.to_le_bytes()).collect()
}

/// One segment of a synthetic image.
struct Seg {
    vaddr: u64,
    flags: u32,
    data: Vec<u8>,
    memsz: u64,
}

/// Assemble a real ELF64 file: a header, a program header table inside the
/// first segment (which is where a real linker puts it, and what makes
/// `AT_PHDR` derivable), and the segments' bytes at page-congruent offsets.
fn elf64(entry: u64, machine: u16, etype: u16, segs: &[Seg]) -> Vec<u8> {
    let phoff = 64u64;
    let phent = 56u64;
    let phnum = segs.len() as u64;
    let mut file = vec![0u8; (phoff + phent * phnum) as usize];

    file[..4].copy_from_slice(b"\x7fELF");
    file[4] = 2; // ELFCLASS64
    file[5] = 1; // ELFDATA2LSB
    file[6] = 1; // EV_CURRENT
    file[16..18].copy_from_slice(&etype.to_le_bytes());
    file[18..20].copy_from_slice(&machine.to_le_bytes());
    file[20..24].copy_from_slice(&1u32.to_le_bytes());
    file[24..32].copy_from_slice(&entry.to_le_bytes());
    file[32..40].copy_from_slice(&phoff.to_le_bytes());
    file[52..54].copy_from_slice(&64u16.to_le_bytes());
    file[54..56].copy_from_slice(&(phent as u16).to_le_bytes());
    file[56..58].copy_from_slice(&(phnum as u16).to_le_bytes());

    for (i, seg) in segs.iter().enumerate() {
        // The first segment starts at file offset 0 so it carries the ELF
        // header and the program headers; the rest go at the next offset
        // congruent to their virtual address modulo the page size, which is
        // what `p_offset` and `p_vaddr` have to agree on for a real loader.
        let offset = if i == 0 {
            0
        } else {
            let want = seg.vaddr % PAGE_SIZE;
            let mut off = page_up(file.len() as u64) + want;
            if off < file.len() as u64 {
                off += PAGE_SIZE;
            }
            off
        };
        if i == 0 {
            file.extend_from_slice(&seg.data);
        } else {
            file.resize(offset as usize, 0);
            file.extend_from_slice(&seg.data);
        }
        let filesz = if i == 0 {
            file.len() as u64
        } else {
            seg.data.len() as u64
        };
        let ph = (phoff + phent * i as u64) as usize;
        file[ph..ph + 4].copy_from_slice(&PT_LOAD.to_le_bytes());
        file[ph + 4..ph + 8].copy_from_slice(&seg.flags.to_le_bytes());
        file[ph + 8..ph + 16].copy_from_slice(&offset.to_le_bytes());
        file[ph + 16..ph + 24].copy_from_slice(&seg.vaddr.to_le_bytes());
        file[ph + 24..ph + 32].copy_from_slice(&seg.vaddr.to_le_bytes());
        file[ph + 32..ph + 40].copy_from_slice(&filesz.to_le_bytes());
        file[ph + 40..ph + 48].copy_from_slice(&seg.memsz.max(filesz).to_le_bytes());
        file[ph + 48..ph + 56].copy_from_slice(&PAGE_SIZE.to_le_bytes());
    }
    file
}

/// The base every synthetic image is linked at.
const BASE: u64 = 0x1_0000;

/// Assemble a one-segment image whose code is written by `body`, which is
/// handed the address the code will end up at.
///
/// Two passes, because a program that names an address inside itself cannot
/// know it until the code is as long as it is going to be. Both architectures'
/// `li` emits a fixed number of instructions whatever the constant, so the
/// second pass is the same length as the first — and that is asserted rather
/// than assumed, because the day it stops being true is the day the message
/// address silently points into the middle of an instruction.
fn assemble(arch: &Arch, body: impl Fn(u64) -> Vec<u32>, tail: &[u8]) -> Vec<u8> {
    let header = 64 + 56;
    let first = body(0);
    let code_at = BASE + header;
    let code = body(code_at + first.len() as u64 * 4);
    assert_eq!(
        code.len(),
        first.len(),
        "an address inside the code moved the code"
    );
    let mut data = words(&code);
    data.extend_from_slice(tail);
    elf64(
        code_at,
        arch.machine,
        ET_EXEC,
        &[Seg {
            vaddr: BASE,
            flags: PF_R | PF_X,
            data,
            memsz: 0,
        }],
    )
}

/// `write(1, msg, len)` then `exit_group(0)`, as a complete ELF64 file.
fn hello_elf(arch: &Arch, message: &[u8]) -> Vec<u8> {
    let asm = arch.asm;
    let len = message.len() as u64;
    assemble(
        arch,
        move |msg_at| {
            let mut c = Vec::new();
            c.extend((asm.li)(0, 1)); // fd 1
            c.extend((asm.li)(1, msg_at));
            c.extend((asm.li)(2, len));
            c.extend((asm.li)(Asm::NR, nr::WRITE));
            c.push(asm.syscall);
            c.extend((asm.li)(0, 0));
            c.extend((asm.li)(Asm::NR, nr::EXIT_GROUP));
            c.push(asm.syscall);
            c
        },
        message,
    )
}

fn run_synthetic(arch: &'static Arch, file: &[u8]) -> LoadResult<Outcome> {
    run(
        arch,
        file,
        &["guest"],
        &[],
        Arc::new(Journal::new()),
        counting_entropy(),
        1_000_000,
    )
}

#[test]
fn a_real_elf_file_loads_and_runs_with_no_toolchain() {
    for arch in ARCHES {
        let out = run_synthetic(arch, &hello_elf(arch, b"hello from a real ELF\n"))
            .unwrap_or_else(|e| panic!("{}: {e}", arch.name));
        assert_eq!(out.stdout, b"hello from a real ELF\n", "{}", arch.name);
        assert!(out.stderr.is_empty(), "fd 2 is a separate sink");
        assert_eq!(out.status, 0);
        assert!(out.refused.is_empty(), "refused {:?}", out.refused);
        assert_eq!(out.threads, 1, "one program, one thread");
    }
}

#[test]
fn p_memsz_beyond_p_filesz_is_zeroed() {
    // Two segments: text, and a data segment whose `p_memsz` reaches a page
    // past its `p_filesz`. The guest exits with a word loaded out of that
    // gap, which must be zero — `.bss` is where a static binary keeps every
    // uninitialised global it has.
    let bss_base = 0x2_0000u64;
    for arch in ARCHES {
        let asm = arch.asm;
        let text = 64 + 56 * 2;
        let code = {
            let mut c = Vec::new();
            c.extend((asm.li)(Asm::TMP, bss_base + 0x800));
            c.push((asm.ld)(0, Asm::TMP));
            c.extend((asm.li)(Asm::NR, nr::EXIT_GROUP));
            c.push(asm.syscall);
            c
        };
        let file = elf64(
            BASE + text,
            arch.machine,
            ET_EXEC,
            &[
                Seg {
                    vaddr: BASE,
                    flags: PF_R | PF_X,
                    data: words(&code),
                    memsz: 0,
                },
                Seg {
                    vaddr: bss_base,
                    flags: PF_R | PF_W,
                    data: vec![0x5a; 8],
                    memsz: 0x1000,
                },
            ],
        );
        let out = run_synthetic(arch, &file).unwrap_or_else(|e| panic!("{}: {e}", arch.name));
        assert_eq!(out.status, 0, "{}: the bss word was not zero", arch.name);

        // And the *initialised* half of the same segment survived: the zeroing
        // covers `p_filesz..p_memsz` and not a byte below it.
        let mem = UserMemory::new(48);
        let image = load(&mem, &file, arch.machine).unwrap();
        let mut buf = [0u8; 8];
        mem.read_bytes(bss_base, &mut buf).unwrap();
        assert_eq!(buf, [0x5a; 8]);
        assert_eq!(image.brk, page_up(bss_base + 0x1000));
    }
}

#[test]
fn segments_get_the_permissions_their_flags_asked_for() {
    for arch in ARCHES {
        let mem = UserMemory::new(48);
        let file = hello_elf(arch, b"x");
        load(&mem, &file, arch.machine).unwrap();
        let maps = mem.mappings();
        assert_eq!(maps.len(), 1, "one segment, one range: {maps:?}");
        assert_eq!(maps[0].prot, Prot::RX);
        // A guest store into it is refused by the address space itself, with
        // no cooperation from the core — which is what makes `Prot` worth
        // carrying.
        assert!(mem.write_bytes(maps[0].base, b"!").is_err());
    }
}

#[test]
fn two_segments_sharing_a_page_get_the_union_of_their_permissions() {
    // A linker is allowed to end a read-only segment and start a writable one
    // inside the same page. Mapping each segment separately would make the
    // second erase the first; taking the union of their page ranges and of
    // their flags is what a page-granular map can actually express.
    for arch in ARCHES {
        let text = 64 + 56 * 2;
        let first = (arch.asm.li)(0, 0);
        let file = elf64(
            BASE + text,
            arch.machine,
            ET_EXEC,
            &[
                Seg {
                    vaddr: BASE,
                    flags: PF_R | PF_X,
                    data: words(&first),
                    memsz: 0,
                },
                Seg {
                    vaddr: BASE + 0xf00,
                    flags: PF_R | PF_W,
                    data: vec![0x11; 16],
                    memsz: 0,
                },
            ],
        );
        let mem = UserMemory::new(48);
        load(&mem, &file, arch.machine).unwrap();
        let maps = mem.mappings();
        assert_eq!(maps[0].base, BASE);
        assert_eq!(
            maps[0].prot,
            Prot::RWX,
            "the shared page permits what both segments asked for: {maps:?}"
        );
        // Both segments' bytes are there: neither mapping wiped the other.
        let mut buf = [0u8; 4];
        mem.read_bytes(BASE + text, &mut buf).unwrap();
        assert_eq!(
            u32::from_le_bytes(buf),
            first[0],
            "the first segment's code"
        );
        mem.read_bytes(BASE + 0xf00, &mut buf).unwrap();
        assert_eq!(buf, [0x11; 4], "the second segment's data");
    }
}

#[test]
fn at_phdr_points_at_the_program_headers_in_guest_memory() {
    for arch in ARCHES {
        let mem = UserMemory::new(48);
        let file = hello_elf(arch, b"x");
        let image = load(&mem, &file, arch.machine).unwrap();
        assert_eq!(image.phdr, BASE + 64);
        assert_eq!(image.phent, 56);
        assert_eq!(image.phnum, 1);
        // The bytes at that guest address are the program header table itself
        // — the check that catches a malformed auxv before a guest faults.
        let mut buf = [0u8; 56];
        mem.read_bytes(image.phdr, &mut buf).unwrap();
        assert_eq!(&buf[..], &file[64..64 + 56]);
    }
}

#[test]
fn a_hostile_or_wrong_image_is_refused_rather_than_mapped() {
    /// `EM_386`, which is never the architecture under test.
    const SOMEBODY_ELSE: u16 = 3;

    for arch in ARCHES {
        let mem = || UserMemory::new(48);
        let ok = hello_elf(arch, b"x");

        let cases: &[(&str, Vec<u8>)] = &[
            ("bad magic", b"not an elf file at all".to_vec()),
            ("empty", Vec::new()),
            ("truncated header", ok[..32].to_vec()),
            ("class", {
                let mut f = ok.clone();
                f[4] = 1;
                f
            }),
            ("data encoding", {
                let mut f = ok.clone();
                f[5] = 2;
                f
            }),
            ("e_type", {
                let mut f = ok.clone();
                f[16..18].copy_from_slice(&ET_DYN.to_le_bytes());
                f
            }),
            ("e_machine", {
                let mut f = ok.clone();
                f[18..20].copy_from_slice(&SOMEBODY_ELSE.to_le_bytes());
                f
            }),
            ("e_phentsize", {
                let mut f = ok.clone();
                f[54..56].copy_from_slice(&8u16.to_le_bytes());
                f
            }),
            ("e_phoff off the end", {
                let mut f = ok.clone();
                f[32..40].copy_from_slice(&u64::MAX.to_le_bytes());
                f
            }),
            ("p_filesz off the end", {
                let mut f = ok.clone();
                f[64 + 32..64 + 40].copy_from_slice(&(1u64 << 40).to_le_bytes());
                f
            }),
            ("p_filesz above p_memsz", {
                let mut f = ok.clone();
                f[64 + 40..64 + 48].copy_from_slice(&1u64.to_le_bytes());
                f
            }),
            ("p_vaddr wraps", {
                let mut f = ok.clone();
                f[64 + 16..64 + 24].copy_from_slice(&(u64::MAX - 8).to_le_bytes());
                f
            }),
            ("no program headers", {
                let mut f = ok.clone();
                f[56..58].copy_from_slice(&0u16.to_le_bytes());
                f
            }),
            ("PT_INTERP", {
                let mut f = ok.clone();
                f[64..68].copy_from_slice(&PT_INTERP.to_le_bytes());
                f
            }),
        ];
        for (what, bytes) in cases {
            let m = mem();
            let err = load(&m, bytes, arch.machine)
                .expect_err(&format!("{}: {what} should have been refused", arch.name));
            assert!(err.starts_with("ELF:"), "{what}: {err}");
        }
    }
}

// ---------------------------------------------------------------------------
// The initial stack
// ---------------------------------------------------------------------------

#[test]
fn the_initial_stack_is_the_layout_every_start_walks() {
    let mem = UserMemory::new(48);
    let top = 0x1_0000_0000u64;
    mem.map_at(top - 0x10000, 0x10000, Prot::RW, "[stack]")
        .unwrap();
    let random: Vec<u8> = (0..16).collect();
    let sp = build_stack(
        &mem,
        top,
        &["prog", "one"],
        &["PATH=/", "HOME=/root"],
        &[
            (auxv::PAGESZ, Aux::Num(PAGE_SIZE)),
            (auxv::RANDOM, Aux::Bytes(random.clone())),
        ],
    )
    .unwrap();

    assert_eq!(sp % 16, 0, "the psABI requires a 16-byte aligned sp");
    let word = |i: u64| mem.read_u64(sp + i * 8).unwrap();
    let cstr = |mut at: u64| {
        let mut out = Vec::new();
        loop {
            let mut b = [0u8; 1];
            mem.read_bytes(at, &mut b).unwrap();
            if b[0] == 0 {
                return String::from_utf8(out).unwrap();
            }
            out.push(b[0]);
            at += 1;
        }
    };

    assert_eq!(word(0), 2, "argc");
    assert_eq!(cstr(word(1)), "prog");
    assert_eq!(cstr(word(2)), "one");
    assert_eq!(word(3), 0, "argv is NULL terminated");
    assert_eq!(cstr(word(4)), "PATH=/");
    assert_eq!(cstr(word(5)), "HOME=/root");
    assert_eq!(word(6), 0, "envp is NULL terminated");
    assert_eq!((word(7), word(8)), (auxv::PAGESZ, PAGE_SIZE));
    assert_eq!(word(9), auxv::RANDOM);
    let mut bytes = [0u8; 16];
    mem.read_bytes(word(10), &mut bytes).unwrap();
    assert_eq!(&bytes[..], &random[..], "AT_RANDOM points at the bytes");
    assert_eq!((word(11), word(12)), (auxv::NULL, 0));
}

// ---------------------------------------------------------------------------
// The policy
// ---------------------------------------------------------------------------

/// A kernel over a scratch map with one thread, for the calls that need no
/// guest executing.
fn scratch_kernel(arch: &'static Arch) -> (Arc<UserMemory>, Kernel) {
    let mem = Arc::new(UserMemory::new(48));
    mem.map_at(0x1000, 0x1000, Prot::RW, "scratch").unwrap();
    let main = (arch.start)(&mem, 0, 0);
    let kernel = Kernel::new(
        arch,
        Arc::clone(&mem),
        Arc::new(GuestClock::new()),
        Arc::new(Journal::new()),
        counting_entropy(),
        0x10_0000,
        main,
    );
    (mem, kernel)
}

/// The first architecture this build has, for the tests that are about the
/// policy rather than about an instruction set.
fn any_arch() -> &'static Arch {
    ARCHES[0]
}

const ENOENT: i64 = -2;
const EBADF: i64 = -9;

#[test]
fn no_host_path_resolves_however_it_is_asked_for() {
    // The policy in one test: there is no host filesystem, so the answer does
    // not depend on the path, the directory fd, or the flags — and the paths
    // below are the ones a guest would try if it were looking.
    let (mem, mut kernel) = scratch_kernel(any_arch());
    for path in [
        "/etc/shadow",
        "/proc/self/exe",
        "/proc/self/mem",
        "../../../etc/passwd",
        "/dev/urandom",
        "/",
        "",
    ] {
        let mut bytes = path.as_bytes().to_vec();
        bytes.push(0);
        mem.write_bytes(0x1000, &bytes).unwrap();
        for nr in [nr::OPENAT, nr::FACCESSAT, nr::READLINKAT, nr::NEWFSTATAT] {
            assert_eq!(
                kernel.ask(nr, &[u64::MAX, 0x1000, 0, 0]),
                ENOENT,
                "{nr} on {path:?}"
            );
        }
    }
    // A path pointer the guest made up is refused rather than followed.
    assert_eq!(
        kernel.ask(nr::OPENAT, &[u64::MAX, 0xdead_0000, 0, 0]),
        ENOENT
    );
    assert!(kernel.files.iter().all(Option::is_none));
}

#[test]
fn the_one_openable_path_describes_the_guest_and_not_the_host() {
    // `/proc/self/maps`, served out of `UserMemory::mappings` — the reason
    // that method exists, and the whole of what a level-3 guest may be told.
    let (mem, mut kernel) = scratch_kernel(any_arch());
    mem.map_at(0x50_0000, 0x1000, Prot::RX, "elf").unwrap();
    mem.map_at(0x60_0000, 0x2000, Prot::RW, "[stack]").unwrap();
    mem.write_bytes(0x1000, b"/proc/self/maps\0").unwrap();

    let fd = kernel.ask(nr::OPENAT, &[u64::MAX, 0x1000, 0, 0]);
    assert_eq!(fd, 3, "the first descriptor above the standard three");
    let fd = fd as u64;

    // Read it the way stdio would, in pieces.
    let mut text = Vec::new();
    loop {
        let n = kernel.ask(nr::READ, &[fd, 0x1000, 64]);
        assert!(n >= 0, "read failed with {n}");
        if n == 0 {
            break;
        }
        let mut buf = alloc::vec![0u8; n as usize];
        mem.read_bytes(0x1000, &mut buf).unwrap();
        text.extend_from_slice(&buf);
    }
    let text = String::from_utf8(text).unwrap();
    assert!(
        text.contains("0000000000500000-0000000000501000 r-xp"),
        "{text}"
    );
    assert!(text.contains("[stack]"), "{text}");
    assert!(
        !text.contains("elf"),
        "only bracketed names are printed: {text}"
    );

    assert_eq!(kernel.ask(nr::CLOSE, &[fd]), 0);
    // And it is gone: a closed descriptor is not a descriptor.
    assert_eq!(kernel.ask(nr::READ, &[fd, 0x1000, 1]), EBADF);
}

#[test]
fn an_alternate_stack_query_says_there_is_none() {
    // The stub that returned zero and wrote nothing was worse than wrong: a
    // caller reads its own uninitialised buffer as "an alternate stack is
    // already installed" and skips installing one. Comparing the emulated
    // trace against the same program's native trace is what found it, and
    // this is the assertion that keeps it found.
    let (mem, mut kernel) = scratch_kernel(any_arch());
    mem.write_bytes(0x1000, &[0xff; 24]).unwrap();
    assert_eq!(kernel.ask(nr::SIGALTSTACK, &[0, 0x1000]), 0);
    let mut out = [0u8; 24];
    mem.read_bytes(0x1000, &mut out).unwrap();
    assert_eq!(u64::from_le_bytes(out[..8].try_into().unwrap()), 0);
    assert_eq!(
        u32::from_le_bytes(out[8..12].try_into().unwrap()),
        2,
        "SS_DISABLE"
    );

    // Install one, and it comes back.
    let mut new = [0u8; 24];
    new[..8].copy_from_slice(&0x4000u64.to_le_bytes());
    new[16..24].copy_from_slice(&0x2000u64.to_le_bytes());
    mem.write_bytes(0x1000, &new).unwrap();
    kernel.ask(nr::SIGALTSTACK, &[0x1000, 0]);
    kernel.ask(nr::SIGALTSTACK, &[0, 0x1000]);
    mem.read_bytes(0x1000, &mut out).unwrap();
    assert_eq!(u64::from_le_bytes(out[..8].try_into().unwrap()), 0x4000);
    assert_eq!(u64::from_le_bytes(out[16..24].try_into().unwrap()), 0x2000);
    assert_eq!(u32::from_le_bytes(out[8..12].try_into().unwrap()), 0);
}

#[test]
fn a_file_backed_mapping_is_refused_because_there_are_no_files() {
    let (_mem, mut kernel) = scratch_kernel(any_arch());
    const ENODEV: i64 = -19;
    // MAP_PRIVATE with a descriptor, and MAP_ANONYMOUS with one: both are a
    // guest trying to map something that is not its own memory.
    for (flags, fd) in [(0x02u64, 3u64), (0x22, 3)] {
        assert_eq!(kernel.ask(nr::MMAP, &[0, 0x1000, 3, flags, fd, 0]), ENODEV);
    }
    // Anonymous, with the -1 every libc passes, is granted.
    let base = kernel.ask(nr::MMAP, &[0, 0x2000, 3, 0x22, u64::MAX, 0]);
    assert!(base > 0, "mmap returned {base:#x}");
    assert_eq!(kernel.mem.mapping_at(base as u64).unwrap().prot, Prot::RW);
}

#[test]
fn only_the_three_standard_descriptors_exist() {
    let (mem, mut kernel) = scratch_kernel(any_arch());
    mem.write_bytes(0x1000, b"nope").unwrap();
    for fd in [3u64, 42, u64::MAX] {
        assert_eq!(kernel.ask(nr::WRITE, &[fd, 0x1000, 4]), EBADF, "fd {fd}");
    }
    // fd 0 is at end of file rather than an error: a program that reads
    // standard input gets a clean EOF, not a mystery.
    assert_eq!(kernel.ask(nr::READ, &[0, 0x1000, 4]), 0);
    assert!(kernel.stdout.is_empty() && kernel.stderr.is_empty());
}

#[test]
fn the_heap_grows_and_shrinks_through_brk() {
    let (mem, mut kernel) = scratch_kernel(any_arch());
    let base = 0x10_0000u64;
    assert_eq!(
        kernel.ask(nr::BRK, &[0]) as u64,
        base,
        "brk(0) reports where it is"
    );
    assert_eq!(kernel.ask(nr::BRK, &[base + 0x2800]) as u64, base + 0x2800);
    // The guest can now use every byte it asked for.
    mem.write_bytes(base + 0x27ff, b"!").unwrap();
    assert!(mem.write_bytes(base + 0x3000, b"!").is_err());
    // And giving it back unmaps the pages that are wholly above the new break.
    assert_eq!(kernel.ask(nr::BRK, &[base]) as u64, base);
    assert!(mem.write_bytes(base + 0x27ff, b"!").is_err());
}

#[test]
fn uname_reports_the_architecture_the_guest_is_actually_on() {
    // Not decoration: a `uname` that answered "riscv64" on an AArch64 guest is
    // the kind of thing that only surfaces three layers up, in a build script
    // that picks the wrong code path.
    for arch in ARCHES {
        let (mem, mut kernel) = scratch_kernel(arch);
        assert_eq!(kernel.ask(nr::UNAME, &[0x1000]), 0);
        let mut out = vec![0u8; 6 * 65];
        mem.read_bytes(0x1000, &mut out).unwrap();
        let field = |i: usize| {
            let f = &out[i * 65..(i + 1) * 65];
            String::from_utf8_lossy(&f[..f.iter().position(|b| *b == 0).unwrap()]).into_owned()
        };
        assert_eq!(field(0), "Linux");
        assert_eq!(field(4), arch.uname);
    }
}

// ---------------------------------------------------------------------------
// Threads
// ---------------------------------------------------------------------------

#[test]
fn a_clone_that_is_not_a_thread_is_refused_rather_than_half_done() {
    // `fork` needs a process model, and this stand-in has one process. The
    // refusal is explicit because the alternative — creating a thread that
    // shares an address space the caller asked *not* to share — is a guest
    // that corrupts itself much later.
    let (_mem, mut kernel) = scratch_kernel(any_arch());
    const EINVAL: i64 = -22;
    // No flags at all: a `fork`.
    assert_eq!(kernel.ask(nr::CLONE, &[0, 0x4000, 0, 0, 0]), EINVAL);
    // A thread with no stack of its own.
    let thread = cl::VM | cl::THREAD | cl::SIGHAND;
    assert_eq!(kernel.ask(nr::CLONE, &[thread, 0, 0, 0, 0]), EINVAL);
    assert_eq!(kernel.threads.len(), 1);
}

#[test]
fn a_futex_wait_blocks_and_a_wake_releases_exactly_who_it_said() {
    // The scheduler contract, driven directly: three threads park on one word
    // and a wake of two releases the first two, in arrival order. Arrival
    // order is a *choice* — the kernel promises only "some waiter" — and it
    // has to be one, or a replay diverges the first time two threads contend.
    let arch = any_arch();
    let (mem, mut kernel) = scratch_kernel(arch);
    let word = 0x1000u64;
    mem.write_bytes(word, &7u32.to_le_bytes()).unwrap();

    // Three siblings, each parked on the word.
    let mut ids = vec![kernel.current];
    for _ in 0..2 {
        let child = kernel.main_thread().spawn(0x2000);
        let id = kernel.threads.insert(child.core());
        kernel.tasks.insert(
            id,
            Task {
                thread: child,
                clear_child_tid: 0,
                altstack: None,
                sigmask: 0,
            },
        );
        ids.push(id);
    }
    for id in &ids {
        kernel.current = *id;
        assert_eq!(kernel.futex(word, fx::WAIT | fx::PRIVATE_FLAG, 7, 0), 0);
        assert_eq!(
            kernel.threads.state(*id),
            Some(ThreadState::Blocked { until: None })
        );
    }
    // Nothing is runnable, and rsemu says so rather than spinning.
    assert!(kernel.threads.run_next().is_none());

    assert_eq!(kernel.futex(word, fx::WAKE | fx::PRIVATE_FLAG, 2, 0), 2);
    assert_eq!(kernel.threads.state(ids[0]), Some(ThreadState::Runnable));
    assert_eq!(kernel.threads.state(ids[1]), Some(ThreadState::Runnable));
    assert_eq!(
        kernel.threads.state(ids[2]),
        Some(ThreadState::Blocked { until: None }),
        "a wake of two woke two"
    );
    // And a wait whose word has already moved on does not block at all.
    mem.write_bytes(word, &8u32.to_le_bytes()).unwrap();
    kernel.current = ids[0];
    assert_eq!(
        kernel.futex(word, fx::WAIT | fx::PRIVATE_FLAG, 7, 0),
        -errno::AGAIN
    );
}

#[test]
fn a_timed_futex_wait_reports_a_timeout_by_jumping_virtual_time() {
    // No host sleeps anywhere: the deadline is a virtual instant, `run_next`
    // jumps the clock to it because nothing else is runnable, and the thread
    // resumes with the answer that was preloaded when it blocked.
    let arch = any_arch();
    let (mem, mut kernel) = scratch_kernel(arch);
    let word = 0x1000u64;
    mem.write_bytes(word, &0u32.to_le_bytes()).unwrap();
    // A `timespec` of ten milliseconds, at a scratch address above the word.
    let ts = 0x1100u64;
    mem.write_bytes(ts, &0u64.to_le_bytes()).unwrap();
    mem.write_bytes(ts + 8, &10_000_000u64.to_le_bytes())
        .unwrap();

    let id = kernel.current;
    assert_eq!(
        kernel.futex(word, fx::WAIT | fx::PRIVATE_FLAG, 0, ts),
        -errno::TIMEDOUT,
        "the answer a wait that is never woken gets, written when it blocks"
    );
    let before = kernel.clock.ticks();
    assert!(matches!(
        kernel.threads.state(id),
        Some(ThreadState::Blocked { until: Some(_) })
    ));
    // The thread has no code under it, so it stops immediately — but virtual
    // time has moved to the deadline, which is the whole point.
    kernel.threads.run_next();
    assert!(
        kernel.clock.nanos() >= 10_000_000,
        "virtual time jumped to the deadline: {} ns from {before} ticks",
        kernel.clock.nanos()
    );
}

/// The reservation race, hand-assembled, with no toolchain and no guest
/// program: two threads, one word, one preemption in the wrong place.
///
/// What [`a_siblings_store_breaks_this_cores_reservation`] asserts, built here
/// rather than taken from the threaded guest because the guest only reaches
/// the sequence on one of the two architectures — `rustc` emits a
/// single-instruction `amoadd.d` on RISC-V and an `ldxr`/`stxr` loop on an
/// AArch64 part without `FEAT_LSE`. Neither compiler and neither instruction
/// set was ever the point: it was that **the exclusive monitor was per core**,
/// and level-3 threads are one core each over one map.
fn reservation_race(arch: &'static Arch) -> (u64, u64, u64) {
    let asm = arch.asm;
    /// Where the contended word and the two status words live.
    const DATA: u64 = 0x2_0000;
    /// Long enough that the first thread's quantum ends inside it, and that
    /// the second thread has finished by the time the first gets back.
    const PAD: usize = 400;
    const A_WROTE: u64 = 0xaaaa;
    const B_WROTE: u64 = 0xbbbb;

    // The thread that reserves, is preempted, and stores anyway.
    let mut a = Vec::new();
    a.extend((asm.li)(Asm::TMP, DATA));
    a.extend((asm.li)(2, A_WROTE));
    a.push((asm.lr)(0, Asm::TMP));
    a.extend(core::iter::repeat_n(asm.nop, PAD));
    a.push((asm.sc)(1, 2, Asm::TMP));
    a.extend((asm.li)(3, DATA + 8));
    a.push((asm.st)(1, 3));
    a.extend((asm.li)(0, 0));
    a.extend((asm.li)(Asm::NR, nr::EXIT_GROUP));
    a.push(asm.syscall);

    // The thread that runs while it is preempted, and does its own complete
    // reserve-and-store on the same word.
    let mut b = Vec::new();
    b.extend((asm.li)(Asm::TMP, DATA));
    b.extend((asm.li)(2, B_WROTE));
    b.push((asm.lr)(0, Asm::TMP));
    b.push((asm.sc)(1, 2, Asm::TMP));
    b.extend((asm.li)(3, DATA + 16));
    b.push((asm.st)(1, 3));
    b.extend((asm.li)(0, 0));
    b.extend((asm.li)(Asm::NR, nr::EXIT_GROUP));
    b.push(asm.syscall);

    let header = 64 + 56 * 2;
    let entry_a = BASE + header;
    let entry_b = entry_a + a.len() as u64 * 4;
    let mut text = words(&a);
    text.extend_from_slice(&words(&b));
    let file = elf64(
        entry_a,
        arch.machine,
        ET_EXEC,
        &[
            Seg {
                vaddr: BASE,
                flags: PF_R | PF_X,
                data: text,
                memsz: 0,
            },
            Seg {
                vaddr: DATA,
                flags: PF_R | PF_W,
                data: vec![0; 24],
                memsz: 0x1000,
            },
        ],
    );

    let mem = Arc::new(UserMemory::new(48));
    mem.set_placement(PAGE_SIZE, STACK_TOP - STACK_SIZE)
        .unwrap();
    load(&mem, &file, arch.machine).unwrap();
    mem.map_at(STACK_TOP - STACK_SIZE, STACK_SIZE, Prot::RW, "[stack]")
        .unwrap();

    let clock = Arc::new(GuestClock::new());
    let first = (arch.start)(&mem, entry_a, STACK_TOP);
    let second = first.spawn(STACK_TOP - 0x10000);
    second.core().set_pc(entry_b);

    let threads = ThreadSet::new(Arc::clone(&clock));
    // Short enough that the first thread is preempted inside its pad, which is
    // the whole experiment. A real consumer's quantum is thousands of times
    // this and only makes the race rarer, never impossible.
    threads.set_quantum(8);
    threads.insert(first.core());
    threads.insert(second.core());

    let mut live = 2;
    while live > 0 {
        assert!(
            clock.ticks() < 1_000_000,
            "{}: the race never finished",
            arch.name
        );
        let Some(stop) = threads.run_next() else {
            panic!("{}: nothing is runnable", arch.name)
        };
        let Some(exit) = stop.exit else { continue };
        assert_eq!(
            exit.reason,
            ExitReason::SYSCALL,
            "{}: unexpected exit at {:#x}",
            arch.name,
            exit.pc
        );
        threads.remove(stop.thread);
        live -= 1;
    }

    (
        mem.read_u64(DATA).unwrap(),
        mem.read_u64(DATA + 8).unwrap(),
        mem.read_u64(DATA + 16).unwrap(),
    )
}

#[test]
fn a_siblings_store_breaks_this_cores_reservation() {
    // **The ledger entry, shrunk.**
    //
    // This test used to assert the defect. The exclusive monitor lived in the
    // core's own execution state — RISC-V's `reservation`, AArch64's
    // `State::exclusive` — and was broken only by a store *that core* made, so
    // a `sc`/`stxr` the architecture requires to fail succeeded instead and the
    // sibling's update was lost. Level-3 threads are one core each over one
    // `UserMemory`, which is how it was found.
    //
    // `core::space::ExclusiveMonitor` is the fix: a reservation table on the
    // address space, one slot per core, consulted by every store that reaches
    // `SpaceView::write_span`. That is AArch64's *global* monitor (DDI 0487
    // B2.9) and RISC-V's reservation set — the same object seen twice. The
    // core's own field stays as the local monitor, and a store-conditional now
    // needs both to agree.
    //
    // So the assertions are inverted, on both architectures: the first
    // thread's store-conditional must now *fail*, and the word must still hold
    // what the second thread put there.
    for arch in ARCHES {
        let (word, a_status, b_status) = reservation_race(arch);
        assert_eq!(
            b_status, 0,
            "{}: the second thread's own pair should store",
            arch.name
        );
        assert_ne!(
            a_status, 0,
            "{}: the first thread's store-conditional succeeded even though a \
             sibling wrote the reservation set in between — the global \
             exclusive monitor is not being consulted",
            arch.name
        );
        assert_eq!(
            word, 0xbbbb,
            "{}: the second thread's update survived, because the first \
             thread's conditional store did not happen",
            arch.name
        );
    }
}

// ---------------------------------------------------------------------------
// Determinism: the two doors entropy comes through
// ---------------------------------------------------------------------------

#[test]
fn at_random_and_getrandom_both_go_through_the_journal() {
    // A guest that asks for eight bytes and writes them to fd 1, so both
    // doors are visible in one run: `AT_RANDOM` while the stack is built, and
    // `getrandom` from the program.
    for arch in ARCHES {
        let asm = arch.asm;
        let buf = 0x2_0000u64;
        let text = 64 + 56 * 2;
        let code = {
            let mut c = Vec::new();
            c.extend((asm.li)(0, buf));
            c.extend((asm.li)(1, 8));
            c.extend((asm.li)(2, 0));
            c.extend((asm.li)(Asm::NR, nr::GETRANDOM));
            c.push(asm.syscall);
            c.extend((asm.li)(0, 1));
            c.extend((asm.li)(1, buf));
            c.extend((asm.li)(2, 8));
            c.extend((asm.li)(Asm::NR, nr::WRITE));
            c.push(asm.syscall);
            c.extend((asm.li)(0, 0));
            c.extend((asm.li)(Asm::NR, nr::EXIT_GROUP));
            c.push(asm.syscall);
            c
        };
        let file = elf64(
            BASE + text,
            arch.machine,
            ET_EXEC,
            &[
                Seg {
                    vaddr: BASE,
                    flags: PF_R | PF_X,
                    data: words(&code),
                    memsz: 0,
                },
                Seg {
                    vaddr: buf,
                    flags: PF_R | PF_W,
                    data: vec![0; 8],
                    memsz: 0x1000,
                },
            ],
        );

        // Record, with an entropy source that is *not* a function of the
        // program.
        let recording = Arc::new(Journal::with_mode(JournalMode::Record));
        let mut counter = 0u8;
        let live = run(
            arch,
            &file,
            &["g"],
            &[],
            Arc::clone(&recording),
            alloc::boxed::Box::new(move |len| {
                counter = counter.wrapping_add(1);
                vec![counter; len]
            }),
            1_000_000,
        )
        .unwrap_or_else(|e| panic!("{}: {e}", arch.name));
        assert_eq!(live.stdout, vec![2u8; 8]);
        assert_eq!(live.random, vec![1u8; 16]);
        assert_eq!(recording.len(), 2, "AT_RANDOM and one getrandom");

        // Replay, with a source that panics. Nothing else in the run may reach
        // the host, and the output is identical.
        recording.set_mode(JournalMode::Replay);
        let replayed = run(
            arch,
            &file,
            &["g"],
            &[],
            Arc::clone(&recording),
            Kernel::replay_guard(),
            1_000_000,
        )
        .unwrap();
        assert_eq!(replayed.stdout, live.stdout);
        assert_eq!(replayed.random, live.random);
        assert_eq!(replayed.trace, live.trace);
        assert_eq!(replayed.ticks, live.ticks);
        assert_eq!(recording.remaining(), 0);

        // A recording survives the file format, which is what makes `rsemu
        // replay` on another host a thing rather than an aspiration.
        let bytes = recording.encode().unwrap();
        let decoded = Arc::new(Journal::decode(&bytes).unwrap());
        decoded.set_mode(JournalMode::Replay);
        let from_file = run(
            arch,
            &file,
            &["g"],
            &[],
            decoded,
            Kernel::replay_guard(),
            1_000_000,
        )
        .unwrap();
        assert_eq!(from_file.stdout, live.stdout);
    }
}

#[test]
fn a_run_is_a_function_of_the_program_alone() {
    // Two runs, two everythings, and the whole trace agrees — including the
    // virtual clock, which is what `clock_gettime` would have answered from.
    for arch in ARCHES {
        let file = hello_elf(arch, b"deterministic\n");
        let first = run_synthetic(arch, &file).unwrap();
        let second = run_synthetic(arch, &file).unwrap();
        assert_eq!(first.stdout, second.stdout);
        assert_eq!(first.trace, second.trace);
        assert_eq!(first.ticks, second.ticks);
        assert!(first.ticks > 0);
    }
}

// ---------------------------------------------------------------------------
// The milestone: real statically linked Linux binaries
// ---------------------------------------------------------------------------

/// Run the named built guest on every architecture this build has one for,
/// reporting what it did.
///
/// The corpus rule applies even though these are *built* rather than
/// downloaded (CLAUDE.md, Testing): a compiler's output is not a source file
/// and does not belong in the repository, so without the fixture the test says
/// what to run and passes. Everything above this line runs unconditionally.
#[cfg(feature = "std")]
fn run_built_guest(
    name: &str,
    argv: &[&str],
    envp: &[&str],
    budget: u64,
    check: impl Fn(&'static Arch, &Outcome),
) {
    let mut ran = 0;
    for arch in ARCHES {
        let Some(bytes) = guest_binary(&std::format!("{name}-{}", arch.suffix)) else {
            continue;
        };
        let journal = Arc::new(Journal::with_mode(JournalMode::Record));
        let out = run(
            arch,
            &bytes,
            argv,
            envp,
            Arc::clone(&journal),
            counting_entropy(),
            budget,
        )
        .unwrap_or_else(|e| panic!("{}: {e}", arch.name));

        std::eprintln!(
            "usermode/{name} on {}: {} syscall(s), {} thread(s), {} tick(s); refused {:?}",
            arch.name,
            out.trace.len(),
            out.threads,
            out.ticks,
            out.refused
        );
        std::eprintln!(
            "usermode/{name} on {}: stdout {:?}",
            arch.name,
            String::from_utf8_lossy(&out.stdout)
        );
        // The trace is the discovery tool, and it is how this module was
        // written: run the same program natively under `strace` and compare
        // call for call. It is behind a variable only because a hundred and
        // thirty pairs are noise in a passing run.
        if std::env::var("RSEMU_USERMODE_TRACE").is_ok() {
            std::eprintln!("usermode/{name} on {}: TRACE {:?}", arch.name, out.trace);
        }
        if !out.stderr.is_empty() {
            std::eprintln!(
                "usermode/{name} on {}: stderr {:?}",
                arch.name,
                String::from_utf8_lossy(&out.stderr)
            );
        }
        check(arch, &out);

        // The whole run replays with the host unplugged.
        journal.set_mode(JournalMode::Replay);
        let replayed = run(
            arch,
            &bytes,
            argv,
            envp,
            Arc::clone(&journal),
            Kernel::replay_guard(),
            budget,
        )
        .expect("the guest replayed");
        assert_eq!(replayed.stdout, out.stdout);
        assert_eq!(replayed.trace, out.trace);
        assert_eq!(replayed.ticks, out.ticks);
        ran += 1;
    }
    if ran == 0 {
        std::eprintln!(
            "usermode: no {name} guest for any architecture in this build. Build one with\n    \
             scripts/fetch-testdata.sh usermode-guests"
        );
    }
}

#[cfg(feature = "std")]
#[test]
fn a_real_static_linux_binary_prints_and_exits_zero() {
    run_built_guest(
        "hello",
        &["hello"],
        &["RSEMU=1"],
        4_000_000_000,
        |_, out| {
            assert_eq!(out.status, 0, "the guest exited {}", out.status);
            assert!(out.stderr.is_empty(), "the guest wrote to fd 2");
            assert!(out.refused.is_empty(), "refused {:?}", out.refused);
            assert_eq!(
                String::from_utf8_lossy(&out.stdout),
                "hello from level 3\nargv = [\"hello\"]\nRSEMU = Some(\"1\")\n",
                "the guest read its own argv and environment off the stack the \
             loader built, so a stack that is subtly wrong fails here rather \
             than later"
            );
        },
    );
}

#[cfg(feature = "std")]
#[test]
fn a_real_threaded_binary_spawns_joins_and_agrees_on_the_answer() {
    /// Four threads, ten thousand increments each — `tests/usermode/threads.rs`.
    const TOTAL: u64 = 40_000;

    run_built_guest("threads", &["threads"], &[], 20_000_000_000, |arch, out| {
        assert_eq!(out.status, 0, "the guest exited {}", out.status);
        assert!(out.stderr.is_empty(), "the guest wrote to fd 2");
        assert!(out.refused.is_empty(), "refused {:?}", out.refused);
        assert_eq!(
            out.threads, 8,
            "one main thread, four workers and three sleepers"
        );

        let stdout = String::from_utf8_lossy(&out.stdout).into_owned();
        let lines: Vec<&str> = stdout.lines().collect();
        assert_eq!(
            lines.len(),
            3,
            "{}: the guest printed {stdout:?}",
            arch.name
        );
        // Every worker ran its closure and every `join` returned: that is
        // `clone`, the child's `clear_child_tid`, and the `futex` the joiner
        // was parked on, all three or none.
        assert_eq!(lines[0], "joined [0, 1, 2, 3]", "{}", arch.name);
        // Three threads blocked on a condition variable with no deadline and
        // one `FUTEX_WAKE` released them. A thread blocked with no deadline is
        // only ever runnable again because somebody woke it, so reaching this
        // line at all is the wake having worked.
        assert_eq!(lines[2], "rendezvous ok", "{}", arch.name);

        let counter: u64 = lines[1]
            .strip_prefix("counter = ")
            .and_then(|n| n.parse().ok())
            .unwrap_or_else(|| panic!("{}: {:?}", arch.name, lines[1]));
        // **No ledger line here any more, on either architecture.** This used
        // to accept any count on a target whose `fetch_add` is an `ldxr`/`stxr`
        // loop — `aarch64-unknown-linux-musl`'s baseline is Armv8.0 without
        // `FEAT_LSE`, so it is — because the exclusive monitor was core-local
        // and a preemption between the two let a sibling's increment be
        // overwritten. It landed 32038 of 40000. With
        // `core::space::ExclusiveMonitor` the sibling's store breaks the
        // reservation, the `stxr` fails, and the loop retries: one arithmetic
        // answer, whatever the compiler emitted for it.
        assert_eq!(
            counter, TOTAL,
            "{}: every increment must land — whether `fetch_add` is one \
             instruction or an `ldxr`/`stxr` loop is the compiler's business, \
             not the answer's",
            arch.name
        );
    });
}

/// Read the named guest binary, or `None` if this checkout has not built one.
///
/// `RSEMU_USERMODE_GUEST` names a file directly; otherwise
/// `$RSEMU_TESTDATA/usermode/<name>` is where the fetch script puts it.
#[cfg(feature = "std")]
fn guest_binary(name: &str) -> Option<Vec<u8>> {
    if let Ok(path) = std::env::var("RSEMU_USERMODE_GUEST") {
        return std::fs::read(&path).ok();
    }
    let root = std::env::var("RSEMU_TESTDATA")
        .unwrap_or_else(|_| std::format!("{}/testdata", std::env!("CARGO_MANIFEST_DIR")));
    std::fs::read(std::format!("{root}/usermode/{name}")).ok()
}
