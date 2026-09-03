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

use crate::core::exec::{ExitMask, ExitReason, ExitingCore};
use crate::cpu::riscv::csr::Priv;
use crate::cpu::riscv::{Config, Hart};

use super::{
    Answer, GuestClock, Journal, JournalMode, PAGE_SIZE, Prot, Tag, ThreadSet, UserMemory,
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

/// `EM_RISCV`.
const EM_RISCV: u16 = 243;

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
    pub(super) const CLOCK_GETTIME: u64 = 113;
    pub(super) const SCHED_GETAFFINITY: u64 = 123;
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
    pub(super) const GETRANDOM: u64 = 278;
    pub(super) const RSEQ: u64 = 293;
}

/// The errno values this stand-in returns. Negated into `a0`, which is how
/// every `asm-generic` architecture reports a failure.
mod errno {
    /// No such file or directory. The answer to every path.
    pub(super) const NOENT: i64 = 2;
    /// Bad file descriptor.
    pub(super) const BADF: i64 = 9;
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

/// The consumer's half: file descriptors, errno, a heap, and a syscall table.
///
/// Everything §2.1 says is nixvm's, written out here in the smallest form that
/// runs a real binary, so rsemu's half has something to be proven against.
struct Kernel {
    mem: Arc<UserMemory>,
    clock: Arc<GuestClock>,
    journal: Arc<Journal>,
    entropy: Entropy,
    /// Everything the guest wrote to fd 1.
    stdout: Vec<u8>,
    /// Everything the guest wrote to fd 2.
    stderr: Vec<u8>,
    brk_base: u64,
    brk: u64,
    tid_address: u64,
    /// The alternate signal stack the guest last installed, if any.
    altstack: Option<(u64, u64)>,
    /// Descriptors above 2, each a snapshot of something the *guest* can
    /// legitimately be told about itself. Never a host file — see the module
    /// documentation.
    files: Vec<Option<Vfile>>,
    /// `(number, return value)` for every call serviced, in order. The
    /// discovery tool: *implement from a trace, not from a list*.
    trace: Vec<(u64, i64)>,
    /// Numbers this stand-in refused, deduplicated, in first-asked order.
    refused: Vec<u64>,
}

impl Kernel {
    /// A kernel over `mem`, with a heap starting at `brk_base`.
    fn new(
        mem: Arc<UserMemory>,
        clock: Arc<GuestClock>,
        journal: Arc<Journal>,
        entropy: Entropy,
        brk_base: u64,
    ) -> Kernel {
        Kernel {
            mem,
            clock,
            journal,
            entropy,
            stdout: Vec::new(),
            stderr: Vec::new(),
            brk_base,
            brk: brk_base,
            tid_address: 0,
            altstack: None,
            files: Vec::new(),
            trace: Vec::new(),
            refused: Vec::new(),
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

    /// Service the call the hart has just exited on. `Some(status)` when the
    /// guest asked to exit.
    fn service(&mut self, hart: &Hart) -> Option<i32> {
        let a = |i: u32| hart.x(10 + i);
        let nr = hart.x(17);
        let ret = match nr {
            nr::EXIT | nr::EXIT_GROUP => {
                self.trace.push((nr, 0));
                return Some(a(0) as i32);
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
                self.tid_address = a(0);
                1
            }
            nr::SET_ROBUST_LIST => 0,
            nr::CLOCK_GETTIME => self.clock_gettime(a(1)),
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
            nr::RT_SIGACTION | nr::RT_SIGPROCMASK => 0,
            nr::SIGALTSTACK => self.sigaltstack(a(0), a(1)),
            nr::FUTEX => 0,
            nr::TGKILL => 0,
            nr::PRLIMIT64 => 0,
            nr::SCHED_GETAFFINITY => self.sched_getaffinity(a(1), a(2)),
            nr::GETPID | nr::GETTID => 1,
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
        hart.set_x(10, ret as u64);
        None
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
            "riscv64",
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

    /// `sigaltstack(new, old)`, stored and reported back.
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
            match self.altstack {
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
            self.altstack = if flags & SS_DISABLE != 0 {
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

/// The `AT_HWCAP` bitmap for an RV64GC hart: one bit per single-letter
/// extension, bit 0 being `A`, as the kernel's `ELF_HWCAP` defines it. Claimed
/// honestly — a guest that reads it is asking what this hart has.
const HWCAP_GC: u64 = (1 << 0)   // A
    | (1 << 2)                    // C
    | (1 << 3)                    // D
    | (1 << 5)                    // F
    | (1 << 8)                    // I
    | (1 << 12); // M

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
    /// Every number this stand-in had to refuse.
    refused: Vec<u64>,
    /// Virtual ticks consumed.
    ticks: u64,
    /// Where the auxiliary vector's `AT_RANDOM` pointed, and to what.
    random: Vec<u8>,
}

/// Load `file`, build its initial process image, and run it to `exit_group`.
///
/// The whole consumer, end to end, through rsemu's public surface and nothing
/// else. `budget` caps virtual ticks so a guest that loops is a test failure
/// rather than a hung suite.
fn run(
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
    let image = load(&mem, file, EM_RISCV)?;
    mem.map_at(STACK_TOP - STACK_SIZE, STACK_SIZE, Prot::RW, "[stack]")
        .map_err(|e| e.to_string())?;

    let clock = Arc::new(GuestClock::new());
    let mut kernel = Kernel::new(
        Arc::clone(&mem),
        Arc::clone(&clock),
        journal,
        entropy,
        image.brk,
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
            (auxv::HWCAP, Aux::Num(HWCAP_GC)),
            (auxv::CLKTCK, Aux::Num(100)),
            (auxv::SECURE, Aux::Num(0)),
            (auxv::RANDOM, Aux::Bytes(random.clone())),
        ],
    )?;

    let cfg = Config {
        pmp_count: 0,
        ..Config::rv64gc()
    }
    .with_reset_vector(image.entry);
    let hart = Arc::new(Hart::new(cfg));
    hart.attach_space(Arc::clone(mem.space()));
    hart.set_pc(image.entry);
    hart.set_x(2, sp);
    let mut csrs = hart.csrs();
    csrs.priv_mode = Priv::User;
    hart.set_csrs(csrs);
    hart.set_exit_mask(ExitMask::USER);

    let threads = ThreadSet::new(Arc::clone(&clock));
    let id = threads.insert(Arc::clone(&hart) as Arc<dyn ExitingCore>);

    loop {
        if clock.ticks() > budget {
            return Err(format!(
                "the guest ran past its {budget}-tick budget after {} syscall(s); \
                 the last few were {:?}",
                kernel.trace.len(),
                &kernel.trace[kernel.trace.len().saturating_sub(8)..]
            ));
        }
        let stop = threads.run_next().ok_or("nothing is runnable")?;
        let Some(exit) = stop.exit else { continue };
        match exit.reason {
            ExitReason::SYSCALL => {
                if let Some(status) = kernel.service(&hart) {
                    threads.remove(id);
                    return Ok(Outcome {
                        status,
                        stdout: kernel.stdout,
                        stderr: kernel.stderr,
                        trace: kernel.trace,
                        refused: kernel.refused,
                        ticks: clock.ticks(),
                        random,
                    });
                }
            }
            ExitReason::FAULT => {
                return Err(format!(
                    "the guest faulted at pc {:#x}: {:?} of {:#x} (cause {}), after \
                     {} syscall(s), the last few being {:?}",
                    exit.pc,
                    exit.access,
                    exit.address,
                    exit.detail,
                    kernel.trace.len(),
                    &kernel.trace[kernel.trace.len().saturating_sub(8)..]
                ));
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
// nothing installed — and so the real-binary test below is measuring the guest
// rather than the loader.

/// `addi rd, rs1, imm` — I-type, and with `rs1 = x0` the `li` a hand-assembled
/// program is mostly made of. Volume I, "Integer Register-Immediate".
const fn addi(rd: u32, rs1: u32, imm: i32) -> u32 {
    ((imm as u32) << 20) | (rs1 << 15) | (rd << 7) | 0b001_0011
}

/// `lui rd, imm20` — U-type.
const fn lui(rd: u32, imm20: u32) -> u32 {
    (imm20 << 12) | (rd << 7) | 0b011_0111
}

/// `ld rd, imm(rs1)` — I-type, opcode `0000011`, funct3 `011`.
const fn ld(rd: u32, rs1: u32, imm: i32) -> u32 {
    ((imm as u32) << 20) | (rs1 << 15) | (0b011 << 12) | (rd << 7) | 0b000_0011
}

/// `ecall`.
const ECALL: u32 = 0x0000_0073;

/// The `lui`/`addi` pair a `li` expands to. The `addi` immediate is sign
/// extended, so the upper half is pre-compensated when the lower half's top
/// bit is set — a fact about the encoding, not a trick.
fn li(rd: u32, value: u64) -> [u32; 2] {
    let lo = (value & 0xfff) as i32;
    let lo = if lo & 0x800 != 0 { lo - 0x1000 } else { lo };
    let hi = ((value as i64 - i64::from(lo)) >> 12) as u32 & 0xf_ffff;
    [lui(rd, hi), addi(rd, rd, lo)]
}

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

/// `write(1, msg, len)` then `exit(0)`, as a complete ELF64 file.
fn hello_elf(message: &[u8]) -> Vec<u8> {
    // One segment: the ELF header, the single program header, the code, and
    // the message. The message's address is known before the code is
    // assembled because the header sizes are fixed.
    let text = 64 + 56;
    let code_words = 4 * 2 + 1 + 2 * 2 + 1; // four `li`, ecall, two `li`, ecall
    let msg_at = BASE + text + code_words * 4;
    let code = {
        let mut c = Vec::new();
        c.extend(li(10, 1));
        c.extend(li(11, msg_at));
        c.extend(li(12, message.len() as u64));
        c.extend(li(17, 64)); // __NR_write
        c.push(ECALL);
        c.extend(li(10, 0));
        c.extend(li(17, 94)); // __NR_exit_group
        c.push(ECALL);
        c
    };
    assert_eq!(
        code.len() as u64,
        code_words,
        "the message address is fixed"
    );
    let mut data = words(&code);
    data.extend_from_slice(message);
    elf64(
        BASE + text,
        EM_RISCV,
        ET_EXEC,
        &[Seg {
            vaddr: BASE,
            flags: PF_R | PF_X,
            data,
            memsz: 0,
        }],
    )
}

fn run_synthetic(file: &[u8]) -> LoadResult<Outcome> {
    run(
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
    let out = run_synthetic(&hello_elf(b"hello from a real ELF\n")).unwrap();
    assert_eq!(out.stdout, b"hello from a real ELF\n");
    assert!(out.stderr.is_empty(), "fd 2 is a separate sink");
    assert_eq!(out.status, 0);
    assert!(out.refused.is_empty(), "refused {:?}", out.refused);
}

#[test]
fn p_memsz_beyond_p_filesz_is_zeroed() {
    // Two segments: text, and a data segment whose `p_memsz` reaches a page
    // past its `p_filesz`. The guest exits with a word loaded out of that
    // gap, which must be zero — `.bss` is where a static binary keeps every
    // uninitialised global it has.
    let bss_base = 0x2_0000u64;
    let text = 64 + 56 * 2;
    let code = {
        let mut c = Vec::new();
        c.extend(li(5, bss_base + 0x800));
        c.push(ld(10, 5, 0));
        c.extend(li(17, 94));
        c.push(ECALL);
        c
    };
    let file = elf64(
        BASE + text,
        EM_RISCV,
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
    let out = run_synthetic(&file).unwrap();
    assert_eq!(out.status, 0, "the bss word was not zero");

    // And the *initialised* half of the same segment survived: the zeroing
    // covers `p_filesz..p_memsz` and not a byte below it.
    let mem = UserMemory::new(48);
    let image = load(&mem, &file, EM_RISCV).unwrap();
    let mut buf = [0u8; 8];
    mem.read_bytes(bss_base, &mut buf).unwrap();
    assert_eq!(buf, [0x5a; 8]);
    assert_eq!(image.brk, page_up(bss_base + 0x1000));
}

#[test]
fn segments_get_the_permissions_their_flags_asked_for() {
    let mem = UserMemory::new(48);
    let file = hello_elf(b"x");
    load(&mem, &file, EM_RISCV).unwrap();
    let maps = mem.mappings();
    assert_eq!(maps.len(), 1, "one segment, one range: {maps:?}");
    assert_eq!(maps[0].prot, Prot::RX);
    // A guest store into it is refused by the address space itself, with no
    // cooperation from the core — which is what makes `Prot` worth carrying.
    assert!(mem.write_bytes(maps[0].base, b"!").is_err());
}

#[test]
fn two_segments_sharing_a_page_get_the_union_of_their_permissions() {
    // A linker is allowed to end a read-only segment and start a writable one
    // inside the same page. Mapping each segment separately would make the
    // second erase the first; taking the union of their page ranges and of
    // their flags is what a page-granular map can actually express.
    let text = 64 + 56 * 2;
    let file = elf64(
        BASE + text,
        EM_RISCV,
        ET_EXEC,
        &[
            Seg {
                vaddr: BASE,
                flags: PF_R | PF_X,
                data: words(&li(10, 0)),
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
    load(&mem, &file, EM_RISCV).unwrap();
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
        lui(10, 0),
        "the first segment's code"
    );
    mem.read_bytes(BASE + 0xf00, &mut buf).unwrap();
    assert_eq!(buf, [0x11; 4], "the second segment's data");
}

#[test]
fn at_phdr_points_at_the_program_headers_in_guest_memory() {
    let mem = UserMemory::new(48);
    let file = hello_elf(b"x");
    let image = load(&mem, &file, EM_RISCV).unwrap();
    assert_eq!(image.phdr, BASE + 64);
    assert_eq!(image.phent, 56);
    assert_eq!(image.phnum, 1);
    // The bytes at that guest address are the program header table itself —
    // the check that catches a malformed auxv before a guest faults on it.
    let mut buf = [0u8; 56];
    mem.read_bytes(image.phdr, &mut buf).unwrap();
    assert_eq!(&buf[..], &file[64..64 + 56]);
}

#[test]
fn a_hostile_or_wrong_image_is_refused_rather_than_mapped() {
    let mem = || UserMemory::new(48);
    let ok = hello_elf(b"x");

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
            f[18..20].copy_from_slice(&62u16.to_le_bytes()); // EM_X86_64
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
        let err = load(&m, bytes, EM_RISCV).expect_err(&format!("{what} should have been refused"));
        assert!(err.starts_with("ELF:"), "{what}: {err}");
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

/// A kernel over a scratch map, for the calls that need no guest.
fn scratch_kernel() -> (Arc<UserMemory>, Kernel) {
    let mem = Arc::new(UserMemory::new(48));
    mem.map_at(0x1000, 0x1000, Prot::RW, "scratch").unwrap();
    let kernel = Kernel::new(
        Arc::clone(&mem),
        Arc::new(GuestClock::new()),
        Arc::new(Journal::new()),
        counting_entropy(),
        0x10_0000,
    );
    (mem, kernel)
}

/// A hart parked at nothing, used only as a register file to hand the syscall
/// table its arguments — which is exactly what it is to a syscall kernel.
fn arg_hart(nr: u64, args: &[u64]) -> Arc<Hart> {
    let hart = Arc::new(Hart::new(Config {
        pmp_count: 0,
        ..Config::rv64gc()
    }));
    hart.set_x(17, nr);
    for (i, a) in args.iter().enumerate() {
        hart.set_x(10 + i as u32, *a);
    }
    hart
}

const ENOENT: u64 = -2i64 as u64;

#[test]
fn no_host_path_resolves_however_it_is_asked_for() {
    // The policy in one test: there is no host filesystem, so the answer does
    // not depend on the path, the directory fd, or the flags — and the paths
    // below are the ones a guest would try if it were looking.
    let (mem, mut kernel) = scratch_kernel();
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
            let hart = arg_hart(nr, &[u64::MAX, 0x1000, 0, 0]);
            assert!(kernel.service(&hart).is_none());
            assert_eq!(hart.x(10), ENOENT, "{nr} on {path:?}");
        }
    }
    // A path pointer the guest made up is refused rather than followed.
    let hart = arg_hart(nr::OPENAT, &[u64::MAX, 0xdead_0000, 0, 0]);
    assert!(kernel.service(&hart).is_none());
    assert_eq!(hart.x(10), ENOENT);
    assert!(kernel.files.iter().all(Option::is_none));
}

#[test]
fn the_one_openable_path_describes_the_guest_and_not_the_host() {
    // `/proc/self/maps`, served out of `UserMemory::mappings` — the reason
    // that method exists, and the whole of what a level-3 guest may be told.
    let (mem, mut kernel) = scratch_kernel();
    mem.map_at(0x50_0000, 0x1000, Prot::RX, "elf").unwrap();
    mem.map_at(0x60_0000, 0x2000, Prot::RW, "[stack]").unwrap();
    mem.write_bytes(0x1000, b"/proc/self/maps\0").unwrap();

    let hart = arg_hart(nr::OPENAT, &[u64::MAX, 0x1000, 0, 0]);
    assert!(kernel.service(&hart).is_none());
    let fd = hart.x(10);
    assert_eq!(fd, 3, "the first descriptor above the standard three");

    // Read it the way stdio would, in pieces.
    let mut text = Vec::new();
    loop {
        let hart = arg_hart(nr::READ, &[fd, 0x1000, 64]);
        assert!(kernel.service(&hart).is_none());
        let n = hart.x(10) as i64;
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

    let hart = arg_hart(nr::CLOSE, &[fd]);
    assert!(kernel.service(&hart).is_none());
    assert_eq!(hart.x(10), 0);
    // And it is gone: a closed descriptor is not a descriptor.
    let hart = arg_hart(nr::READ, &[fd, 0x1000, 1]);
    assert!(kernel.service(&hart).is_none());
    assert_eq!(hart.x(10), -9i64 as u64, "EBADF");
}

#[test]
fn an_alternate_stack_query_says_there_is_none() {
    // The stub that returned zero and wrote nothing was worse than wrong: a
    // caller reads its own uninitialised buffer as "an alternate stack is
    // already installed" and skips installing one. Comparing the emulated
    // trace against the same program's native trace is what found it, and
    // this is the assertion that keeps it found.
    let (mem, mut kernel) = scratch_kernel();
    mem.write_bytes(0x1000, &[0xff; 24]).unwrap();
    let hart = arg_hart(nr::SIGALTSTACK, &[0, 0x1000]);
    assert!(kernel.service(&hart).is_none());
    assert_eq!(hart.x(10), 0);
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
    let hart = arg_hart(nr::SIGALTSTACK, &[0x1000, 0]);
    assert!(kernel.service(&hart).is_none());
    let hart = arg_hart(nr::SIGALTSTACK, &[0, 0x1000]);
    assert!(kernel.service(&hart).is_none());
    mem.read_bytes(0x1000, &mut out).unwrap();
    assert_eq!(u64::from_le_bytes(out[..8].try_into().unwrap()), 0x4000);
    assert_eq!(u64::from_le_bytes(out[16..24].try_into().unwrap()), 0x2000);
    assert_eq!(u32::from_le_bytes(out[8..12].try_into().unwrap()), 0);
}

#[test]
fn a_file_backed_mapping_is_refused_because_there_are_no_files() {
    let (_mem, mut kernel) = scratch_kernel();
    const ENODEV: u64 = -19i64 as u64;
    // MAP_PRIVATE with a descriptor, and MAP_ANONYMOUS with one: both are a
    // guest trying to map something that is not its own memory.
    for (flags, fd) in [(0x02u64, 3u64), (0x22, 3)] {
        let hart = arg_hart(nr::MMAP, &[0, 0x1000, 3, flags, fd, 0]);
        assert!(kernel.service(&hart).is_none());
        assert_eq!(hart.x(10), ENODEV);
    }
    // Anonymous, with the -1 every libc passes, is granted.
    let hart = arg_hart(nr::MMAP, &[0, 0x2000, 3, 0x22, u64::MAX, 0]);
    assert!(kernel.service(&hart).is_none());
    let base = hart.x(10);
    assert!((base as i64) > 0, "mmap returned {:#x}", base);
    assert_eq!(kernel.mem.mapping_at(base).unwrap().prot, Prot::RW);
}

#[test]
fn only_the_three_standard_descriptors_exist() {
    let (mem, mut kernel) = scratch_kernel();
    mem.write_bytes(0x1000, b"nope").unwrap();
    const EBADF: u64 = -9i64 as u64;
    for fd in [3u64, 42, u64::MAX] {
        let hart = arg_hart(nr::WRITE, &[fd, 0x1000, 4]);
        assert!(kernel.service(&hart).is_none());
        assert_eq!(hart.x(10), EBADF, "fd {fd}");
    }
    // fd 0 is at end of file rather than an error: a program that reads
    // standard input gets a clean EOF, not a mystery.
    let hart = arg_hart(nr::READ, &[0, 0x1000, 4]);
    assert!(kernel.service(&hart).is_none());
    assert_eq!(hart.x(10), 0);
    assert!(kernel.stdout.is_empty() && kernel.stderr.is_empty());
}

#[test]
fn the_heap_grows_and_shrinks_through_brk() {
    let (mem, mut kernel) = scratch_kernel();
    let base = 0x10_0000u64;
    let ask = |kernel: &mut Kernel, want: u64| {
        let hart = arg_hart(nr::BRK, &[want]);
        kernel.service(&hart);
        hart.x(10)
    };
    assert_eq!(ask(&mut kernel, 0), base, "brk(0) reports where it is");
    assert_eq!(ask(&mut kernel, base + 0x2800), base + 0x2800);
    // The guest can now use every byte it asked for.
    mem.write_bytes(base + 0x27ff, b"!").unwrap();
    assert!(mem.write_bytes(base + 0x3000, b"!").is_err());
    // And giving it back unmaps the pages that are wholly above the new break.
    assert_eq!(ask(&mut kernel, base), base);
    assert!(mem.write_bytes(base + 0x27ff, b"!").is_err());
}

// ---------------------------------------------------------------------------
// Determinism: the two doors entropy comes through
// ---------------------------------------------------------------------------

#[test]
fn at_random_and_getrandom_both_go_through_the_journal() {
    // A guest that asks for eight bytes and writes them to fd 1, so both
    // doors are visible in one run: `AT_RANDOM` while the stack is built, and
    // `getrandom` from the program.
    let buf = 0x2_0000u64;
    let text = 64 + 56 * 2;
    let code = {
        let mut c = Vec::new();
        c.extend(li(10, buf));
        c.extend(li(11, 8));
        c.extend(li(12, 0));
        c.extend(li(17, 278)); // __NR_getrandom
        c.push(ECALL);
        c.extend(li(10, 1));
        c.extend(li(11, buf));
        c.extend(li(12, 8));
        c.extend(li(17, 64)); // __NR_write
        c.push(ECALL);
        c.extend(li(10, 0));
        c.extend(li(17, 94));
        c.push(ECALL);
        c
    };
    let file = elf64(
        BASE + text,
        EM_RISCV,
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

    // Record, with an entropy source that is *not* a function of the program.
    let recording = Arc::new(Journal::with_mode(JournalMode::Record));
    let mut counter = 0u8;
    let live = run(
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
    .unwrap();
    assert_eq!(live.stdout, vec![2u8; 8]);
    assert_eq!(live.random, vec![1u8; 16]);
    assert_eq!(recording.len(), 2, "AT_RANDOM and one getrandom");

    // Replay, with a source that panics. Nothing else in the run may reach
    // the host, and the output is identical.
    recording.set_mode(JournalMode::Replay);
    let replayed = run(
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

#[test]
fn a_run_is_a_function_of_the_program_alone() {
    // Two runs, two everythings, and the whole trace agrees — including the
    // virtual clock, which is what `clock_gettime` would have answered from.
    let file = hello_elf(b"deterministic\n");
    let first = run_synthetic(&file).unwrap();
    let second = run_synthetic(&file).unwrap();
    assert_eq!(first.stdout, second.stdout);
    assert_eq!(first.trace, second.trace);
    assert_eq!(first.ticks, second.ticks);
    assert!(first.ticks > 0);
}

// ---------------------------------------------------------------------------
// The milestone: a real statically linked Linux binary
// ---------------------------------------------------------------------------

/// Where `scripts/fetch-testdata.sh usermode-guests` puts what it builds, and
/// the variable that overrides it.
///
/// The corpus rule applies even though this one is *built* rather than
/// downloaded (CLAUDE.md, Testing): a compiler's output is not a source file
/// and does not belong in the repository, so without the fixture the test says
/// what to run and passes. Everything above this line runs unconditionally.
#[cfg(feature = "std")]
#[test]
fn a_real_static_linux_binary_prints_and_exits_zero() {
    let Some(bytes) = guest_binary("hello-riscv64") else {
        std::eprintln!(
            "usermode: no guest binary. Build one with\n    \
             scripts/fetch-testdata.sh usermode-guests\n  \
             or point RSEMU_USERMODE_GUEST at a static riscv64 ELF."
        );
        return;
    };

    let journal = Arc::new(Journal::with_mode(JournalMode::Record));
    let out = run(
        &bytes,
        &["hello"],
        &["RSEMU=1"],
        Arc::clone(&journal),
        counting_entropy(),
        2_000_000_000,
    )
    .expect("the guest ran");

    std::eprintln!(
        "usermode: {} syscall(s), {} tick(s); refused {:?}",
        out.trace.len(),
        out.ticks,
        out.refused
    );
    std::eprintln!("usermode: TRACE {:?}", out.trace);
    let stdout = String::from_utf8_lossy(&out.stdout).into_owned();
    std::eprintln!("usermode: stdout {stdout:?}");
    std::eprintln!(
        "usermode: stderr {:?}",
        String::from_utf8_lossy(&out.stderr)
    );

    assert_eq!(out.status, 0, "the guest exited {}", out.status);
    assert!(out.stderr.is_empty(), "the guest wrote to fd 2");
    assert!(out.refused.is_empty(), "refused {:?}", out.refused);
    assert_eq!(
        stdout, "hello from level 3\nargv = [\"hello\"]\nRSEMU = Some(\"1\")\n",
        "the guest read its own argv and environment off the stack the loader \
         built, so a stack that is subtly wrong fails here rather than later"
    );

    // The whole run replays with the host unplugged.
    journal.set_mode(JournalMode::Replay);
    let replayed = run(
        &bytes,
        &["hello"],
        &["RSEMU=1"],
        Arc::clone(&journal),
        Kernel::replay_guard(),
        2_000_000_000,
    )
    .expect("the guest replayed");
    assert_eq!(replayed.stdout, out.stdout);
    assert_eq!(replayed.ticks, out.ticks);
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
