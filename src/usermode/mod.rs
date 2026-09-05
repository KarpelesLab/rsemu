//! Level-3 execution: running a program with no guest kernel under it.
//!
//! `ROADMAP.md` §2 describes three depths a guest can be fed to rsemu at.
//! Level 1 emulates a machine, level 2 skips its firmware, and **level 3 has no
//! guest kernel at all**: the guest's `ecall` / `syscall` / `svc` does not
//! vector to a handler inside the guest, it leaves the core, and something in
//! Rust services it. There is no interrupt controller, no timer chip and no
//! block device, because nothing in the guest can see one.
//!
//! # Where the line is
//!
//! §2.1 settles which crate owns which half, with one test — *is it hardware?*
//! rsemu supplies the machine; [`KarpelesLab/nixvm`] supplies the Linux kernel
//! that runs on it and depends on this crate for the rest. So **this module
//! contains no syscall table, no file descriptors, no errno, no `procfs`, no
//! ELF loader and no process model.** It contains the four things a consumer
//! cannot build for itself:
//!
//! | | |
//! | --- | --- |
//! | [`core::exec`](crate::core::exec) | a core that stops *at* a syscall instruction and says so |
//! | [`mem`] | a memory map with no devices in it |
//! | [`sched`] | when guest threads run, and for how long |
//! | [`journal`] | the one door non-deterministic answers come through |
//!
//! Everything here is public, documented, and shaped to be driven from another
//! crate. That is not a nice property of this module, it is the point of it:
//! §2.1 calls a downstream consumer *"the strongest available test"* of §2's
//! claim that embedding rsemu is a supported use rather than a fork.
//!
//! [`KarpelesLab/nixvm`]: https://github.com/KarpelesLab/nixvm
//!
//! # Determinism is designed in, not added
//!
//! A syscall's result crossing into the guest is §0's *"non-deterministic input
//! crossing into the machine"*, and phase 5b makes that a gate rather than an
//! aspiration. Three decisions here discharge it, and they reinforce each
//! other:
//!
//! * **Time is virtual.** [`GuestClock`] advances by executed ticks, so a
//!   consumer's `clock_gettime` is a function of the program. There is nothing
//!   to record because there is nothing non-deterministic left.
//! * **Preemption is in ticks.** [`ThreadSet`] gives each thread a tick
//!   quantum, so the interleaving is a function of the program too. A
//!   wall-clock quantum — the obvious implementation — makes instruction counts
//!   depend on how busy the host was, and quietly destroys replayability.
//! * **Everything else goes through [`Journal::ask`].** One funnel, three
//!   modes, and a replay that reports the first divergence instead of
//!   producing wrong output.
//!
//! # What that adds up to, measured
//!
//! The four pieces above are enough to run **real Linux binaries, statically
//! and dynamically linked, on two architectures**.
//!
//! An ordinary `std` Rust `hello world` starts through `musl`'s
//! `__libc_start_main`, finds its own program headers through the auxiliary
//! vector, sets up thread-local storage, installs a stack-overflow handler,
//! sizes a heap with `brk`, prints, and exits zero — twenty-five syscalls,
//! matching the same program's `strace` on the host one for one. An ordinary
//! `std` Rust program that spawns four threads, contends an atomic, joins
//! them, and then blocks three more on a condition variable does 166 calls
//! against the host's 168, and every difference is a `futex` retry a genuinely
//! parallel host made and this one did not.
//!
//! Both run identically on `riscv64gc-unknown-linux-musl` and
//! `aarch64-unknown-linux-musl`, and the second architecture needed **no new
//! syscall, no loader change and no policy change** — only five register
//! numbers and the unprivileged state a kernel would have established. §2.1's
//! claim that a syscall exit is *"a property of a core"* rather than a
//! property of RISC-V is now measured.
//!
//! A **dynamically linked** program runs too, under a real `ld.so` taken from
//! the host: an `ET_DYN` executable with a `PT_INTERP`, one `DT_NEEDED`, a
//! data relocation and a function relocation, in twenty-two syscalls, with the
//! loader opening the library by path, mapping its segments out of a
//! descriptor, trimming them to their 64 KiB alignment and resolving both
//! relocations. Nothing here processes a relocation; the consumer places two
//! images and builds an auxiliary vector that describes each to the other,
//! which is the whole of what a kernel does for a dynamically linked process.
//!
//! That needed the one policy change level 3 has had. A dynamic loader opens
//! files, so *"the guest may be told about itself"* became *"the guest may be
//! told about itself and about what it was handed"* — a set of `(guest path,
//! bytes)` fixed before the first instruction. What is unchanged is the reason
//! the rule existed: the syscall kernel still does not link `std`, so there is
//! no code path from a guest pointer to a host path. `docs/system/usermode-abi.md`
//! argues it.
//!
//! None of the code that does that is in this module, and that is the result.
//! The ELF loader, the interpreter, the sandbox policy, the syscall table, the
//! descriptors, the errno values and
//! the process model live in `src/usermode/proof.rs`, which is `#[cfg(test)]`
//! and is the *consumer's* half written out longhand — §2.1's line, held, with
//! working programs on the far side of it. `docs/system/usermode-abi.md` has
//! the ABI sources, the host-filesystem policy, the trace comparisons and the
//! one thing this exercise found that is **not** on the consumer's side: the
//! exclusive monitor is per core, so two guest threads' `lr`/`sc` pairs are
//! not coherent with each other.
//!
//! # Driving it
//!
//! The loop is the consumer's; rsemu is pulled from, never called back into.
//!
//! ```no_run
//! use alloc::sync::Arc;
//! # extern crate alloc;
//! use rsemu::core::exec::{ExitMask, ExitReason, ExitingCore};
//! use rsemu::usermode::{GuestClock, Prot, ThreadSet, UserMemory};
//!
//! # fn demo(core: Arc<dyn ExitingCore>) -> rsemu::Result<()> {
//! let mem = Arc::new(UserMemory::new(48));
//! mem.map_at(0x1000, 0x1000, Prot::RX, "text")?;
//!
//! core.set_exit_mask(ExitMask::USER);
//! let threads = ThreadSet::new(Arc::new(GuestClock::new()));
//! let id = threads.insert(core);
//!
//! while let Some(stop) = threads.run_next() {
//!     match stop.exit.map(|e| e.reason) {
//!         Some(ExitReason::SYSCALL) => { /* service it, write the result register */ }
//!         Some(ExitReason::FAULT) => { /* map a page, or deliver a signal */ }
//!         _ => {} // the quantum ended; go round again
//!     }
//! }
//! # let _ = id;
//! # Ok(())
//! # }
//! ```

pub mod clock;
pub mod journal;
pub mod mem;
pub mod sched;

/// The consumer's half, written out longhand so rsemu's half can be proven
/// against a real binary. `#[cfg(test)]` and staying that way: §2.1 puts a
/// syscall table, an ELF loader and a process model in the *consumer*, and
/// this module exists to demonstrate that line is holdable, not to cross it.
#[cfg(all(test, any(feature = "cpu-riscv", feature = "cpu-arm-a64")))]
mod proof;
#[cfg(test)]
mod tests;

pub use clock::{DEFAULT_HZ, GuestClock};
pub use journal::{Answer, Journal, JournalMode, Tag};
pub use mem::{MappingInfo, PAGE_SIZE, Prot, UserMemory};
pub use sched::{DEFAULT_QUANTUM, Stop, ThreadId, ThreadSet, ThreadState};
