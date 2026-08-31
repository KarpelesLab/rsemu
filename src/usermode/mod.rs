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

#[cfg(test)]
mod tests;

pub use clock::{DEFAULT_HZ, GuestClock};
pub use journal::{Answer, Journal, JournalMode, Tag};
pub use mem::{MappingInfo, PAGE_SIZE, Prot, UserMemory};
pub use sched::{DEFAULT_QUANTUM, Stop, ThreadId, ThreadSet, ThreadState};
