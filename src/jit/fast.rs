//! The seam a host offers a code generator, so its loads and stores need no
//! call into it.
//!
//! `ROADMAP.md` §9.1's first mechanism is the software TLB, and the sentence
//! that matters is *"the fast path is inlined into generated code: mask,
//! compare, add, load"*. [`Tlb`](crate::jit::Tlb) is the table; this is how a
//! backend is allowed to read it.
//!
//! # Why the host publishes its own TLB rather than the backend owning one
//!
//! A guest access is not just a lookup. It is a lookup *plus* whatever the
//! core does around it: the guest MMU's own translation, the split a
//! misaligned access becomes, the tick each bus cycle costs, the segment base
//! x86 adds first. All of that is the host's, and `IrHost::load` is where it
//! lives. A backend that kept a second table would have to reproduce every one
//! of those rules to keep a hit and a miss indistinguishable — which is
//! exactly the property `ROADMAP.md` §0 requires of the JIT against the
//! interpreter.
//!
//! So the split is: the host says *here is the table my loads resolve
//! through, and here is what one aligned access costs me*, and the backend
//! inlines only the case it can prove identical — an aligned, in-page load of
//! at most eight bytes, unsegmented, in the ordinary memory space, hitting an
//! entry that already resolves to plain little-endian RAM. Everything else
//! calls [`IrHost::load`](crate::ir::IrHost::load) and gets the host's answer,
//! including the fill that makes the *next* access fast.
//!
//! # What a host with a *guest* MMU owes on top of that
//!
//! A bare-mode host publishes a plan and is done: an aligned load of plain RAM
//! costs one bus cycle whether it was cached or not. A **paged** host does
//! not, and the reason this seam went unimplemented on the one core that
//! matters for a year is worth stating, because the obvious reading of it is
//! wrong.
//!
//! The obvious reading is that a walk cannot be skipped per access, so a paged
//! host can never publish. But the plan is not per access — it is **per page**,
//! and the entry is written by the miss that walked. The condition is
//! therefore not *"no walk is owed"* but *"a hit here implies a hit in the
//! table that owes the walk"*, and a host makes that true by writing this
//! table in lockstep with its own translation cache: same index, same page,
//! same moment, so an eviction there is an eviction here. `cpu::riscv::mmu`'s
//! `Tlb::attach_shadow` is that, and the tick
//! [`FastMem::note_fast_load`] charges is then the whole of what an inlined
//! load still owes.
//!
//! Two more things belong to the host rather than to this table, and both are
//! silent when they are wrong: a **protection check the topology knows nothing
//! about** (RISC-V's PMP, whose answer may differ within one page — see
//! `mmu::pmp_page_uniform`), and the guest's own **fence**, which is
//! [`Epoch::translation`](crate::jit::Epoch::translation) and rides in the tag
//! rather than costing a flush.
//!
//! # Not implementing this is the default
//!
//! Every method is defaulted, so a host that has no TLB — a `no_std` board, a
//! differential harness with a raw address space, a device test — writes
//! `impl FastMem for MyHost {}` and every access takes the call. That is the
//! honest default: a host that published a table it did not actually use for
//! its own loads would make compiled and interpreted execution disagree, which
//! is the one thing this crate does not tolerate.

use crate::jit::tlb::FastSet;

/// The parts of a host's memory path a backend may inline, for one access
/// type.
///
/// One type for loads and for stores because it is one *encoding* — a set and
/// the tag bits a hit compares against — and two copies of an encoding are two
/// chances for them to disagree. What differs between the two is the promise
/// the host makes by publishing it, and that is written on
/// [`FastMem::load_plan`] and [`FastMem::store_plan`] rather than in the type.
#[derive(Debug, Clone, Copy)]
pub struct MemPlan {
    /// The set of the software TLB the accesses this plan covers resolve
    /// through.
    ///
    /// Valid for as long as the borrow it came from, and until the TLB is
    /// flushed. A flush comes from [`Tlb::sync`](crate::jit::Tlb::sync), which
    /// a host calls at a block boundary — never inside one — so a plan taken
    /// at the top of a block stays good for that block.
    pub set: FastSet,
    /// Everything a hit's tag carries besides the page number: the world those
    /// accesses happen in, and the stamp of the guest MMU's generation.
    ///
    /// [`Tlb::tag_bits`](crate::jit::Tlb::tag_bits) is what produces it, and a
    /// backend must load it per block rather than bake it in: the stamp moves
    /// every time the guest fences its translations.
    pub tag: u64,
}

/// A host whose accesses a backend may serve without calling it.
///
/// Every method is defaulted to *no*, which is always correct — and the
/// defaults are how the one dangerous combination arises, so it is named here
/// rather than left to be discovered: a host that returns `Some` from
/// [`FastMem::store_plan`] and does **not** override
/// [`FastMem::note_fast_store`] gets a no-op, and loses the tick, the store's
/// dirty bitmap and the self-modifying-code check on every inlined store. The
/// two are one decision. A host implements both or neither.
pub trait FastMem {
    /// The table this host's loads resolve through, if a backend may use it.
    ///
    /// Returning `Some` is a promise: that a load this host performs at an
    /// aligned address, in the ordinary memory space, with no segment, resolves
    /// through exactly this set under exactly this context, and that a hit on
    /// an entry carrying a host address produces the bytes at that address.
    /// Breaking it makes compiled and interpreted execution disagree about
    /// guest memory, which no test in this crate will forgive.
    fn load_plan(&mut self) -> Option<MemPlan> {
        None
    }

    /// The table this host's **stores** resolve through, if a backend may
    /// write guest memory through it.
    ///
    /// A far stronger promise than [`FastMem::load_plan`], and every clause is
    /// something an inlined store would otherwise skip in silence:
    ///
    /// * the entry was admitted on
    ///   [`Perms::WRITE`](crate::core::space::Perms::WRITE), and on the guest
    ///   MMU's own **write** permission — a different bit from the read one, in
    ///   a different set;
    /// * whatever the architecture owes a *first* write to a page has already
    ///   been paid. On RISC-V that is the PTE's dirty bit, and the reason a
    ///   store plan can promise it is that a store entry only exists because a
    ///   walk **for a store** filled it, and that walk set `D`;
    /// * writing through the entry's host address is allowed *provided* the
    ///   caller then reports the write, so publishing this obliges a host to
    ///   implement [`FastMem::note_fast_store`] as well.
    ///   `RamStore::host_ptr` is documented read-only precisely because a
    ///   write through it skips the store's own dirty bitmap and the block
    ///   cache's self-modifying-code check, and `note_fast_store` is where a
    ///   host pays both.
    ///
    /// Returning `None` — the default — means every store takes the call.
    fn store_plan(&mut self) -> Option<MemPlan> {
        None
    }

    /// One aligned store was served inline at guest address `addr`, `bytes`
    /// wide; do everything the host's own store path does apart from moving
    /// the bytes.
    ///
    /// Three obligations, and a host that skips any of them is broken in a way
    /// no unit test finds:
    ///
    /// * **the tick**, exactly as [`FastMem::note_fast_load`] charges one;
    /// * **the store's dirty bitmap**, which
    ///   [`RamStore::write_at`](crate::core::space::RamStore::write_at) would
    ///   have marked and a host pointer does not — the only record a
    ///   framebuffer refresh or a live snapshot has (`ROADMAP.md` §4.1);
    /// * **the guest-physical dirty log**
    ///   ([`StoreLog`](crate::jit::StoreLog)), which the dispatcher drains at
    ///   the next block boundary to invalidate translations of the page just
    ///   written. A guest that writes its own code — Linux does, in module
    ///   loading and in alternatives patching — depends on it.
    ///
    /// Anything else the host's store path does that the guest can observe
    /// belongs here too. On RISC-V that is the reservation: a store into the
    /// reserved range breaks it, and an `sc` that succeeded because an inlined
    /// store forgot to would be a lost update in guest software.
    ///
    /// Called *after* the bytes have landed, which is unobservable: nothing
    /// runs in between, and the fast path cannot fault.
    fn note_fast_store(&mut self, addr: u64, bytes: u64) {
        let _ = (addr, bytes);
    }

    /// One aligned load was served inline; charge for it.
    ///
    /// Called once per inlined access, in place of everything
    /// [`IrHost::load`](crate::ir::IrHost::load) would have done, so this must
    /// account for exactly what that path accounts for and no more: on the
    /// RISC-V harness one tick, because one bus access is one cycle; on the
    /// x86 one the bus-clock count. Getting it wrong is a cycle-counter
    /// divergence, which `ROADMAP.md` §0 makes a state-hash divergence, which
    /// the differential harnesses fail on.
    fn note_fast_load(&mut self) {}
}
