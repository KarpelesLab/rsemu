//! The seam a host offers a code generator, so its loads need no call.
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
//! Both methods are defaulted, so a host that has no TLB — a `no_std` board, a
//! differential harness with a raw address space, a device test — writes
//! `impl FastMem for MyHost {}` and every load takes the call. That is the
//! honest default: a host that published a table it did not actually use for
//! its own loads would make compiled and interpreted execution disagree, which
//! is the one thing this crate does not tolerate.

use crate::jit::tlb::FastSet;

/// The parts of a host's memory path a backend may inline.
#[derive(Debug, Clone, Copy)]
pub struct LoadPlan {
    /// The load set of the software TLB this host's loads resolve through.
    ///
    /// Valid for as long as the borrow it came from, and until the TLB is
    /// flushed. A flush comes from [`Tlb::sync`](crate::jit::Tlb::sync), which
    /// a host calls at a block boundary — never inside one — so a plan taken
    /// at the top of a block stays good for that block.
    pub set: FastSet,
    /// Everything a hit's tag carries besides the page number: the world those
    /// loads happen in, and the stamp of the guest MMU's generation.
    ///
    /// [`Tlb::tag_bits`](crate::jit::Tlb::tag_bits) is what produces it, and a
    /// backend must load it per block rather than bake it in: the stamp moves
    /// every time the guest fences its translations.
    pub tag: u64,
}

/// A host whose loads a backend may serve without calling it.
///
/// Every method is defaulted to *no*, which is always correct.
pub trait FastMem {
    /// The table this host's loads resolve through, if a backend may use it.
    ///
    /// Returning `Some` is a promise: that a load this host performs at an
    /// aligned address, in the ordinary memory space, with no segment, resolves
    /// through exactly this set under exactly this context, and that a hit on
    /// an entry carrying a host address produces the bytes at that address.
    /// Breaking it makes compiled and interpreted execution disagree about
    /// guest memory, which no test in this crate will forgive.
    fn load_plan(&mut self) -> Option<LoadPlan> {
        None
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
