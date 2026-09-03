//! The dispatcher: the loop that keeps a guest inside translated code.
//!
//! Lift on a miss, cache under `(pc, key)`, patch the exit to its successor,
//! and go round again without leaving for the interpreter. This is where the
//! three mechanisms in [`jit`](super) meet, and it is deliberately the only
//! place that knows the order they run in.
//!
//! # What is generic, and what a guest supplies
//!
//! Nothing here knows what a RISC-V is. A guest supplies a [`Frontend`] —
//! which world it is in ([`Frontend::key`], [`Frontend::epoch`]), how to lift
//! one block ([`Frontend::translate`]), what a block owes before it runs
//! ([`Frontend::enter`]), and which slot the guest PC lands in at a block exit
//! ([`Frontend::pc_slot`]) — and an [`IrHost`](crate::ir::IrHost) that also
//! implements [`StoreLog`], so guest writes can be matched against cached
//! translations.
//!
//! [`Frontend::enter`] is the one that makes chaining reachable, and it is
//! worth saying why a *cache* needed a hook at all. Following a patched exit
//! skips the hash lookup, which was never the expensive part; what it really
//! skips is everything a caller does *between* two `Dispatcher::run` calls.
//! But a guest whose instruction fetch translates owes that translation on
//! every block execution — a cached block that skipped it would cost fewer
//! ticks than the uncached one it replaced — so before this hook existed the
//! only honest budget was one block, and `DispatchStats::chained` was zero in
//! every run of a real guest. Now it is called once per block iteration, the
//! chained ones included.
//!
//! # Why self-modifying code is reported rather than intercepted
//!
//! A guest store goes through [`IrHost::store`](crate::ir::IrHost::store),
//! which the dispatcher never sees, and putting the block cache behind a lock
//! so the store path could reach it would put a lock on the one path that
//! cannot afford one. So a host **accumulates** the guest-physical pages it
//! wrote ([`DirtyPages`] is a ready-made accumulator) and the dispatcher
//! drains them at each block boundary.
//!
//! Draining at a boundary rather than at the store is the granularity RISC-V
//! asks for, and now the *only* one available: the ISA requires a `FENCE.I`
//! between a store to instruction memory and executing it, so a store's effect
//! on **later** blocks is all it promises. That used to be belt and braces —
//! the lifter ended a block at its first access, so nothing after a store in
//! the same block existed to be modified — and superblocks spend the braces: a
//! trace runs to its end on the bytes it was lifted from, and a store it made
//! into its own page invalidates it for the *next* execution. A guest that
//! wants otherwise owes a `FENCE.I`. An x86 frontend needs the check *within*
//! a block — x86 makes coherent instruction caches architectural — and will
//! need a finer hook than this one; that is recorded here rather than
//! discovered later.
//!
//! # Safe points
//!
//! A [`Dispatcher`] carrying an [`ExitFlag`] tests it at each block boundary
//! and stops with [`Stop::Exit`]. That is §4.7's protocol exactly: a
//! generation counter plus a per-CPU flag checked at block boundaries, never a
//! signal, because wasm has none.
//!
//! A trace has *fewer* boundaries than the basic blocks it replaces, so the
//! delay before a raised flag is honoured is bounded by a frontend's own
//! instruction limit rather than by a basic block's length — sixty-four guest
//! instructions for the RISC-V frontend. That is the price of merging, it is
//! bounded, and it is checked by
//! `a_raised_exit_flag_stops_within_one_block_however_long_the_block_is`.

use alloc::vec::Vec;

use crate::core::error::Result;
use crate::core::sched::ExitFlag;
use crate::ir::{Block, Fault, Interp, IrHost, Opcode, Outcome, RegSlot};
use crate::jit::cache::{BlockCache, BlockId, CacheStats};
use crate::jit::fast::FastMem;
use crate::jit::tlb::{Epoch, PAGE_MASK, PAGE_SIZE};

#[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
use crate::jit::x86::Engine;

/// One freshly lifted block, and what the cache needs to know about it.
#[derive(Debug)]
pub struct Translation {
    /// The block.
    pub block: Block,
    /// The guest-**physical** page its bytes were read from.
    ///
    /// Physical, not virtual: a guest write is matched against this, and a
    /// write arrives at a physical address. A block never leaves the page it
    /// started on, so one page is the whole answer.
    pub page: u64,
    /// How many guest instructions the block covers.
    ///
    /// Zero means the frontend could not lift the instruction at the entry PC,
    /// and the dispatcher stops with [`Stop::Untranslatable`] rather than
    /// spinning on a block that cannot advance the PC.
    ///
    /// A **static** count, and not the one [`Run::insns`] reports: a
    /// superblock covers every instruction on the path it inlined, and a run
    /// that leaves through a side exit retires fewer of them. What retired is
    /// counted by [`Interp::boundaries`](crate::ir::Interp::boundaries).
    pub insns: usize,
}

/// What a frontend says when a block boundary is reached.
///
/// The answer to [`Frontend::enter`]. `Leave` is not an error and not a fault:
/// it is a guest whose *next* block should not run — the instruction at that
/// PC is outside the lifted subset, or the block's worst case does not fit
/// what is left of the caller's budget, or entering it trapped. The dispatcher
/// stops with [`Stop::Declined`], and what that means is the frontend's own
/// business; for a CPU core it means its own interpreter takes over.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Entry {
    /// Execute the block at this PC.
    Ready,
    /// Do not. Stop the run here, with the guest at this PC.
    Leave,
}

/// What a dispatcher needs from a guest.
///
/// Generic over the host, and *only* so that [`Frontend::enter`] can be handed
/// it. Entering a block is a hart action — it translates the entry fetch, it
/// may walk a page table, it charges for the walk and it can trap — and the
/// state all four of those need belongs to the [`IrHost`], not to the lifter.
/// Every other method ignores the parameter, which is why a frontend with
/// nothing to do at a boundary reads `impl<H> Frontend<H> for …`.
pub trait Frontend<H: ?Sized> {
    /// The counters this guest's translations are stale against.
    ///
    /// Read at **every** block boundary, not once per [`Dispatcher::run`], so
    /// a stop-the-world retopology is observed before the next block rather
    /// than after the chain that followed it. It is therefore on the hot path:
    /// answer it with an atomic load, not with a lock or a walk.
    fn epoch(&mut self) -> Epoch;

    /// Everything a block owes *before* it runs, on every execution.
    ///
    /// Called once per block iteration — including the chained ones, which is
    /// the whole reason it exists. A translated block skips its own entry
    /// fetch, but a guest whose fetch *translates* still owes that translation
    /// every time the block runs: a cached block that skipped it would cost
    /// fewer ticks than the uncached one it replaced, and the two engines
    /// would stop agreeing on the cycle counter (`ROADMAP.md` §0). Without a
    /// hook here a dispatcher can only be driven one block at a time, which is
    /// what gave up §9's second mechanism — `chained: 0` in every run of a
    /// real guest, on the one machine the JIT exists for.
    ///
    /// It runs **after** the budget and safe-point checks and **before**
    /// [`Frontend::key`], so a frontend may compute the key here out of what
    /// the entry resolved to. The RISC-V engine does exactly that: a block is
    /// keyed on the physical page its entry translation just produced.
    ///
    /// The default is [`Entry::Ready`]. A guest with nothing to do at a
    /// boundary says so by not implementing this.
    ///
    /// # Errors
    ///
    /// Whatever the frontend says. A dispatcher does not try to recover — an
    /// ordinary guest condition is [`Entry::Leave`] rather than an error.
    fn enter(&mut self, pc: u64, host: &mut H) -> Result<Entry> {
        let _ = (pc, host);
        Ok(Entry::Ready)
    }

    /// The rest of the cache key beside the guest PC — the value the frontend
    /// puts in [`Block::key`](crate::ir::Block::key).
    fn key(&mut self) -> u64;

    /// The slot a block leaves the guest PC in at its exit boundary.
    fn pc_slot(&self) -> RegSlot;

    /// Lift the block at `pc`.
    ///
    /// # Errors
    ///
    /// Whatever the frontend says. A dispatcher does not try to recover.
    fn translate(&mut self, pc: u64) -> Result<Translation>;
}

/// A host that reports which guest-physical pages its stores touched.
///
/// The self-modifying-code half of the contract. A host that cannot write
/// guest memory implements this as an empty method.
pub trait StoreLog {
    /// Hand over the pages stored to since the last call, and forget them.
    fn drain_dirty(&mut self, sink: &mut dyn FnMut(u64));
}

/// A ready-made accumulator a host can embed to satisfy [`StoreLog`].
///
/// Records pages, not addresses, and de-duplicates against the most recent —
/// a guest memcpy walks one page for hundreds of stores, and a list with one
/// entry per store would be the expensive part of the mechanism.
#[derive(Debug, Clone, Default)]
pub struct DirtyPages {
    pages: Vec<u64>,
}

impl DirtyPages {
    /// An empty log.
    #[must_use]
    pub fn new() -> DirtyPages {
        DirtyPages::default()
    }

    /// Record a store of `len` bytes at guest-physical `phys`.
    #[inline]
    pub fn note(&mut self, phys: u64, len: u64) {
        if len == 0 {
            return;
        }
        let first = phys & !PAGE_MASK;
        let last = phys.saturating_add(len - 1) & !PAGE_MASK;
        let mut page = first;
        loop {
            if self.pages.last() != Some(&page) {
                self.pages.push(page);
            }
            if page >= last {
                break;
            }
            page = page.saturating_add(PAGE_SIZE);
        }
    }

    /// Whether anything has been recorded.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.pages.is_empty()
    }
}

impl StoreLog for DirtyPages {
    fn drain_dirty(&mut self, sink: &mut dyn FnMut(u64)) {
        for page in self.pages.drain(..) {
            sink(page);
        }
    }
}

/// Why a run stopped.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum Stop {
    /// The block budget ran out. The guest is mid-flight and the PC is live.
    Budget,
    /// The safe-point flag was raised (`ROADMAP.md` §4.7).
    Exit,
    /// A guest access faulted. The guest's own fault path takes it from here.
    Fault(Fault),
    /// The block reached an op this backend does not implement.
    Unsupported {
        /// The op.
        op: Opcode,
        /// Its index in the block.
        at: usize,
    },
    /// The frontend could not lift the instruction at this PC, so the guest's
    /// own interpreter has to execute it.
    Untranslatable {
        /// The guest PC.
        pc: u64,
    },
    /// [`Frontend::enter`] answered [`Entry::Leave`]: the block at
    /// [`Run::pc`] was not entered, and why is the frontend's own business.
    Declined,
}

/// What a run did.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Run {
    /// The guest PC to resume at.
    pub pc: u64,
    /// How many blocks executed.
    pub blocks: usize,
    /// How many guest instructions those blocks **retired**.
    ///
    /// Counted from the boundaries the backend actually passed, not summed
    /// from [`Translation::insns`]: a trace that leaves through a side exit
    /// retires fewer instructions than it covers, and a block that faulted
    /// retires everything before the faulting instruction and no more. A
    /// caller that steps an oracle this many times — the differential harness
    /// does — gets a wrong answer from the static number and a right one from
    /// this.
    pub insns: usize,
    /// Why it stopped.
    pub stop: Stop,
}

/// What a dispatcher has been asked to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DispatchStats {
    /// Blocks executed.
    pub blocks: u64,
    /// Blocks reached by following a patched exit.
    pub chained: u64,
    /// Blocks reached by a hash lookup.
    pub looked_up: u64,
    /// Blocks translated.
    pub translated: u64,
    /// Blocks invalidated by a guest store.
    pub smc: u64,
    /// Times the epoch moved and the caches were resynchronised.
    pub resyncs: u64,
    /// Blocks executed as compiled host code rather than interpreted.
    ///
    /// The two engines are indistinguishable to the guest, so this is a
    /// statistic and never a behaviour — but a backend whose coverage is
    /// unmeasured is a backend whose coverage rots, which is why it is
    /// counted rather than assumed.
    pub compiled: u64,
}

/// The loop that keeps a guest inside translated code.
#[derive(Debug)]
pub struct Dispatcher {
    cache: BlockCache,
    interp: Interp,
    /// The host code generator, when there is one and it has been given.
    ///
    /// `None` is the whole of the portable path: every block is interpreted,
    /// which is what `no_std`, wasm and any host without a backend do.
    #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
    backend: Option<Engine>,
    exit: Option<ExitFlag>,
    stats: DispatchStats,
}

impl Dispatcher {
    /// A dispatcher over a default-sized cache.
    #[must_use]
    pub fn new() -> Dispatcher {
        Dispatcher::with_cache(BlockCache::new())
    }

    /// A dispatcher over `cache`.
    #[must_use]
    pub fn with_cache(cache: BlockCache) -> Dispatcher {
        Dispatcher {
            cache,
            interp: Interp::new(),
            #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
            backend: None,
            exit: None,
            stats: DispatchStats::default(),
        }
    }

    /// The same dispatcher, compiling blocks with `engine` where it can.
    ///
    /// A block the engine refuses runs on [`Interp`](crate::ir::Interp), and
    /// the two are indistinguishable to the guest — same registers, same
    /// memory, same faults, same ticks, in the same order — which is the claim
    /// both differential harnesses check. So this is a speed knob and never a
    /// semantic one.
    #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
    #[cfg_attr(docsrs, doc(cfg(feature = "jit-x86")))]
    #[must_use]
    pub fn with_backend(mut self, engine: Engine) -> Dispatcher {
        self.backend = Some(engine);
        self
    }

    /// The host code generator, if this dispatcher has one.
    #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
    #[cfg_attr(docsrs, doc(cfg(feature = "jit-x86")))]
    #[inline]
    #[must_use]
    pub fn backend(&self) -> Option<&Engine> {
        self.backend.as_ref()
    }

    /// The same dispatcher, unwinding when `flag` is raised.
    #[must_use]
    pub fn with_exit_flag(mut self, flag: ExitFlag) -> Dispatcher {
        self.exit = Some(flag);
        self
    }

    /// The block cache, for statistics and for a caller that invalidates.
    #[inline]
    #[must_use]
    pub fn cache(&self) -> &BlockCache {
        &self.cache
    }

    /// The block cache, mutably.
    #[inline]
    pub fn cache_mut(&mut self) -> &mut BlockCache {
        &mut self.cache
    }

    /// What this dispatcher has been asked to do.
    #[inline]
    #[must_use]
    pub fn stats(&self) -> DispatchStats {
        self.stats
    }

    /// The cache's own statistics.
    #[inline]
    #[must_use]
    pub fn cache_stats(&self) -> CacheStats {
        self.cache.stats()
    }

    /// Run at most `budget` blocks from `pc`.
    ///
    /// # Errors
    ///
    /// Whatever [`Frontend::translate`] or
    /// [`Interp::run`](crate::ir::Interp::run) said. Neither is recoverable
    /// here: a frontend that cannot lift says so with `insns == 0`, and a
    /// backend error is a malformed block.
    ///
    /// # Panics
    ///
    /// Never: every block reached through the cache is one this run inserted
    /// or found, and both are checked.
    pub fn run<F, H>(
        &mut self,
        front: &mut F,
        host: &mut H,
        mut pc: u64,
        budget: usize,
    ) -> Result<Run>
    where
        F: Frontend<H> + ?Sized,
        H: IrHost + StoreLog + FastMem,
    {
        let pc_slot = front.pc_slot();
        let mut from: Option<BlockId> = None;
        let mut blocks = 0usize;
        let mut insns = 0usize;

        let stop = loop {
            if blocks >= budget {
                break Stop::Budget;
            }
            if self.exit.as_ref().is_some_and(ExitFlag::raised) {
                break Stop::Exit;
            }
            // Per block, not per run. A guest store can remap an address
            // space, a store ends its block, and a chained successor would
            // otherwise be served out of a cache lifted through the topology
            // that store replaced — a window that did not exist while a run
            // was one block long. The predecessor goes with it: a flush
            // retires every id, so following a link from before one would
            // reach whatever took the slot.
            if self.cache.sync(front.epoch()) {
                self.stats.resyncs += 1;
                from = None;
            }
            // What this block owes before it exists as far as this loop is
            // concerned: the entry translation, and whatever else the guest
            // decides at a boundary. It is inside the loop rather than before
            // it because a *chained* successor owes exactly the same thing,
            // and a dispatcher that only charged the first block would make a
            // chain cheaper than the blocks it replaced.
            if front.enter(pc, host)? == Entry::Leave {
                break Stop::Declined;
            }

            let key = front.key();
            let (id, chained) = match from.and_then(|f| self.cache.follow(f, pc, key)) {
                Some(id) => (id, true),
                None => match self.cache.lookup(pc, key) {
                    Some(id) => {
                        self.stats.looked_up += 1;
                        (id, false)
                    }
                    None => {
                        let t = front.translate(pc)?;
                        self.stats.translated += 1;
                        if t.insns == 0 {
                            break Stop::Untranslatable { pc };
                        }
                        (self.cache.insert(pc, key, t.page, t.insns, t.block), false)
                    }
                },
            };
            if chained {
                self.stats.chained += 1;
            } else if let Some(f) = from {
                // The patch. Next time this predecessor exits to this PC it
                // reaches the successor with no lookup at all.
                self.cache.link(f, pc, id);
            }

            // Compiled if there is a backend and it takes this block, and
            // interpreted otherwise. The two are indistinguishable to the
            // guest, including in cycle accounting, so `retired` is read off
            // whichever ran rather than off a fixed engine — a run that read
            // the wrong one would tell an oracle to step the wrong number of
            // times, which is the bug the retired count exists to avoid.
            let (outcome, retired) = self.execute(id, host)?;
            self.stats.blocks += 1;
            blocks += 1;
            // Every exit is preceded by one boundary that begins no guest
            // instruction, and exactly one exit is reached, so this is what
            // retired — at a fault too, where the faulting instruction opened
            // its boundary and did not retire.
            insns += retired;

            // Guest stores land before the next block is chosen, so a block
            // invalidated by one is never served afterwards.
            let cache = &mut self.cache;
            let mut hit = 0usize;
            host.drain_dirty(&mut |page| hit += cache.note_write(page, 1));
            self.stats.smc += hit as u64;
            let survived = self.cache.block(id).is_some();

            match outcome {
                Outcome::Exit => pc = host.read_slot(pc_slot) as u64,
                Outcome::Goto { pc: next } | Outcome::Lookup { pc: next } => pc = next,
                Outcome::Fault(f) => break Stop::Fault(f),
                Outcome::Unsupported { op, at } => break Stop::Unsupported { op, at },
            }
            // A block that wrote into its own page is gone, and the id that
            // named it may already have been reused, so it cannot be the
            // predecessor of the next link.
            from = survived.then_some(id);
        };

        Ok(Run {
            pc,
            blocks,
            insns,
            stop,
        })
    }

    /// Execute the block `id` names, and say what it did and how many guest
    /// instructions it retired.
    ///
    /// The one place that chooses an engine. A backend that takes the block
    /// runs it; anything else — no backend, a refusal, a code handle that a
    /// buffer reset invalidated — interprets, which is `ROADMAP.md` §9's
    /// *"degrades in speed rather than failing to run"* applied one level down,
    /// to a block rather than to a host.
    fn execute<H>(&mut self, id: BlockId, host: &mut H) -> Result<(Outcome, usize)>
    where
        H: IrHost + FastMem,
    {
        #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
        {
            // Destructured, because compiling reads the block out of the cache
            // while the engine is borrowed mutably, and the two are different
            // fields of the same struct.
            let Dispatcher {
                cache,
                backend,
                stats,
                ..
            } = self;
            if let Some(engine) = backend.as_mut() {
                let block = cache
                    .block(id)
                    .expect("a block just found or just inserted is resident");
                let code = match cache.code(id).filter(|c| engine.is_live(*c)) {
                    Some(code) => Some(code),
                    // A refusal is not an error and is not recorded against
                    // the block: the engine counts it, and the next time this
                    // block is reached it is refused again for the same
                    // reason, which costs a compile attempt and nothing else.
                    None => engine.compile(block).ok(),
                };
                if let Some(code) = code {
                    cache.set_code(id, code);
                    let block = cache
                        .block(id)
                        .expect("a block just found or just inserted is resident");
                    if let Some(outcome) = engine.run(block, code, host) {
                        stats.compiled += 1;
                        let retired = engine.boundaries().saturating_sub(1) as usize;
                        return Ok((outcome?, retired));
                    }
                }
            }
        }
        let block = self
            .cache
            .block(id)
            .expect("a block just found or just inserted is resident");
        let outcome = self.interp.run(block, host)?;
        Ok((outcome, self.interp.boundaries().saturating_sub(1) as usize))
    }
}

impl Default for Dispatcher {
    fn default() -> Dispatcher {
        Dispatcher::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::error::BusError;
    use crate::core::space::MemResult;
    use crate::ir::{BlockBuilder, Const, InsnStart, MemOp, Type};
    use alloc::vec;

    const PC: RegSlot = RegSlot(0);

    /// A block that leaves `next` in the PC slot and exits.
    fn straight(pc: u64, next: u64) -> Block {
        let mut b = BlockBuilder::new(pc, 0);
        b.insn_start(InsnStart {
            pc,
            next_pc: next,
            ticks: 0,
            live: Vec::new(),
        });
        b.charge(1);
        let t = b.imm(Type::I64, Const::Int(u128::from(next)));
        b.insn_start(InsnStart {
            pc: next,
            next_pc: next,
            ticks: 1,
            live: vec![(PC, t)],
        });
        b.exit_tb();
        b.finish()
    }

    /// A frontend over a fixed straight-line chain of blocks.
    struct Chain {
        /// `pc -> next pc`, for as many blocks as the test wants.
        step: u64,
        limit: u64,
        epoch: Epoch,
        key: u64,
        translated: Vec<u64>,
    }

    impl<H: ?Sized> Frontend<H> for Chain {
        fn epoch(&mut self) -> Epoch {
            self.epoch
        }
        fn key(&mut self) -> u64 {
            self.key
        }
        fn pc_slot(&self) -> RegSlot {
            PC
        }
        fn translate(&mut self, pc: u64) -> Result<Translation> {
            self.translated.push(pc);
            let next = if pc + self.step >= self.limit {
                0x1000
            } else {
                pc + self.step
            };
            Ok(Translation {
                block: straight(pc, next),
                page: pc & !PAGE_MASK,
                insns: 1,
            })
        }
    }

    #[derive(Default)]
    struct Host {
        slots: [u64; 4],
        ticks: u64,
        dirty: DirtyPages,
    }

    impl IrHost for Host {
        fn read_slot(&mut self, slot: RegSlot) -> u128 {
            u128::from(self.slots[slot.0 as usize])
        }
        fn write_slot(&mut self, slot: RegSlot, value: u128) {
            self.slots[slot.0 as usize] = value as u64;
        }
        fn load(&mut self, _mem: &MemOp, _addr: u64) -> MemResult<u64> {
            Err(BusError::Unassigned)
        }
        fn store(&mut self, mem: &MemOp, addr: u64, _value: u64) -> MemResult {
            self.dirty.note(addr, mem.size.bytes());
            Ok(())
        }
        fn charge(&mut self, ticks: u64) {
            self.ticks += ticks;
        }
        fn insn_start(&mut self, _mark: &InsnStart) {}
    }

    impl StoreLog for Host {
        fn drain_dirty(&mut self, sink: &mut dyn FnMut(u64)) {
            self.dirty.drain_dirty(sink);
        }
    }

    // No software TLB here, so no fast path to publish: this host's loads all
    // take the call, which is the default and always correct.
    impl FastMem for Host {}

    fn chain(step: u64, limit: u64) -> Chain {
        Chain {
            step,
            limit,
            epoch: Epoch::default(),
            key: 0,
            translated: Vec::new(),
        }
    }

    #[test]
    fn a_loop_is_translated_once_and_then_chained() {
        let mut d = Dispatcher::with_cache(BlockCache::with_capacity(64));
        let mut f = chain(4, 0x1010);
        let mut h = Host::default();
        let run = d.run(&mut f, &mut h, 0x1000, 400).expect("runs");
        assert_eq!(run.blocks, 400);
        assert_eq!(run.insns, 400);
        assert_eq!(run.stop, Stop::Budget);
        // Four distinct blocks in the loop, translated once each.
        assert_eq!(f.translated.len(), 4);
        assert_eq!(d.stats().translated, 4);
        // and after the first time round, every edge is a patched exit.
        assert!(
            d.stats().chained >= 390,
            "chained {} of {}",
            d.stats().chained,
            run.blocks
        );
        assert_eq!(d.cache_stats().stale_links, 0);
        d.cache().check().expect("consistent");
    }

    #[test]
    fn every_tick_is_charged_whether_the_block_was_cached_or_not() {
        // A cache hit and a cache miss must be indistinguishable to the guest,
        // including in cycle accounting (`ROADMAP.md` §0). Each block charges
        // one, so the total is the block count however the blocks were found.
        let mut d = Dispatcher::new();
        let mut f = chain(4, 0x1010);
        let mut h = Host::default();
        let run = d.run(&mut f, &mut h, 0x1000, 97).expect("runs");
        assert_eq!(h.ticks, run.blocks as u64);
        assert!(d.stats().chained > 0, "and chaining really happened");
    }

    /// The same guard, with the blocks executed as host code.
    ///
    /// `ROADMAP.md` §0 requires a bit-identical state hash *across the
    /// interpreter and the JIT for the same guest*, and the cycle counter is in
    /// that hash. So the two engines must charge the same ticks at the same
    /// points, whichever ran the block — and the run is done twice, once each
    /// way, on the same programs, so the numbers are compared rather than
    /// merely asserted.
    #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
    #[test]
    fn a_compiled_block_charges_exactly_what_an_interpreted_one_charges() {
        let mut interpreted = Dispatcher::new();
        let mut hi = Host::default();
        let a = interpreted
            .run(&mut chain(4, 0x1010), &mut hi, 0x1000, 97)
            .expect("runs");

        let mut compiled = Dispatcher::new()
            .with_backend(crate::jit::x86::Engine::new().expect("a W^X code buffer"));
        let mut hc = Host::default();
        let b = compiled
            .run(&mut chain(4, 0x1010), &mut hc, 0x1000, 97)
            .expect("runs");

        assert!(compiled.stats().compiled > 0, "nothing was compiled");
        assert_eq!(hi.ticks, hc.ticks, "the cycle counters must agree");
        assert_eq!(
            a.insns, b.insns,
            "and so must the retired instruction count"
        );
        assert_eq!(a.pc, b.pc);
        assert_eq!(a.stop, b.stop);
        assert_eq!(hi.slots, hc.slots, "and the guest's own state");
    }

    /// A block covering `insns` guest instructions, with a side exit taken
    /// when `leave_at` is reached.
    ///
    /// The superblock shape in miniature: several boundaries, two terminators,
    /// and a forward branch over the first exit sequence.
    fn trace(pc: u64, insns: u64, leave_at: Option<u64>, after: u64) -> Block {
        let mut b = BlockBuilder::new(pc, 0);
        // The branch jumps *over* the exit sequence, so a zero here is the
        // side exit being taken — the inversion `cpu::riscv::lift` emits.
        let skip = b.imm(Type::I1, Const::Int(0));
        let mut ticks = 0u64;
        for i in 0..insns {
            b.insn_start(InsnStart {
                pc: pc + i * 4,
                next_pc: pc + (i + 1) * 4,
                ticks,
                live: Vec::new(),
            });
            b.charge(1);
            ticks += 1;
            if leave_at == Some(i) {
                // The side exit, inline and branched over — exactly the shape
                // `cpu::riscv::lift` emits.
                let over = b.emit_raw(
                    Opcode::BRCOND,
                    Type::I64,
                    None,
                    None,
                    &[skip],
                    None,
                    None,
                    0,
                );
                let t = b.imm(Type::I64, Const::Int(u128::from(after)));
                b.insn_start(InsnStart {
                    pc: after,
                    next_pc: after,
                    ticks,
                    live: vec![(PC, t)],
                });
                b.exit_tb();
                b.patch_aux(over, b.next_index() as u32);
            }
        }
        let t = b.imm(Type::I64, Const::Int(u128::from(after)));
        b.insn_start(InsnStart {
            pc: after,
            next_pc: after,
            ticks,
            live: vec![(PC, t)],
        });
        b.exit_tb();
        b.finish()
    }

    /// A frontend serving one trace, over and over.
    struct Traces {
        insns: u64,
        leave_at: Option<u64>,
        epoch: Epoch,
    }

    impl<H: ?Sized> Frontend<H> for Traces {
        fn epoch(&mut self) -> Epoch {
            self.epoch
        }
        fn key(&mut self) -> u64 {
            0
        }
        fn pc_slot(&self) -> RegSlot {
            PC
        }
        fn translate(&mut self, pc: u64) -> Result<Translation> {
            Ok(Translation {
                block: trace(pc, self.insns, self.leave_at, pc),
                page: pc & !PAGE_MASK,
                // Deliberately the *static* count, which is what a superblock
                // covers and not what a run through it retires.
                insns: self.insns as usize,
            })
        }
    }

    #[test]
    fn a_side_exit_retires_fewer_instructions_than_the_trace_covers() {
        // The static count would say sixteen a block; the run leaves through
        // the side exit after five. A dispatcher that reported the static
        // number would tell an oracle to step three times too far.
        let mut d = Dispatcher::new();
        let mut f = Traces {
            insns: 16,
            leave_at: Some(4),
            epoch: Epoch::default(),
        };
        let mut h = Host::default();
        let run = d.run(&mut f, &mut h, 0x1000, 10).expect("runs");
        assert_eq!(run.blocks, 10);
        assert_eq!(
            run.insns, 50,
            "five guest instructions a block, not sixteen"
        );
        // and the ticks agree with the instructions, not with the coverage.
        assert_eq!(h.ticks, 50);
    }

    #[test]
    fn a_trace_that_runs_to_its_end_retires_everything_it_covers() {
        let mut d = Dispatcher::new();
        let mut f = Traces {
            insns: 16,
            leave_at: None,
            epoch: Epoch::default(),
        };
        let mut h = Host::default();
        let run = d.run(&mut f, &mut h, 0x1000, 10).expect("runs");
        assert_eq!(run.insns, 160);
        assert_eq!(h.ticks, 160);
    }

    #[test]
    fn a_raised_exit_flag_stops_within_one_block_however_long_the_block_is() {
        // A trace has fewer boundaries than the basic blocks it replaces, so
        // the safe-point protocol's promise weakens from "one basic block" to
        // "one translation" — bounded by a frontend's instruction limit
        // (`ROADMAP.md` §4.7, and `cpu::riscv::lift::MAX_INSNS`). Bounded is
        // the claim, so this asserts the bound rather than the old wording.
        let flag = ExitFlag::default();
        let mut d = Dispatcher::new().with_exit_flag(flag.clone());
        let mut f = Traces {
            insns: 64,
            leave_at: None,
            epoch: Epoch::default(),
        };
        let mut h = Host::default();
        d.run(&mut f, &mut h, 0x1000, 1).expect("runs");
        flag.raise();
        let run = d.run(&mut f, &mut h, 0x1000, 100).expect("runs");
        assert_eq!(run.stop, Stop::Exit);
        assert_eq!(run.blocks, 0, "no block starts once the flag is up");
    }

    /// The safe-point bound, with the blocks executed as host code.
    ///
    /// A compiled block has exactly the same boundaries as the interpreted one
    /// — the code generator changes how a block runs, never where it ends — so
    /// `ROADMAP.md` §4.7's protocol is unchanged and the delay before a raised
    /// flag is honoured is still bounded by a frontend's own instruction limit.
    /// That is asserted rather than argued, because "the backend did not change
    /// it" is the kind of claim that stops being true quietly.
    #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
    #[test]
    fn a_raised_exit_flag_stops_a_compiled_run_within_one_block_too() {
        let flag = ExitFlag::default();
        let mut d = Dispatcher::new()
            .with_exit_flag(flag.clone())
            .with_backend(crate::jit::x86::Engine::new().expect("a W^X code buffer"));
        let mut f = Traces {
            insns: 64,
            leave_at: None,
            epoch: Epoch::default(),
        };
        let mut h = Host::default();
        let run = d.run(&mut f, &mut h, 0x1000, 4).expect("runs");
        assert_eq!(run.insns, 4 * 64, "every merged instruction retired");
        assert!(d.stats().compiled > 0, "the blocks really were compiled");
        flag.raise();
        let run = d.run(&mut f, &mut h, 0x1000, 100).expect("runs");
        assert_eq!(run.stop, Stop::Exit);
        assert_eq!(run.blocks, 0, "no block starts once the flag is up");
    }

    #[test]
    fn a_raised_exit_flag_stops_at_a_block_boundary() {
        let flag = ExitFlag::default();
        let mut d = Dispatcher::new().with_exit_flag(flag.clone());
        let mut f = chain(4, 0x1010);
        let mut h = Host::default();
        assert_eq!(
            d.run(&mut f, &mut h, 0x1000, 10).expect("runs").stop,
            Stop::Budget
        );
        flag.raise();
        let run = d.run(&mut f, &mut h, 0x1000, 10).expect("runs");
        assert_eq!(run.stop, Stop::Exit);
        assert_eq!(run.blocks, 0, "no block starts once the flag is up");
    }

    #[test]
    fn an_epoch_change_between_runs_resynchronises_the_cache() {
        let mut d = Dispatcher::new();
        let mut f = chain(4, 0x1010);
        let mut h = Host::default();
        d.run(&mut f, &mut h, 0x1000, 20).expect("runs");
        assert_eq!(d.stats().translated, 4);
        f.epoch.topology += 1;
        d.run(&mut f, &mut h, 0x1000, 20).expect("runs");
        assert_eq!(d.stats().resyncs, 1);
        assert_eq!(d.stats().translated, 8, "every block was lifted again");
    }

    /// A [`Chain`] that records every PC it was entered at and refuses to
    /// enter the `limit`th.
    struct Gate {
        chain: Chain,
        seen: Vec<u64>,
        limit: usize,
    }

    impl<H: ?Sized> Frontend<H> for Gate {
        fn epoch(&mut self) -> Epoch {
            Frontend::<H>::epoch(&mut self.chain)
        }
        fn enter(&mut self, pc: u64, _host: &mut H) -> Result<Entry> {
            self.seen.push(pc);
            Ok(if self.seen.len() > self.limit {
                Entry::Leave
            } else {
                Entry::Ready
            })
        }
        fn key(&mut self) -> u64 {
            Frontend::<H>::key(&mut self.chain)
        }
        fn pc_slot(&self) -> RegSlot {
            Frontend::<H>::pc_slot(&self.chain)
        }
        fn translate(&mut self, pc: u64) -> Result<Translation> {
            Frontend::<H>::translate(&mut self.chain, pc)
        }
    }

    #[test]
    fn a_boundary_hook_is_called_once_per_block_chained_or_not() {
        // The property `cpu::riscv::engine` depends on for its cycle counter:
        // a chained successor is entered exactly as an unchained one is, so a
        // guest that charges for its entry fetch charges the same whichever
        // way the block was reached.
        let mut d = Dispatcher::with_cache(BlockCache::with_capacity(64));
        let mut f = Gate {
            chain: chain(4, 0x1010),
            seen: Vec::new(),
            limit: usize::MAX,
        };
        let mut h = Host::default();
        let run = d.run(&mut f, &mut h, 0x1000, 40).expect("runs");
        assert_eq!(run.blocks, 40);
        assert_eq!(f.seen.len(), 40, "one entry per block, chained included");
        assert!(d.stats().chained > 0, "and chaining really happened");
        // The PCs are the ones the blocks ran at, in order, round the loop.
        assert_eq!(&f.seen[..5], &[0x1000, 0x1004, 0x1008, 0x100c, 0x1000]);
    }

    /// A frontend that computes its key in `enter`, as the RISC-V engine does,
    /// and records what `key` was asked for and whether `enter` had run.
    struct Ordered {
        chain: Chain,
        entered: Option<u64>,
        asked: Vec<(Option<u64>, u64)>,
    }

    impl<H: ?Sized> Frontend<H> for Ordered {
        fn epoch(&mut self) -> Epoch {
            self.chain.epoch
        }
        fn enter(&mut self, pc: u64, _host: &mut H) -> Result<Entry> {
            self.entered = Some(pc);
            Ok(Entry::Ready)
        }
        fn key(&mut self) -> u64 {
            let key = self.entered.unwrap_or(u64::MAX);
            self.asked.push((self.entered, key));
            key
        }
        fn pc_slot(&self) -> RegSlot {
            Frontend::<H>::pc_slot(&self.chain)
        }
        fn translate(&mut self, pc: u64) -> Result<Translation> {
            Frontend::<H>::translate(&mut self.chain, pc)
        }
    }

    #[test]
    fn a_boundary_hook_runs_before_the_key_it_computes_is_read() {
        // The order is documented and load-bearing: `cpu::riscv::engine`
        // resolves its entry fetch to a physical page in `enter` and *is* that
        // page in `key`. Asked the other way round, every block would be
        // cached under its predecessor's world — which on a guest whose blocks
        // share a page is invisible until one of them does not.
        let mut d = Dispatcher::with_cache(BlockCache::with_capacity(64));
        let mut f = Ordered {
            chain: chain(4, 0x1010),
            entered: None,
            asked: Vec::new(),
        };
        let mut h = Host::default();
        let run = d.run(&mut f, &mut h, 0x1000, 12).expect("runs");
        assert_eq!(run.blocks, 12);
        assert_eq!(f.asked.len(), 12, "one key per block");
        assert!(
            f.asked.iter().all(|&(entered, key)| entered == Some(key)),
            "`key` was asked before `enter` set it: {:?}",
            f.asked
        );
    }

    #[test]
    fn a_frontend_that_declines_a_boundary_stops_the_run_there() {
        let mut d = Dispatcher::with_cache(BlockCache::with_capacity(64));
        let mut f = Gate {
            chain: chain(4, 0x1010),
            seen: Vec::new(),
            limit: 3,
        };
        let mut h = Host::default();
        let run = d.run(&mut f, &mut h, 0x1000, 40).expect("runs");
        assert_eq!(run.stop, Stop::Declined);
        assert_eq!(run.blocks, 3, "the declined block did not run");
        assert_eq!(run.insns, 3);
        assert_eq!(run.pc, 0x100c, "and the guest is left standing at it");
        assert_eq!(h.ticks, 3, "the declined block charged nothing");
    }

    /// A [`Chain`] whose topology generation moves partway through a run.
    struct Shifting {
        chain: Chain,
        seen: usize,
        at: usize,
    }

    impl<H: ?Sized> Frontend<H> for Shifting {
        fn epoch(&mut self) -> Epoch {
            self.seen += 1;
            if self.seen > self.at {
                self.chain.epoch.topology = 1;
            }
            self.chain.epoch
        }
        fn key(&mut self) -> u64 {
            Frontend::<H>::key(&mut self.chain)
        }
        fn pc_slot(&self) -> RegSlot {
            Frontend::<H>::pc_slot(&self.chain)
        }
        fn translate(&mut self, pc: u64) -> Result<Translation> {
            Frontend::<H>::translate(&mut self.chain, pc)
        }
    }

    #[test]
    fn a_retopology_partway_through_a_run_is_seen_before_the_next_block() {
        // The window chaining opens and this closes: a guest store can remap
        // an address space, a store ends its block, and the *next* block of
        // the chain would otherwise come out of a cache lifted through the
        // topology that store replaced. The epoch is therefore read at every
        // boundary rather than once per run.
        let mut d = Dispatcher::with_cache(BlockCache::with_capacity(64));
        let mut f = Shifting {
            chain: chain(4, 0x1010),
            seen: 0,
            at: 5,
        };
        let mut h = Host::default();
        let run = d.run(&mut f, &mut h, 0x1000, 20).expect("runs");
        assert_eq!(run.blocks, 20, "the run still finishes");
        assert_eq!(d.stats().resyncs, 1, "and resynchronised inside it");
        assert_eq!(
            d.stats().translated,
            8,
            "four blocks before the flush and four after"
        );
        // The predecessor is dropped with the cache, so no link is followed
        // into a slot the flush retired.
        assert_eq!(d.cache_stats().stale_links, 0);
        d.cache().check().expect("consistent");
    }

    #[test]
    fn a_key_change_is_a_different_translation_at_the_same_pc() {
        let mut d = Dispatcher::new();
        let mut f = chain(4, 0x1010);
        let mut h = Host::default();
        d.run(&mut f, &mut h, 0x1000, 20).expect("runs");
        f.key = 1;
        d.run(&mut f, &mut h, 0x1000, 20).expect("runs");
        assert_eq!(d.stats().translated, 8);
        assert_eq!(d.cache_stats().stale_links, 0);
        d.cache().check().expect("consistent");
    }
}
