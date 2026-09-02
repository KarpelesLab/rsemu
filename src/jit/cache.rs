//! The translation block cache: lookup, chaining, and — the part that is
//! actually hard — unchaining.
//!
//! `ROADMAP.md` §9.1's second and third mechanisms. A block is cached under
//! `(guest PC, Block::key)`, exits are **patched directly to their
//! successors**, and a guest write into a page holding translations throws
//! away the blocks lifted from it.
//!
//! # The key
//!
//! §9 says `(guest PC, relevant CPU flags)`, and
//! [`Block::key`](crate::ir::Block::key) is where a frontend already says what
//! "relevant" means for it. The RISC-V lifter puts four things there — the `C`
//! extension, the misalignment policy, the width, and the `Origin` naming the
//! translation generation the bytes were read under — and that question was
//! settled carefully enough that this module does not add to it. It adds
//! exactly one thing the key cannot express, and it adds it as a flush rather
//! than a key bit: see [`BlockCache::sync`] and the module docs of
//! [`jit`](super).
//!
//! # Chaining, and why unchaining is the whole difficulty
//!
//! A chain link is `predecessor exit → successor block`. Following one skips
//! the hash lookup entirely, which with the IR interpreter as the executor is
//! what "patch the exit jump" reduces to; with a code generator it becomes a
//! real patched branch and the bookkeeping below is unchanged.
//!
//! Making a link is easy. Removing one is where translation caches go wrong,
//! because a link is a pointer held by a block that is *not* the one being
//! invalidated. So every block keeps its **back edges**: the list of
//! `(predecessor, exit slot)` pairs that point at it. Invalidating a block
//! walks that list and clears each predecessor's slot before the block goes;
//! freeing a block also removes itself from every successor's list, so no back
//! edge outlives its source.
//!
//! That is the mechanism. It is not, on its own, evidence — so each link also
//! carries the **stamp** its target had when the link was made. Stamps come
//! from one monotonic counter rather than from the slot, so a slot that is
//! freed and refilled — or emptied by a flush and refilled — never carries a
//! number it carried before, and no id or link from the old occupant can name
//! the new one. Following a link whose stamp no longer matches counts a
//! [`CacheStats::stale_links`] and takes the slow path instead of executing
//! the wrong block. The tests and the fuzz target assert that counter is zero:
//! the belt is the back edges, and the braces are there to prove the belt
//! held.
//!
//! # Self-modifying code
//!
//! Every block records the guest-**physical** page its bytes came from — a
//! block never leaves the page it started on (`cpu::riscv::lift`), so one page
//! is the whole answer. [`BlockCache::note_write`] is the hook a guest store
//! goes through: a **bit filter** over pages that have ever held a translation
//! rejects the overwhelmingly common case in one load and a test, and only a
//! set bit costs a map lookup. The filter is allowed false positives — a bit
//! is cleared only by a flush — because a false positive costs a lookup that
//! finds nothing and a false negative would be a stale translation.
//!
//! # Determinism
//!
//! The page index is a [`BTreeMap`], bucket chains are in insertion order, and
//! eviction is FIFO. Nothing here is iterated in hash order, and nothing here
//! is guest-visible anyway: a hit and a miss produce the same block.

use alloc::collections::{BTreeMap, VecDeque};
use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;

use crate::ir::Block;
use crate::jit::tlb::{Epoch, PAGE_MASK, PAGE_SIZE};

/// A block's identity within one cache.
///
/// Carries the slot's **stamp** as well as its index, so an id that outlived
/// its block cannot name whatever took the slot next. That matters because a
/// dispatcher holds an id across the block's own execution, and a block that
/// writes into its own page invalidates itself while that id is live.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct BlockId {
    slot: u32,
    stamp: u64,
}

/// A handle on one block's compiled host code.
///
/// Plain data — an index and a generation — so that this module can hold one
/// beside the block it names without knowing what a code buffer is, and so
/// that `jit/cache.rs` stays `no_std` while the thing it refers to is behind a
/// feature and a target `cfg`. The generation is what makes a handle from
/// before a code buffer was reset *rejected* rather than followed into
/// whatever took its place; the backend checks it, not this cache.
///
/// A handle lives and dies with its slot: [`BlockCache::insert`] starts it at
/// `None`, and an invalidated or evicted block drops it, so the one thing that
/// cannot happen is code left attached to a block that has gone.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CodeRef {
    /// The backend's index for this code.
    pub index: u32,
    /// The code buffer generation it was emitted in.
    pub generation: u64,
}

/// How many successors one block may be chained to.
///
/// Two: the taken and not-taken sides of a conditional branch, which is the
/// shape that matters. A computed jump with many targets keeps whichever two
/// it saw most recently and pays a hash lookup for the rest.
pub const EXITS: usize = 2;

/// The sentinel for "no block".
const NONE: u32 = u32::MAX;

/// One chain link: an exit, patched.
#[derive(Debug, Clone, Copy)]
struct Link {
    /// The guest PC this exit produces when it takes this successor.
    pc: u64,
    /// The successor, or [`NONE`].
    target: u32,
    /// The successor slot's stamp when the link was made.
    stamp: u64,
}

impl Link {
    const fn empty() -> Link {
        Link {
            pc: 0,
            target: NONE,
            stamp: 0,
        }
    }
}

/// One cached translation.
#[derive(Debug)]
struct Slot {
    block: Block,
    pc: u64,
    key: u64,
    /// The guest-physical page the bytes were read from — what a guest write
    /// is matched against.
    page: u64,
    /// How many guest instructions the block covers. Zero means the frontend
    /// could not lift the instruction at `pc`.
    insns: usize,
    /// Bumped every time this slot is reused, so a link into it can tell.
    stamp: u64,
    /// The next slot in this slot's hash bucket, or [`NONE`].
    next: u32,
    /// Where this block's exits are patched to.
    exits: [Link; EXITS],
    /// Who is patched to this block: `(predecessor, exit slot)`.
    ///
    /// The back edges. Without them an invalidated block leaves live pointers
    /// behind it in blocks that are perfectly valid themselves.
    preds: Vec<(u32, u8)>,
    /// This block's compiled host code, once a backend has produced some.
    code: Option<CodeRef>,
}

/// What a [`BlockCache`] has been asked to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct CacheStats {
    /// Lookups that found a block.
    pub hits: u64,
    /// Lookups that did not.
    pub misses: u64,
    /// Exits followed straight to a successor, with no lookup at all.
    pub chained: u64,
    /// Links made.
    pub links: u64,
    /// Links cleared because their target went away.
    pub unlinks: u64,
    /// Blocks inserted.
    pub inserts: u64,
    /// Blocks invalidated by a guest write into their page.
    pub smc: u64,
    /// Blocks evicted because the cache was full.
    pub evictions: u64,
    /// Whole-cache invalidations.
    pub flushes: u64,
    /// Guest writes the page filter rejected without a map lookup.
    pub filtered: u64,
    /// Links found pointing at a reused slot.
    ///
    /// **Must be zero.** A back edge that was not cleared shows up here rather
    /// than as a wrong block executed; the tests and the fuzz target assert
    /// it, which is what makes the unchaining a checked claim rather than an
    /// intention.
    pub stale_links: u64,
}

/// A cache of translation blocks, chained.
#[derive(Debug)]
pub struct BlockCache {
    slots: Vec<Option<Slot>>,
    /// The stamp the next block inserted will carry.
    ///
    /// Global and monotonic rather than per-slot, which is what makes an id
    /// from before a flush unable to name a block from after one *and* makes
    /// [`BlockCache::flush`] cost the buckets and the filter rather than the
    /// whole slot table. A per-slot counter has to be walked to be bumped, and
    /// a cache that is flushed once per block — a translator running with no
    /// cache, which is exactly the baseline the benchmark measures — then
    /// spends quadratic time in the flush.
    next_stamp: u64,
    free: Vec<u32>,
    /// Hash buckets holding the head of each chain, or [`NONE`].
    buckets: Vec<u32>,
    /// `buckets.len() - 1`.
    bucket_mask: u64,
    /// Guest-physical page → the blocks lifted from it.
    pages: BTreeMap<u64, Vec<u32>>,
    /// A bit per hashed page: *this page may hold a translation*.
    filter: Vec<u64>,
    /// Insertion order, for FIFO eviction. Holds `(slot, stamp)` so an entry
    /// for a slot that has since been reused can be skipped.
    order: VecDeque<(u32, u64)>,
    capacity: usize,
    epoch: Epoch,
    stats: CacheStats,
}

/// How many blocks a cache holds before it starts evicting.
///
/// Bounded because a key carrying a translation generation makes every block
/// lifted under a replaced mapping permanently unreachable but still resident
/// (`cpu::riscv::lift`'s `Origin`), so an unbounded cache is a leak with a
/// guest-controlled rate. FIFO, because it is deterministic and because a
/// translation cache's access pattern makes recency a poor predictor anyway
/// once the working set exceeds the cache.
pub const DEFAULT_CAPACITY: usize = 8192;

impl BlockCache {
    /// A cache holding [`DEFAULT_CAPACITY`] blocks.
    #[must_use]
    pub fn new() -> BlockCache {
        BlockCache::with_capacity(DEFAULT_CAPACITY)
    }

    /// A cache holding `capacity` blocks.
    #[must_use]
    pub fn with_capacity(capacity: usize) -> BlockCache {
        let capacity = capacity.max(1);
        let buckets = capacity.next_power_of_two();
        // The page filter is sized from the capacity rather than fixed: a
        // flush clears it, so a small cache paying for a large filter would
        // make flushing the most expensive thing the cache does. Eight words
        // is the floor and 512 the ceiling — 32 768 bits, which is far more
        // pages than a cache of any size here will ever hold blocks from.
        let filter_words = (capacity / 4).clamp(8, 512);
        BlockCache {
            slots: Vec::new(),
            next_stamp: 1,
            free: Vec::new(),
            buckets: vec![NONE; buckets],
            bucket_mask: (buckets - 1) as u64,
            pages: BTreeMap::new(),
            filter: vec![0u64; filter_words],
            order: VecDeque::new(),
            capacity,
            epoch: Epoch::default(),
            stats: CacheStats::default(),
        }
    }

    /// What this cache has been asked to do.
    #[inline]
    #[must_use]
    pub fn stats(&self) -> CacheStats {
        self.stats
    }

    /// How many blocks are resident.
    #[must_use]
    pub fn len(&self) -> usize {
        self.slots.len() - self.free.len()
    }

    /// Whether nothing is resident.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// The epoch every resident block was translated under.
    #[inline]
    #[must_use]
    pub fn epoch(&self) -> Epoch {
        self.epoch
    }

    /// Adopt `epoch`, throwing every block away if the **topology** half
    /// changed.
    ///
    /// The translation half is deliberately not acted on: it is already in
    /// [`Block::key`](crate::ir::Block::key) through the frontend's `Origin`,
    /// so a block lifted under a replaced mapping is unreachable rather than
    /// wrong, and flushing on every `SFENCE.VMA` would throw away every
    /// bare-mode block a supervisor guest has too. What it *does* leave behind
    /// is unreachable blocks occupying slots, which is why the cache has a
    /// capacity and evicts.
    ///
    /// The topology half has no such protection — `Origin::Bare` contributes
    /// nothing to the key, so a machine-mode block lifted before a remap keys
    /// identically to one lifted after — so it flushes. Returns whether
    /// anything was thrown away.
    pub fn sync(&mut self, epoch: Epoch) -> bool {
        let flush = epoch.topology != self.epoch.topology;
        self.epoch = epoch;
        if flush {
            self.flush();
        }
        flush
    }

    /// Throw every block away.
    ///
    /// Every outstanding [`BlockId`] is dead afterwards, and provably so: the
    /// next block inserted takes a stamp no slot has ever carried.
    pub fn flush(&mut self) {
        self.slots.clear();
        self.free.clear();
        self.buckets.fill(NONE);
        self.pages.clear();
        self.filter.fill(0);
        self.order.clear();
        self.stats.flushes += 1;
    }

    /// The block cached for `(pc, key)`, if any.
    pub fn lookup(&mut self, pc: u64, key: u64) -> Option<BlockId> {
        match self.find(pc, key) {
            Some(id) => {
                self.stats.hits += 1;
                Some(id)
            }
            None => {
                self.stats.misses += 1;
                None
            }
        }
    }

    /// [`BlockCache::lookup`] without the statistics, for the paths that look
    /// a block up in order to replace it rather than to run it.
    fn find(&self, pc: u64, key: u64) -> Option<BlockId> {
        let mut at = self.buckets[self.bucket(pc, key)];
        while at != NONE {
            let slot = self.slots[at as usize].as_ref()?;
            if slot.pc == pc && slot.key == key {
                return Some(BlockId {
                    slot: at,
                    stamp: slot.stamp,
                });
            }
            at = slot.next;
        }
        None
    }

    /// The live slot an id names, or `None` if the block has gone.
    #[inline]
    fn slot(&self, id: BlockId) -> Option<&Slot> {
        self.slots
            .get(id.slot as usize)?
            .as_ref()
            .filter(|s| s.stamp == id.stamp)
    }

    /// Cache `block`, lifted from guest-physical page `page` and covering
    /// `insns` guest instructions.
    ///
    /// A block already cached for the same `(pc, key)` is replaced, which
    /// keeps the cache from holding two answers to one question.
    pub fn insert(&mut self, pc: u64, key: u64, page: u64, insns: usize, block: Block) -> BlockId {
        if let Some(old) = self.find(pc, key) {
            self.remove(old.slot);
        }
        while self.len() >= self.capacity {
            if !self.evict_one() {
                break;
            }
        }
        let page = page & !PAGE_MASK;
        let stamp = self.next_stamp;
        self.next_stamp = self.next_stamp.wrapping_add(1);
        let fresh = Slot {
            block,
            pc,
            key,
            page,
            insns,
            stamp,
            next: NONE,
            exits: [Link::empty(); EXITS],
            preds: Vec::new(),
            code: None,
        };
        let id = match self.free.pop() {
            Some(i) => {
                self.slots[i as usize] = Some(fresh);
                i
            }
            None => {
                let i = u32::try_from(self.slots.len()).unwrap_or(NONE);
                self.slots.push(Some(fresh));
                i
            }
        };
        let bucket = self.bucket(pc, key);
        let head = self.buckets[bucket];
        if let Some(slot) = self.slots[id as usize].as_mut() {
            slot.next = head;
        }
        self.buckets[bucket] = id;
        self.pages.entry(page).or_default().push(id);
        self.mark_filter(page);
        self.order.push_back((id, stamp));
        self.stats.inserts += 1;
        BlockId { slot: id, stamp }
    }

    /// The block behind an id, or `None` if it has been invalidated.
    #[inline]
    #[must_use]
    pub fn block(&self, id: BlockId) -> Option<&Block> {
        self.slot(id).map(|s| &s.block)
    }

    /// How many guest instructions the block covers.
    #[inline]
    #[must_use]
    pub fn insns(&self, id: BlockId) -> Option<usize> {
        self.slot(id).map(|s| s.insns)
    }

    /// The compiled code attached to a block, if any has been.
    #[inline]
    #[must_use]
    pub fn code(&self, id: BlockId) -> Option<CodeRef> {
        self.slot(id).and_then(|s| s.code)
    }

    /// Attach compiled code to a block.
    ///
    /// A no-op for an id whose block has gone, which is what makes it safe to
    /// call after a run: a block that wrote into its own page invalidated
    /// itself while its id was still live.
    pub fn set_code(&mut self, id: BlockId, code: CodeRef) {
        if let Some(slot) = self
            .slots
            .get_mut(id.slot as usize)
            .and_then(Option::as_mut)
            .filter(|s| s.stamp == id.stamp)
        {
            slot.code = Some(code);
        }
    }

    /// The guest-physical page the block was lifted from.
    #[inline]
    #[must_use]
    pub fn page(&self, id: BlockId) -> Option<u64> {
        self.slot(id).map(|s| s.page)
    }

    /// Patch `from`'s exit for guest PC `pc` to `to`.
    ///
    /// Idempotent, and a no-op when both exits are already patched to other
    /// successors — which keeps a computed jump with a hundred targets from
    /// churning the two slots it has.
    pub fn link(&mut self, from: BlockId, pc: u64, to: BlockId) {
        if self.slot(to).is_none() || self.slot(from).is_none() {
            return;
        }
        let Some(slot) = self
            .slots
            .get_mut(from.slot as usize)
            .and_then(|s| s.as_mut())
        else {
            return;
        };
        // The exit for this PC if it has one, otherwise the first free slot.
        // A block with neither keeps the two successors it has: a computed
        // jump with a hundred targets must not churn them.
        let mut chosen = None;
        for (i, link) in slot.exits.iter().enumerate() {
            if link.target != NONE && link.pc == pc {
                if link.target == to.slot && link.stamp == to.stamp {
                    return;
                }
                chosen = Some(i);
                break;
            }
            if link.target == NONE && chosen.is_none() {
                chosen = Some(i);
            }
        }
        let Some(i) = chosen else {
            return;
        };
        let old = slot.exits[i];
        slot.exits[i] = Link {
            pc,
            target: to.slot,
            stamp: to.stamp,
        };
        if old.target != NONE {
            self.drop_back_edge(old.target, old.stamp, from.slot, i as u8);
        }
        if let Some(target) = self
            .slots
            .get_mut(to.slot as usize)
            .and_then(|s| s.as_mut())
        {
            target.preds.push((from.slot, i as u8));
        }
        self.stats.links += 1;
    }

    /// Follow `from`'s patched exit for `(pc, key)`, if it has one.
    ///
    /// The whole point of chaining: no hash, no bucket walk, no compare
    /// against every block in a chain. The key is checked anyway, because a
    /// world change between two blocks would otherwise be a way to execute a
    /// block from the previous world.
    pub fn follow(&mut self, from: BlockId, pc: u64, key: u64) -> Option<BlockId> {
        let link = *self
            .slot(from)?
            .exits
            .iter()
            .find(|l| l.target != NONE && l.pc == pc)?;
        let id = BlockId {
            slot: link.target,
            stamp: link.stamp,
        };
        let Some(live) = self.slot(id) else {
            // The back edges should have cleared this. That they did not is a
            // bug in this module rather than in the caller — so it is counted,
            // and the caller falls back to a lookup instead of running a block
            // that is not the one the link named.
            if self
                .slots
                .get(link.target as usize)
                .is_some_and(Option::is_some)
            {
                self.stats.stale_links += 1;
            }
            return None;
        };
        if live.pc != pc || live.key != key {
            return None;
        }
        self.stats.chained += 1;
        Some(id)
    }

    /// Invalidate every block lifted from the page holding `phys`.
    ///
    /// Returns how many went.
    pub fn invalidate_page(&mut self, phys: u64) -> usize {
        let page = phys & !PAGE_MASK;
        let Some(ids) = self.pages.remove(&page) else {
            return 0;
        };
        let n = ids.len();
        for id in ids {
            self.remove_keeping_page(id);
        }
        n
    }

    /// The self-modifying-code hook: a guest store of `len` bytes at
    /// guest-physical `phys`.
    ///
    /// Returns how many translations it invalidated. The bit filter answers
    /// the common case — a store into a page nothing was ever lifted from — in
    /// one load and a test, which is what makes this affordable on every
    /// store.
    pub fn note_write(&mut self, phys: u64, len: u64) -> usize {
        if len == 0 {
            return 0;
        }
        let first = phys & !PAGE_MASK;
        let last = phys.saturating_add(len - 1) & !PAGE_MASK;
        let mut hit = 0;
        let mut page = first;
        loop {
            if self.filter_set(page) {
                hit += self.invalidate_page(page);
            } else {
                self.stats.filtered += 1;
            }
            if page >= last {
                break;
            }
            page = page.saturating_add(PAGE_SIZE);
        }
        self.stats.smc += hit as u64;
        hit
    }

    /// Whether any block was ever lifted from `page`.
    ///
    /// False positives are allowed and cost a map lookup that finds nothing; a
    /// false negative would be a stale translation, so the bit is only ever
    /// cleared by [`BlockCache::flush`].
    #[inline]
    fn filter_set(&self, page: u64) -> bool {
        let bit = filter_bit(page, self.filter.len());
        self.filter[bit / 64] & (1u64 << (bit % 64)) != 0
    }

    #[inline]
    fn mark_filter(&mut self, page: u64) {
        let bit = filter_bit(page, self.filter.len());
        self.filter[bit / 64] |= 1u64 << (bit % 64);
    }

    /// Invalidate one block by id.
    pub fn invalidate(&mut self, id: BlockId) -> bool {
        if self.slot(id).is_none() {
            return false;
        }
        self.remove(id.slot);
        true
    }

    /// Drop the oldest resident block. Returns whether one went.
    fn evict_one(&mut self) -> bool {
        while let Some((id, stamp)) = self.order.pop_front() {
            let live = self
                .slots
                .get(id as usize)
                .and_then(|s| s.as_ref())
                .is_some_and(|s| s.stamp == stamp);
            if live {
                self.remove(id);
                self.stats.evictions += 1;
                return true;
            }
        }
        false
    }

    fn remove(&mut self, id: u32) {
        let page = self
            .slots
            .get(id as usize)
            .and_then(|s| s.as_ref())
            .map(|s| s.page);
        if let Some(page) = page
            && let Some(list) = self.pages.get_mut(&page)
        {
            list.retain(|x| *x != id);
            if list.is_empty() {
                self.pages.remove(&page);
            }
        }
        self.remove_keeping_page(id);
    }

    /// The half of removal that does not touch the page index, so
    /// [`BlockCache::invalidate_page`] can drop the whole list at once.
    fn remove_keeping_page(&mut self, id: u32) {
        let Some(slot) = self.slots.get_mut(id as usize).and_then(|s| s.take()) else {
            return;
        };
        // Clear every exit this block held, so its successors do not keep back
        // edges to a slot that no longer exists.
        for (i, link) in slot.exits.iter().enumerate() {
            if link.target != NONE {
                self.drop_back_edge(link.target, link.stamp, id, i as u8);
            }
        }
        // Clear every exit pointing *at* this block. This is the direction
        // that is easy to forget and impossible to notice without the stamp.
        for (pred, i) in &slot.preds {
            if let Some(p) = self.slots.get_mut(*pred as usize).and_then(|s| s.as_mut())
                && let Some(link) = p.exits.get_mut(*i as usize)
                && link.target == id
            {
                *link = Link::empty();
                self.stats.unlinks += 1;
            }
        }
        // Unlink from the hash chain.
        let bucket = self.bucket(slot.pc, slot.key);
        let mut at = self.buckets[bucket];
        if at == id {
            self.buckets[bucket] = slot.next;
        } else {
            while at != NONE {
                let next = match self.slots.get(at as usize).and_then(|s| s.as_ref()) {
                    Some(s) => s.next,
                    None => break,
                };
                if next == id {
                    if let Some(s) = self.slots.get_mut(at as usize).and_then(|s| s.as_mut()) {
                        s.next = slot.next;
                    }
                    break;
                }
                at = next;
            }
        }
        self.free.push(id);
    }

    /// Remove `(pred, i)` from `target`'s back edges.
    fn drop_back_edge(&mut self, target: u32, stamp: u64, pred: u32, i: u8) {
        if let Some(t) = self.slots.get_mut(target as usize).and_then(|s| s.as_mut())
            && t.stamp == stamp
        {
            t.preds.retain(|(p, s)| !(*p == pred && *s == i));
        }
    }

    #[inline]
    fn bucket(&self, pc: u64, key: u64) -> usize {
        // A fixed multiplicative mix, so the bucket a block lands in is the
        // same on every host and in every run. Nothing iterates buckets, but
        // a cache whose collision behaviour varied by host would make a
        // reproducer stop reproducing.
        let mut h = pc ^ key.wrapping_mul(0x9e37_79b9_7f4a_7c15);
        h ^= h >> 29;
        h = h.wrapping_mul(0xbf58_476d_1ce4_e5b9);
        h ^= h >> 32;
        (h & self.bucket_mask) as usize
    }

    /// Check every invariant this module maintains, and say which one broke.
    ///
    /// Exported because the fuzz target and the tests both drive it: a cache
    /// that survives ten thousand random inserts and invalidations with every
    /// back edge still symmetric is evidence, where a passing lookup is not.
    ///
    /// # Errors
    ///
    /// A description of the first inconsistency found.
    pub fn check(&self) -> Result<(), String> {
        use alloc::format;
        for (i, slot) in self.slots.iter().enumerate() {
            let Some(slot) = slot else { continue };
            let i = i as u32;
            for (n, link) in slot.exits.iter().enumerate() {
                if link.target == NONE {
                    continue;
                }
                let Some(t) = self
                    .slots
                    .get(link.target as usize)
                    .and_then(|s| s.as_ref())
                else {
                    return Err(format!("block {i} exit {n} points at a freed slot"));
                };
                if t.stamp != link.stamp {
                    return Err(format!("block {i} exit {n} points at a reused slot"));
                }
                if !t.preds.contains(&(i, n as u8)) {
                    return Err(format!("block {i} exit {n} has no matching back edge"));
                }
            }
            for (pred, n) in &slot.preds {
                let Some(p) = self.slots.get(*pred as usize).and_then(|s| s.as_ref()) else {
                    return Err(format!("block {i} has a back edge from a freed slot"));
                };
                let Some(link) = p.exits.get(*n as usize) else {
                    return Err(format!(
                        "block {i} has a back edge to exit {n}, which does not exist"
                    ));
                };
                if link.target != i {
                    return Err(format!(
                        "block {i} has a back edge from {pred} exit {n}, which points elsewhere"
                    ));
                }
            }
            match self.pages.get(&slot.page) {
                Some(list) if list.contains(&i) => {}
                _ => return Err(format!("block {i} is not in its page's index")),
            }
            if !self.filter_set(slot.page) {
                return Err(format!("block {i}'s page is not in the filter"));
            }
        }
        for (page, list) in &self.pages {
            for id in list {
                match self.slots.get(*id as usize).and_then(|s| s.as_ref()) {
                    Some(s) if s.page == *page => {}
                    _ => return Err(format!("page {page:#x} indexes a block that is not there")),
                }
            }
        }
        if self.stats.stale_links != 0 {
            return Err(format!(
                "{} stale links were followed, so a back edge was not cleared",
                self.stats.stale_links
            ));
        }
        Ok(())
    }
}

impl Default for BlockCache {
    fn default() -> BlockCache {
        BlockCache::new()
    }
}

/// Which bit of the page filter a page occupies.
#[inline]
fn filter_bit(page: u64, words: usize) -> usize {
    let bits = (words * 64) as u64;
    let mixed = (page >> 12).wrapping_mul(0x9e37_79b9_7f4a_7c15);
    ((mixed >> 32) % bits) as usize
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ir::{BlockBuilder, Const, Type};

    /// A distinguishable one-instruction block.
    fn block(pc: u64) -> Block {
        let mut b = BlockBuilder::new(pc, 0);
        let _ = b.imm(Type::I64, Const::Int(u128::from(pc)));
        b.exit_tb();
        b.finish()
    }

    fn cache() -> BlockCache {
        BlockCache::with_capacity(16)
    }

    #[test]
    fn a_block_comes_back_under_its_own_key_and_no_other() {
        let mut c = cache();
        let id = c.insert(0x1000, 7, 0x1000, 1, block(0x1000));
        assert_eq!(c.lookup(0x1000, 7), Some(id));
        assert_eq!(
            c.lookup(0x1000, 8),
            None,
            "a different key is a different block"
        );
        assert_eq!(c.lookup(0x1004, 7), None);
        assert_eq!(c.block(id).map(|b| b.entry_pc), Some(0x1000));
        c.check().expect("consistent");
    }

    #[test]
    fn a_chained_exit_skips_the_lookup() {
        let mut c = cache();
        let a = c.insert(0x1000, 0, 0x1000, 1, block(0x1000));
        let b = c.insert(0x1004, 0, 0x1000, 1, block(0x1004));
        c.link(a, 0x1004, b);
        assert_eq!(c.follow(a, 0x1004, 0), Some(b));
        assert_eq!(c.stats().chained, 1);
        assert_eq!(c.follow(a, 0x2000, 0), None, "an unpatched exit");
        c.check().expect("consistent");
    }

    #[test]
    fn a_chain_link_is_cleared_when_its_target_is_invalidated() {
        // The unpatch. Without the back edges `a` keeps a live index into a
        // freed slot, and the next block to take that slot is executed at
        // `a`'s exit.
        let mut c = cache();
        let a = c.insert(0x1000, 0, 0x1000, 1, block(0x1000));
        let b = c.insert(0x1004, 0, 0x2000, 1, block(0x1004));
        c.link(a, 0x1004, b);
        assert_eq!(c.invalidate_page(0x2000), 1);
        assert_eq!(c.follow(a, 0x1004, 0), None);
        assert_eq!(c.stats().unlinks, 1);
        assert_eq!(c.stats().stale_links, 0, "cleared, not merely detected");
        c.check().expect("consistent");
    }

    #[test]
    fn invalidating_a_predecessor_leaves_no_back_edge_behind() {
        let mut c = cache();
        let a = c.insert(0x1000, 0, 0x1000, 1, block(0x1000));
        let b = c.insert(0x1004, 0, 0x2000, 1, block(0x1004));
        c.link(a, 0x1004, b);
        c.invalidate(a);
        c.check().expect("b's back edge went with a");
        // The slot a occupied is reused; b must not now believe a points at it.
        let d = c.insert(0x3000, 0, 0x3000, 1, block(0x3000));
        assert_eq!(c.follow(d, 0x1004, 0), None);
        c.check().expect("consistent");
    }

    #[test]
    fn a_reused_slot_is_never_reached_through_an_old_link() {
        let mut c = cache();
        let a = c.insert(0x1000, 0, 0x1000, 1, block(0x1000));
        let b = c.insert(0x1004, 0, 0x2000, 1, block(0x1004));
        c.link(a, 0x1004, b);
        c.invalidate(b);
        // The freed slot is taken by a completely different block.
        let e = c.insert(0x9000, 0, 0x9000, 1, block(0x9000));
        assert_ne!(c.follow(a, 0x1004, 0), Some(e));
        assert_eq!(c.stats().stale_links, 0);
        c.check().expect("consistent");
    }

    #[test]
    fn a_guest_write_into_a_page_holding_translations_invalidates_them() {
        let mut c = cache();
        let a = c.insert(0x1000, 0, 0x1000, 1, block(0x1000));
        let b = c.insert(0x1010, 0, 0x1000, 1, block(0x1010));
        let elsewhere = c.insert(0x5000, 0, 0x5000, 1, block(0x5000));
        assert_eq!(c.note_write(0x1008, 4), 2, "both blocks on the page went");
        assert_eq!(c.lookup(0x1000, 0), None);
        assert_eq!(c.lookup(0x1010, 0), None);
        assert_eq!(c.lookup(0x5000, 0), Some(elsewhere));
        let _ = (a, b);
        c.check().expect("consistent");
    }

    #[test]
    fn a_write_spanning_two_pages_invalidates_both() {
        let mut c = cache();
        c.insert(0x1000, 0, 0x1000, 1, block(0x1000));
        c.insert(0x2000, 0, 0x2000, 1, block(0x2000));
        assert_eq!(c.note_write(0x1ffe, 8), 2);
        c.check().expect("consistent");
    }

    #[test]
    fn a_write_into_a_page_with_no_translations_costs_a_filter_test() {
        let mut c = cache();
        c.insert(0x1000, 0, 0x1000, 1, block(0x1000));
        assert_eq!(c.note_write(0x8_0000, 8), 0);
        assert!(c.stats().filtered >= 1);
        c.check().expect("consistent");
    }

    #[test]
    fn a_topology_change_invalidates_every_cached_block() {
        // `Origin::Bare` contributes nothing to `Block::key`, so a block
        // lifted before a remap keys identically to one lifted after. If this
        // flush goes away, a machine-mode guest gets its old translation of a
        // page that has since been replaced.
        let mut c = cache();
        c.insert(0x1000, 0, 0x1000, 1, block(0x1000));
        assert!(c.sync(Epoch {
            topology: 1,
            translation: 0
        }));
        assert_eq!(c.lookup(0x1000, 0), None);
        assert!(c.is_empty());
    }

    #[test]
    fn a_translation_generation_bump_leaves_bare_blocks_alone() {
        // A block's key already carries the translation generation, so an
        // `SFENCE.VMA` makes a virtual block unreachable without touching the
        // physical ones. Flushing here would be correct and slow.
        let mut c = cache();
        let bare = c.insert(0x1000, 0, 0x1000, 1, block(0x1000));
        assert!(!c.sync(Epoch {
            topology: 0,
            translation: 9
        }));
        assert_eq!(c.lookup(0x1000, 0), Some(bare));
    }

    #[test]
    fn the_cache_evicts_in_insertion_order_and_stays_consistent() {
        let mut c = BlockCache::with_capacity(4);
        let mut ids = Vec::new();
        for n in 0..8u64 {
            ids.push(c.insert(n * 0x1000, 0, n * 0x1000, 1, block(n * 0x1000)));
            if n > 0 {
                c.link(ids[(n - 1) as usize], n * 0x1000, ids[n as usize]);
            }
            c.check().expect("consistent at every step");
        }
        assert_eq!(c.len(), 4);
        assert_eq!(c.lookup(0, 0), None, "the first went first");
        assert_eq!(c.stats().evictions, 4);
        assert_eq!(c.stats().stale_links, 0);
    }

    #[test]
    fn reinserting_the_same_key_replaces_rather_than_duplicates() {
        let mut c = cache();
        let first = c.insert(0x1000, 0, 0x1000, 1, block(0x1000));
        let second = c.insert(0x1000, 0, 0x1000, 1, block(0x1000));
        assert_ne!(first, second);
        assert_eq!(c.len(), 1);
        assert_eq!(c.lookup(0x1000, 0), Some(second));
        c.check().expect("consistent");
    }

    #[test]
    fn a_block_with_more_successors_than_exits_keeps_two_and_looks_the_rest_up() {
        let mut c = cache();
        let a = c.insert(0x9_0000, 0, 0x9_0000, 1, block(0x9_0000));
        let targets: Vec<BlockId> = (1..4)
            .map(|n| c.insert(n * 0x1000, 0, n * 0x1000, 1, block(n * 0x1000)))
            .collect();
        for (n, t) in targets.iter().enumerate() {
            c.link(a, (n as u64 + 1) * 0x1000, *t);
        }
        let followed = (1..4)
            .filter(|n| c.follow(a, n * 0x1000, 0).is_some())
            .count();
        assert_eq!(followed, EXITS);
        c.check().expect("consistent");
    }

    #[test]
    fn a_link_replaced_at_the_same_exit_pc_drops_the_old_back_edge() {
        let mut c = cache();
        let a = c.insert(0x1000, 0, 0x1000, 1, block(0x1000));
        let b = c.insert(0x2000, 0, 0x2000, 1, block(0x2000));
        c.link(a, 0x2000, b);
        c.invalidate(b);
        // Same exit PC, a freshly translated successor.
        let b2 = c.insert(0x2000, 0, 0x2000, 1, block(0x2000));
        c.link(a, 0x2000, b2);
        assert_eq!(c.follow(a, 0x2000, 0), Some(b2));
        c.check().expect("consistent");
        assert_eq!(c.stats().stale_links, 0);
    }

    #[test]
    fn a_flush_leaves_nothing_pointing_anywhere() {
        let mut c = cache();
        let a = c.insert(0x1000, 0, 0x1000, 1, block(0x1000));
        let b = c.insert(0x2000, 0, 0x2000, 1, block(0x2000));
        c.link(a, 0x2000, b);
        c.flush();
        assert!(c.is_empty());
        assert_eq!(c.lookup(0x1000, 0), None);
        c.check().expect("consistent");
    }
}
