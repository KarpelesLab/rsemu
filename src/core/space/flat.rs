//! Flattening: turning the region tree into a sorted, non-overlapping view.
//!
//! The tree is what the machine file describes; the flat view is a derived
//! cache, rebuilt on retopology and never on rebase (`ROADMAP.md` §4.1, and
//! §15 invariant 3 — it must be reconstructible from architectural state
//! alone, and it is: [`FlatView::build`] is a pure function of the region
//! tree).
//!
//! # Where a rebase lands
//!
//! Each flat leaf keeps its offset into the backing store in an
//! [`AtomicU64`], computed as a fixed part plus the current value of every
//! alias cell the flattening walked through. Sliding an alias therefore costs
//! *one atomic store per affected flat entry* and touches nothing else: not
//! the entry list, not its ordering, not the dispatch table, not the
//! generation counter.

use super::attrs::{AccessConstraints, MemAttrs, MemOps, MemResult};
use super::region::{AliasId, CombinePolicy, Mapping, RegionKind, RegionRef, RomWrite};
use super::store::{RamStore, RomStore};
use crate::core::error::{BusError, Error};
use crate::core::value::{Endian, Width};
use alloc::collections::BTreeMap;
use alloc::string::ToString;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::sync::atomic::{AtomicU64, Ordering};

/// How deep the region tree may nest before flattening gives up.
///
/// Cycles are impossible by construction, so this only bounds absurd
/// descriptions — and bounds the recursion, which matters on a small wasm
/// stack.
const MAX_DEPTH: u32 = 64;

/// Where a flat entry ultimately sends an access.
#[derive(Debug, Clone)]
#[non_exhaustive]
pub enum FlatTarget {
    /// Writable memory. This is the fast path.
    Ram(Arc<RamStore>),
    /// Read-only memory.
    Rom {
        /// The contents.
        store: Arc<RomStore>,
        /// What a write does.
        on_write: RomWrite,
    },
    /// MMIO: a call, every time.
    Io(Arc<dyn MemOps>),
}

/// One resolved leaf: a backing store plus the offset the entry starts at.
#[derive(Debug)]
pub struct FlatLeaf {
    target: FlatTarget,
    /// Cached `fixed + sum(terms)`. Updated in place by a rebase.
    offset: AtomicU64,
    fixed: u64,
    terms: Vec<(AliasId, Arc<AtomicU64>)>,
    /// Set for a repeating window: the offset wraps at this many bytes.
    period: Option<u64>,
    constraints: AccessConstraints,
}

impl FlatLeaf {
    /// The backing store this leaf resolves to.
    #[must_use]
    pub fn target(&self) -> &FlatTarget {
        &self.target
    }

    /// The offset into the backing store corresponding to the entry's start
    /// address.
    #[inline]
    #[must_use]
    pub fn offset(&self) -> u64 {
        self.offset.load(Ordering::Relaxed)
    }

    /// What this leaf accepts.
    #[inline]
    #[must_use]
    pub fn constraints(&self) -> AccessConstraints {
        self.constraints
    }

    /// Whether any alias offset feeds into this leaf's offset.
    #[must_use]
    pub fn is_rebasable(&self) -> bool {
        !self.terms.is_empty()
    }

    /// The repeat period, if this leaf is reached through a repeating window.
    #[inline]
    #[must_use]
    pub fn period(&self) -> Option<u64> {
        self.period
    }

    /// The offset in the backing store for a byte `rel` past the entry start.
    #[inline]
    #[must_use]
    pub fn offset_of(&self, rel: u64) -> u64 {
        let raw = self.offset().wrapping_add(rel);
        match self.period {
            Some(p) => raw % p,
            None => raw,
        }
    }

    /// How many bytes may be transferred from `rel` in one call before the
    /// window wraps.
    #[inline]
    #[must_use]
    pub fn run_len(&self, rel: u64) -> u64 {
        match self.period {
            Some(p) => p - self.offset_of(rel) % p,
            None => u64::MAX,
        }
    }

    /// Recompute the cached offset from the alias cells. One relaxed store.
    ///
    /// Relaxed is deliberate. Making a rebase visible to another CPU thread at
    /// a defined point is the safe-point protocol's job (`ROADMAP.md` §4.7);
    /// paying for an acquire on every guest load to half-solve it here would
    /// buy nothing.
    #[inline]
    pub(super) fn recompute(&self) {
        let mut v = self.fixed;
        for (_, cell) in &self.terms {
            v = v.wrapping_add(cell.load(Ordering::Relaxed));
        }
        self.offset.store(v, Ordering::Relaxed);
    }

    #[inline]
    fn check(&self, off: u64, len: u64, width: Option<Width>, attrs: MemAttrs) -> MemResult {
        match width {
            Some(w) => self.constraints.check(off, w, attrs),
            None => self.constraints.check_bulk(off, len, attrs),
        }
    }

    /// Read from this leaf, `rel` bytes past the entry's start.
    #[inline]
    pub fn read(
        &self,
        rel: u64,
        dst: &mut [u8],
        attrs: MemAttrs,
        width: Option<Width>,
    ) -> MemResult {
        let off = self.offset_of(rel);
        self.check(off, dst.len() as u64, width, attrs)?;
        match &self.target {
            FlatTarget::Ram(s) => s.read_at(off, dst),
            FlatTarget::Rom { store, .. } => store.read_at(off, dst),
            FlatTarget::Io(ops) => ops.read(off, dst, attrs),
        }
    }

    /// Write to this leaf, `rel` bytes past the entry's start.
    #[inline]
    pub fn write(&self, rel: u64, src: &[u8], attrs: MemAttrs, width: Option<Width>) -> MemResult {
        let off = self.offset_of(rel);
        self.check(off, src.len() as u64, width, attrs)?;
        match &self.target {
            FlatTarget::Ram(s) => s.write_at(off, src),
            FlatTarget::Rom { on_write, .. } => match on_write {
                RomWrite::Ignore => Ok(()),
                RomWrite::Fault => Err(BusError::BadAccess),
            },
            FlatTarget::Io(ops) => ops.write(off, src, attrs),
        }
    }
}

/// What a flat entry dispatches to: one leaf, or several that combine.
#[derive(Debug)]
#[non_exhaustive]
pub enum EntryKind {
    /// Exactly one region covers this range — the [`CombinePolicy::Priority`]
    /// outcome, and the only shape a machine that is not an open-bus system
    /// ever produces.
    Single(FlatLeaf),
    /// Several regions cover this range and the container asked for them to be
    /// combined rather than ranked.
    Combine {
        /// How to combine them.
        policy: CombinePolicy,
        /// Members, highest priority first.
        members: Vec<FlatLeaf>,
    },
}

/// One range of the flat view: contiguous, non-overlapping, sorted by
/// `start`.
#[derive(Debug)]
pub struct FlatEntry {
    start: u64,
    len: u64,
    kind: EntryKind,
    conflicts: AtomicU64,
}

impl FlatEntry {
    /// First address covered.
    #[inline]
    #[must_use]
    pub fn start(&self) -> u64 {
        self.start
    }

    /// Length in bytes.
    #[inline]
    #[must_use]
    pub fn len(&self) -> u64 {
        self.len
    }

    /// Whether the entry is zero-sized. It never is; the accessor exists
    /// because `len` without `is_empty` is a lint.
    #[inline]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// One past the last address covered.
    #[inline]
    #[must_use]
    pub fn end(&self) -> u64 {
        self.start.wrapping_add(self.len)
    }

    /// What this entry dispatches to.
    #[must_use]
    pub fn kind(&self) -> &EntryKind {
        &self.kind
    }

    /// The single leaf, when there is one.
    #[inline]
    #[must_use]
    pub fn leaf(&self) -> Option<&FlatLeaf> {
        match &self.kind {
            EntryKind::Single(l) => Some(l),
            EntryKind::Combine { .. } => None,
        }
    }

    /// Byte order for a width-typed access here.
    ///
    /// A combined entry answers with its highest-priority member: mixing byte
    /// orders inside one wired-or range is not a configuration the core tries
    /// to make sense of.
    #[inline]
    #[must_use]
    pub fn endian(&self) -> Endian {
        match &self.kind {
            EntryKind::Single(l) => l.constraints.endian,
            EntryKind::Combine { members, .. } => members
                .first()
                .map_or(Endian::Little, |m| m.constraints.endian),
        }
    }

    /// Whether a read of this entry drives the master's external data bus.
    ///
    /// A combined entry answers with its highest-priority member, the same way
    /// [`Entry::endian`] does: a wired-or of an on-die register and an external
    /// one is not a board the core tries to make sense of.
    #[inline]
    #[must_use]
    pub fn drives_data_bus(&self) -> bool {
        match &self.kind {
            EntryKind::Single(l) => l.constraints.drives_data_bus,
            EntryKind::Combine { members, .. } => members
                .first()
                .is_none_or(|m| m.constraints.drives_data_bus),
        }
    }

    /// Whether this entry is plain RAM, with no combining — the case the
    /// dispatch table flags as its fast path.
    #[inline]
    #[must_use]
    pub fn is_direct_ram(&self) -> bool {
        matches!(&self.kind, EntryKind::Single(l) if matches!(l.target, FlatTarget::Ram(_)))
    }

    /// How many bytes may be transferred in one call starting `rel` bytes
    /// into this entry.
    ///
    /// The entry's remaining length, unless a repeating window wraps sooner: a
    /// transfer that crossed the wrap would have to be split, and splitting it
    /// silently inside a device call is not something the device could see.
    #[inline]
    #[must_use]
    pub fn run_len(&self, rel: u64) -> u64 {
        let avail = self.len.saturating_sub(rel);
        let bound = match &self.kind {
            EntryKind::Single(l) => l.run_len(rel),
            EntryKind::Combine { members, .. } => members
                .iter()
                .map(|m| m.run_len(rel))
                .min()
                .unwrap_or(u64::MAX),
        };
        avail.min(bound)
    }

    /// How many times a [`CombinePolicy::Conflict`] range saw more than one
    /// responder.
    #[must_use]
    pub fn conflicts(&self) -> u64 {
        self.conflicts.load(Ordering::Relaxed)
    }

    /// Read `dst.len()` bytes starting `rel` bytes into this entry.
    pub fn read(
        &self,
        rel: u64,
        dst: &mut [u8],
        attrs: MemAttrs,
        width: Option<Width>,
    ) -> MemResult {
        match &self.kind {
            EntryKind::Single(l) => l.read(rel, dst, attrs, width),
            EntryKind::Combine { policy, members } => {
                let fill = if matches!(policy, CombinePolicy::WiredAnd) {
                    0xffu8
                } else {
                    0x00
                };
                dst.fill(fill);
                let mut scratch = alloc::vec![0u8; dst.len()];
                let mut responders = 0u32;
                let mut first_err = None;
                for m in members {
                    match m.read(rel, &mut scratch, attrs, width) {
                        Ok(()) => {
                            responders += 1;
                            for (d, s) in dst.iter_mut().zip(scratch.iter()) {
                                *d = match policy {
                                    CombinePolicy::WiredAnd => *d & *s,
                                    _ => *d | *s,
                                };
                            }
                        }
                        // A retry once another member has already been read is
                        // a retry after a side effect, and is rejected.
                        Err(BusError::Retry) if responders > 0 => {
                            return Err(BusError::BadAccess);
                        }
                        Err(e) => first_err = first_err.or(Some(e)),
                    }
                }
                if responders == 0 {
                    return Err(first_err.unwrap_or(BusError::Unassigned));
                }
                if responders > 1 && matches!(policy, CombinePolicy::Conflict) {
                    self.conflicts.fetch_add(1, Ordering::Relaxed);
                }
                Ok(())
            }
        }
    }

    /// Write `src` starting `rel` bytes into this entry.
    ///
    /// A combined entry broadcasts: every member sees the write, and the
    /// first error is reported once they all have. That is what a wired bus
    /// does, and stopping halfway would leave the members disagreeing.
    pub fn write(&self, rel: u64, src: &[u8], attrs: MemAttrs, width: Option<Width>) -> MemResult {
        match &self.kind {
            EntryKind::Single(l) => l.write(rel, src, attrs, width),
            EntryKind::Combine { members, .. } => {
                let mut accepted = 0u32;
                let mut first_err = None;
                for m in members {
                    match m.write(rel, src, attrs, width) {
                        Ok(()) => accepted += 1,
                        Err(BusError::Retry) if accepted > 0 => return Err(BusError::BadAccess),
                        Err(e) => first_err = first_err.or(Some(e)),
                    }
                }
                if accepted == 0 {
                    return Err(first_err.unwrap_or(BusError::Unassigned));
                }
                Ok(())
            }
        }
    }

    fn for_each_leaf(&self, mut f: impl FnMut(usize, &FlatLeaf)) {
        match &self.kind {
            EntryKind::Single(l) => f(0, l),
            EntryKind::Combine { members, .. } => {
                for (i, m) in members.iter().enumerate() {
                    f(i, m);
                }
            }
        }
    }

    fn leaf_at(&self, index: usize) -> Option<&FlatLeaf> {
        match &self.kind {
            EntryKind::Single(l) if index == 0 => Some(l),
            EntryKind::Single(_) => None,
            EntryKind::Combine { members, .. } => members.get(index),
        }
    }
}

/// Which flat leaves an alias's offset feeds into.
///
/// A `BTreeMap` rather than a hash map because iteration order of anything
/// that can affect guest-visible state has to be deterministic (`CLAUDE.md`).
pub(super) type RebaseIndex = BTreeMap<AliasId, Vec<(u32, u32)>>;

/// The flattened, sorted, non-overlapping view of an address space.
#[derive(Debug, Default)]
pub struct FlatView {
    entries: Vec<FlatEntry>,
}

impl FlatView {
    /// Flatten `children` — the root container's mappings — into a view
    /// covering `[0, limit)`.
    ///
    /// # Errors
    ///
    /// If the tree nests deeper than the flattener will walk.
    pub fn build(
        children: &[Mapping],
        limit: u64,
        combine: CombinePolicy,
    ) -> Result<(FlatView, RebaseIndex), Error> {
        let mut pieces = Vec::new();
        resolve_children(children, limit, 0, limit, combine, 0, &mut pieces)?;

        let entries: Vec<FlatEntry> = pieces
            .into_iter()
            .map(|p| FlatEntry {
                start: p.start,
                len: p.len,
                kind: match p.leaves.len() {
                    1 => EntryKind::Single(p.leaves.into_iter().next().expect("len 1").into_leaf()),
                    _ => EntryKind::Combine {
                        policy: p.combine,
                        members: p.leaves.into_iter().map(LeafSpec::into_leaf).collect(),
                    },
                },
                conflicts: AtomicU64::new(0),
            })
            .collect();

        let mut index: RebaseIndex = BTreeMap::new();
        for (e, entry) in entries.iter().enumerate() {
            entry.for_each_leaf(|m, leaf| {
                for (id, _) in &leaf.terms {
                    index
                        .entry(*id)
                        .or_default()
                        .push((e as u32, u32::try_from(m).unwrap_or(u32::MAX)));
                }
            });
        }
        Ok((FlatView { entries }, index))
    }

    /// The entries, sorted by start address.
    #[must_use]
    pub fn entries(&self) -> &[FlatEntry] {
        &self.entries
    }

    /// How many entries the view has.
    #[inline]
    #[must_use]
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether nothing is mapped.
    #[inline]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// The entry covering `addr`, by index.
    ///
    /// A binary search over a sorted array: this is the slow path, and the
    /// only path for a space with no dense dispatch table.
    #[inline]
    #[must_use]
    pub fn find(&self, addr: u64) -> Option<usize> {
        let i = self.entries.partition_point(|e| e.start <= addr);
        if i == 0 {
            return None;
        }
        let e = &self.entries[i - 1];
        if addr < e.end() { Some(i - 1) } else { None }
    }

    /// The entry at `index`.
    #[inline]
    #[must_use]
    pub fn entry(&self, index: usize) -> Option<&FlatEntry> {
        self.entries.get(index)
    }

    /// One past the highest mapped address.
    #[must_use]
    pub fn extent(&self) -> u64 {
        self.entries.last().map_or(0, FlatEntry::end)
    }

    /// Apply a rebase: refresh every leaf that `id`'s offset feeds.
    pub(super) fn rebase(&self, index: &RebaseIndex, id: AliasId) {
        let Some(targets) = index.get(&id) else {
            return;
        };
        for (e, m) in targets {
            if let Some(entry) = self.entries.get(*e as usize)
                && let Some(leaf) = entry.leaf_at(*m as usize)
            {
                leaf.recompute();
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Flattening
// ---------------------------------------------------------------------------

/// A leaf under construction, before its atomic offset cell exists.
#[derive(Debug, Clone)]
struct LeafSpec {
    target: FlatTarget,
    fixed: u64,
    terms: Vec<(AliasId, Arc<AtomicU64>)>,
    period: Option<u64>,
    constraints: AccessConstraints,
}

impl LeafSpec {
    fn offset(&self) -> u64 {
        let mut v = self.fixed;
        for (_, cell) in &self.terms {
            v = v.wrapping_add(cell.load(Ordering::Relaxed));
        }
        v
    }

    fn into_leaf(self) -> FlatLeaf {
        let offset = AtomicU64::new(self.offset());
        FlatLeaf {
            target: self.target,
            offset,
            fixed: self.fixed,
            terms: self.terms,
            period: self.period,
            constraints: self.constraints,
        }
    }

    /// Whether two specs are the same store at continuing offsets, so the
    /// pieces they back can be merged into one entry.
    fn continues(&self, next: &LeafSpec, len: u64) -> bool {
        // A repeating window is already one piece; never fuse one with
        // anything, because "the offsets continue" is not true across a wrap.
        if self.period.is_some() || next.period.is_some() {
            return false;
        }
        let same_target = match (&self.target, &next.target) {
            (FlatTarget::Ram(a), FlatTarget::Ram(b)) => Arc::ptr_eq(a, b),
            (
                FlatTarget::Rom {
                    store: a,
                    on_write: wa,
                },
                FlatTarget::Rom {
                    store: b,
                    on_write: wb,
                },
            ) => Arc::ptr_eq(a, b) && wa == wb,
            (FlatTarget::Io(a), FlatTarget::Io(b)) => Arc::ptr_eq(a, b),
            _ => false,
        };
        same_target
            && self.constraints == next.constraints
            && self.terms.len() == next.terms.len()
            && self
                .terms
                .iter()
                .zip(next.terms.iter())
                .all(|(a, b)| a.0 == b.0 && Arc::ptr_eq(&a.1, &b.1))
            && self.fixed.wrapping_add(len) == next.fixed
    }
}

/// A resolved, non-overlapping range, relative to the window that was asked
/// for.
#[derive(Debug, Clone)]
struct Piece {
    start: u64,
    len: u64,
    leaves: Vec<LeafSpec>,
    combine: CombinePolicy,
}

/// A candidate placement inside a container, before overlaps are resolved.
///
/// Carries the whole piece's leaf list rather than one leaf, so that a nested
/// combining container survives being placed in a parent.
#[derive(Debug, Clone)]
struct Cand {
    start: u64,
    len: u64,
    priority: i32,
    seq: usize,
    leaves: Vec<LeafSpec>,
    combine: CombinePolicy,
}

/// The backing store of a leaf region, or `None` if it is a tree.
fn leaf_target(region: &RegionRef) -> Option<FlatTarget> {
    match region.kind() {
        RegionKind::Ram(store) => Some(FlatTarget::Ram(store.clone())),
        RegionKind::Rom { store, on_write } => Some(FlatTarget::Rom {
            store: store.clone(),
            on_write: *on_write,
        }),
        RegionKind::Io(ops) => Some(FlatTarget::Io(ops.clone())),
        RegionKind::Alias(_) | RegionKind::Container(_) => None,
    }
}

fn resolve_region(
    region: &RegionRef,
    off: u64,
    len: u64,
    depth: u32,
    out: &mut Vec<Piece>,
) -> Result<(), Error> {
    if depth > MAX_DEPTH {
        return Err(Error::Config {
            at: region.name().to_string(),
            message: "region tree nests too deeply".to_string(),
        });
    }
    // Never look past the region's own end: a container decoded larger than
    // its contents leaves a hole, it does not wrap.
    let avail = region.len().saturating_sub(off);
    let len = len.min(avail);
    if len == 0 {
        return Ok(());
    }
    let constraints = region.constraints();
    match region.kind() {
        RegionKind::Ram(store) => out.push(Piece {
            start: 0,
            len,
            leaves: alloc::vec![LeafSpec {
                target: FlatTarget::Ram(store.clone()),
                fixed: off,
                terms: Vec::new(),
                period: None,
                constraints,
            }],
            combine: CombinePolicy::Priority,
        }),
        RegionKind::Rom { store, on_write } => out.push(Piece {
            start: 0,
            len,
            leaves: alloc::vec![LeafSpec {
                target: FlatTarget::Rom {
                    store: store.clone(),
                    on_write: *on_write,
                },
                fixed: off,
                terms: Vec::new(),
                period: None,
                constraints,
            }],
            combine: CombinePolicy::Priority,
        }),
        RegionKind::Io(ops) => out.push(Piece {
            start: 0,
            len,
            leaves: alloc::vec![LeafSpec {
                target: FlatTarget::Io(ops.clone()),
                fixed: off,
                terms: Vec::new(),
                period: None,
                constraints,
            }],
            combine: CombinePolicy::Priority,
        }),
        RegionKind::Alias(alias) if alias.repeats() => {
            // A repeating window is one entry with a modulus, not `len/period`
            // entries: the NES would otherwise flatten to 1024 PPU mirrors.
            let period = alias.period().expect("a repeating alias has a period");
            let target = leaf_target(alias.target()).ok_or_else(|| Error::Config {
                at: region.name().to_string(),
                message: "a repeating window's target must be a leaf".to_string(),
            })?;
            out.push(Piece {
                start: 0,
                len,
                leaves: alloc::vec![LeafSpec {
                    target,
                    fixed: off % period,
                    terms: Vec::new(),
                    period: Some(period),
                    constraints,
                }],
                combine: CombinePolicy::Priority,
            });
        }
        RegionKind::Alias(alias) => {
            let cur = alias.offset();
            let mut sub = Vec::new();
            resolve_region(
                alias.target(),
                cur.wrapping_add(off),
                len,
                depth + 1,
                &mut sub,
            )?;
            if constraints != alias.target().constraints() {
                // The window was given constraints of its own — a narrower
                // aperture onto a wider device — so they replace the target's.
                for piece in &mut sub {
                    for leaf in &mut piece.leaves {
                        leaf.constraints = constraints;
                    }
                }
            }
            if alias.is_rebasable() {
                // The alias's current offset is already inside every `fixed`;
                // move it out into a term so that sliding the cell moves the
                // window without a reflatten. Safe only because a rebasable
                // alias resolves to a leaf, so which store a piece points at
                // cannot depend on the offset.
                for piece in &mut sub {
                    for leaf in &mut piece.leaves {
                        leaf.fixed = leaf.fixed.wrapping_sub(cur);
                        leaf.terms.push((alias.id(), alias.cell().clone()));
                    }
                }
            }
            out.append(&mut sub);
        }
        RegionKind::Container(container) => {
            resolve_children(
                container.children(),
                region.len(),
                off,
                len,
                container.combine(),
                depth,
                out,
            )?;
        }
    }
    Ok(())
}

fn resolve_children(
    children: &[Mapping],
    limit: u64,
    off: u64,
    len: u64,
    combine: CombinePolicy,
    depth: u32,
    out: &mut Vec<Piece>,
) -> Result<(), Error> {
    let want_end = off.saturating_add(len);
    let mut cands: Vec<Cand> = Vec::new();
    for (seq, m) in children.iter().enumerate() {
        let child_start = m.base;
        let child_end = m.end().min(limit);
        let a = child_start.max(off);
        let b = child_end.min(want_end);
        if a >= b {
            continue;
        }
        let mut sub = Vec::new();
        resolve_region(&m.region, a - child_start, b - a, depth + 1, &mut sub)?;
        for p in sub {
            cands.push(Cand {
                start: a - off + p.start,
                len: p.len,
                priority: m.priority,
                seq,
                leaves: p.leaves,
                combine: p.combine,
            });
        }
    }
    resolve_overlaps(cands, combine, out);
    Ok(())
}

/// Sweep-line: cut the candidate set at every boundary, decide each elementary
/// interval, then merge the cuts that did not actually change anything.
fn resolve_overlaps(cands: Vec<Cand>, combine: CombinePolicy, out: &mut Vec<Piece>) {
    if cands.is_empty() {
        return;
    }
    let mut bounds: Vec<u64> = Vec::with_capacity(cands.len() * 2);
    for c in &cands {
        bounds.push(c.start);
        bounds.push(c.start.saturating_add(c.len));
    }
    bounds.sort_unstable();
    bounds.dedup();

    let mut pieces: Vec<Piece> = Vec::new();
    for w in bounds.windows(2) {
        let (a, b) = (w[0], w[1]);
        let mut covering: Vec<&Cand> = cands
            .iter()
            .filter(|c| c.start <= a && c.start.saturating_add(c.len) >= b)
            .collect();
        if covering.is_empty() {
            continue;
        }
        // Highest priority first; a later mapping wins a tie, which is the
        // rule a machine file can reason about.
        covering.sort_by(|x, y| y.priority.cmp(&x.priority).then_with(|| y.seq.cmp(&x.seq)));
        let take = if matches!(combine, CombinePolicy::Priority) {
            1
        } else {
            covering.len()
        };
        let leaves: Vec<LeafSpec> = covering[..take]
            .iter()
            .flat_map(|c| {
                c.leaves.iter().map(move |leaf| {
                    let mut leaf = leaf.clone();
                    leaf.fixed = leaf.fixed.wrapping_add(a - c.start);
                    leaf
                })
            })
            .collect();
        // A nested combining container keeps its own policy when the parent
        // ranks by priority; otherwise the parent's policy governs the group.
        let policy = if matches!(combine, CombinePolicy::Priority) {
            covering[0].combine
        } else {
            combine
        };
        let piece = Piece {
            start: a,
            len: b - a,
            leaves,
            combine: policy,
        };
        match pieces.last_mut() {
            Some(prev)
                if prev.start.wrapping_add(prev.len) == piece.start
                    && prev.combine == piece.combine
                    && prev.leaves.len() == piece.leaves.len()
                    && prev
                        .leaves
                        .iter()
                        .zip(piece.leaves.iter())
                        .all(|(x, y)| x.continues(y, prev.len)) =>
            {
                prev.len += piece.len;
            }
            _ => pieces.push(piece),
        }
    }
    out.append(&mut pieces);
}
